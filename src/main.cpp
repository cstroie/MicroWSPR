/**
  MicroWSPR - Arduino Nano GPS-disciplined WSPR beacon

  Copyright (C) 2021-2026 Costin STROIE <costinstroie@eridu.eu.org>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include "config.h"

#ifndef CALIBRATION
#define CALIBRATION (0)
#endif

#include <JTEncode.h>
#include <SoftwareSerial.h>
#include <Wire.h>

/**
 * Minimal Si5351 driver for MicroWSPR
 * Based on uSDX implementation - uses Wire library for I2C
 * Optimized for WSPR tone generation on CLK2
 */
#define SI5351_ADDR 0x60
#define F_XTAL 25004000UL

/**
 * Si5351 frequency synthesizer driver
 * 
 * Uses Integer-N PLL mode for WSPR tones which only need coarse frequency setting.
 * This avoids the heavy fractional-N math of the full library, saving ~10KB flash.
 * 
 * Key functions:
 *   init()       - Initialize I2C and disable all outputs
 *   set_freq()   - Set output frequency (fout in Hz)
 *   output_enable() - Enable/disable clock output
 *   set_correction() - Apply frequency correction (calibration)
 */
class SI5351 {
private:
  volatile int32_t _fout;
  volatile uint8_t _div;
  volatile uint16_t _msa128min512;
  volatile uint32_t _msb128;
  int16_t iqmsa;
  uint32_t fxtal;
  #define _MSC 0x80000

  /**
   * Write single byte to Si5351 register
   */
  void SendRegister(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(SI5351_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }

  /**
   * Write multiple bytes to Si5351 registers
   */
  void SendRegisterBulk(uint8_t reg, uint8_t* data, uint8_t n) {
    Wire.beginTransmission(SI5351_ADDR);
    Wire.write(reg);
    while (n--) Wire.write(*data++);
    Wire.endTransmission();
  }

public:
  /**
   * Initialize Si5351
   * @param csLoad - Crystal load capacitance (8 = 8pF, ignored in this minimal driver)
   * @param - ignored parameter for compatibility
   * @param - ignored parameter for compatibility
   * @return true (always succeeds in minimal driver)
   */
  bool init(uint8_t, uint32_t, int32_t) {
    Wire.begin();
    Wire.setClock(400000UL);
    SendRegister(3, 0xFF);
    for (uint8_t i = 0; i < 6; i++) SendRegister(16 + i, 0x80);
    SendRegister(3, 0xFF);
    return true;
  }

  /**
   * Set frequency correction (calibration offset)
   * @param corr - Correction value in Hz (subtracted from nominal crystal frequency)
   * @param - ignored parameter for compatibility
   */
  void set_correction(int32_t corr, uint8_t) {
    fxtal = F_XTAL - corr;
  }

  /**
   * Set output frequency
   * @param fout - Desired output frequency in Hz
   * @param clk - Clock output (0, 1, or 2) - only CLK2 used for WSPR
   * 
   * Uses integer-N PLL mode for simplicity. For WSPR tones this provides
   * adequate precision. The frequency is first divided to stay within
   * the PLL's usable range, then multiplied back up.
   */
  void set_freq(uint32_t fout, uint8_t clk) {
    uint8_t rdiv = 0;
    if (fout < 500000) { rdiv = 7; fout *= 128; }
    uint16_t d = (16 * fxtal) / fout;
    if (fout > 30000000) d = (34 * fxtal) / fout;
    if ((d * (fout - 5000) / fxtal) != (d * (fout + 5000) / fxtal)) d--;
    uint32_t fvcoa = d * fout;
    uint8_t msa = fvcoa / fxtal;
    uint32_t msb = ((uint64_t)(fvcoa % fxtal) * _MSC * 128) / fxtal;
    uint32_t msp1 = 128 * msa + 128 * msb / _MSC - 512;
    uint32_t msp2 = 128 * msb - 128 * msb / _MSC * _MSC;
    uint8_t pll_regs[8] = {
      (uint8_t)((_MSC >> 8) & 0xFF),
      (uint8_t)(_MSC & 0xFF),
      (uint8_t)(msp1 >> 16),
      (uint8_t)(msp1 >> 8),
      (uint8_t)(msp1),
      (uint8_t)(((_MSC >> 12) & 0xF0) | (msp2 >> 16)),
      (uint8_t)(msp2 >> 8),
      (uint8_t)(msp2)
    };
    SendRegisterBulk(34, pll_regs, 8);  // PLLB only (CLK2 uses PLLB per reg 18)
    msp1 = (128 * msa - 512) | (((uint32_t)rdiv) << 20);
    uint8_t ms_regs[8] = {0, 1, (uint8_t)(msp1 >> 16), (uint8_t)(msp1 >> 8), (uint8_t)(msp1), 0, 0, 0};
    SendRegisterBulk(58, ms_regs, 8);   // MS2 only (42+16, CLK2)
    SendRegister(18, 0x6C);             // CLK2: PLLB, integer mode, inverted, 6mA
    if (iqmsa != msa) { iqmsa = msa; SendRegister(177, 0xA0); }
    _fout = fout; _div = d; _msa128min512 = msa * 128 - 512; _msb128 = msb;
  }

  /**
   * Enable or disable clock output
   * @param clk - Clock output (0, 1, or 2)
   * @param enable - 1 to enable, 0 to disable
   */
  void output_enable(uint8_t clk, uint8_t enable) {
    if (enable) SendRegister(3, ~(1 << clk));  // clear bit → enable output
    else        SendRegister(3, 0xFF);          // all bits set → all disabled
  }

  /**
   * Set drive strength
   * @param clk - Clock output (0, 1, or 2)
   * @param strength - Drive strength (0=2mA, 1=4mA, 2=6mA, 3=8mA)
   */
  void drive_strength(uint8_t clk, uint8_t strength) {
    uint8_t val = RecvRegister(16 + clk);
    SendRegister(16 + clk, (val & 0xF9) | (strength << 1));
  }

  /**
   * Read single byte from Si5351 register
   */
  uint8_t RecvRegister(uint8_t reg) {
    Wire.beginTransmission(SI5351_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(SI5351_ADDR, (uint8_t)1);
    return Wire.read();
  }
};

SI5351 DDS;

uint8_t txBuf[WSPR_SYMBOL_COUNT];

const uint16_t wsprToneSep = round(1000UL * 12000 / 8192);  // 1.4648 Hz (stored as mHz)
const uint16_t wsprToneDur = round(1000UL * 8192 / 12000);  // 683 ms
const uint32_t wsprBaseFrq[] = {
  0UL,
  136000UL,   474200UL,   1836600UL,  3568600UL,
  5287200UL,  7038600UL,  10138700UL, 14095600UL, 18104600UL,
  21094600UL, 24924600UL, 28124600UL, 50293000UL, 144489000UL
};

enum HAM_BANDS {
  BAND_OFF,
  BAND_2190, BAND_630,  BAND_160,
  BAND_80,   BAND_60,   BAND_40,  BAND_30,  BAND_20,
  BAND_17,   BAND_15,   BAND_12,  BAND_10,  BAND_6,   BAND_2
};

// Working locator: set from cfg.locator on boot, updated by GPS when cfg.locator is empty
char     loc[7];
// Current band (HAM_BANDS value, 1-14); 0 = no band enabled
uint8_t  curBand  = 0;

// Transmission scheduling
uint32_t nextTX   = 0UL;
uint8_t  countTX  = 0;

const char DEVNAME[] = "MicroWSPR";
const char VERSION[] = "v1.0";
const char DATE[]    = __DATE__;

JTEncode JT;
SoftwareSerial SoftSerial(3, 4);

/**
 * GPS RMC sentence data structure
 * Minimal storage for $GPRMC NMEA sentences (no satellite count)
 */
struct GPRMCData {
  uint32_t time;       // HHMMSS as 6-digit integer (HH*10000 + MM*100 + SS)
  uint32_t date;       // DDMMYY
  long lat;            // Raw DDMM.MMMM digits (decimal stripped); to degrees: (lat/1000000) + (lat%1000000)/600000.0
  long lon;            // Raw DDDMM.MMMM digits; same conversion
  char lat_ns;         // N or S
  char lon_ew;         // E or W
  bool valid;          // A = valid fix, V = invalid
};

volatile GPRMCData gpsData = {0, 0, 0, 0, 'N', 'E', false};

/**
 * Parse single character from GPS NMEA stream
 * Extracts time, validity, lat/lon from $GPRMC sentences.
 * Uses comma-field counting; all variable data is accumulated into `acc`
 * and committed to gpsData on each ',' separator.
 * @return true when a complete sentence with all needed fields is parsed
 */
bool parseGPRMC(char c) {
  static uint8_t field  = 0xFF;  // 0xFF = waiting for '$'
  static uint8_t pos    = 0;     // position within current field
  static uint8_t hdrIdx = 0;     // index into "GPRMC" header
  static long    acc    = 0;     // digit accumulator

  if (c == '$') { field = 0; pos = 0; hdrIdx = 0; acc = 0; return false; }
  if (field == 0xFF) return false;
  if (c == '\r' || c == '\n' || c == '*') { field = 0xFF; return false; }

  if (c == ',') {
    switch (field) {
      case 1: gpsData.time = acc; break;   // HHMMSS
      case 3: gpsData.lat  = acc; break;   // DDMMmmmm
      case 5: gpsData.lon  = acc; break;   // DDDMMmmmm
    }
    field++; pos = 0; acc = 0;
    return false;
  }

  switch (field) {
    case 0: {  // verify "GPRMC" header
      static const char hdr[] = "GPRMC";
      if (hdrIdx < 5 && c != hdr[hdrIdx++]) field = 0xFF;
      break;
    }
    case 1:  // time: HHMMSS.ss — accumulate first 6 integer digits only
      if (c != '.' && pos < 6 && c >= '0' && c <= '9') { acc = acc * 10 + (c - '0'); pos++; }
      break;
    case 2:  // validity: A or V
      if (pos == 0) { gpsData.valid = (c == 'A'); pos++; }
      break;
    case 3:  // lat: DDMM.MMMM — accumulate all digits, skip decimal point
    case 5:  // lon: DDDMM.MMMM
      if (c != '.' && c >= '0' && c <= '9') acc = acc * 10 + (c - '0');
      break;
    case 4:  // N or S
      if (pos == 0) { gpsData.lat_ns = c; pos++; }
      break;
    case 6:  // E or W — last field needed; signal completion
      if (pos == 0) { gpsData.lon_ew = c; return true; }
      break;
  }
  return false;
}

// ── band cycling ─────────────────────────────────────────────────────────────

/**
 * Advance to next enabled band
 * Cycles through cfg.bands bitmask, wrapping around. Sets curBand to 0 if no bands enabled.
 */
void advanceBand() {
  if (!cfg.bands) { curBand = 0; return; }
  for (int i = 1; i <= 14; i++) {
    uint8_t b = (curBand - 1 + i) % 14 + 1;
    if (cfg.bands & (1 << b)) { curBand = b; return; }
  }
  curBand = 0;
}

// ── transmit ─────────────────────────────────────────────────────────────────

/**
 * Transmit WSPR symbols on specified band
 * Encodes message and transmits at random frequency within WSPR channel
 * @param band - HAM_BANDS value (1-14), 0 = no transmission
 */
void transmit(uint8_t band = 0) {
  uint32_t nextSym;
  float wsprChanFrq = 1400 + (random(25) + 5) * (4.0 * 12000UL / 8192);
  float wsprSymbFrq;
#ifdef DEBUG
  Serial.print(F("Base frequency: "));
  Serial.print(wsprChanFrq, 3);
  Serial.print(F(" "));
  Serial.println(wsprBaseFrq[band] + wsprChanFrq, 3);
#endif
  DDS.output_enable(2, 1);
  digitalWrite(LED_BUILTIN, HIGH);
  nextSym = millis();
  for (uint8_t i = 0; i < WSPR_SYMBOL_COUNT; i++) {
    wsprSymbFrq = wsprBaseFrq[band] + wsprChanFrq + (txBuf[i] * wsprToneSep / 1000.0);
    DDS.set_freq(wsprSymbFrq * 100, 2);
    nextSym += wsprToneDur;
#ifdef DEBUG
    Serial.print(i); Serial.print(' ');
    Serial.print(txBuf[i]); Serial.print(' ');
    Serial.println(wsprSymbFrq, 3);
#endif
    while (millis() < nextSym);
  }
  digitalWrite(LED_BUILTIN, LOW);
  DDS.output_enable(2, 0);
}

// ── GPS helpers ───────────────────────────────────────────────────────────────
// Convert lat/lon to Maidenhead locator (4-character grid square)
void getLocator(char *loc, float lat, float lng) {
  float rem;
  rem = lng + 180.0;
  int o1 = (int)(rem / 20.0);
  rem -= (float)o1 * 20.0;
  int o2 = (int)(rem / 2.0);
  rem = lat + 90.0;
  int a1 = (int)(rem / 10.0);
  rem -= (float)a1 * 10.0;
  int a2 = (int)(rem);
  loc[0] = (char)o1 + 'A';
  loc[1] = (char)a1 + 'A';
  loc[2] = (char)o2 + '0';
  loc[3] = (char)a2 + '0';
  loc[4] = '\0';
}

// Generate entropy from floating analog input for randomSeed()
long getRandomSeed(int numBits = 31) {
  if (numBits > 31 || numBits < 1) numBits = 31;
  const int  baseIntervalMs    = 1;
  const byte sampleSignificant = 7;
  const byte sampleMultiplier  = 10;
  const byte hashIterations    = 3;
  int  intervalMs = 0;
  long result     = 0;
  int  tempBit    = 0;
  pinMode(A0, INPUT_PULLUP);
  pinMode(A0, INPUT);
  delay(200);
  for (int bits = 0; bits < numBits; bits++) {
    for (int i = 0; i < hashIterations; i++) {
      delay(baseIntervalMs + intervalMs);
      unsigned long reading = analogRead(A0);
      tempBit ^= reading & 1;
      intervalMs = (reading % sampleSignificant) * sampleMultiplier;
    }
    result |= (long)(tempBit & 1) << bits;
  }
#ifdef DEBUG
  Serial.print(F("Entropy: 0x"));
  Serial.println(result, HEX);
#endif
  return result;
}

// ── setup ────────────────────────────────────────────────────────────────────
// Initialize Serial, GPS, Si5351, load config, start scheduler
void setup() {
  Serial.begin(115200);
  SoftSerial.begin(9600);

  Serial.println();
  Serial.print(DEVNAME); Serial.print(' ');
  Serial.print(VERSION); Serial.print(F(" SI5351 ("));
  Serial.print(DATE);
  Serial.println(')');

  DDS.init(8, 0, 0);
#if CALIBRATION != 0
  DDS.set_correction(CALIBRATION, 0);
#endif
  DDS.drive_strength(2, 2);  // strength: 0=2mA, 1=4mA, 2=6mA, 3=8mA
  DDS.output_enable(2, 0);
  pinMode(LED_BUILTIN, OUTPUT);

  randomSeed(getRandomSeed());

  // Load config from EEPROM (or defaults on first boot)
  configLoad();

  // Initialize working locator from stored config; GPS will override if empty
  strncpy(loc, cfg.locator, sizeof(loc));

  // Set to first enabled band
  advanceBand();

  // Boot-time config window
  Serial.println(F("Press any key for configuration..."));
  uint32_t deadline = millis() + 5000UL;
  while (millis() < deadline) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      configTUI();
      // Re-apply locator and band after config change
      strncpy(loc, cfg.locator, sizeof(loc));
      curBand = 0;
      advanceBand();
      nextTX = 0;
      break;
    }
  }
}

// ── loop ─────────────────────────────────────────────────────────────────────
// Main scheduler: check TX timing, read GPS, schedule transmissions
void loop() {
  if (loc[0] != '\0' && millis() >= nextTX && nextTX > 0) {
    nextTX += (uint32_t)cfg.decimation * 120 * 1000UL;
    memset(txBuf, 0, sizeof(txBuf));
    JT.wspr_encode(cfg.callsign, loc, cfg.dbm, txBuf);
#ifdef DEBUG
    Serial.print(F("Symbols: "));
    for (uint8_t i = 0; i < WSPR_SYMBOL_COUNT; i++)
      Serial.print(txBuf[i]);
    Serial.println();
#endif
    if (curBand > 0)
      transmit(curBand);
    advanceBand();
    countTX++;
  }
#ifdef DEBUG
  if (nextTX > millis()) {
    Serial.print(F("Next in "));
    Serial.print((nextTX - millis()) / 1000);
    Serial.println('s');
  }
#endif

  bool newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (SoftSerial.available()) {
      char c = SoftSerial.read();
#ifdef DEBUG_GPS
      Serial.write(c);
#endif
      if (parseGPRMC(c))
        newData = true;
    }
  }

  if (newData) {
    Serial.println();
    Serial.print(F("GPS: "));
    Serial.print('-');  // No satellite count in GPRMC
    Serial.print(',');
    
    float lat = 0.0, lon = 0.0;
    if (gpsData.valid && gpsData.lat != 0 && gpsData.lon != 0) {
      lat = (float)(gpsData.lat / 1000000L) + (float)(gpsData.lat % 1000000L) / 600000.0f;
      if (gpsData.lat_ns == 'S') lat = -lat;
      lon = (float)(gpsData.lon / 1000000L) + (float)(gpsData.lon % 1000000L) / 600000.0f;
      if (gpsData.lon_ew == 'W') lon = -lon;
      Serial.print(lat, 6); Serial.print(',');
      Serial.print(lon, 6); Serial.print(',');
      if (!cfg.locator[0])
        getLocator(loc, lat, lon);
    } else {
      Serial.print(F("*,*,"));
    }
    if (loc[0]) { Serial.print(loc); Serial.print(','); }
    else          Serial.print(F("*,"));

    if (gpsData.valid && gpsData.time > 0) {
      uint32_t t = gpsData.time;
      uint8_t hour   = t / 10000;
      uint8_t minute = (t / 100) % 100;
      uint8_t second = t % 100;
      uint8_t rem = ((minute % 2 == 0) ? 120 : 60) - second;
      char buf[16];
      sprintf(buf, "%02d:%02d:%02d,%ds", hour, minute, second, rem);
      Serial.println(buf);
      if (nextTX == 0 || countTX * cfg.decimation >= 30) {
        nextTX   = millis() + (rem + 1) * 1000UL;
        countTX  = 0;
      }
    }
  }
}
