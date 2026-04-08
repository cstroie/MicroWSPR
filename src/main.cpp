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

// Compile-time constant — set via build_flags, not stored in EEPROM
#ifndef CALIBRATION
#define CALIBRATION (0)
#endif

#ifdef USE_AD9833
#include <MD_AD9833.h>
#include <SPI.h>
#endif
#ifdef USE_SI5351
#include <si5351.h>
#endif
#include <JTEncode.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

uint8_t txBuf[255];

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

#ifdef USE_AD9833
const int FSYNC = 10;
const int DATA  = 11;
const int CLK   = 13;
MD_AD9833 DDS(FSYNC);
#endif
#ifdef USE_SI5351
Si5351 DDS;
#endif

JTEncode JT;
TinyGPS gps;
SoftwareSerial SoftSerial(3, 4);
float lat = 0.0, lon = 0.0;

// ── band cycling ─────────────────────────────────────────────────────────────

// Advance curBand to the next enabled band in cfg.bands, wrapping around.
// Sets curBand to 0 if no band is enabled.
void advanceBand() {
  if (!cfg.bands) { curBand = 0; return; }
  for (int i = 1; i <= 14; i++) {
    uint8_t b = (curBand - 1 + i) % 14 + 1;
    if (cfg.bands & (1 << b)) { curBand = b; return; }
  }
  curBand = 0;
}

// ── transmit ─────────────────────────────────────────────────────────────────

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
#ifdef USE_AD9833
  DDS.setMode(MD_AD9833::MODE_SINE);
#endif
#ifdef USE_SI5351
  DDS.output_enable(SI5351_CLK2, 1);
  digitalWrite(LED_BUILTIN, HIGH);
#endif
  nextSym = millis();
  for (uint8_t i = 0; i < WSPR_SYMBOL_COUNT; i++) {
    wsprSymbFrq = wsprBaseFrq[band] + wsprChanFrq + (txBuf[i] * wsprToneSep / 1000.0);
#ifdef USE_AD9833
    DDS.setFrequency(MD_AD9833::CHAN_0, wsprSymbFrq);
#endif
#ifdef USE_SI5351
    DDS.set_freq(wsprSymbFrq * 100, SI5351_CLK2);
#endif
    nextSym += wsprToneDur;
#ifdef DEBUG
    Serial.print(i); Serial.print(' ');
    Serial.print(txBuf[i]); Serial.print(' ');
    Serial.println(wsprSymbFrq, 3);
#endif
    while (millis() < nextSym);
  }
#ifdef USE_AD9833
  DDS.setMode(MD_AD9833::MODE_OFF);
#endif
#ifdef USE_SI5351
  digitalWrite(LED_BUILTIN, LOW);
  DDS.output_enable(SI5351_CLK2, 0);
#endif
}

// ── GPS helpers ───────────────────────────────────────────────────────────────

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

void setup() {
  Serial.begin(115200);
  SoftSerial.begin(9600);

  Serial.println();
  Serial.print(DEVNAME); Serial.print(' ');
  Serial.print(VERSION); Serial.print(' ');
#ifdef USE_AD9833
  Serial.print(F("AD9833"));
#endif
#ifdef USE_SI5351
  Serial.print(F("SI5351"));
#endif
  Serial.print(F(" ("));
  Serial.print(DATE);
  Serial.println(')');

#ifdef USE_AD9833
  DDS.begin();
  DDS.setMode(MD_AD9833::MODE_OFF);
#endif
#ifdef USE_SI5351
  if (!DDS.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0))
    Serial.println(F("Si5351 not found on I2C bus!"));
#if CALIBRATION != 0
  DDS.set_correction(CALIBRATION, SI5351_PLL_INPUT_XO);
  DDS.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
#endif
  DDS.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);
  DDS.output_enable(SI5351_CLK2, 0);
  pinMode(LED_BUILTIN, OUTPUT);
#endif

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
      if (gps.encode(c))
        newData = true;
    }
  }

  if (newData) {
    uint32_t age;
    uint16_t year;
    uint8_t  month, day, hour, minute, second, hndrds;
    Serial.println();
    Serial.print(F("GPS: "));
    Serial.print(gps.satellites());
    Serial.print(',');
    gps.f_get_position(&lat, &lon, &age);
    if (age != TinyGPS::GPS_INVALID_AGE) {
      Serial.print(lat, 6); Serial.print(',');
      Serial.print(lon, 6); Serial.print(',');
      // Only update working locator from GPS if no fixed locator is configured
      if (!cfg.locator[0])
        getLocator(loc, lat, lon);
    } else {
      Serial.print(F("*,*,"));
    }
    if (loc[0]) { Serial.print(loc); Serial.print(','); }
    else          Serial.print(F("*,"));

    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hndrds, &age);
    if (age != TinyGPS::GPS_INVALID_AGE) {
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
