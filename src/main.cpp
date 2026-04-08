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
#include "gps.h"
#include "si5351.h"

#ifndef CALIBRATION
#define CALIBRATION (0)
#endif

#include <JTEncode.h>

// ── constants ────────────────────────────────────────────────────────────────

const char DEVNAME[] = "MicroWSPR";
const char VERSION[] = "v1.0";
const char DATE[]    = __DATE__;

enum HAM_BANDS {
  BAND_OFF,
  BAND_2190, BAND_630,  BAND_160,
  BAND_80,   BAND_60,   BAND_40,  BAND_30,  BAND_20,
  BAND_17,   BAND_15,   BAND_12,  BAND_10,  BAND_6,   BAND_2
};

// ── LED status ───────────────────────────────────────────────────────────────

enum LedState {
  LED_FAULT,     // fast blink 100/100 ms — hardware not detected
  LED_NO_FIX,    // brief flash every 2 s — waiting for GPS fix
  LED_TX,        // solid on — transmitting
  LED_IDLE       // off — fix acquired, between transmissions
};

LedState ledState = LED_NO_FIX;

/** Drive the LED according to ledState; call from loop() on every iteration. */
void ledUpdate() {
  static uint32_t lastToggle = 0;
  static bool     ledOn      = false;
  uint32_t now = millis();

  switch (ledState) {
    case LED_FAULT:
      // 100 ms on / 100 ms off
      if (now - lastToggle >= 100) {
        ledOn = !ledOn;
        digitalWrite(LED_BUILTIN, ledOn);
        lastToggle = now;
      }
      break;
    case LED_NO_FIX:
      // 50 ms flash every 2 s
      if (!ledOn && now - lastToggle >= 2000) {
        digitalWrite(LED_BUILTIN, HIGH);
        ledOn = true;
        lastToggle = now;
      } else if (ledOn && now - lastToggle >= 50) {
        digitalWrite(LED_BUILTIN, LOW);
        ledOn = false;
        lastToggle = now;
      }
      break;
    case LED_TX:
      digitalWrite(LED_BUILTIN, HIGH);
      ledOn = true;
      break;
    case LED_IDLE:
      digitalWrite(LED_BUILTIN, LOW);
      ledOn = false;
      break;
  }
}

// ── WSPR ─────────────────────────────────────────────────────────────────────

// WSPR base frequencies indexed by HAM_BANDS enum (index 0 unused)
const uint32_t wsprBaseFrq[] = {
  0UL,
  136000UL,   474200UL,   1836600UL,  3568600UL,
  5287200UL,  7038600UL,  10138700UL, 14095600UL, 18104600UL,
  21094600UL, 24924600UL, 28124600UL, 50293000UL, 144489000UL
};

const uint16_t wsprToneSep = round(1000UL * 12000 / 8192);  // 1.4648 Hz (stored as mHz)
const uint16_t wsprToneDur = round(1000UL * 8192 / 12000);  // 683 ms per symbol

uint8_t  txBuf[WSPR_SYMBOL_COUNT];
JTEncode JT;

// Current band (HAM_BANDS index, 1-14); 0 = no band enabled
uint8_t  curBand = 0;
// Transmission scheduling
uint32_t nextTX  = 0UL;
uint8_t  countTX = 0;

/** Advance curBand to the next enabled band in cfg.bands, wrapping around. */
void advanceBand() {
  if (!cfg.bands) { curBand = 0; return; }
  for (int i = 1; i <= 14; i++) {
    uint8_t b = (curBand - 1 + i) % 14 + 1;
    if (cfg.bands & (1 << b)) { curBand = b; return; }
  }
  curBand = 0;
}

/** Transmit encoded WSPR symbols on band (1-14) at a random offset within the WSPR channel. */
void transmit(uint8_t band = 0) {
  uint32_t nextSym;
  float wsprChanFrq = 1400 + (random(25) + 5) * (4.0 * 12000UL / 8192);
  float wsprSymbFrq;
  Serial.print(F("  freq: "));
  Serial.print((wsprBaseFrq[band] + wsprChanFrq) / 1000.0, 3);
  Serial.println(F(" kHz"));
  ledState = LED_TX;
  ledUpdate();
  DDS.outputEnable(2, 1);
  nextSym = millis();
  for (uint8_t i = 0; i < WSPR_SYMBOL_COUNT; i++) {
    wsprSymbFrq = wsprBaseFrq[band] + wsprChanFrq + (txBuf[i] * wsprToneSep / 1000.0);
    DDS.setFreq(wsprSymbFrq * 100, 2);
    nextSym += wsprToneDur;
    while (millis() < nextSym);
  }
  DDS.outputEnable(2, 0);
  ledState = LED_IDLE;
}

// ── entropy ──────────────────────────────────────────────────────────────────

/** Sample analog noise on A0 to produce numBits bits of entropy for randomSeed(). */
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
  Serial.print(F("Entropy: 0x"));
  Serial.println(result, HEX);
  return result;
}

// ── setup ────────────────────────────────────────────────────────────────────
/** Initialize Serial, GPS, Si5351, load config, and open the boot config window. */
void setup() {
  Serial.begin(115200);

  Serial.println();
  Serial.print(DEVNAME); Serial.print(' ');
  Serial.print(VERSION); Serial.print(F(" SI5351 ("));
  Serial.print(DATE);
  Serial.println(')');

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.print(F("GPS    : "));
  bool gpsDetected = gpsInit();
  if (gpsDetected) {
    Serial.println(F("detected"));
  } else {
    Serial.println(F("no data!"));
    ledState = LED_FAULT;
  }

  Serial.print(F("Si5351 : "));
  if (DDS.init(8, 0, 0)) {
    Serial.println(F("OK"));
  } else {
    Serial.println(F("not found!"));
    ledState = LED_FAULT;
  }
#if CALIBRATION != 0
  DDS.setCorrection(CALIBRATION, 0);
#endif
  DDS.driveStrength(2, 2);  // strength: 0=2mA, 1=4mA, 2=6mA, 3=8mA
  DDS.outputEnable(2, 0);

  randomSeed(getRandomSeed());

  // Load config from EEPROM (or defaults on first boot)
  configLoad();

  // Initialize working locator from stored config; GPS will override if empty
  strncpy(loc, cfg.locator, sizeof(loc));

  // Set to first enabled band
  advanceBand();

  // Print config summary so the user knows what will be transmitted
  configSummary();

  // Boot-time config window (optional, 5-second window)
  Serial.println(F("Press any key for configuration..."));
  uint32_t deadline = millis() + 5000UL;
  while (millis() < deadline) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      configTUI();
      strncpy(loc, cfg.locator, sizeof(loc));
      curBand = 0; advanceBand(); nextTX = 0;
      break;
    }
  }

  // Require a real callsign — loop until one is set
  while (cfg.callsign[0] == '\0' || strcmp(cfg.callsign, "N0CALL") == 0) {
    Serial.println(F("Callsign not set. Please configure."));
    configTUI();
    strncpy(loc, cfg.locator, sizeof(loc));
    curBand = 0; advanceBand(); nextTX = 0;
  }

  // Require a locator when GPS is unavailable — loop until one is set
  if (!gpsDetected && cfg.locator[0] == '\0') {
    Serial.println(F("No GPS detected and no locator set. Please configure."));
    while (cfg.locator[0] == '\0') {
      configTUI();
      strncpy(loc, cfg.locator, sizeof(loc));
    }
    curBand = 0; advanceBand(); nextTX = 0;
  }

  if (cfg.locator[0])
    Serial.println(F("Waiting for GPS time..."));
  else
    Serial.println(F("Waiting for GPS fix..."));
}

// ── loop ─────────────────────────────────────────────────────────────────────
/** Check TX timing, read GPS, and schedule the next transmission window. */
void loop() {
  ledUpdate();

  if (loc[0] != '\0' && millis() >= nextTX && nextTX > 0) {
    nextTX += (uint32_t)cfg.decimation * 120 * 1000UL;
    memset(txBuf, 0, sizeof(txBuf));
    JT.wspr_encode(cfg.callsign, loc, cfg.dbm, txBuf);
    if (curBand > 0) {
      Serial.print(F("TX: "));
      Serial.print(getBandName(curBand));
      Serial.print(F(" ("));
      Serial.print(wsprBaseFrq[curBand] / 1000.0, 1);
      Serial.println(F(" kHz)"));
      transmit(curBand);
      Serial.println(F("TX done."));
    }
    advanceBand();
    countTX++;
  }

  int rem = gpsUpdate();
  if (rem >= 0) {
    if (ledState == LED_NO_FIX)
      ledState = LED_IDLE;
  } else {
    // GPS time lost — cancel schedule and signal no-fix until time returns
    if (nextTX > 0) {
      Serial.println(F("GPS time lost, TX suspended."));
      nextTX = 0;
      countTX = 0;
    }
    ledState = LED_NO_FIX;
  }

  // Auto-save GPS-derived locator once per boot if it differs from stored value
  static bool locatorSaved = false;
  if (!locatorSaved && loc[0] != '\0' && strcmp(loc, cfg.locator) != 0) {
    strncpy(cfg.locator, loc, sizeof(cfg.locator));
    Serial.print(F("Locator auto-saved: ")); Serial.println(cfg.locator);
    configSave();
    locatorSaved = true;
  }

  // Schedule the next transmission window if we have a valid time and
  // either no previous schedule or we've reached the decimation count
  if (rem >= 0 && (nextTX == 0 || countTX * cfg.decimation >= 30)) {
    nextTX  = millis() + (rem + 1) * 1000UL;
    countTX = 0;
    Serial.print(F("Next TX: "));
    Serial.print(getBandName(curBand));
    Serial.print(F(" in "));
    Serial.print(rem);
    Serial.println('s');
  }
}
