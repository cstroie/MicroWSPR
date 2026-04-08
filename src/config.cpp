/**
  config.cpp - EEPROM-backed configuration + serial TUI

  Copyright (C) 2021-2026 Costin STROIE <costinstroie@eridu.eu.org>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#include "config.h"
#include <EEPROM.h>

Config cfg;

// Band names indexed by HAM_BANDS enum value (0=OFF, 1=2190m … 14=2m)
static const char* const bandName[] = {
  "OFF",
  "2190m", "630m",  "160m", "80m",  "60m",
  "40m",   "30m",   "20m",  "17m",  "15m",
  "12m",   "10m",   "6m",   "2m"
};

// ── helpers ──────────────────────────────────────────────────────────────────

/**
 * Read one line of input from Serial into buf (at most maxLen chars + NUL).
 *
 * Behaviour:
 *   - Blocks until CR or LF is received.
 *   - CR is consumed; a trailing LF immediately following a CR is also discarded
 *     so that \r\n line endings from Windows terminals work cleanly.
 *   - Printable characters (≥ 0x20) are echoed and appended up to maxLen.
 *   - Backspace (0x08) and DEL (0x7F) erase the last character with a
 *     "backspace–space–backspace" sequence.
 *   - Returns the number of characters stored (not counting the NUL terminator).
 */
static uint8_t readLine(char *buf, uint8_t maxLen) {
  uint8_t i = 0;
  while (true) {
    while (!Serial.available());
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (c == '\r') {
        delay(5);
        if (Serial.available() && Serial.peek() == '\n')
          Serial.read();
      }
      Serial.println();
      break;
    } else if ((c == '\b' || c == 127) && i > 0) {
      i--;
      Serial.print(F("\b \b"));
    } else if (c >= 32 && i < maxLen) {
      buf[i++] = c;
      Serial.print(c);
    }
  }
  buf[i] = '\0';
  return i;
}

/** Print the names of all enabled bands from cfg.bands, space-separated, or "none". */
static void printBands() {
  bool any = false;
  for (uint8_t b = 1; b <= 14; b++) {
    if (cfg.bands & (1 << b)) {
      if (any) Serial.print(' ');
      Serial.print(bandName[b]);
      any = true;
    }
  }
  if (!any) Serial.print(F("none"));
}

/** Print the full configuration menu with current values. */
static void showConfig() {
  Serial.println(F("\r\n================================"));
  Serial.println(F("   MicroWSPR  -  Configuration  "));
  Serial.println(F("================================"));
  Serial.print(F("  1. Callsign   : ")); Serial.println(cfg.callsign);
  Serial.print(F("  2. Power      : ")); Serial.print(cfg.dbm); Serial.println(F(" dBm"));
  Serial.print(F("  3. Locator    : "));
  if (cfg.locator[0]) Serial.println(cfg.locator);
  else                Serial.println(F("(from GPS)"));
  Serial.print(F("  4. Decimation : ")); Serial.println(cfg.decimation);
  Serial.print(F("  5. Bands      : ")); printBands(); Serial.println();
  Serial.print(F("  6. Calibration: ")); Serial.print(cfg.calibration); Serial.println(F(" Hz"));
  Serial.print(F("  7. CLK output : CLK")); Serial.println(cfg.clkOutput);
  Serial.println(F("--------------------------------"));
  Serial.println(F("  S. Save and exit"));
  Serial.println(F("  Q. Quit without saving"));
  Serial.println(F("================================"));
}

// ── field editors ────────────────────────────────────────────────────────────

/**
 * Prompt for a new callsign (up to 9 chars) and store it upper-cased in
 * cfg.callsign.  Input is rejected silently if the user presses Enter with no
 * characters, leaving the existing value unchanged.
 */
static void editCallsign() {
  char buf[10];
  Serial.print(F("Callsign ["));
  Serial.print(cfg.callsign);
  Serial.print(F("]: "));
  if (readLine(buf, 9) > 0) {
    for (uint8_t i = 0; buf[i]; i++)
      buf[i] = toupper((uint8_t)buf[i]);
    strncpy(cfg.callsign, buf, sizeof(cfg.callsign));
  }
}

/**
 * Prompt for TX power in dBm and update cfg.dbm.
 * Valid range: 0-60 (WSPR encodes power in 3-dB steps; arbitrary values are
 * accepted here and rounded by the encoder).  Out-of-range input is rejected.
 */
static void editPower() {
  char buf[4];
  Serial.print(F("Power dBm (0-60) ["));
  Serial.print(cfg.dbm);
  Serial.print(F("]: "));
  if (readLine(buf, 3) > 0) {
    int v = atoi(buf);
    if (v >= 0 && v <= 60)
      cfg.dbm = (uint8_t)v;
    else
      Serial.println(F("Out of range, unchanged."));
  }
}

/**
 * Prompt for a Maidenhead grid locator and update cfg.locator.
 * Accepts 4-character (grid square) or 6-character (grid subsquare) input,
 * forced to upper case.  An empty entry clears cfg.locator, reverting to
 * GPS-derived position.  Any other length is rejected.
 */
static void editLocator() {
  char buf[7];
  Serial.print(F("Locator (4 or 6 chars, empty = GPS) ["));
  Serial.print(cfg.locator[0] ? cfg.locator : "GPS");
  Serial.print(F("]: "));
  uint8_t len = readLine(buf, 6);
  if (len == 0) {
    cfg.locator[0] = '\0';
  } else if (len == 4 || len == 6) {
    for (uint8_t i = 0; buf[i]; i++)
      buf[i] = toupper((uint8_t)buf[i]);
    strncpy(cfg.locator, buf, sizeof(cfg.locator));
  } else {
    Serial.println(F("Invalid length (need 4 or 6 chars), unchanged."));
  }
}

/**
 * Prompt for the TX decimation factor and update cfg.decimation.
 * Decimation N means transmit once every N consecutive WSPR slots (each slot
 * is 2 minutes), so N=1 transmits every slot, N=2 every 4 minutes, etc.
 * Valid range: 1-99.
 */
static void editDecimation() {
  char buf[4];
  Serial.print(F("Decimation 1-99 ["));
  Serial.print(cfg.decimation);
  Serial.print(F("]: "));
  if (readLine(buf, 3) > 0) {
    int v = atoi(buf);
    if (v >= 1 && v <= 99)
      cfg.decimation = (uint8_t)v;
    else
      Serial.println(F("Out of range, unchanged."));
  }
}

/**
 * Prompt for the Si5351 frequency calibration correction and update cfg.calibration.
 *
 * The correction is a signed integer in Hz added to (or subtracted from) the
 * nominal crystal frequency before computing PLL multipliers.  Determine the
 * value by comparing the beacon's actual output frequency (measured with an
 * accurate receiver or SDR) against the expected WSPR dial frequency and
 * entering the difference here.  Valid range: ±999999 Hz.
 */
static void editCalibration() {
  char buf[8];
  Serial.print(F("Calibration Hz [-999999..999999] ["));
  Serial.print(cfg.calibration);
  Serial.print(F("]: "));
  if (readLine(buf, 7) > 0) {
    long v = atol(buf);
    if (v >= -999999L && v <= 999999L)
      cfg.calibration = (int32_t)v;
    else
      Serial.println(F("Out of range, unchanged."));
  }
}

/**
 * Prompt for the Si5351 output clock (0, 1, or 2) and update cfg.clkOutput.
 * All three outputs are routed through PLLB and are electrically equivalent;
 * the choice depends on which physical pin is wired to the antenna/filter.
 */
static void editClkOutput() {
  char buf[2];
  Serial.print(F("CLK output (0, 1, 2) ["));
  Serial.print(cfg.clkOutput);
  Serial.print(F("]: "));
  if (readLine(buf, 1) > 0) {
    int v = atoi(buf);
    if (v >= 0 && v <= 2)
      cfg.clkOutput = (uint8_t)v;
    else
      Serial.println(F("Out of range, unchanged."));
  }
}

/**
 * Interactively toggle enabled bands in cfg.bands.
 * Prints the full band list with current on/off state, then reads band numbers
 * one at a time, toggling the corresponding bit in cfg.bands.  Enter 0 to
 * finish.  Changes take effect in cfg immediately but are not saved to EEPROM
 * until configSave() is called.
 */
static void editBands() {
  Serial.println(F("\r\nAvailable bands:"));
  for (uint8_t b = 1; b <= 14; b++) {
    Serial.print(F("  "));
    if (b < 10) Serial.print(' ');
    Serial.print(b);
    Serial.print(F(". ["));
    Serial.print((cfg.bands & (1 << b)) ? 'X' : ' ');
    Serial.print(F("] "));
    Serial.println(bandName[b]);
  }
  Serial.println(F("Enter band number to toggle, 0 to finish."));
  while (true) {
    char buf[4];
    Serial.print(F("  Band: "));
    readLine(buf, 3);
    int v = atoi(buf);
    if (v == 0) break;
    if (v >= 1 && v <= 14) {
      cfg.bands ^= (1 << v);
      Serial.print(F("  "));
      Serial.print(bandName[v]);
      Serial.println((cfg.bands & (1 << v)) ? F(" ON") : F(" OFF"));
    } else {
      Serial.println(F("  Invalid number."));
    }
  }
}

// ── public API ───────────────────────────────────────────────────────────────

const char* getBandName(uint8_t band) {
  if (band > 14) band = 0;
  return bandName[band];
}

void configSummary() {
  Serial.print(F("Callsign   : ")); Serial.println(cfg.callsign);
  Serial.print(F("Power      : ")); Serial.print(cfg.dbm); Serial.println(F(" dBm"));
  Serial.print(F("Locator    : "));
  if (cfg.locator[0]) Serial.println(cfg.locator);
  else                Serial.println(F("(from GPS)"));
  Serial.print(F("Bands      : ")); printBands(); Serial.println();
  Serial.print(F("Decimation : ")); Serial.println(cfg.decimation);
  Serial.print(F("Calibration: ")); Serial.print(cfg.calibration); Serial.println(F(" Hz"));
  Serial.print(F("CLK output : CLK")); Serial.println(cfg.clkOutput);
}

void configDefaults() {
  strncpy(cfg.callsign, "N0CALL", sizeof(cfg.callsign));
  cfg.locator[0] = '\0';
  cfg.dbm         = 10;
  cfg.decimation  = 1;
  cfg.bands       = BANDS_DEFAULT;
  cfg.calibration = 0;
  cfg.clkOutput   = 0;
}

void configLoad() {
  uint16_t magic;
  EEPROM.get(0, magic);
  if (magic != CONFIG_MAGIC) {
    // No valid config block — first boot or struct layout changed.
    // Write defaults so future boots find a valid magic number.
    configDefaults();
    configSave();
  } else {
    EEPROM.get(CONFIG_ADDR, cfg);
    // Sanitise string fields: ensure NUL termination in case the stored bytes
    // were written by an older firmware with a shorter field.
    cfg.callsign[sizeof(cfg.callsign) - 1] = '\0';
    cfg.locator[sizeof(cfg.locator) - 1]   = '\0';
    // Clamp numeric fields to valid ranges; 0/out-of-range indicates corruption.
    if (cfg.decimation == 0) cfg.decimation = 1;
    if (cfg.dbm > 60)        cfg.dbm = 10;
    if (cfg.clkOutput > 2)   cfg.clkOutput = 0;
  }
}

void configSave() {
  uint16_t magic = CONFIG_MAGIC;
  EEPROM.put(0, magic);
  EEPROM.put(CONFIG_ADDR, cfg);
  Serial.println(F("Configuration saved."));
}

void configTUI() {
  Serial.setTimeout(30000);
  char buf[2];

  while (true) {
    showConfig();
    Serial.print(F("Choice: "));
    readLine(buf, 1);
    char c = toupper((uint8_t)buf[0]);

    switch (c) {
      case '1': editCallsign();   break;
      case '2': editPower();      break;
      case '3': editLocator();    break;
      case '4': editDecimation(); break;
      case '5': editBands();      break;
      case '6': editCalibration(); break;
      case '7': editClkOutput();   break;
      case 'S': configSave();     return;
      case 'Q':                   return;
      default:  Serial.println(F("Unknown option.")); break;
    }
  }
}
