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

// Band names indexed by HAM_BANDS enum value (0=OFF, 1=2190m ... 14=2m)
static const char* const bandName[] = {
  "OFF",
  "2190m", "630m",  "160m", "80m",  "60m",
  "40m",   "30m",   "20m",  "17m",  "15m",
  "12m",   "10m",   "6m",   "2m"
};

// ── helpers ──────────────────────────────────────────────────────────────────

/** Read a line from Serial with echo and backspace support; returns char count. */
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

/** Print the enabled band names from cfg.bands, or "none". */
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
  Serial.println(F("--------------------------------"));
  Serial.println(F("  S. Save and exit"));
  Serial.println(F("  Q. Quit without saving"));
  Serial.println(F("================================"));
}

// ── field editors ────────────────────────────────────────────────────────────

/** Prompt for a new callsign and update cfg.callsign. */
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

/** Prompt for TX power in dBm (0-60) and update cfg.dbm. */
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

/** Prompt for a 4- or 6-character Maidenhead locator, or empty to use GPS. */
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

/** Prompt for TX decimation (1-99 TX intervals) and update cfg.decimation. */
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

/** Interactively toggle enabled bands in cfg.bands. */
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

void configDefaults() {
  strncpy(cfg.callsign, "N0CALL", sizeof(cfg.callsign));
  cfg.locator[0] = '\0';
  cfg.dbm        = 10;
  cfg.decimation = 1;
  cfg.bands      = BANDS_DEFAULT;
}

void configLoad() {
  uint16_t magic;
  EEPROM.get(0, magic);
  if (magic != CONFIG_MAGIC) {
    configDefaults();
    configSave();
  } else {
    EEPROM.get(CONFIG_ADDR, cfg);
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
      case 'S': configSave();     return;
      case 'Q':                   return;
      default:  Serial.println(F("Unknown option.")); break;
    }
  }
}
