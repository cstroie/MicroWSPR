/**
  config.h - EEPROM-backed configuration

  Copyright (C) 2021-2026 Costin STROIE <costinstroie@eridu.eu.org>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#pragma once
#include <Arduino.h>

// Bump this when the Config struct layout changes — all devices will reset to defaults
#define CONFIG_MAGIC 0xAB01
#define CONFIG_ADDR  2      // magic lives at 0-1, struct starts here

// bands bitmask: bit N = HAM_BANDS enum value N is enabled (bits 1-14 valid)
// Default: 40m (bit 6) | 20m (bit 8) | 15m (bit 10)
#define BANDS_DEFAULT ((1 << 6) | (1 << 8) | (1 << 10))

struct Config {
  char     callsign[10];
  char     locator[7];   // empty string = use GPS
  uint8_t  dbm;
  uint8_t  decimation;
  uint16_t bands;
};

extern Config cfg;

/** Return the name string for a HAM_BANDS index (0-14). */
const char* getBandName(uint8_t band);
/** Print a one-screen config summary (callsign, power, locator, bands, decimation). */
void configSummary();
/** Reset cfg to built-in defaults. */
void configDefaults();
/** Load cfg from EEPROM; writes defaults if the magic number is missing. */
void configLoad();
/** Write cfg and magic number to EEPROM. */
void configSave();
/** Run the interactive serial configuration menu. */
void configTUI();
