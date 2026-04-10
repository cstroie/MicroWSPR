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

/**
 * Magic number written at EEPROM address 0-1 to mark a valid config block.
 * Bump this value whenever the Config struct layout changes — mismatched magic
 * causes configLoad() to discard EEPROM contents and write factory defaults.
 *
 * EEPROM layout:
 *   0x0000-0x0001  CONFIG_MAGIC (uint16_t)
 *   0x0002-…       Config struct (CONFIG_ADDR)
 */
#define CONFIG_MAGIC 0xAB04
#define CONFIG_ADDR  2

/**
 * Default enabled-bands bitmask for cfg.bands.
 * Bit N corresponds to HAM_BANDS enum value N (bits 1-14 are valid).
 * Default: 40 m (bit 6), 20 m (bit 8), 15 m (bit 10).
 */
#define BANDS_DEFAULT ((1 << 6) | (1 << 8) | (1 << 10))

/**
 * Persistent beacon configuration, stored verbatim in EEPROM at CONFIG_ADDR.
 *
 * Fields:
 *   callsign    — amateur radio callsign, NUL-terminated, up to 9 chars.
 *   locator     — Maidenhead grid square (4 or 6 chars + NUL).
 *                 Empty string means derive the locator from GPS position.
 *   dbm         — TX power in dBm (0-60); encoded into the WSPR message.
 *   decimation  — transmit every Nth WSPR slot (1 = every 2-minute slot).
 *   bands       — bitmask of enabled HAM_BANDS values; bit N set = band N active.
 *   calibration — Si5351 crystal frequency correction in Hz (signed).
 *                 Applied at startup via DDS.setCorrection(); 0 = no correction.
 *   clkOutput   — Si5351 output clock to use for TX: 0=CLK0, 1=CLK1, 2=CLK2.
 *                 Default 0 (CLK0).  All three outputs are routed through PLLB.
 *   gpsRxPin    — Arduino digital pin connected to GPS TX (SoftwareSerial RX).
 *                 Default 3.
 *   gpsTxPin    — Arduino digital pin connected to GPS RX (SoftwareSerial TX,
 *                 unused but required by the SoftwareSerial constructor).
 *                 Default 4.
 */
struct Config {
  char     callsign[10];
  char     locator[7];
  uint8_t  dbm;
  uint8_t  decimation;
  uint16_t bands;
  int32_t  calibration;
  uint8_t  clkOutput;
  uint8_t  gpsRxPin;
  uint8_t  gpsTxPin;
};

/** Active configuration; loaded from EEPROM by configLoad(). */
extern Config cfg;

/** Return the display name string for a HAM_BANDS index (0-14); 0 → "OFF". */
const char* getBandName(uint8_t band);

/** Print a one-line-per-field summary of cfg to Serial. */
void configSummary();

/** Reset cfg to built-in factory defaults (does not write to EEPROM). */
void configDefaults();

/**
 * Load cfg from EEPROM.
 * If the magic number is absent or mismatched, writes factory defaults to EEPROM
 * and uses them for this session.  Also sanitises loaded values to guard against
 * struct layout changes or flash corruption.
 */
void configLoad();

/**
 * Write the current cfg and CONFIG_MAGIC to EEPROM.
 * Uses EEPROM.put() which only erases/writes bytes that have changed, preserving
 * EEPROM endurance.
 */
void configSave();

/**
 * Run the interactive serial configuration TUI.
 * Presents a numbered menu, reads single-character choices, dispatches to field
 * editors, and returns when the user selects Save (S) or Quit (Q).
 * Serial timeout is set to 30 s so the loop does not block indefinitely.
 *
 * testToneFn — optional callback invoked by the 'T' test-tone option.
 *   Called with the user-entered frequency in Hz and the configured CLK output.
 *   Pass nullptr to disable the test-tone option.
 */
void configTUI(void (*testToneFn)(uint32_t freqHz, uint8_t clk) = nullptr);
