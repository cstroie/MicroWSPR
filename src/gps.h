/**
  gps.h - GPS NMEA parsing and Maidenhead locator

  Copyright (C) 2021-2026 Costin STROIE <costinstroie@eridu.eu.org>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#pragma once
#include <Arduino.h>

/** Parsed fields from a $GPRMC NMEA sentence. */
struct GPRMCData {
  uint32_t time;       // HHMMSS as 6-digit integer (HH*10000 + MM*100 + SS)
  uint32_t date;       // DDMMYY
  long lat;            // Raw DDMM.MMMM digits (decimal stripped); to degrees: (lat/1000000) + (lat%1000000)/600000.0
  long lon;            // Raw DDDMM.MMMM digits; same conversion
  char lat_ns;         // N or S
  char lon_ew;         // E or W
  bool valid;          // A = valid fix, V = invalid
};

extern volatile GPRMCData gpsData;
// Working locator: set from cfg.locator on boot, updated by GPS when cfg.locator is empty
extern char loc[7];

/** Initialize the GPS serial port; returns true if any data is received within 1 s. */
bool gpsInit();
/**
 * Poll GPS serial for up to 1 second and print a status line on new data.
 * Returns seconds to the next even 2-minute WSPR slot, or -1 if no valid time.
 */
int gpsUpdate();
/** Compute a 4-character Maidenhead locator from decimal lat/lon into loc[5]. */
void getLocator(char *loc, float lat, float lng);
