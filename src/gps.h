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

/**
 * Parsed fields extracted from a $GPRMC NMEA sentence.
 *
 * $GPRMC sentence format (fields separated by commas):
 *   $GPRMC,HHMMSS.ss,A,DDMM.MMMM,N,DDDMM.MMMM,E,spd,crs,DDMMYY,,,*CS
 *   field:  1=time    2 3=lat      4  5=lon       6
 *
 * Only fields 1-6 are captured; speed, course and date are ignored for WSPR.
 */
struct GPRMCData {
  uint32_t time;    // UTC time as HHMMSS integer (HH*10000 + MM*100 + SS); e.g. 123456 = 12:34:56
  uint32_t date;    // UTC date as DDMMYY integer; not used by WSPR scheduler
  long lat;         // Latitude raw digits with decimal stripped: DDMM.MMMM → DDMMmmmm
                    // Convert to decimal degrees: (lat/1000000) + (lat%1000000)/600000.0
  long lon;         // Longitude raw digits: DDDMM.MMMM → DDDMMmmmm; same conversion
  char lat_ns;      // Hemisphere: 'N' (positive) or 'S' (negate result)
  char lon_ew;      // Hemisphere: 'E' (positive) or 'W' (negate result)
  bool valid;       // Fix status from field 2: true='A' (active/valid), false='V' (void)
};

/** Last successfully parsed $GPRMC sentence; updated by gpsUpdate(). */
extern volatile GPRMCData gpsData;

/**
 * Active Maidenhead locator (4 characters + NUL).
 * Initialised from cfg.locator at boot; overwritten by GPS when cfg.locator is empty.
 * Empty string means no locator is available yet — TX is withheld until it is set.
 */
extern char loc[7];

/** Open the GPS software serial port at 9600 baud; returns true if any byte is received within 1 s. */
bool gpsInit();

/**
 * Poll the GPS serial port for 1 second, parse incoming $GPRMC sentences,
 * update gpsData and loc[], and print a status line on new data.
 * Returns seconds until the next even-minute WSPR TX slot, or -1 if no valid time.
 */
int gpsUpdate();

/**
 * Compute a 4-character Maidenhead grid square from decimal lat/lon.
 * Result is written into buf[5] (4 characters + NUL terminator).
 */
void getLocator(char *buf, float lat, float lng);
