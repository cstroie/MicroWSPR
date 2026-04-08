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

/**
 * Initialise the GPS software serial port at 9600 baud and detect the module.
 *
 * Opens SoftwareSerial on the configured RX/TX pins and listens for up to 1 s.
 * Any byte received within that window is sufficient to confirm the GPS module
 * is powered and transmitting — full sentence parsing is not required here.
 *
 * Returns true if the module was detected, false if no data arrived within 1 s.
 * The serial port remains open regardless; gpsUpdate() can be called either way.
 */
bool gpsInit();

/**
 * Poll the GPS serial port for exactly 1 second, parse $GPRMC sentences, and
 * return the time remaining until the next WSPR TX slot.
 *
 * Each call blocks for 1 s, consuming all available bytes and feeding them
 * through the character-level parseGPRMC() state machine.  If no complete
 * sentence is received during that window, returns -1 immediately.
 *
 * When new data arrives:
 *   - Position fix (gpsData.valid + non-zero lat/lon):
 *       Converts raw DDMMmmmm integers to decimal degrees and, if no fixed
 *       locator is configured in EEPROM (cfg.locator empty), recomputes loc[]
 *       via getLocator().  Prints "GPS fix acquired!" once on the fix→no-fix
 *       transition edge.
 *   - Time (gpsData.time > 0):
 *       Decodes HHMMSS into hours/minutes/seconds, computes the remainder to
 *       the next even-minute WSPR slot boundary, and prints "GPS time acquired:
 *       HH:MM:SS UTC" once on the no-time→time transition edge.
 *   - Status line printed every call:
 *       Full fix  → "GPS: lat,lon  locator  HH:MM:SS UTC  next slot: Ns"
 *       Time only → "GPS: HH:MM:SS UTC  loc: locator  next slot: Ns"
 *       No data   → "GPS: no fix  [HH:MM:SS UTC]"
 *
 * WSPR slot timing:
 *   Slots start at second :00 of every even UTC minute (0, 2, 4, …).
 *   rem = (even minute ? 120 : 60) − current_second
 *   This gives the number of seconds until the next slot boundary.
 *
 * Returns seconds to the next WSPR TX slot (≥ 0), or -1 if no valid time.
 */
int gpsUpdate();

/**
 * Compute a 4-character Maidenhead grid square from decimal lat/lon.
 * Result is written into buf[5] (4 characters + NUL terminator).
 */
void getLocator(char *buf, float lat, float lng);
