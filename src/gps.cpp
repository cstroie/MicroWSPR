/**
  gps.cpp - GPS NMEA parsing and Maidenhead locator

  Copyright (C) 2021-2026 Costin STROIE <costinstroie@eridu.eu.org>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#include "gps.h"
#include "config.h"
#include <SoftwareSerial.h>

volatile GPRMCData gpsData = {0, 0, 0, 0, 'N', 'E', false};

// Working locator: set from cfg.locator on boot, updated by GPS when cfg.locator is empty
char loc[7];

static SoftwareSerial SoftSerial(3, 4);

// ── parser ───────────────────────────────────────────────────────────────────

/**
 * Feed one character from the GPS NMEA stream into the $GPRMC parser.
 * Accumulates fields into gpsData; returns true when a complete sentence is parsed.
 */
static bool parseGPRMC(char c) {
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

// ── public API ───────────────────────────────────────────────────────────────

bool gpsInit() {
  SoftSerial.begin(9600);
  // Listen for 1 second; any byte received means the GPS module is alive
  for (unsigned long start = millis(); millis() - start < 1000;)
    if (SoftSerial.available()) return true;
  return false;
}

int gpsUpdate() {
  bool newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (SoftSerial.available()) {
      char c = SoftSerial.read();
      if (parseGPRMC(c))
        newData = true;
    }
  }

  if (!newData)
    return -1;

  static bool hadFix = false;

  float lat = 0.0, lon = 0.0;
  bool hasFix = gpsData.valid && gpsData.lat != 0 && gpsData.lon != 0;

  if (hasFix) {
    lat = (float)(gpsData.lat / 1000000L) + (float)(gpsData.lat % 1000000L) / 600000.0f;
    if (gpsData.lat_ns == 'S') lat = -lat;
    lon = (float)(gpsData.lon / 1000000L) + (float)(gpsData.lon % 1000000L) / 600000.0f;
    if (gpsData.lon_ew == 'W') lon = -lon;
    if (!cfg.locator[0])
      getLocator(loc, lat, lon);
    if (!hadFix) {
      Serial.println(F("GPS fix acquired!"));
      hadFix = true;
    }
  } else {
    hadFix = false;
  }

  int rem = -1;
  char timebuf[9] = "";  // "HH:MM:SS"
  if (gpsData.valid && gpsData.time > 0) {
    uint32_t t  = gpsData.time;
    uint8_t hour   = t / 10000;
    uint8_t minute = (t / 100) % 100;
    uint8_t second = t % 100;
    rem = (int)(((minute % 2 == 0) ? 120 : 60) - second);
    sprintf(timebuf, "%02d:%02d:%02d", hour, minute, second);
  }

  if (hasFix) {
    Serial.print(F("GPS: "));
    Serial.print(lat, 6); Serial.print(','); Serial.print(lon, 6);
    Serial.print(F("  ")); Serial.print(loc);
    if (timebuf[0]) {
      Serial.print(F("  ")); Serial.print(timebuf); Serial.print(F(" UTC"));
      Serial.print(F("  next slot: ")); Serial.print(rem); Serial.print('s');
    }
  } else {
    Serial.print(F("GPS: no fix"));
    if (timebuf[0]) {
      Serial.print(F("  ")); Serial.print(timebuf); Serial.print(F(" UTC"));
    }
  }
  Serial.println();

  return rem;
}

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

