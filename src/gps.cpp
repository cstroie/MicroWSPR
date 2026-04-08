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

// Working locator: initialised from cfg.locator at boot; GPS updates it when cfg.locator is empty
char loc[7];

static SoftwareSerial *SoftSerial = nullptr;

// ── helpers ──────────────────────────────────────────────────────────────────

static inline uint32_t hhmmssToSec(uint32_t t) {
  return (t / 10000) * 3600UL + ((t / 100) % 100) * 60UL + (t % 100);
}

/**
 * Print "  next TX: Xs" when an alarm is scheduled, or "  next slot: Xs" when
 * waiting for the first schedule.  Handles midnight rollover like alarmReached().
 */
static void printSlot(int rem, uint32_t nextTXTime) {
  if (nextTXTime > 0) {
    int32_t cur  = (int32_t)hhmmssToSec(gpsData.time);
    int32_t alm  = (int32_t)hhmmssToSec(nextTXTime);
    int32_t diff = alm - cur;
    if (diff >  43200) diff -= 86400;
    if (diff < -43200) diff += 86400;
    if (diff < 0) diff = 0;
    Serial.print(F("  next TX: ")); Serial.print((int)diff); Serial.print('s');
  } else {
    Serial.print(F("  next slot: ")); Serial.print(rem); Serial.print('s');
  }
}

// ── parser ───────────────────────────────────────────────────────────────────

/**
 * Character-at-a-time $GPRMC sentence parser.
 *
 * The parser is a simple state machine driven by comma separators. Each call
 * processes one character and returns true only when the last needed field (E/W
 * hemisphere, field 6) has been received — meaning gpsData holds a fresh sentence.
 *
 * State variables (all static, persist between calls):
 *   field  — current comma-field index (0=sentence ID, 1=time … 6=E/W);
 *            0xFF means waiting for the next '$' start-of-sentence marker.
 *   pos    — character position within the current field (used to take only the
 *            first character of single-char fields like validity and hemisphere).
 *   hdrIdx — index into the expected "GPRMC" header string; abandons the
 *            sentence immediately if any character does not match.
 *   acc    — digit accumulator for numeric fields; resets to 0 on each comma.
 *
 * Field mapping ($GPRMC):
 *   0  sentence ID    "GPRMC" — verified character by character
 *   1  UTC time       HHMMSS.ss — only integer part (6 digits) accumulated
 *   2  status         A=valid, V=void
 *   3  latitude       DDMM.MMMM — all digits accumulated, decimal point skipped
 *   4  lat hemisphere N or S
 *   5  longitude      DDDMM.MMMM — same as latitude
 *   6  lon hemisphere E or W — last field we need; triggers return true
 *
 * Numeric storage format for lat/lon:
 *   The decimal point is stripped and all digits are accumulated into a long.
 *   "4437.1234" → acc = 44371234 (stored in gpsData.lat).
 *   To convert to decimal degrees: degrees + minutes/60
 *     = (acc / 1000000) + (acc % 1000000) / 600000.0
 *   where acc/1000000 gives the DD part and acc%1000000 gives MM×10000,
 *   and dividing by 600000 = 60×10000 converts minutes to fractional degrees.
 */
static bool parseGPRMC(char c) {
  static uint8_t field  = 0xFF;  // 0xFF = waiting for '$'
  static uint8_t pos    = 0;     // position within current field
  static uint8_t hdrIdx = 0;     // index into "GPRMC" header
  static long    acc    = 0;     // digit accumulator

  // '$' resets the state machine unconditionally — start of a new sentence
  if (c == '$') { field = 0; pos = 0; hdrIdx = 0; acc = 0; return false; }
  if (field == 0xFF) return false;
  // '*' begins the checksum, '\r'/'\n' end the line — sentence is complete (or abandoned)
  if (c == '\r' || c == '\n' || c == '*') { field = 0xFF; return false; }

  if (c == ',') {
    // Commit accumulated value for numeric fields before advancing
    switch (field) {
      case 1: gpsData.time = acc; break;   // commit HHMMSS
      case 3: gpsData.lat  = acc; break;   // commit DDMMmmmm
      case 5: gpsData.lon  = acc; break;   // commit DDDMMmmmm
    }
    field++; pos = 0; acc = 0;
    return false;
  }

  switch (field) {
    case 0: {  // sentence ID: verify each character against "GPRMC"
      static const char hdr[] = "GPRMC";
      if (hdrIdx < 5 && c != hdr[hdrIdx++]) field = 0xFF;  // mismatch → abandon
      break;
    }
    case 1:  // UTC time: HHMMSS.ss — accumulate first 6 integer digits, ignore sub-seconds
      if (c != '.' && pos < 6 && c >= '0' && c <= '9') { acc = acc * 10 + (c - '0'); pos++; }
      break;
    case 2:  // fix validity: 'A' = active (valid), 'V' = void (no fix)
      if (pos == 0) { gpsData.valid = (c == 'A'); pos++; }
      break;
    case 3:  // latitude DDMM.MMMM → accumulate all digits, skip decimal point
    case 5:  // longitude DDDMM.MMMM → same treatment
      if (c != '.' && c >= '0' && c <= '9') acc = acc * 10 + (c - '0');
      break;
    case 4:  // latitude hemisphere: 'N' or 'S'
      if (pos == 0) { gpsData.lat_ns = c; pos++; }
      break;
    case 6:  // longitude hemisphere: 'E' or 'W' — last field needed; signal completion
      if (pos == 0) { gpsData.lon_ew = c; return true; }
      break;
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
      return true;  // we don't care about any fields after 6, but we need to consume them until the line ends
      break;
  }
  return false;
}

// ── public API ───────────────────────────────────────────────────────────────

bool gpsInit() {
  // Construct SoftwareSerial with the pins stored in cfg (loaded before this call)
  delete SoftSerial;
  SoftSerial = new SoftwareSerial(cfg.gpsRxPin, cfg.gpsTxPin);
  SoftSerial->begin(9600);
  // Wait up to 1 s; any byte is enough to confirm the module is alive
  for (unsigned long start = millis(); millis() - start < 1000;)
    if (SoftSerial->available()) return true;
  return false;
}

int gpsUpdate(uint32_t nextTXTime) {
  // Consume all bytes arriving in the next 1 s window, feeding the parser
  bool newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (SoftSerial->available()) {
      char c = SoftSerial->read();
      //Serial.print(c);  // echo GPS data to Serial for debugging; comment out if not needed
      if (parseGPRMC(c))
        newData = true;
    }
  }

  if (!newData)
    return -1;

  // hadFix/hadTime track transitions to print "acquired" banners once
  static bool hadFix  = false;
  static bool hadTime = false;

  // Convert raw lat/lon integers to decimal degrees
  float lat = 0.0, lon = 0.0;
  bool hasFix  = gpsData.valid && gpsData.lat != 0 && gpsData.lon != 0;
  bool hasTime = gpsData.time > 0;

  if (hasFix) {
    // Raw format: DDMMmmmm (decimal point stripped from DDMM.MMMM)
    // Degrees = integer DD part; fractional degrees = MM.mmmm / 60
    // = (acc % 1000000) / 600000.0  (since mmmm represents 4 decimal places of minutes)
    lat = (float)(gpsData.lat / 1000000L) + (float)(gpsData.lat % 1000000L) / 600000.0f;
    if (gpsData.lat_ns == 'S') lat = -lat;
    lon = (float)(gpsData.lon / 1000000L) + (float)(gpsData.lon % 1000000L) / 600000.0f;
    if (gpsData.lon_ew == 'W') lon = -lon;
    // Derive Maidenhead locator from GPS position only when no fixed locator is configured
    if (!cfg.locator[0])
      getLocator(loc, lat, lon);
    if (!hadFix) {
      Serial.println(F("GPS fix acquired!"));
      hadFix = true;
    }
  } else {
    hadFix = false;
  }

  // Compute seconds to the next WSPR slot boundary (every even UTC minute).
  // At second :00 of an even minute, rem=120 (next boundary is a full 2-minute cycle away).
  // At second :00 of an odd minute, rem=60.  Returns -1 when no GPS time is available.
  int rem = -1;
  char timebuf[9] = "";  // formatted as "HH:MM:SS"
  if (hasTime) {
    uint32_t t     = gpsData.time;
    uint8_t hour   = t / 10000;
    uint8_t minute = (t / 100) % 100;
    uint8_t second = t % 100;
    rem = (int)(((minute % 2 == 0) ? 120 : 60) - second);
    sprintf(timebuf, "%02d:%02d:%02d", hour, minute, second);
    if (!hadTime) {
      Serial.print(F("GPS time acquired: "));
      Serial.print(timebuf);
      Serial.println(F(" UTC"));
      hadTime = true;
    }
  } else {
    hadTime = false;
  }

  // Print status line
  if (hasFix) {
    Serial.print(F("GPS: "));
    Serial.print(lat, 6); Serial.print(','); Serial.print(lon, 6);
    Serial.print(F("  ")); Serial.print(loc);
    if (timebuf[0]) {
      Serial.print(F("  ")); Serial.print(timebuf); Serial.print(F(" UTC"));
      printSlot(rem, nextTXTime);
    }
  } else if (hasTime && loc[0]) {
    // Time acquired and locator available from EEPROM — no position fix needed
    Serial.print(F("GPS: ")); Serial.print(timebuf); Serial.print(F(" UTC"));
    Serial.print(F("  loc: ")); Serial.print(loc);
    printSlot(rem, nextTXTime);
  } else {
    Serial.print(F("GPS: no fix"));
    if (timebuf[0]) {
      Serial.print(F("  ")); Serial.print(timebuf); Serial.print(F(" UTC"));
    }
  }
  Serial.println();

  return rem;
}

/**
 * Maidenhead locator algorithm (4-character grid square).
 *
 * The grid divides the globe into a two-level hierarchy:
 *   Field (2 letters): 18 zones in longitude (20° each) × 18 in latitude (10° each)
 *   Square (2 digits): 10 sub-zones in longitude (2° each) × 10 in latitude (1° each)
 *
 * Computation:
 *   Normalize longitude to 0-360 by adding 180, then latitude to 0-180 by adding 90.
 *   For longitude: field letter = floor(norm_lon / 20),  square digit = floor(remainder / 2)
 *   For latitude:  field letter = floor(norm_lat / 10),  square digit = floor(remainder)
 *   Letters are offset from 'A'; digits are offset from '0'.
 *
 * Example: 44.62°N, 26.08°E → JN75
 */
void getLocator(char *buf, float lat, float lng) {
  float rem;

  // Longitude field and square
  rem = lng + 180.0;                  // normalize to 0-360°
  int o1 = (int)(rem / 20.0);         // field index (0-17 → A-R)
  rem -= (float)o1 * 20.0;            // remainder within field (0-20°)
  int o2 = (int)(rem / 2.0);          // square index (0-9)

  // Latitude field and square
  rem = lat + 90.0;                   // normalize to 0-180°
  int a1 = (int)(rem / 10.0);         // field index (0-17 → A-R)
  rem -= (float)a1 * 10.0;            // remainder within field (0-10°)
  int a2 = (int)(rem);                // square index (0-9)

  buf[0] = (char)o1 + 'A';
  buf[1] = (char)a1 + 'A';
  buf[2] = (char)o2 + '0';
  buf[3] = (char)a2 + '0';
  buf[4] = '\0';
}
