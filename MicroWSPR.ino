/**
  MicroWSPR - Arduino Micro GPS-discplined WSPR beacon

  Copyright (C) 2021 Costin STROIE <costinstroie@eridu.eu.org>

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

// User settings
// Should include at least the line
// #define CALLSIGN "your-callsign"
// #define DBM 27
#include "config.h"
// Safe values
#ifndef CALLSIGN
#define CALLSIGN    ("N0CALL")
#endif
#ifndef DBM
#define DBM         (27)
#endif
#ifndef LOC
#define LOC         ("")
#endif
#ifndef DECIMATION
#define DECIMATION  (5)
#endif

// EEPROM
#include <EEPROM.h>
// AD9833 and SPI
#include <MD_AD9833.h>
#include <SPI.h>
// JT modes
#include <JTEncode.h>
// GPS serial port
#include <SoftwareSerial.h>
// GPS
#include <TinyGPS.h>

/*

  $GPRMC,170852.00,V,,,,,,,040321,,,N*70
  $GPVTG,,,,,,,,,N*30
  $GPGGA,170852.00,,,,,0,00,99.99,,,,,,*6F
  $GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30
  $GPGSV,3,1,09,16,27,312,,18,76,219,40,20,27,169,37,23,,,35*4C
  $GPGSV,3,2,09,24,,,22,25,,,25,26,54,295,40,30,,,22*4B
  $GPGSV,3,3,09,31,,,42*74
  $GPGLL,,,,,170852.00,V,N*43
*/


char call[10] = CALLSIGN;
char loc[5]   = LOC;
uint8_t dBm   = DBM;
uint8_t txBuf[255];

const uint16_t wsprToneSep = round(1000UL * 12000 / 8192);  // 1.4648 Hz
const uint16_t wsprToneDur = round(1000UL * 8192 / 12000);  // 683 ms
const uint32_t wsprBaseFrq[] = {0UL, 136000UL, 474200UL, 1836600UL, 3592600UL,
                                5287200UL, 7038600UL, 10138700UL, 14095600UL, 18104600UL,
                                21094600UL, 24924600UL, 28124600UL, 50293000UL, 144489000UL
                               };

// The next transmission window
uint32_t nextTX = 0UL;
uint8_t decim   = DECIMATION;

// Software name and vesion
const char DEVNAME[]  = "MicroWSPR";
const char VERSION[]  = "v0.4";
const char AUTHOR[]   = "Costin Stroie <costinstroie@eridu.eu.org>";
const char DATE[]     = __DATE__;

// Pin definitions
const int FSYNC     = 10;
const int DATA      = 11;
const int CLK       = 13;

MD_AD9833  DDS(FSYNC);
//MD_AD9833  DDS(DATA, CLK, FSYNC);

// JT
JTEncode JT;

// GPS
TinyGPS gps;
// Software serial for GPS (RX, TX)
SoftwareSerial SoftSerial(3, 4);
// Latitude and Longitude (globals)
float lat = 0.0, lon = 0.0;

/**
  Transmit the payload
*/
void transmit(uint8_t band = 0) {
  uint32_t nextSym;
  // Choose a random 'channel' (there are 34 6Hz wide channels in 200Hz band)
  float wsprChanFrq = wsprBaseFrq[band] + 1400 + random(35) * (4.0 * 12000UL / 8192);
  float wsprSymbFrq;
#ifdef DEBUG
  Serial.print(F("Base frequency: "));
  Serial.println(wsprChanFrq, 3);
#endif
  // Output sine wave
  DDS.setMode(MD_AD9833::MODE_SINE);
  nextSym = millis();
  // Transmit the symbols
  for (uint8_t i = 0; i < WSPR_SYMBOL_COUNT; i++) {
    wsprSymbFrq = wsprChanFrq + (txBuf[i] * wsprToneSep / 1000.0);
    DDS.setFrequency(MD_AD9833::CHAN_0, wsprSymbFrq);
    nextSym += wsprToneDur;
#ifdef DEBUG
    Serial.print(i);
    Serial.print(" ");
    Serial.print(txBuf[i]);
    Serial.print(" ");
    Serial.println(wsprSymbFrq, 3);
#endif
    while (millis() < nextSym);
  }
  // Turn off the output
  DDS.setMode(MD_AD9833::MODE_OFF);
}

/**
  Get the maidenhead locator
*/
void getLocator(char *loc, float lat, float lng) {
  int o1, o2;
  int a1, a2;
  float rem;

  // Longitude
  rem = lng + 180.0;
  o1 = (int)(rem / 20.0);
  rem = rem - (float)o1 * 20.0;
  o2 = (int)(rem / 2.0);

  // Latitude
  rem = lat + 90.0;
  a1 = (int)(rem / 10.0);
  rem = rem - (float)a1 * 10.0;
  a2 = (int)(rem);

  // Fill the locator (5 chars)
  loc[0] = (char)o1 + 'A';
  loc[1] = (char)a1 + 'A';
  loc[2] = (char)o2 + '0';
  loc[3] = (char)a2 + '0';
  loc[4] = (char)0;
}

long getRandomSeed(int numBits = 31) {
  // magic numbers tested 2016-03-28
  // try to speed it up
  // Works Well. Keep!
  //
  if (numBits > 31 or numBits < 1) numBits = 31; // limit input range

  const int baseIntervalMs = 1UL;   // minumum wait time
  const byte sampleSignificant = 7; // modulus of the input sample
  const byte sampleMultiplier = 10; // ms per sample digit difference

  const byte hashIterations = 3;
  int intervalMs = 0;

  unsigned long reading;
  long result = 0;
  int tempBit = 0;

#ifdef DEBUG
  Serial.print(F("Entropy: 0x"));
#endif
  pinMode(A0, INPUT_PULLUP);
  pinMode(A0, INPUT);
  delay(200);
  // Now there will be a slow decay of the voltage,
  // about 8 seconds
  // so pick a point on the curve
  // offset by the processed previous sample:
  for (int bits = 0; bits < numBits; bits++) {
    for (int i = 0; i < hashIterations; i++) {
      //      Serial.print(' ');
      //      Serial.print( hashIterations - i );
      delay(baseIntervalMs + intervalMs);
      // take a sample
      reading = analogRead(A0);
      tempBit ^= reading & 1;
      // take the low "digits" of the reading
      // and multiply it to scale it to
      // map a new point on the decay curve:
      intervalMs = (reading % sampleSignificant) * sampleMultiplier;
    }
    result |= (long)(tempBit & 1) << bits;
  }
#ifdef DEBUG
  Serial.println(result, HEX);
#endif
  return result;
}

/**
  Main Arduino setup function
*/
void setup() {
  // Initialize the serial hardware and software ports
  Serial.begin(115200);
  SoftSerial.begin(9600);

  // Banner
  Serial.println();
  Serial.print(DEVNAME);
  Serial.print(" ");
  Serial.print(VERSION);
  Serial.print(" (");
  Serial.print(DATE);
  Serial.println(")");

  // Initialize AD9833
  DDS.begin();
  // Turn off the output
  DDS.setMode(MD_AD9833::MODE_OFF);

  // Initialize the random seed
  randomSeed(getRandomSeed());
}

/**
  Main Arduino loop
*/
void loop() {
  // Wait until the next transmission window and transmit if the location is valid
  if (loc[0] != '\0' and millis() >= nextTX and nextTX > 0) {
    // Set the next transmission window
    nextTX += decim * 120 * 1000UL;
    // Clear the buffer
    memset(txBuf, 0, 255);
    // Encode the payload
    JT.wspr_encode(call, loc, dBm, txBuf);
#ifdef DEBUG
    // Print the encoded symbols
    Serial.print(F("Symbols: "));
    for (uint8_t i = 0; i < WSPR_SYMBOL_COUNT; i++)
      Serial.print(txBuf[i]);
    Serial.println();
#endif
    // Transmit (specify band)
    transmit(6);
  }
#ifdef DEBUG
  if (nextTX > millis()) {
    // Show how long we will wait
    Serial.print(F("Next in "));
    Serial.print((nextTX - millis()) / 1000);
    Serial.println("s");
#endif
  }

  bool newData = false;
  // For one second we parse GPS data
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (SoftSerial.available()) {
      char c = SoftSerial.read();
#ifdef DEBUG
      // Print the GPS data
      //Serial.write(c);
#endif
      // Did a new valid sentence came in?
      if (gps.encode(c))
        newData = true;
    }
  }
  // Check if we have new GPS data
  if (newData) {
    uint32_t age;
    uint16_t year;
    uint8_t month, day, hour, minute, second, hndrds;
    Serial.print(F("GPS:"));
    // Get the position
    gps.f_get_position(&lat, &lon, &age);
    // Check if the last location fix is valid
    if (age != TinyGPS::GPS_INVALID_AGE) {
      // Print the coordinates
      Serial.print(lat, 6);
      Serial.print(",");
      Serial.print(lon, 6);
      Serial.print(",");
      // Update the grid locator
      getLocator(loc, lat, lon);
    }
    else {
      // No location
      Serial.print("*,*,");
    }
    // Check the grid locator (might be predefined)
    if (loc[0] != '\0') {
      Serial.print(loc);
      Serial.print(",");
    }
    else
      Serial.print("*,");
    // Get the date and time
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hndrds, &age);
    // Check if the last time fix is valid
    if (age != TinyGPS::GPS_INVALID_AGE) {
      // Find how many seconds until next transmission (one second into an even minute)
      uint8_t rem = ((minute % 2 == 0) ? 120 : 60) - second;
      char buf[16];
      sprintf(buf, "%02d:%02d:%02d,%ds", hour, minute, second, rem);
      Serial.println(buf);
      // Compute the next transmission window only if not already set
      if (nextTX < 20000UL)
        nextTX = millis() + rem * 1000UL;
    }
  }
}
