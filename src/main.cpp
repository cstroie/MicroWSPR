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

#include <Arduino.h>

// Safe values (overridden by build_flags in platformio.ini)
#ifndef CALLSIGN
#define CALLSIGN    ("N0CALL")
#endif
#ifndef DBM
#define DBM         (10)
#endif
#ifndef LOC
#define LOC         ("")
#endif
#ifndef DECIMATION
#define DECIMATION  (5)
#endif
#ifndef CALIBRATION
#define CALIBRATION (0)
#endif

#ifdef USE_AD9833
// AD9833 and SPI
#include <MD_AD9833.h>
#include <SPI.h>
#endif
#ifdef USE_SI5351
#include <si5351.h>
#endif
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

  $GPRMC,212620.00,A,4427.64308,N,02607.97828,E,0.012,,040321,,,A*71
  $GPVTG,,T,,M,0.012,N,0.023,K,A*21
  $GPGGA,212620.00,4427.64308,N,02607.97828,E,1,05,2.42,145.4,M,34.8,M,,*50
  $GPGSA,A,3,27,08,16,21,32,,,,,,,,3.55,2.42,2.59*03
  $GPGSV,4,1,15,01,16,287,,08,63,282,34,10,54,052,,14,04,332,*75
  $GPGSV,4,2,15,16,06,200,30,18,01,097,,20,20,058,21,21,40,295,35*7C
  $GPGSV,4,3,15,22,17,239,23,21,054,,24,01,045,18,27,62,180,42*77
  $GPGSV,4,4,15,28,02,337,,32,44,128,30,39,39,181,28*49
  $GPGLL,4427.64308,N,02607.97828,E,212620.00,A,A*6F

*/

// 1623
// 1599

char call[10] = CALLSIGN;
char loc[7]   = LOC;
uint8_t dBm   = DBM;
uint8_t txBuf[255];

const uint16_t wsprToneSep = round(1000UL * 12000 / 8192);  // 1.4648 Hz
const uint16_t wsprToneDur = round(1000UL * 8192 / 12000);  // 683 ms
const uint32_t wsprBaseFrq[] = {0UL, 136000UL, 474200UL, 1836600UL, 3568600UL,
                                5287200UL, 7038600UL, 10138700UL, 14095600UL, 18104600UL,
                                21094600UL, 24924600UL, 28124600UL, 50293000UL, 144489000UL
                               };
// Bands
enum HAM_BANDS {BAND_OFF, BAND_2190, BAND_630, BAND_160,
                BAND_80, BAND_60, BAND_40, BAND_30, BAND_20,
                BAND_17, BAND_15, BAND_12, BAND_10, BAND_6, BAND_2
               };
// Selected bands
const uint8_t selBands[] = {BAND_40, BAND_20, BAND_15};
uint8_t idxBand = 0;

// The next transmission window
uint32_t nextTX  = 0UL;
uint8_t  countTX = 0;
uint8_t  decim   = DECIMATION;

// Software name and vesion
const char DEVNAME[]  = "MicroWSPR";
const char VERSION[]  = "v0.6";
const char AUTHOR[]   = "Costin Stroie <costinstroie@eridu.eu.org>";
const char DATE[]     = __DATE__;

#ifdef USE_AD9833
// Pin definitions
const int FSYNC     = 10;
const int DATA      = 11;
const int CLK       = 13;

MD_AD9833  DDS(FSYNC);
//MD_AD9833  DDS(DATA, CLK, FSYNC);
#endif
#ifdef USE_SI5351
Si5351 DDS;
#endif

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
  float wsprChanFrq = 1400 + (random(25) + 5) * (4.0 * 12000UL / 8192);
  float wsprSymbFrq;
#ifdef DEBUG
  Serial.print(F("Base frequency: "));
  Serial.print(wsprChanFrq, 3);
  Serial.print(F(" "));
  Serial.println(wsprBaseFrq[band] + wsprChanFrq, 3);
#endif
#ifdef USE_AD9833
  // Output sine wave
  DDS.setMode(MD_AD9833::MODE_SINE);
#endif
#ifdef USE_SI5351
  // Turn on the output
  DDS.output_enable(SI5351_CLK2, 1);
  digitalWrite(LED_BUILTIN, HIGH);
#endif
  nextSym = millis();
  // Transmit the symbols
  for (uint8_t i = 0; i < WSPR_SYMBOL_COUNT; i++) {
    wsprSymbFrq = wsprBaseFrq[band] + wsprChanFrq + (txBuf[i] * wsprToneSep / 1000.0);
#ifdef USE_AD9833
    DDS.setFrequency(MD_AD9833::CHAN_0, wsprSymbFrq);
#endif
#ifdef USE_SI5351
    DDS.set_freq(wsprSymbFrq * 100, SI5351_CLK2);
#endif
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
#ifdef USE_AD9833
  DDS.setMode(MD_AD9833::MODE_OFF);
#endif
#ifdef USE_SI5351
  digitalWrite(LED_BUILTIN, LOW);
  DDS.output_enable(SI5351_CLK2, 0);
#endif
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
  Serial.print(F("Entropy: 0x"));
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
  Serial.print(" ");
#ifdef USE_AD9833
  Serial.print("AD9833");
#endif
#ifdef USE_SI5351
  Serial.print("SI5351");
#endif
  Serial.print(" (");
  Serial.print(DATE);
  Serial.println(")");

#ifdef USE_AD9833
  // Initialize AD9833
  DDS.begin();
  // Turn off the output
  DDS.setMode(MD_AD9833::MODE_OFF);
#endif
#ifdef USE_SI5351
  // Initialize the Si5351
  if (! DDS.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0))
    Serial.println("Device not found on I2C bus!");
  // Callibration
#if CALIBRATION != 0
  DDS.set_correction(CALIBRATION, SI5351_PLL_INPUT_XO);
  DDS.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
#endif
  // Set for max power if desired
  DDS.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);
  // Disable the clock initially
  DDS.output_enable(SI5351_CLK2, 0);
  // Use the builtin led to signal TX
  pinMode(LED_BUILTIN, OUTPUT);
#endif

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
    transmit(selBands[idxBand]);
    // Switch band
    if (++idxBand >= sizeof(selBands) / sizeof(selBands[0]))
      idxBand = 0;
    // Count the transmissions before resync
    countTX++;
  }
#ifdef DEBUG
  if (nextTX > millis()) {
    // Show how long we will wait
    Serial.print(F("Next in "));
    Serial.print((nextTX - millis()) / 1000);
    Serial.println("s");
  }
#endif

  bool newData = false;
  // For one second we parse GPS data
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (SoftSerial.available()) {
      char c = SoftSerial.read();
#ifdef DEBUG_GPS
      // Print the GPS data
      Serial.write(c);
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
    Serial.println();
    Serial.print(F("GPS: "));
    Serial.print(gps.satellites());
    Serial.print(",");
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
      if (nextTX == 0 or countTX * decim >= 30) {
        nextTX = millis() + (rem + 1) * 1000UL;
        countTX = 0;
      }
    }
  }
}
