/**
  MicroWSPR - Arduino micro GPS-discplined WSPR beacon

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
#include <MicroNMEA.h>


char call[10] = "N0CALL";
char loc[5]   = "AA00";
uint8_t dBm   = 27;
uint8_t txBuf[255];


const uint16_t wsprToneSep = round(1000 * 12000 / 8192);  // 1.4648 Hz
const uint16_t wsprToneDur = round(1000 * 8192 / 12000);  // 683 ms
const uint32_t wsprBaseFrq[] = {7040000UL, 14097000UL};



// Software name and vesion
const char DEVNAME[]  PROGMEM = "MicroWSPR";
const char VERSION[]  PROGMEM = "v0.1";
const char AUTHOR[]   PROGMEM = "Costin Stroie <costinstroie@eridu.eu.org>";
const char DATE[]     PROGMEM = __DATE__;

// Pin definitions
const int FSYNC     = 10;
const int DATA      = 11;
const int CLK       = 13;

MD_AD9833  DDS(FSYNC);
// MD_AD9833  DDS(DATA, CLK, FSYNC);

// JT
JTEncode JT;


SoftwareSerial SoftSerial(8, 7); // RX, TX

// GPS
char nmeaBuffer[100];
MicroNMEA NMEA(nmeaBuffer, sizeof(nmeaBuffer));

/**
  Transmit the payload
*/
void transmit(uint8_t band = 0) {
  // Output sine wave
  DDS.setMode(MD_AD9833::MODE_SINE);
  // Choose a random 'channe' (there are 34 6Hz wide channels in 200Hz band)
  float wsprChanFrq = wsprBaseFrq[band] + random(35) * (12000 / 8192 * 4);
  // Transmit the symbols
  for (uint8_t i = 0; i < WSPR_SYMBOL_COUNT; i++) {
    DDS.setFrequency(MD_AD9833::CHAN_0, wsprChanFrq + (txBuf[i] * wsprToneSep / 1000.0));
    delay(wsprToneDur);
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

/**
  Main Arduino setup function
*/
void setup() {
  // Init the serial com and print the banner
  Serial.begin(115200);
  SoftSerial.begin(9600);

  // Initialize AD9833
  DDS.begin();

  // Clear the buffer and encode the payload
  memset(txBuf, 0, 255);
  JT.wspr_encode(call, loc, dBm, txBuf);
  // Transmit
  transmit();
}



/**
  Main Arduino loop
*/
void loop() {
}
