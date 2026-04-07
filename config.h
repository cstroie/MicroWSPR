/**
  config.h - Local configuration

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

#ifndef CONFIG_H
#define CONFIG_H

// Debug mode
#define DEBUG
//#define DEBUG_GPS

// Choose one of SI5351 or AD9833
#define USE_SI5351
//#define USE_AD9833

// Callsign
#define CALLSIGN    ("YO9JAZ")
// Power
#define DBM         (10)
// Location
//#define LOC         ("KN24")

// Transmit one in n 2-minute segments
#define DECIMATION  (1)

#define CALIBRATION (67760)

#endif /* CONFIG_H */
