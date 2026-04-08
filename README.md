# MicroWSPR

Arduino Nano GPS-disciplined WSPR beacon using a Si5351 clock generator for
frequency synthesis and a GPS module for time and position.

## Hardware

| Component | Details |
|-----------|---------|
| **MCU** | Arduino Nano (ATmega328P) |
| **DDS** | Si5351 clock generator, I²C address 0x60 |
| **GPS** | Any module outputting NMEA $GPRMC at 9600 baud |

Default wiring:

- Si5351 SDA → A4, SCL → A5 (Arduino Nano hardware I²C)
- GPS TX → D3 (SoftwareSerial RX), GPS RX → D4 (unused)
- Si5351 CLK0 → antenna / bandpass filter (configurable)

All pins and the active clock output are runtime-configurable via the serial TUI.

## Features

- GPS-disciplined WSPR transmission synchronized to even UTC minutes
- All 14 standard WSPR bands from 2190 m to 2 m
- Automatic Maidenhead grid locator derived from GPS position
- **Transmits with GPS time + EEPROM locator** — no position fix required
- GPS-derived locator auto-saved to EEPROM after first fix
- Transmission suspended and rescheduled on GPS time loss
- EEPROM-backed configuration (survives power cycles)
- Interactive serial configuration TUI at boot
- Non-blocking LED status indication
- Hardware entropy from ADC noise for PRNG seeding

## Supported Bands

| Band | Dial frequency (kHz) |
|------|----------------------|
| 2190 m | 136.000 |
| 630 m | 474.200 |
| 160 m | 1836.600 |
| 80 m | 3568.600 |
| 60 m | 5287.200 |
| 40 m | 7038.600 |
| 30 m | 10138.700 |
| 20 m | 14095.600 |
| 17 m | 18104.600 |
| 15 m | 21094.600 |
| 12 m | 24924.600 |
| 10 m | 28124.600 |
| 6 m | 50293.000 |
| 2 m | 144489.000 |

Multiple bands can be enabled simultaneously; the beacon rotates through them
on successive TX slots.

## LED Status

| Pattern | Meaning |
|---------|---------|
| Fast blink 100 ms / 100 ms | Hardware fault — GPS or Si5351 not detected |
| Brief 50 ms flash every 2 s | Waiting for GPS time or fix |
| Solid on | Transmitting |
| Off | GPS time acquired, between transmissions |

## Build

Requires [PlatformIO](https://platformio.org):

```bash
pio run
pio run -t upload
```

## Configuration

Connect a serial terminal at **115200 baud**. On boot, press any key within
the 5-second window to enter the configuration menu. The menu is also entered
automatically if no callsign is set or no locator is available without GPS.

```
================================
   MicroWSPR  -  Configuration
================================
  1. Callsign   : YO8CRA
  2. Power      : 10 dBm
  3. Locator    : KN46
  4. Decimation : 1
  5. Bands      : 40m 20m
  6. Calibration: 0 Hz
  7. CLK output : CLK0
  8. GPS RX pin : 3
  9. GPS TX pin : 4
--------------------------------
  S. Save and exit
  Q. Quit without saving
================================
```

### Configuration options

| # | Parameter | Description |
|---|-----------|-------------|
| 1 | **Callsign** | Amateur callsign, up to 9 characters, forced upper-case |
| 2 | **Power** | TX power in dBm (0–60); encoded into the WSPR message |
| 3 | **Locator** | Maidenhead grid square, 4 or 6 characters. Leave empty to derive from GPS position |
| 4 | **Decimation** | Transmit every Nth WSPR slot (1 = every 2-minute slot, 2 = every 4 minutes, …) |
| 5 | **Bands** | Toggle individual bands on/off; multiple bands are cycled round-robin |
| 6 | **Calibration** | Si5351 crystal frequency correction in Hz (±999999). See [Calibration](#calibration) |
| 7 | **CLK output** | Si5351 output to use: 0 = CLK0, 1 = CLK1, 2 = CLK2 (default 0) |
| 8 | **GPS RX pin** | Arduino pin connected to GPS TX (SoftwareSerial RX); default 3 |
| 9 | **GPS TX pin** | Arduino pin connected to GPS RX (SoftwareSerial TX, unused); default 4 |

Configuration is stored in EEPROM and survives reboots. The EEPROM layout is
versioned with a magic number; a firmware upgrade that changes the struct layout
automatically resets all settings to defaults on the first boot.

## Calibration

The Si5351 uses a 25 MHz crystal whose actual frequency may differ slightly
from nominal, causing the beacon to transmit a few Hz off the expected dial
frequency.

To calibrate:

1. Receive your own beacon with an SDR or accurate receiver.
2. Note the measured centre frequency of the signal.
3. Calculate the error: `error = measured − expected` (Hz).
4. Enter that value as **Calibration** (positive = measured above expected).
5. Save and reboot. Verify with another reception.

Alternatively, use a report from [WSPRnet](https://www.wsprnet.org) — the
reported frequency drift field gives a direct indication of calibration error.

## TX scheduling

WSPR slots start at second :00 of every even UTC minute. The beacon:

1. Waits for GPS time (a position fix is not required if a locator is stored).
2. Schedules the next TX to fire 1 second after the upcoming slot boundary.
3. Encodes callsign, locator, and power into 162 WSPR symbols (~110 s).
4. Transmits, advances to the next enabled band, and reschedules.
5. If GPS time is lost the schedule is cancelled and the LED returns to the
   "waiting" flash pattern. TX resumes automatically when time is regained.

The scheduler resyncs from GPS time every 30 slots (~1 hour) to prevent
`millis()` drift from accumulating.

## License

GNU General Public License v3.0

## Author

Copyright (C) 2021–2026 Costin STROIE <costinstroie@eridu.eu.org>
