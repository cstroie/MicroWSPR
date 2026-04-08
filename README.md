# MicroWSPR

Arduino Nano GPS-disciplined WSPR beacon using Si5351 DDS and GPS for frequency and time synchronization.

## Hardware

- **Arduino Nano** (ATmega328P)
- **Si5351** clock generator (I2C address 0x60)
- **GPS module** (NMEA $GPRMC at 9600 baud)

## Features

- GPS-disciplined frequency and time synchronization
- WSPR transmission on HF bands: 2190m, 630m, 160m, 80m, 60m, 40m, 30m, 20m, 17m, 15m, 12m, 10m, 6m, 2m
- Maidenhead grid locator from GPS coordinates
- EEPROM-stored configuration (callsign, locator, power, bands, TX interval)
- Interactive serial configuration TUI
- LED status indication

## Supported Bands

| Band | Frequency (kHz) |
|------|-----------------|
| 2190m | 136.000 |
| 630m | 474.200 |
| 160m | 1836.600 |
| 80m | 3568.600 |
| 60m | 5287.200 |
| 40m | 7038.600 |
| 30m | 10138.700 |
| 20m | 14095.600 |
| 17m | 18104.600 |
| 15m | 21094.600 |
| 12m | 24924.600 |
| 10m | 28124.600 |
| 6m | 50293.000 |
| 2m | 144489.000 |

## LED Status

- **Fast blink (100/100ms)**: Hardware fault (GPS or Si5351 not detected)
- **Brief flash every 2s**: Waiting for GPS fix
- **Solid on**: Transmitting
- **Off**: Fix acquired, between transmissions

## Build

Requires [PlatformIO](https://platformio.org):

```bash
pio run
```

Upload to Arduino Nano:

```bash
pio run -t upload
```

## Configuration

Connect via serial at 115200 baud. On boot, press any key within 5 seconds to enter the configuration menu.

### TUI Options

- Set callsign (up to 10 characters)
- Set Maidenhead locator (4 characters, e.g. "JO32")
- Set transmit power (dBm)
- Select bands (bitmask)
- Set TX decimation (minutes between transmissions, default: 10)

The configuration is stored in EEPROM and persists across reboots.

## License

GNU General Public License v3.0

## Author

Copyright (C) 2021-2026 Costin STROIE <costinstroie@eridu.eu.org>
