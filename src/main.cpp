/**
  MicroWSPR - Arduino Nano GPS-disciplined WSPR beacon

  Copyright (C) 2021-2026 Costin STROIE <costinstroie@eridu.eu.org>

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
#include "config.h"
#include "gps.h"
#include "si5351.h"

#include <JTEncode.h>

// ── constants ────────────────────────────────────────────────────────────────

const char DEVNAME[] = "MicroWSPR";
const char VERSION[] = "v1.0";
const char DATE[]    = __DATE__;

/**
 * HAM band indices used to index wsprBaseFrq[].
 * BAND_OFF (0) means no band is selected; BAND_2190…BAND_2 map to 1-14.
 * cfg.bands stores a bitmask where bit N corresponds to HAM_BANDS value N.
 */
enum HAM_BANDS {
  BAND_OFF,
  BAND_2190, BAND_630,  BAND_160,
  BAND_80,   BAND_60,   BAND_40,  BAND_30,  BAND_20,
  BAND_17,   BAND_15,   BAND_12,  BAND_10,  BAND_6,   BAND_2
};

// ── LED status ───────────────────────────────────────────────────────────────

/**
 * LED blink patterns used to communicate beacon state at a glance.
 *   LED_FAULT  — fast 100 ms on / 100 ms off: hardware not detected (GPS or Si5351)
 *   LED_NO_FIX — 50 ms flash every 2 s: waiting for GPS time or fix
 *   LED_TX     — solid on: actively transmitting WSPR symbols
 *   LED_IDLE   — off: GPS time acquired, between transmissions
 */
enum LedState {
  LED_FAULT,
  LED_NO_FIX,
  LED_TX,
  LED_IDLE
};

LedState ledState = LED_NO_FIX;

/**
 * Drive LED_BUILTIN according to the current ledState.
 *
 * Must be called on every loop() iteration to maintain accurate timing.
 * Uses static variables to track toggle time and current pin level so it
 * never calls delay() and does not block the main loop.
 *
 * Blink patterns:
 *   LED_FAULT  — 100 ms on / 100 ms off (rapid blink)
 *   LED_NO_FIX — 50 ms on / 1950 ms off (brief flash every 2 s)
 *   LED_TX     — continuously on
 *   LED_IDLE   — continuously off
 */
void ledUpdate() {
  static uint32_t lastToggle = 0;
  static bool     ledOn      = false;
  uint32_t now = millis();

  switch (ledState) {
    case LED_FAULT:
      if (now - lastToggle >= 100) {
        ledOn = !ledOn;
        digitalWrite(LED_BUILTIN, ledOn);
        lastToggle = now;
      }
      break;
    case LED_NO_FIX:
      if (!ledOn && now - lastToggle >= 2000) {
        digitalWrite(LED_BUILTIN, HIGH);
        ledOn = true;
        lastToggle = now;
      } else if (ledOn && now - lastToggle >= 50) {
        digitalWrite(LED_BUILTIN, LOW);
        ledOn = false;
        lastToggle = now;
      }
      break;
    case LED_TX:
      digitalWrite(LED_BUILTIN, HIGH);
      ledOn = true;
      break;
    case LED_IDLE:
      digitalWrite(LED_BUILTIN, LOW);
      ledOn = false;
      break;
  }
}

// ── WSPR ─────────────────────────────────────────────────────────────────────

/**
 * WSPR dial frequencies (Hz) for each HAM_BANDS index.
 * Index 0 is unused (BAND_OFF).  Values are the standard WSPR channel
 * centre frequencies as published by the WSPR protocol specification.
 * The actual TX frequency is offset within the 200 Hz WSPR channel by
 * transmit() to spread beacons and reduce collisions.
 */
const uint32_t wsprBaseFrq[] = {
  0UL,
  136000UL,   474200UL,   1836600UL,  3568600UL,
  5287200UL,  7038600UL,  10138700UL, 14095600UL, 18104600UL,
  21094600UL, 24924600UL, 28124600UL, 50293000UL, 144489000UL
};

/**
 * WSPR tone spacing and symbol duration derived from the protocol constants.
 *
 * WSPR uses 4-FSK with:
 *   Symbol rate  = 12000 / 8192 ≈ 1.4648 baud
 *   Tone spacing = symbol rate  ≈ 1.4648 Hz
 *   Symbol duration = 1 / symbol_rate ≈ 683 ms
 *
 * wsprToneSep is stored in mHz (millihertz) to avoid floating-point in the
 * frequency calculation inner loop.
 * wsprToneDur is stored in ms.
 */
const uint16_t wsprToneSep = round(1000UL * 12000 / 8192);  // 1464.8 mHz ≈ 1.4648 Hz
const uint16_t wsprToneDur = round(1000UL * 8192 / 12000);  // 682.7 ms

uint8_t  txBuf[WSPR_SYMBOL_COUNT];  // encoded 4-FSK symbol buffer (162 symbols)
JTEncode JT;                        // JTEncode instance for wspr_encode()

// Current band index (HAM_BANDS, 1-14); 0 = no band selected
uint8_t  curBand = 0;
// Absolute millis() timestamp for the next scheduled TX; 0 = not yet scheduled
uint32_t nextTX  = 0UL;

/**
 * Advance curBand to the next enabled band in cfg.bands, wrapping around.
 *
 * Iterates forward through band indices 1-14, starting just after curBand,
 * and stops at the first band whose bit is set in cfg.bands.  If no bands
 * are enabled (cfg.bands == 0) or only the current band is set, curBand is
 * set to 0 (BAND_OFF).
 */
void advanceBand() {
  if (!cfg.bands) { curBand = 0; return; }
  for (int i = 1; i <= 14; i++) {
    uint8_t b = (curBand - 1 + i) % 14 + 1;
    if (cfg.bands & (1 << b)) { curBand = b; return; }
  }
  curBand = 0;
}

/**
 * Transmit the pre-encoded WSPR symbol buffer on the given band.
 *
 * Picks a random starting offset within the 200 Hz WSPR channel to reduce
 * collisions with other beacons on the same band:
 *   wsprChanFrq = 1400 + (5..29) × tone_spacing  (Hz above dial frequency)
 *
 * Then clocks out all 162 symbols back-to-back, each held for wsprToneDur ms.
 * The Si5351 CLK2 output is enabled for the duration and disabled afterwards.
 * The LED is set to LED_TX while transmitting and restored to LED_IDLE on exit.
 *
 * Timing: uses a running nextSym timestamp incremented by wsprToneDur on each
 * symbol, then busy-waits until millis() reaches it.  This keeps symbol timing
 * accurate even if setFreq() takes a variable amount of time.
 *
 * Total transmission time: 162 × 683 ms ≈ 110.6 seconds.
 */
void transmit(uint8_t band = 0) {
  uint32_t nextSym;
  float wsprChanFrq = 1400 + (random(25) + 5) * (4.0 * 12000UL / 8192);
  float wsprSymbFrq;
  Serial.print(F("  freq: "));
  Serial.print((wsprBaseFrq[band] + wsprChanFrq) / 1000.0, 3);
  Serial.println(F(" kHz"));
  ledState = LED_TX;
  ledUpdate();
  DDS.outputEnable(cfg.clkOutput, 1);
  nextSym = millis();
  for (uint8_t i = 0; i < WSPR_SYMBOL_COUNT; i++) {
    wsprSymbFrq = wsprBaseFrq[band] + wsprChanFrq + (txBuf[i] * wsprToneSep / 1000.0);
    DDS.setFreq(wsprSymbFrq * 100, cfg.clkOutput);
    nextSym += wsprToneDur;
    while (millis() < nextSym);
  }
  DDS.outputEnable(cfg.clkOutput, 0);
  ledState = LED_IDLE;
}

// ── entropy ──────────────────────────────────────────────────────────────────

/**
 * Collect numBits bits of hardware entropy from ADC noise on pin A0.
 *
 * Algorithm (per bit):
 *   Repeat hashIterations times:
 *     1. Delay a variable interval derived from the previous ADC reading
 *        (introduces timing jitter).
 *     2. Read A0 (floating, driven only by thermal/quantisation noise).
 *     3. XOR the LSB of the reading into tempBit.
 *   Store the final tempBit LSB as one entropy bit in result.
 *
 * The variable delay (reading % sampleSignificant × sampleMultiplier ms)
 * breaks correlation between successive samples by making the sampling instant
 * depend on the noise itself.
 *
 * Returns a long suitable for passing directly to randomSeed().
 */
long getRandomSeed(int numBits = 31) {
  if (numBits > 31 || numBits < 1) numBits = 31;
  const int  baseIntervalMs    = 1;
  const byte sampleSignificant = 7;
  const byte sampleMultiplier  = 10;
  const byte hashIterations    = 3;
  int  intervalMs = 0;
  long result     = 0;
  int  tempBit    = 0;
  pinMode(A0, INPUT_PULLUP);
  pinMode(A0, INPUT);
  delay(200);
  for (int bits = 0; bits < numBits; bits++) {
    for (int i = 0; i < hashIterations; i++) {
      delay(baseIntervalMs + intervalMs);
      unsigned long reading = analogRead(A0);
      tempBit ^= reading & 1;
      intervalMs = (reading % sampleSignificant) * sampleMultiplier;
    }
    result |= (long)(tempBit & 1) << bits;
  }
  Serial.print(F("Entropy: 0x"));
  Serial.println(result, HEX);
  return result;
}

// ── setup ────────────────────────────────────────────────────────────────────

/**
 * One-time initialisation: hardware, config, and pre-flight checks.
 *
 * Sequence:
 *   1. Open Serial at 115200 baud and print the firmware banner.
 *   2. Initialise GPS serial port; set LED_FAULT if no module is detected.
 *   3. Initialise Si5351 (I²C probe only); set LED_FAULT if not found.
 *   4. Seed the PRNG from ADC noise.
 *   5. Load configuration from EEPROM (writes defaults on first boot).
 *   6. Apply cfg.calibration and cfg.clkOutput to the Si5351 now that cfg is
 *      populated; configure drive strength and disable output until TX time.
 *   7. Initialise GPS SoftwareSerial on cfg.gpsRxPin/cfg.gpsTxPin; set
 *      LED_FAULT if no data received within 1 s.
 *   8. Copy cfg.locator into the working loc[] buffer used by gpsUpdate()
 *      and transmit(); GPS will overwrite this when a fix is obtained and
 *      cfg.locator is empty.
 *   9. Select the first enabled band.
 *  10. Print a config summary.
 *  11. Open a 5-second boot-time config window; any serial keypress launches
 *      the interactive TUI.
 *  12. Enforce a valid callsign — loop in the TUI until one is set.
 *  13. If GPS is absent and no locator is stored, loop in the TUI until a
 *      locator is entered (without a position there is nothing to transmit).
 *  14. Print the appropriate "Waiting for GPS…" message and return.
 */
void setup() {
  Serial.begin(115200);

  Serial.println();
  Serial.print(DEVNAME); Serial.print(' ');
  Serial.print(VERSION); Serial.print(F(" SI5351 ("));
  Serial.print(DATE);
  Serial.println(')');

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.print(F("Si5351 : "));
  if (DDS.init(8, 0, 0)) {
    Serial.println(F("OK"));
  } else {
    Serial.println(F("not found!"));
    ledState = LED_FAULT;
  }
  randomSeed(getRandomSeed());

  // Load config from EEPROM (or defaults on first boot)
  configLoad();

  // Apply stored Si5351 settings now that cfg is populated
  if (cfg.calibration != 0)
    DDS.setCorrection(cfg.calibration, 0);
  DDS.driveStrength(cfg.clkOutput, 2);  // strength: 0=2mA, 1=4mA, 2=6mA, 3=8mA
  DDS.outputEnable(cfg.clkOutput, 0);

  // Initialise GPS on the configured pins now that cfg is populated
  Serial.print(F("GPS    : "));
  bool gpsDetected = gpsInit();
  if (gpsDetected) {
    Serial.println(F("detected"));
  } else {
    Serial.println(F("no data!"));
    ledState = LED_FAULT;
  }

  // Initialise working locator from stored config; GPS will override if empty
  strncpy(loc, cfg.locator, sizeof(loc));

  // Select the first enabled band
  advanceBand();

  // Print config summary so the user knows what will be transmitted
  configSummary();

  // 5-second boot window: any keypress opens the interactive config TUI
  Serial.println(F("Press any key for configuration..."));
  uint32_t deadline = millis() + 5000UL;
  while (millis() < deadline) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();  // discard the trigger byte(s)
      configTUI();
      strncpy(loc, cfg.locator, sizeof(loc));
      curBand = 0; advanceBand(); nextTX = 0;
      break;
    }
  }

  // Enforce a real callsign — N0CALL is the factory default placeholder
  while (cfg.callsign[0] == '\0' || strcmp(cfg.callsign, "N0CALL") == 0) {
    Serial.println(F("Callsign not set. Please configure."));
    configTUI();
    strncpy(loc, cfg.locator, sizeof(loc));
    curBand = 0; advanceBand(); nextTX = 0;
  }

  // Without GPS and without a stored locator there is no position to encode —
  // keep the user in the TUI until a locator is manually entered
  if (!gpsDetected && cfg.locator[0] == '\0') {
    Serial.println(F("No GPS detected and no locator set. Please configure."));
    while (cfg.locator[0] == '\0') {
      configTUI();
      strncpy(loc, cfg.locator, sizeof(loc));
    }
    curBand = 0; advanceBand(); nextTX = 0;
  }

  if (cfg.locator[0])
    Serial.println(F("Waiting for GPS time..."));
  else
    Serial.println(F("Waiting for GPS fix..."));
}

// ── loop ─────────────────────────────────────────────────────────────────────

/**
 * Main beacon loop: transmit on schedule, track GPS time, and resync.
 *
 * Called repeatedly by the Arduino runtime.  Each iteration:
 *
 *   1. ledUpdate() — maintain the non-blocking LED blink pattern.
 *
 *   2. TX gate — if a locator is available, nextTX is set, and millis() has
 *      reached nextTX, fire a transmission:
 *        - Clear nextTX to 0 (rescheduled below from fresh GPS time).
 *        - Encode callsign / locator / power into txBuf via wspr_encode().
 *        - Call transmit() for the current band (~110 s blocking call).
 *        - Advance to the next enabled band; set txFired flag.
 *
 *   3. gpsUpdate() — poll the GPS for 1 s; returns seconds to the next
 *      even-minute WSPR slot, or -1 if no valid time.
 *        - rem >= 0: GPS time is valid → restore LED_IDLE if it was LED_NO_FIX.
 *        - rem < 0:  GPS time lost → cancel nextTX, set LED_NO_FIX.  The
 *          "GPS time lost" message is printed only once (guarded by nextTX > 0).
 *
 *   4. Locator auto-save — once per boot, if the GPS-derived loc[] differs
 *      from the stored cfg.locator, write it to EEPROM so future cold starts
 *      can transmit without waiting for a position fix.
 *
 *   5. TX scheduling — whenever nextTX == 0 and GPS time is available:
 *        - Initial / after GPS recovery: nextTX = millis() + (rem+1) s.
 *        - After a TX (txFired): nextTX = millis() + (rem+1) s
 *          + (decimation-1) × 120 s, so the gap to the next TX is exactly
 *          decimation × 120 s measured from the current slot boundary.
 *      Scheduling always derives from fresh GPS time, so millis() drift is
 *      corrected after every transmission — no separate resync counter needed.
 */
void loop() {
  ledUpdate();

  bool txFired = false;
  if (loc[0] != '\0' && millis() >= nextTX && nextTX > 0) {
    nextTX = 0;  // cleared here; rescheduled below from fresh GPS time
    memset(txBuf, 0, sizeof(txBuf));
    JT.wspr_encode(cfg.callsign, loc, cfg.dbm, txBuf);
    if (curBand > 0) {
      Serial.print(F("TX: "));
      Serial.print(getBandName(curBand));
      Serial.print(F(" ("));
      Serial.print(wsprBaseFrq[curBand] / 1000.0, 1);
      Serial.println(F(" kHz)"));
      transmit(curBand);
      Serial.println(F("TX done."));
    }
    advanceBand();
    txFired = true;
  }

  int rem = gpsUpdate();
  if (rem >= 0) {
    if (ledState == LED_NO_FIX)
      ledState = LED_IDLE;
  } else {
    // GPS time lost — cancel schedule and signal no-fix until time returns
    if (nextTX > 0) {
      Serial.println(F("GPS time lost, TX suspended."));
      nextTX = 0;
    }
    ledState = LED_NO_FIX;
  }

  // Auto-save GPS-derived locator once per boot if it differs from stored value
  static bool locatorSaved = false;
  if (!locatorSaved && loc[0] != '\0' && strcmp(loc, cfg.locator) != 0) {
    strncpy(cfg.locator, loc, sizeof(cfg.locator));
    Serial.print(F("Locator auto-saved: ")); Serial.println(cfg.locator);
    configSave();
    locatorSaved = true;
  }

  // Schedule next TX from GPS time whenever nextTX is unset (startup, after
  // GPS loss recovery, or after a transmission).
  // After a TX: skip (decimation-1) additional slots beyond the next boundary
  // so the total gap is exactly decimation × 120 s, GPS-disciplined each time.
  if (rem >= 0 && nextTX == 0) {
    uint32_t skipMs = txFired ? (uint32_t)(cfg.decimation - 1) * 120 * 1000UL : 0UL;
    nextTX = millis() + (rem + 1) * 1000UL + skipMs;
    Serial.print(F("Next TX: "));
    Serial.print(getBandName(curBand));
    Serial.print(F(" in "));
    Serial.print(rem + 1 + (txFired ? (cfg.decimation - 1) * 120 : 0));
    Serial.println('s');
  }
}
