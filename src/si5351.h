/**
  si5351.h - Minimal Si5351 driver for WSPR tone generation

  Copyright (C) 2021-2026 Costin STROIE <costinstroie@eridu.eu.org>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#pragma once
#include <Arduino.h>

#define SI5351_ADDR 0x60       // I2C address (A0/A1 pins low)
#define F_XTAL      25004000UL // Nominal crystal frequency (Hz); trimmed by setCorrection()

/**
 * Minimal Si5351 clock generator driver for WSPR tone generation on CLK2.
 *
 * Hardware frequency chain:
 *   crystal (fxtal) → PLLB (fvcoa = d * fout) → MultiSynth 2 (÷d) → CLK2 output
 *
 * Only PLLB and MS2 are used; PLLA and CLK0/CLK1 are left powered down.
 * Integer-N PLL mode is used throughout: the fractional part of the MultiSynth
 * divider is fixed at a constant denominator (_MSC), keeping the math in 32-bit
 * integers and saving ~10 KB flash vs the full Etherkit library.
 *
 * Frequency resolution is sufficient for WSPR tones (1.46 Hz spacing): the
 * PLL VCO is set to an exact integer multiple of fout, so tone switching only
 * requires updating PLLB — not resetting it — which avoids phase glitches
 * between symbols as long as d (the VCO multiplier) stays constant.
 */

// Fractional denominator for MSP2 register encoding (fixed constant, must be < 2^20)
#define _MSC 0x80000

class SI5351 {
private:
  // Cached state used to detect when a PLL reset is required
  volatile int32_t  _fout;          // last requested output frequency
  volatile uint8_t  _div;           // last VCO multiplier d
  volatile uint16_t _msa128min512;  // last MSP1 integer term (128*msa - 512)
  volatile uint32_t _msb128;        // last MSP2 fractional numerator * 128
  int16_t  iqmsa;   // last msa value written; PLL reset triggered when this changes
  uint32_t fxtal;   // effective crystal frequency after correction (Hz)

  /** Write a single byte to Si5351 register reg over I2C. */
  void sendRegister(uint8_t reg, uint8_t val);
  /** Write n consecutive bytes to Si5351 registers starting at reg. */
  void sendRegisterBulk(uint8_t reg, uint8_t* data, uint8_t n);
  /** Read a single byte from Si5351 register reg over I2C. */
  uint8_t recvRegister(uint8_t reg);

public:
  /**
   * Initialise I2C at 400 kHz, probe the device, and power down all outputs.
   * Extra parameters are accepted for API compatibility but ignored.
   * Returns true if the Si5351 acknowledges its I2C address, false otherwise.
   */
  bool init(uint8_t, uint32_t, int32_t);

  /**
   * Apply a frequency correction to the crystal reference.
   * corr (Hz) is subtracted from F_XTAL; positive values shift output down.
   * The second parameter is accepted for API compatibility but ignored.
   */
  void setCorrection(int32_t corr, uint8_t);

  /**
   * Set the output frequency of clock clk (0-2) to fout (Hz).
   * Only CLK2 is used for WSPR; clk is accepted for API compatibility.
   * Computes PLL and MultiSynth register values for integer-N mode and
   * resets PLLB only when the VCO multiplier d changes between calls.
   */
  void setFreq(uint32_t fout, uint8_t clk);

  /**
   * Enable or disable clock output clk (0-2).
   * enable=1 turns on clk only; enable=0 disables all outputs (reg 3 = 0xFF).
   */
  void outputEnable(uint8_t clk, uint8_t enable);

  /**
   * Set output drive strength for clk (0-2).
   * strength: 0=2mA, 1=4mA, 2=6mA, 3=8mA (bits 1-0 of CLKn control register).
   */
  void driveStrength(uint8_t clk, uint8_t strength);
};

extern SI5351 DDS;
