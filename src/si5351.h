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

/**
 * Minimal Si5351 driver: Integer-N PLL mode for WSPR tone generation on CLK2.
 * Saves ~10 KB flash vs the full library by dropping fractional-N math.
 */
#define SI5351_ADDR 0x60
#define F_XTAL 25004000UL

class SI5351 {
private:
  volatile int32_t _fout;
  volatile uint8_t _div;
  volatile uint16_t _msa128min512;
  volatile uint32_t _msb128;
  int16_t iqmsa;
  uint32_t fxtal;
  #define _MSC 0x80000

  /** Write a byte to a Si5351 register. */
  void sendRegister(uint8_t reg, uint8_t val);
  /** Write n bytes to consecutive Si5351 registers starting at reg. */
  void sendRegisterBulk(uint8_t reg, uint8_t* data, uint8_t n);
  /** Read a byte from a Si5351 register. */
  uint8_t recvRegister(uint8_t reg);

public:
  /** Initialize Si5351: bring up I2C at 400 kHz and disable all outputs. */
  bool init(uint8_t, uint32_t, int32_t);
  /** Apply frequency correction; corr (Hz) is subtracted from the crystal frequency. */
  void setCorrection(int32_t corr, uint8_t);
  /**
   * Set output frequency on clk (0-2); only CLK2 is used for WSPR.
   * Integer-N PLL mode gives adequate precision for WSPR tones with minimal code.
   */
  void setFreq(uint32_t fout, uint8_t clk);
  /** Enable (1) or disable (0) clock output clk (0-2). */
  void outputEnable(uint8_t clk, uint8_t enable);
  /** Set drive strength for clk (0-2): 0=2mA, 1=4mA, 2=6mA, 3=8mA. */
  void driveStrength(uint8_t clk, uint8_t strength);
};

extern SI5351 DDS;
