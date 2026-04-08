/**
  si5351.cpp - Minimal Si5351 driver for WSPR tone generation

  Copyright (C) 2021-2026 Costin STROIE <costinstroie@eridu.eu.org>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#include "si5351.h"
#include <Wire.h>

SI5351 DDS;

// ── private ──────────────────────────────────────────────────────────────────

void SI5351::sendRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(SI5351_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void SI5351::sendRegisterBulk(uint8_t reg, uint8_t* data, uint8_t n) {
  Wire.beginTransmission(SI5351_ADDR);
  Wire.write(reg);
  while (n--) Wire.write(*data++);
  Wire.endTransmission();
}

uint8_t SI5351::recvRegister(uint8_t reg) {
  Wire.beginTransmission(SI5351_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(SI5351_ADDR, (uint8_t)1);
  return Wire.read();
}

// ── public ───────────────────────────────────────────────────────────────────

bool SI5351::init(uint8_t, uint32_t, int32_t) {
  Wire.begin();
  Wire.setClock(400000UL);
  // Probe: device must ACK its address before we proceed
  Wire.beginTransmission(SI5351_ADDR);
  if (Wire.endTransmission() != 0)
    return false;
  sendRegister(3, 0xFF);
  for (uint8_t i = 0; i < 6; i++) sendRegister(16 + i, 0x80);
  sendRegister(3, 0xFF);
  return true;
}

void SI5351::setCorrection(int32_t corr, uint8_t) {
  fxtal = F_XTAL - corr;
}

void SI5351::setFreq(uint32_t fout, uint8_t clk) {
  uint8_t rdiv = 0;
  if (fout < 500000) { rdiv = 7; fout *= 128; }
  uint16_t d = (16 * fxtal) / fout;
  if (fout > 30000000) d = (34 * fxtal) / fout;
  if ((d * (fout - 5000) / fxtal) != (d * (fout + 5000) / fxtal)) d--;
  uint32_t fvcoa = d * fout;
  uint8_t msa = fvcoa / fxtal;
  uint32_t msb = ((uint64_t)(fvcoa % fxtal) * _MSC * 128) / fxtal;
  uint32_t msp1 = 128 * msa + 128 * msb / _MSC - 512;
  uint32_t msp2 = 128 * msb - 128 * msb / _MSC * _MSC;
  uint8_t pll_regs[8] = {
    (uint8_t)((_MSC >> 8) & 0xFF),
    (uint8_t)(_MSC & 0xFF),
    (uint8_t)(msp1 >> 16),
    (uint8_t)(msp1 >> 8),
    (uint8_t)(msp1),
    (uint8_t)(((_MSC >> 12) & 0xF0) | (msp2 >> 16)),
    (uint8_t)(msp2 >> 8),
    (uint8_t)(msp2)
  };
  sendRegisterBulk(34, pll_regs, 8);  // PLLB only (CLK2 uses PLLB per reg 18)
  msp1 = (128 * msa - 512) | (((uint32_t)rdiv) << 20);
  uint8_t ms_regs[8] = {0, 1, (uint8_t)(msp1 >> 16), (uint8_t)(msp1 >> 8), (uint8_t)(msp1), 0, 0, 0};
  sendRegisterBulk(58, ms_regs, 8);   // MS2 only (42+16, CLK2)
  sendRegister(18, 0x6C);             // CLK2: PLLB, integer mode, inverted, 6mA
  if (iqmsa != msa) { iqmsa = msa; sendRegister(177, 0xA0); }
  _fout = fout; _div = d; _msa128min512 = msa * 128 - 512; _msb128 = msb;
}

void SI5351::outputEnable(uint8_t clk, uint8_t enable) {
  if (enable) sendRegister(3, ~(1 << clk));  // clear bit → enable output
  else        sendRegister(3, 0xFF);          // all bits set → all disabled
}

void SI5351::driveStrength(uint8_t clk, uint8_t strength) {
  uint8_t val = recvRegister(16 + clk);
  sendRegister(16 + clk, (val & 0xF9) | (strength << 1));
}
