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
  // Probe: device must ACK its I2C address before proceeding
  Wire.beginTransmission(SI5351_ADDR);
  if (Wire.endTransmission() != 0)
    return false;
  sendRegister(3, 0xFF);                              // reg 3: disable all clock outputs
  for (uint8_t i = 0; i < 6; i++)
    sendRegister(16 + i, 0x80);                       // regs 16-21: power down CLK0-CLK5
  sendRegister(3, 0xFF);                              // re-confirm all outputs disabled
  return true;
}

void SI5351::setCorrection(int32_t corr, uint8_t) {
  fxtal = F_XTAL - corr;  // positive corr shifts the effective crystal frequency down
}

/**
 * PLL / MultiSynth frequency calculation (integer-N mode)
 *
 * The Si5351 frequency chain for CLK2:
 *   fout = fxtal * (msa + msb/msc) / d     [general form]
 *   fout = fxtal / d * d = fxtal            [integer-N: VCO = d * fout]
 *
 * Steps:
 *   1. Choose VCO multiplier d so that fvcoa = d * fout falls in ~600-900 MHz.
 *   2. Compute msa (integer) and msb (fractional numerator) of fvcoa / fxtal.
 *   3. Encode msa/msb into MSP1/MSP2 register format (Si5351 AN619).
 *   4. Write PLLB registers (34-41) with the VCO parameters.
 *   5. Write MS2 registers (58-65) with the output divider d (integer mode).
 *   6. Write CLK2 control register (18).
 *   7. Reset PLLB only when d changes (msa changes), to avoid phase glitches.
 */
void SI5351::setFreq(uint32_t fout, uint8_t clk) {
  // ── step 1: output R-divider for sub-500 kHz frequencies ──────────────────
  // rdiv encodes R = 2^rdiv in bits 22-20 of MS2 parameter register.
  // For fout < 500 kHz use R=128 (rdiv=7): scale fout up so PLL math works,
  // the hardware divides the output back down by 128.
  uint8_t rdiv = 0;
  if (fout < 500000) { rdiv = 7; fout *= 128; }

  // ── step 2: choose VCO multiplier d ───────────────────────────────────────
  // Target: fvcoa = d * fout in the Si5351 PLL lock range (~600-900 MHz).
  // d ≈ 16 * fxtal / fout keeps fvcoa near 400 MHz for HF; use 34× for
  // VHF (>30 MHz) where a smaller d is needed to stay in range.
  uint16_t d = (16 * fxtal) / fout;
  if (fout > 30000000) d = (34 * fxtal) / fout;
  // Nudge d down by 1 if fout straddles a division boundary (avoids aliasing
  // where rounding would place fvcoa outside the PLL lock range).
  if ((d * (fout - 5000) / fxtal) != (d * (fout + 5000) / fxtal)) d--;

  // ── step 3: compute VCO parameters ────────────────────────────────────────
  uint32_t fvcoa = d * fout;                          // VCO frequency (Hz)
  uint8_t  msa   = fvcoa / fxtal;                     // integer part of fvcoa/fxtal
  // Fractional numerator: msb/msc = (fvcoa % fxtal) / fxtal
  // Scaled by 128 to fit Si5351 register encoding (AN619 eq. 26-27).
  uint32_t msb   = ((uint64_t)(fvcoa % fxtal) * _MSC * 128) / fxtal;

  // ── step 4: encode PLLB register parameters (AN619 eq. 26-27) ─────────────
  // MSP1 = 128 * msa + floor(128 * msb / _MSC) - 512
  // MSP2 = 128 * msb - _MSC * floor(128 * msb / _MSC)
  uint32_t msp1 = 128 * msa + 128 * msb / _MSC - 512;
  uint32_t msp2 = 128 * msb - 128 * msb / _MSC * _MSC;

  // PLLB parameter registers (34-41): [MSC_hi, MSC_lo, MSP1_hi, MSP1_mid,
  //   MSP1_lo, (MSC_hi<<4)|MSP2_hi, MSP2_mid, MSP2_lo]
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
  sendRegisterBulk(34, pll_regs, 8);  // PLLB at regs 34-41 (all CLKn use PLLB per control reg)

  // ── step 5: encode MSn output divider in integer mode ─────────────────────
  // In integer mode MSP2=0, MSC=1, MSP1 = 128*d - 512 (AN619 eq. 24).
  // rdiv is placed in bits 22-20 of the first MS parameter word.
  // MS register base: 42 + clk*8  (CLK0→42, CLK1→50, CLK2→58)
  msp1 = (128 * d - 512) | (((uint32_t)rdiv) << 20);
  uint8_t ms_regs[8] = {0, 1, (uint8_t)(msp1 >> 16), (uint8_t)(msp1 >> 8), (uint8_t)(msp1), 0, 0, 0};
  sendRegisterBulk(42 + clk * 8, ms_regs, 8);

  // ── step 6: CLKn control register ─────────────────────────────────────────
  // Reg 16+clk = 0x6C | _drv:
  //   bit 7 = 0: CLKn powered on
  //   bit 6 = 1: integer mode
  //   bit 5 = 1: PLLB as source
  //   bit 4 = 0: not inverted
  //   bits 3-2 = 11: MultiSynth N as clock source
  //   bits 1-0 = drive strength (from driveStrength())
  sendRegister(16 + clk, 0x6C | _drv);

  // ── step 7: reset PLLB only when the VCO multiplier d changes ─────────────
  // Changing d means fvcoa changes, so the PLL must relock. When d is stable
  // (tone changes within the same WSPR band), msa stays constant and no reset
  // is needed, avoiding phase discontinuities between symbols.
  // Reg 177: bit 7 resets PLLA, bit 5 resets PLLB.
  if (iqmsa != (int16_t)msa) { iqmsa = msa; sendRegister(177, 0xA0); }

  // Cache state for next call
  _fout = fout; _div = d; _msa128min512 = msa * 128 - 512; _msb128 = msb;
}

void SI5351::outputEnable(uint8_t clk, uint8_t enable) {
  // Reg 3 is the output enable control: a 0 bit enables the corresponding clock.
  if (enable) sendRegister(3, ~(1 << clk));  // clear clk bit → enable that output only
  else        sendRegister(3, 0xFF);          // all bits set → all outputs disabled
}

void SI5351::driveStrength(uint8_t clk, uint8_t strength) {
  // Cache bits 1-0 (drive current: 0=2mA, 1=4mA, 2=6mA, 3=8mA).
  // setFreq() applies _drv to the CLK control register on each call, so
  // caching here is sufficient — no need to write the register immediately.
  _drv = strength & 0x03;
}
