/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "max31865.h"  // NOLINT

namespace bfs {

void Max31865::Config(SPIClass *spi, const uint8_t cs) {
  spi_ = spi;
  cs_ = cs;
}

bool Max31865::Begin(const RtdWire wires, const float r0, const float resist) {
  pinMode(cs_, OUTPUT);
  /* Copy the RTD resistance at 0C */
  r0_ = r0;
  /* Copy the reference resistor resistance */
  ref_resistance_ = resist;
  /* Set the number of wires */
  if (wires == RTD_3WIRE) {
    if (!Enable3WireRtd()) {return false;}
  } else {
    if (!Disable3WireRtd()) {return false;}
  }
  /* Set a 50 Hz filter */
  if (!Select50HzFilter()) {return false;}
  /* Enable VBIAS */
  if (!EnableBiasVoltage()) {return false;}
  delay(10);
  /* Enable auto conversion */
  if (!EnableAutoConversion()) {return false;}
  /* Compute polynomial coefficients */
  Z1_ = -RTD_A_;
  Z2_ = RTD_A_ * RTD_A_ - 4.0f * RTD_B_;
  Z3_ = (4.0f * RTD_B_) / r0_;
  Z4_ = 2 * RTD_B_;
  return true;
}

bool Max31865::EnableBiasVoltage() {
  if (!ReadRegisters(CONFIG_ADDR_, 1, buf_)) {return false;}
  buf_[0] |= CONFIG_VBIAS_BIT_;
  return WriteRegister(CONFIG_ADDR_, buf_[0]);
}

bool Max31865::DisableBiasVoltage() {
  if (!ReadRegisters(CONFIG_ADDR_, 1, buf_)) {return false;}
  buf_[0] &= ~CONFIG_VBIAS_BIT_;
  return WriteRegister(CONFIG_ADDR_, buf_[0]);
}

bool Max31865::EnableAutoConversion() {
  if (!ReadRegisters(CONFIG_ADDR_, 1, buf_)) {return false;}
  buf_[0] |= CONFIG_CONVERSION_MODE_BIT_;
  if (!WriteRegister(CONFIG_ADDR_, buf_[0])) {return false;}
  delay(10);
  if (!ReadRegisters(CONFIG_ADDR_, 1, &buf_[1])) {return false;}
  if (buf_[0] != buf_[1]) {return false;}
  return true;
}

bool Max31865::DisableAutoConversion() {
  if (!ReadRegisters(CONFIG_ADDR_, 1, buf_)) {return false;}
  buf_[0] &= ~CONFIG_CONVERSION_MODE_BIT_;
  if (!WriteRegister(CONFIG_ADDR_, buf_[0])) {return false;}
  delay(10);
  if (!ReadRegisters(CONFIG_ADDR_, 1, &buf_[1])) {return false;}
  if (buf_[0] != buf_[1]) {return false;}
  return true;
}

bool Max31865::ConfigFilter(const Filter filter) {
  if (filter == FILTER_60HZ) {
    return Select60HzFilter();
  } else {
    return Select50HzFilter();
  }
}

bool Max31865::Read() {
  if (!ReadRtd()) {return false;}
  /* Measured resistance */
  r_ = ref_resistance_ * static_cast<float>(rtd_val_) / RTD_MAX_VAL_;
  /* Convert resistance to temperature */
  temp_c_ = (Z1_ + sqrtf(Z2_ + Z3_ * r_)) / Z4_;
  if (temp_c_ < -12.5f) {
    r_ /= r0_;
    r_ *= 100.0f;
    temp_c_ = -242.97f + 2.2838 * r_ + 1.4727e-3f * r_ * r_;
  }
  return true;
}

bool Max31865::GetFault(uint8_t * const fault) {
  if (!fault) {return false;}
  if (ReadFaultStatus()) {
    *fault = fault_status_;
    return true;
  } else {
    return false;
  }
}

bool Max31865::ReadRtd() {
  if (!ReadRegisters(RTD_MSB_ADDR_, 2, buf_)) {return false;}
  fault_ = buf_[1] & 0x01;
  rtd_val_ = (static_cast<uint16_t>(buf_[0]) << 8 | buf_[1]) >> 1;
  return true;
}

bool Max31865::ReadFaultStatus() {
  if (!ReadRegisters(FAULT_STATUS_ADDR_, 1, &fault_status_)) {return false;}
  return true;
}

bool Max31865::Enable3WireRtd() {
  if (!ReadRegisters(CONFIG_ADDR_, 1, buf_)) {return false;}
  buf_[0] |= CONFIG_3WIRE_RTD_BIT_;
  if (!WriteRegister(CONFIG_ADDR_, buf_[0])) {return false;}
  delay(10);
  if (!ReadRegisters(CONFIG_ADDR_, 1, &buf_[1])) {return false;}
  if (buf_[0] != buf_[1]) {return false;}
  return true;
}
bool Max31865::Disable3WireRtd() {
  if (!ReadRegisters(CONFIG_ADDR_, 1, buf_)) {return false;}
  buf_[0] &= ~CONFIG_3WIRE_RTD_BIT_;
  if (!WriteRegister(CONFIG_ADDR_, buf_[0])) {return false;}
  delay(10);
  if (!ReadRegisters(CONFIG_ADDR_, 1, &buf_[1])) {return false;}
  if (buf_[0] != buf_[1]) {return false;}
  return true;
}

bool Max31865::ClearFaultStatus() {
  if (!ReadRegisters(CONFIG_ADDR_, 1, buf_)) {return false;}
  buf_[0] |= CONFIG_FAULT_STATUS_BIT_;
  return WriteRegister(CONFIG_ADDR_, buf_[0]);
}

bool Max31865::Trigger1Shot() {
  if (!ReadRegisters(CONFIG_ADDR_, 1, buf_)) {return false;}
  buf_[0] |= CONFIG_1SHOT_BIT_;
  return WriteRegister(CONFIG_ADDR_, buf_[0]);
}

bool Max31865::Select50HzFilter() {
  if (!ReadRegisters(CONFIG_ADDR_, 1, buf_)) {return false;}
  buf_[0] |= CONFIG_FILTER_BIT_;
  if (!WriteRegister(CONFIG_ADDR_, buf_[0])) {return false;}
  delay(10);
  if (!ReadRegisters(CONFIG_ADDR_, 1, &buf_[1])) {return false;}
  if (buf_[0] != buf_[1]) {return false;}
  return true;}

bool Max31865::Select60HzFilter() {
  if (!ReadRegisters(CONFIG_ADDR_, 1, buf_)) {return false;}
  buf_[0] &= ~CONFIG_FILTER_BIT_;
  if (!WriteRegister(CONFIG_ADDR_, buf_[0])) {return false;}
  delay(10);
  if (!ReadRegisters(CONFIG_ADDR_, 1, &buf_[1])) {return false;}
  if (buf_[0] != buf_[1]) {return false;}
  return true;
}

bool Max31865::WriteRegister(const uint8_t addr, const uint8_t data) {
  spi_->beginTransaction(SPISettings(SPI_CLOCK_, MSBFIRST, SPI_MODE1));
  #if defined(TEENSYDUINO)
  digitalWriteFast(cs_, LOW);
  #else
  digitalWrite(cs_, LOW);
  #endif
  spi_->transfer(addr | WRITE_);
  spi_->transfer(data);
  #if defined(TEENSYDUINO)
  digitalWriteFast(cs_, HIGH);
  #else
  digitalWrite(cs_, HIGH);
  #endif
  spi_->endTransaction();
  return true;
}

bool Max31865::ReadRegisters(const uint8_t addr, const uint8_t count,
                             uint8_t * const data) {
  if ((!data) || (count == 0)) {return false;}
  spi_->beginTransaction(SPISettings(SPI_CLOCK_, MSBFIRST, SPI_MODE1));
  #if defined(TEENSYDUINO)
  digitalWriteFast(cs_, LOW);
  #else
  digitalWrite(cs_, LOW);
  #endif
  spi_->transfer(addr);
  spi_->transfer(data, count);
  #if defined(TEENSYDUINO)
  digitalWriteFast(cs_, HIGH);
  #else
  digitalWrite(cs_, HIGH);
  #endif
  spi_->endTransaction();
  return true;
}

}  // namespace bfs
