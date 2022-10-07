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

#ifndef MAX31865_SRC_MAX31865_H_  // NOLINT
#define MAX31865_SRC_MAX31865_H_

#if defined(ARDUINO)
#include <Arduino.h>
#include "SPI.h"
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif

namespace bfs {

class Max31865 {
 public:
  enum RtdWire {
    RTD_4WIRE,
    RTD_3WIRE,
    RTD_2WIRE
  };
  enum Filter {
    FILTER_50HZ,
    FILTER_60HZ
  };
  Max31865() {}
  Max31865(SPIClass *spi, const uint8_t cs) : spi_(spi), cs_(cs) {}
  void Config(SPIClass *spi, const uint8_t cs);
  bool Begin(const RtdWire wires, const float r0, const float resist);
  bool ConfigFilter(const Filter filter);
  bool EnableBiasVoltage();
  bool DisableBiasVoltage();
  bool EnableAutoConversion();
  bool DisableAutoConversion();
  bool Trigger1Shot();
  bool Read();
  inline float temp_c() const {return temp_c_;}
  inline bool fault() const {return fault_;}
  bool GetFault(uint8_t * const fault);

 private:
  /* Communication */
  SPIClass *spi_;
  uint8_t cs_;
  static constexpr int32_t SPI_CLOCK_ = 5000000;
  static constexpr uint8_t WRITE_ = 0x80;
  /* Data */
  bool fault_;
  uint8_t buf_[2];
  uint16_t rtd_val_;
  uint8_t fault_status_;
  float r0_;
  float ref_resistance_;
  float r_;
  float temp_c_;
  static constexpr float RTD_MAX_VAL_ = 32768.0f;
  static constexpr float RTD_A_ = 3.9083e-3;
  static constexpr float RTD_B_ = -5.775e-7;
  float Z1_, Z2_, Z3_, Z4_;
  /* Registers */
  static constexpr uint8_t CONFIG_ADDR_ = 0x00;
  static constexpr uint8_t RTD_MSB_ADDR_ = 0x01;
  static constexpr uint8_t RTD_LSB_ADDR_ = 0x02;
  static constexpr uint8_t HIGH_FAULT_THRESH_MSB_ADDR_ = 0x03;
  static constexpr uint8_t HIGH_FAULT_THRESH_LSB_ADDR_ = 0x04;
  static constexpr uint8_t LOW_FAULT_THRESH_MSB_ADDR_ = 0x05;
  static constexpr uint8_t LOW_FAULT_THRESH_LSB_ADDR_ = 0x06;
  static constexpr uint8_t FAULT_STATUS_ADDR_ = 0x07;
  /* Config Masks */
  static constexpr uint8_t CONFIG_VBIAS_BIT_ = 0x80;
  static constexpr uint8_t CONFIG_CONVERSION_MODE_BIT_ = 0x40;
  static constexpr uint8_t CONFIG_1SHOT_BIT_ = 0x20;
  static constexpr uint8_t CONFIG_3WIRE_RTD_BIT_ = 0x10;
  static constexpr uint8_t CONFIG_FAULT_DET_MASK_ = 0x0C;
  static constexpr uint8_t CONFIG_FAULT_STATUS_BIT_ = 0x02;
  static constexpr uint8_t CONFIG_FILTER_BIT_ = 0x01;
  /* Utility functions */
  bool ReadRtd();
  bool ReadFaultStatus();
  bool Enable3WireRtd();
  bool Disable3WireRtd();
  bool ClearFaultStatus();
  bool Select50HzFilter();
  bool Select60HzFilter();
  bool ReadRegisters(const uint8_t addr, const uint8_t count,
                     uint8_t * const data);
  bool WriteRegister(const uint8_t addr, const uint8_t data);
};

}  // namespace bfs

#endif  // MAX31865_SRC_MAX31865_H_ NOLINT
