[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![Bolder Flight Systems Logo](img/logo-words_75.png) &nbsp; &nbsp; ![Arduino Logo](img/arduino_logo_75.png)

# Max31865
This library communicates with the [MAX31865](https://www.maximintegrated.com/en/products/interface/sensor-interface/MAX31865.html) RTD front-end. This library is compatible with Arduino and CMake build systems. This library supports single-shot and continuous measurement modes and is non-blocking for good performance and integration with other software.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Installation

## Arduino
Use the Arduino Library Manager to install this library or clone to your Arduino/libraries folder. This library is added as:

```C++
#include "max31865.h"
```

Example Arduino executables are located in: *examples/arduino/*. Teensy 3.x, 4.x, and LC devices are used for testing under Arduino and this library should be compatible with other Arduino devices.

## CMake
CMake is used to build this library, which is exported as a library target called *max31865*. This library is added as:

```C++
#include "max31865.h"
```

The library can be also be compiled stand-alone using the CMake idiom of creating a *build* directory and then, from within that directory issuing:

```
cmake .. -DMCU=MK66FX1M0
make
```

This will build the library and example executables called *max31865_example*. The example executable source files are located at *examples/cmake*. Notice that the *cmake* command includes a define specifying the microcontroller the code is being compiled for. This is required to correctly configure the code, CPU frequency, and compile/linker options. The available MCUs are:
   * MK20DX128
   * MK20DX256
   * MK64FX512
   * MK66FX1M0
   * MKL26Z64
   * IMXRT1062_T40
   * IMXRT1062_T41
   * IMXRT1062_MMOD

These are known to work with the same packages used in Teensy products. Also switching packages is known to work well, as long as it's only a package change.

The example targets create executables for communicating with the sensor using I2C or SPI communication, using the data ready interrupt, and using the wake on motion interrupt, respectively. Each target also has a *_hex*, for creating the hex file to upload to the microcontroller, and an *_upload* for using the [Teensy CLI Uploader](https://www.pjrc.com/teensy/loader_cli.html) to flash the Teensy. Please note that instructions for setting up your build environment can be found in our [build-tools repo](https://github.com/bolderflight/build-tools).

# Namespace
This library is within the namespace *bfs*.

# Max31865

## Methods

**Max31865()** Default constructor, requires calling the *Config* method to setup the SPI bus and chip select pin.

**Max31865(SPIClass &ast;spi, const uint8_t cs)** Creates a Max31865 object. This constructor is used for the SPI communication interface. A pointer to the SPI bus object is passed along with the chip select pin of the sensor. Any pin capable of digital I/O can be used as a chip select pin.

```C++
/* MAX31865 on SPI1 CS pin 29 */
bfs::Max31865 rtd(&SPI1, 29);
```

**void Config(SPIClass &ast;spi, const uint8_t cs)** This is required when using the default constructor and sets up the SPI bus and chip select pin.

**bool Begin(const RtdWire wires, const float r0, const float resist)** Initializes communication and configures the MAX31865. The communication bus is not initialized within this library and must be initialized seperately; this enhances compatibility with other sensors that may on the same bus. Takes an enum for the number of RTD sensor wires, the RTD resistance at 0C, and the reference resistor resistance. Returns true if communication is able to be established and the sensor successfully configured, otherwise false is returned.

The RTD wires are an enum:

| Enum | Description |
| --- | --- |
| RTD_4WIRE | 4 wire RTD sensor |
| RTD_3WIRE | 3 wire RTD sensor |
| RTD_2WIRE | 2 wire RTD sensor |

RTD resistance at 0C is typically, 100 Ohm, 500 Ohm, or 1000 Ohm.

Default configuration sets the filter at 50 Hz and enables continuous measurement mode.

```C++
/* Init SPI and MAX31865 */
SPI1.begin();
if (!rtd.Begin(bfs::Max31865::RTD_3WIRE, 100.0f, 402.0f)) {
  Serial.println("ERROR initializing RTD");
  while (1) {}
}
```

**bool ConfigFilter(const Filter filter)** Configures the filter, which can be set to 50 Hz or 60 Hz. The filter is selected with an enum:

| Enum | Description |
| --- | --- |
| FILTER_50HZ | 50 Hz filter |
| FILTER_60HZ | 60 Hz filter |

### Continous Measurement
Continuous measurement is enabled by default and provides the highest sampling rate. The downside of continuous measurement is that the bias voltage can lead to self heating and effect the measurements.

**bool DisableAutoConversion()** Disables continuous measurement. Returns true on success or false on failure.

**bool EnableAutoConversion()** Enables continuous measurement. Returns true on success or false on failure.

## Single Shot Measurement
Single shot measurement can be used to minimize self heating. In this case, the steps are:
1. Enable the voltage bias, then wait 10x the input time constant plus 1ms
2. Trigger the single shot measurement, then wait 65ms
3. Read the data and disable the voltage bias

**bool EnableBiasVoltage()** Enables the voltage bias. Returns true on success or false on failure.

**bool DisableBiasVoltage()** Disables the voltage bias. Returns true on success or false on failure.

**bool Trigger1Shot()** Triggers the single shot measurement.

### Reading the data

**bool Read()** Reads data from the sensor and stores the data in the object. Note, that the bool return value simply indicates whether the read was successful, not whether the data has been updated.

```C++
/* Read and print temperature */
if (rtd.Read()) {
  Serial.println(rtd.temp_c());
  delay(50);
}
```

**float temp_c()** Returns the most recent temperature measurement, C

**bool fault()** Returns whether a fault was encountered.

**bool GetFault(uint8_t * const fault)** Gets the fault code and passes it via a pointer. Returns true on successfully getting the fault code. The fault code meanings can be found in the datasheet based on the specific RTD probe type used.
