/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * Based on Based on Adafruit MAX31865 library:
 *
 * This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865
 * Designed specifically to work with the Adafruit RTD Sensor
 * https://www.adafruit.com/products/3328
 *
 * This sensor uses SPI to communicate, 4 pins are required to interface.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * Modifications by JoAnn Manges (@GadgetAngel)
 * Copyright (c) 2020, JoAnn Manges
 * All rights reserved.
 */

<<<<<<< HEAD
// Useful for RTD debugging.
//#define MAX31865_DEBUG
//#define MAX31865_DEBUG_SPI

=======
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
#include "../inc/MarlinConfig.h"

#if HAS_MAX31865 && !USE_ADAFRUIT_MAX31865

<<<<<<< HEAD
//#include <SoftwareSPI.h> // TODO: switch to SPIclass/SoftSPI
#include "MAX31865.h"

// The maximum speed the MAX31865 can do is 5 MHz
SPISettings MAX31865::spiConfig = SPISettings(
  #if defined(TARGET_LPC1768)
    SPI_QUARTER_SPEED
  #elif defined(ARDUINO_ARCH_STM32)
    SPI_CLOCK_DIV4
  #else
    500000
  #endif
  , MSBFIRST
  , SPI_MODE_1 // CPOL0 CPHA1
);

#ifndef LARGE_PINMAP
=======
#include "MAX31865.h"

#ifndef MAX31865_MIN_SAMPLING_TIME_MSEC
  #define MAX31865_MIN_SAMPLING_TIME_MSEC 0
#endif

#ifdef TARGET_LPC1768
  #include <SoftwareSPI.h>
#endif

#define DEBUG_OUT ENABLED(DEBUG_MAX31865)
#include "../core/debug_out.h"

// The maximum speed the MAX31865 can do is 5 MHz
SPISettings MAX31865::spiConfig = SPISettings(
  TERN(TARGET_LPC1768, SPI_QUARTER_SPEED, TERN(ARDUINO_ARCH_STM32, SPI_CLOCK_DIV4, 500000)),
  MSBFIRST,
  SPI_MODE1 // CPOL0 CPHA1
);

#if DISABLED(LARGE_PINMAP)
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5

  /**
   * Create the interface object using software (bitbang) SPI for PIN values
   * less than or equal to 127.
   *
   * @param spi_cs    the SPI CS pin to use
   * @param spi_mosi  the SPI MOSI pin to use
   * @param spi_miso  the SPI MISO pin to use
   * @param spi_clk   the SPI clock pin to use
  */
  MAX31865::MAX31865(int8_t spi_cs, int8_t spi_mosi, int8_t spi_miso, int8_t spi_clk) {
<<<<<<< HEAD
    _cs = spi_cs;
    _mosi = spi_mosi;
    _miso = spi_miso;
    _sclk = spi_clk;
=======
    cselPin = spi_cs;
    mosiPin = spi_mosi;
    misoPin = spi_miso;
    sclkPin = spi_clk;
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
  }

  /**
   * Create the interface object using hardware SPI for PIN for PIN values less
   * than or equal to 127.
   *
   * @param spi_cs  the SPI CS pin to use along with the default SPI device
   */
  MAX31865::MAX31865(int8_t spi_cs) {
<<<<<<< HEAD
    _cs = spi_cs;
    _sclk = _miso = _mosi = -1;
  }

#else
=======
    cselPin = spi_cs;
    sclkPin = misoPin = mosiPin = -1;
  }

#else // LARGE_PINMAP
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5

  /**
   * Create the interface object using software (bitbang) SPI for PIN values
   * which are larger than 127. If you have PIN values less than or equal to
   * 127 use the other call for SW SPI.
   *
   * @param spi_cs       the SPI CS pin to use
   * @param spi_mosi     the SPI MOSI pin to use
   * @param spi_miso     the SPI MISO pin to use
   * @param spi_clk      the SPI clock pin to use
   * @param pin_mapping  set to 1 for positive pin values
   */
<<<<<<< HEAD
  MAX31865::MAX31865(uint32_t spi_cs, uint32_t spi_mosi,
                     uint32_t spi_miso, uint32_t spi_clk,
                     uint8_t pin_mapping) {
    _cs = spi_cs;
    _mosi = spi_mosi;
    _miso = spi_miso;
    _sclk = spi_clk;
=======
  MAX31865::MAX31865(uint32_t spi_cs, uint32_t spi_mosi, uint32_t spi_miso, uint32_t spi_clk, uint8_t pin_mapping) {
    cselPin = spi_cs;
    mosiPin = spi_mosi;
    misoPin = spi_miso;
    sclkPin = spi_clk;
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
  }

  /**
   * Create the interface object using hardware SPI for PIN values which are
   * larger than 127. If you have PIN values less than or equal to 127 use
   * the other call for HW SPI.
   *
   * @param spi_cs       the SPI CS pin to use along with the default SPI device
   * @param pin_mapping  set to 1 for positive pin values
   */
  MAX31865::MAX31865(uint32_t spi_cs, uint8_t pin_mapping) {
<<<<<<< HEAD
    _cs = spi_cs;
    _sclk = _miso = _mosi = -1UL;  //-1UL or 0xFFFFFFFF or 4294967295
=======
    cselPin = spi_cs;
    sclkPin = misoPin = mosiPin = -1UL;  //-1UL or 0xFFFFFFFF or 4294967295
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
  }

#endif // LARGE_PINMAP

<<<<<<< HEAD

=======
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
/**
 *
 * Instance & Class methods
 *
 */

<<<<<<< HEAD

=======
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
/**
 * Initialize the SPI interface and set the number of RTD wires used
 *
 * @param wires  The number of wires in enum format. Can be MAX31865_2WIRE, MAX31865_3WIRE, or MAX31865_4WIRE.
 * @param zero   The resistance of the RTD at 0 degC, in ohms.
 * @param ref    The resistance of the reference resistor, in ohms.
<<<<<<< HEAD
 */
void MAX31865::begin(max31865_numwires_t wires, float zero, float ref) {
  Rzero = zero;
  Rref = ref;

  OUT_WRITE(_cs, HIGH);

  if (_sclk != TERN(LARGE_PINMAP, -1UL, -1)) {
    // Define pin modes for Software SPI
    #ifdef MAX31865_DEBUG
      SERIAL_ECHOLN("Initializing MAX31865 Software SPI");
    #endif

    OUT_WRITE(_sclk, LOW);
    SET_OUTPUT(_mosi);
    SET_INPUT(_miso);
  }
  else {
    // Start and configure hardware SPI
    #ifdef MAX31865_DEBUG
      SERIAL_ECHOLN("Initializing MAX31865 Hardware SPI");
    #endif

    SPI.begin();
  }

  setWires(wires);
  enableBias(false);
  autoConvert(false);
  clearFault();

  #ifdef MAX31865_DEBUG_SPI
    #ifndef LARGE_PINMAP
      SERIAL_ECHOLNPGM(
        "Regular begin call with _cs: ", _cs,
        " _miso: ", _miso,
        " _sclk: ", _sclk,
        " _mosi: ", _mosi
      );
    #else
      SERIAL_ECHOLNPGM(
        "LARGE_PINMAP begin call with _cs: ", _cs,
        " _miso: ", _miso,
        " _sclk: ", _sclk,
        " _mosi: ", _mosi
      );
    #endif // LARGE_PINMAP

    SERIAL_ECHOLNPGM("config: ", readRegister8(MAX31856_CONFIG_REG));
    SERIAL_EOL();
  #endif // MAX31865_DEBUG_SPI
}

/**
 * Read the raw 8-bit FAULTSTAT register
 *
 * @return The raw unsigned 8-bit FAULT status register
 */
uint8_t MAX31865::readFault() {
  return readRegister8(MAX31856_FAULTSTAT_REG);
}

/**
 * Clear all faults in FAULTSTAT.
 */
void MAX31865::clearFault() {
  setConfig(MAX31856_CONFIG_FAULTSTAT, 1);
}

/**
 * Whether we want to have continuous conversions (50/60 Hz)
 *
 * @param b  If true, auto conversion is enabled
 */
void MAX31865::autoConvert(bool b) {
  setConfig(MAX31856_CONFIG_MODEAUTO, b);
}

/**
 * Whether we want filter out 50Hz noise or 60Hz noise
 *
 * @param b  If true, 50Hz noise is filtered, else 60Hz(default)
 */
void MAX31865::enable50HzFilter(bool b) {
  setConfig(MAX31856_CONFIG_FILT50HZ, b);
=======
 * @param wire   The resistance of the wire connecting the sensor to the RTD, in ohms.
 */
void MAX31865::begin(max31865_numwires_t wires, float zero_res, float ref_res, float wire_res) {
  zeroRes = zero_res;
  refRes = ref_res;
  wireRes = wire_res;

  pinMode(cselPin, OUTPUT);
  digitalWrite(cselPin, HIGH);

  if (sclkPin != TERN(LARGE_PINMAP, -1UL, 255))
    softSpiBegin(SPI_QUARTER_SPEED); // Define pin modes for Software SPI
  else {
    DEBUG_ECHOLNPGM("Initializing MAX31865 Hardware SPI");
    SPI.begin();    // Start and configure hardware SPI
  }

  initFixedFlags(wires);

  clearFault(); // also initializes flags

  #if DISABLED(MAX31865_USE_AUTO_MODE) // make a proper first 1 shot read to initialize _lastRead

    enableBias();
    DELAY_US(11500);
    oneShot();
    DELAY_US(65000);
    uint16_t rtd = readRegister16(MAX31865_RTDMSB_REG);

    if (rtd & 1) {
      lastRead = 0xFFFF; // some invalid value
      lastFault = readRegister8(MAX31865_FAULTSTAT_REG);
      clearFault(); // also clears the bias voltage flag, so no further action is required

      DEBUG_ECHOLNPGM("MAX31865 read fault: ", rtd);
    }
    else {
      DEBUG_ECHOLNPGM("RTD MSB:", (rtd >> 8), "  RTD LSB:", (rtd & 0x00FF));

      resetFlags();

      lastRead = rtd;
      nextEvent = SETUP_BIAS_VOLTAGE;
      millis_t now = millis();
      nextEventStamp = now + MAX31865_MIN_SAMPLING_TIME_MSEC;

      TERN_(MAX31865_USE_READ_ERROR_DETECTION, lastReadStamp = now);
    }

  #endif // !MAX31865_USE_AUTO_MODE

  DEBUG_ECHOLNPGM(
    TERN(LARGE_PINMAP, "LARGE_PINMAP", "Regular")
    " begin call with cselPin: ", cselPin,
    " misoPin: ", misoPin,
    " sclkPin: ", sclkPin,
    " mosiPin: ", mosiPin,
    " config: ", readRegister8(MAX31865_CONFIG_REG)
  );
}

/**
 * Return and clear the last fault value
 *
 * @return The raw unsigned 8-bit FAULT status register or spike fault
 */
uint8_t MAX31865::readFault() {
  uint8_t r = lastFault;
  lastFault = 0;
  return r;
}

/**
 * Clear last fault
 */
void MAX31865::clearFault() {
  setConfig(MAX31865_CONFIG_FAULTSTAT, 1);
}

/**
 * Reset flags
 */
void MAX31865::resetFlags() {
  writeRegister8(MAX31865_CONFIG_REG, stdFlags);
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
}

/**
 * Enable the bias voltage on the RTD sensor
<<<<<<< HEAD
 *
 * @param b  If true bias is enabled, else disabled
 */
void MAX31865::enableBias(bool b) {
  setConfig(MAX31856_CONFIG_BIAS, b);

  // From the datasheet:
  // Note that if VBIAS is off (to reduce supply current between conversions), any filter
  // capacitors at the RTDIN inputs need to charge before an accurate conversion can be
  // performed. Therefore, enable VBIAS and wait at least 10.5 time constants of the input
  // RC network plus an additional 1ms before initiating the conversion.
  if (b)
    DELAY_US(11500); //11.5ms
=======
 */
void MAX31865::enableBias() {
  setConfig(MAX31865_CONFIG_BIAS, 1);
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
}

/**
 * Start a one-shot temperature reading.
 */
void MAX31865::oneShot() {
<<<<<<< HEAD
  setConfig(MAX31856_CONFIG_1SHOT, 1);

  // From the datasheet:
  // Note that a single conversion requires approximately 52ms in 60Hz filter
  // mode or 62.5ms in 50Hz filter mode to complete. 1-Shot is a self-clearing bit.
  // TODO: switch this out depending on the filter mode.
  DELAY_US(65000); // 65ms
}

/**
 * How many wires we have in our RTD setup, can be MAX31865_2WIRE,
 * MAX31865_3WIRE, or MAX31865_4WIRE
 *
 * @param wires The number of wires in enum format
 */
void MAX31865::setWires(max31865_numwires_t wires) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (wires == MAX31865_3WIRE)
    t |= MAX31856_CONFIG_3WIRE;
  else // 2 or 4 wire
    t &= ~MAX31856_CONFIG_3WIRE;
  writeRegister8(MAX31856_CONFIG_REG, t);
=======
  setConfig(MAX31865_CONFIG_1SHOT | MAX31865_CONFIG_BIAS, 1);
}

/**
 * Initialize standard flags with flags that will not change during operation (Hz, polling mode and no. of wires)
 *
 * @param wires The number of wires in enum format
 */
void MAX31865::initFixedFlags(max31865_numwires_t wires) {

  // set config-defined flags (same for all sensors)
  stdFlags = TERN(MAX31865_50HZ_FILTER, MAX31865_CONFIG_FILT50HZ, MAX31865_CONFIG_FILT60HZ) |
              TERN(MAX31865_USE_AUTO_MODE, MAX31865_CONFIG_MODEAUTO | MAX31865_CONFIG_BIAS, MAX31865_CONFIG_MODEOFF);

  if (wires == MAX31865_3WIRE)
    stdFlags |= MAX31865_CONFIG_3WIRE;
  else  // 2 or 4 wire
    stdFlags &= ~MAX31865_CONFIG_3WIRE;
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
}

/**
 * Read the raw 16-bit value from the RTD_REG in one shot mode. This will include
 * the fault bit, D0.
 *
 * @return The raw unsigned 16-bit register value with ERROR bit attached, NOT temperature!
 */
uint16_t MAX31865::readRaw() {
<<<<<<< HEAD
  clearFault();
  enableBias(true);

  oneShot();
  uint16_t rtd = readRegister16(MAX31856_RTDMSB_REG);

  #ifdef MAX31865_DEBUG
    SERIAL_ECHOLNPGM("RTD MSB:", (rtd >> 8), "  RTD LSB:", (rtd & 0x00FF));
  #endif

  // Disable the bias to lower power dissipation between reads.
  // If the ref resistor heats up, the temperature reading will be skewed.
  enableBias(false);

  return rtd;
=======

  #if ENABLED(MAX31865_USE_AUTO_MODE)

    const uint16_t rtd = readRegister16(MAX31865_RTDMSB_REG);
    DEBUG_ECHOLNPGM("MAX31865 RTD MSB:", (rtd >> 8), " LSB:", (rtd & 0x00FF));

    if (rtd & 1) {
      lastFault = readRegister8(MAX31865_FAULTSTAT_REG);
      lastRead |= 1;
      clearFault(); // also clears the bias voltage flag, so no further action is required
      DEBUG_ECHOLNPGM("MAX31865 read fault: ", rtd);
    }
    #if ENABLED(MAX31865_USE_READ_ERROR_DETECTION)
      else if (ABS(lastRead - rtd) > 500 && PENDING(millis(), lastReadStamp + 1000)) { // if two readings within a second differ too much (~20°C), consider it a read error.
        lastFault = 0x01;
        lastRead |= 1;
        DEBUG_ECHOLNPGM("MAX31865 read error: ", rtd);
      }
    #endif
    else {
      lastRead = rtd;
      TERN_(MAX31865_USE_READ_ERROR_DETECTION, lastReadStamp = millis());
    }

  #else

    if (PENDING(millis(), nextEventStamp)) {
      DEBUG_ECHOLNPGM("MAX31865 waiting for event ", nextEvent);
      return lastRead;
    }

    switch (nextEvent) {
      case SETUP_BIAS_VOLTAGE:
        enableBias();
        nextEventStamp = millis() + 11; // wait at least 11msec before enabling 1shot
        nextEvent = SETUP_1_SHOT_MODE;
        DEBUG_ECHOLN("MAX31865 bias voltage enabled");
        break;

      case SETUP_1_SHOT_MODE:
        oneShot();
        nextEventStamp = millis() + 65; // wait at least 65msec before reading RTD register
        nextEvent = READ_RTD_REG;
        DEBUG_ECHOLN("MAX31865 1 shot mode enabled");
        break;

      case READ_RTD_REG: {
        const uint16_t rtd = readRegister16(MAX31865_RTDMSB_REG);
        DEBUG_ECHOLNPGM("MAX31865 RTD MSB:", (rtd >> 8), " LSB:", (rtd & 0x00FF));

        if (rtd & 1) {
          lastFault = readRegister8(MAX31865_FAULTSTAT_REG);
          lastRead |= 1;
          clearFault(); // also clears the bias voltage flag, so no further action is required
          DEBUG_ECHOLNPGM("MAX31865 read fault: ", rtd);
        }
        #if ENABLED(MAX31865_USE_READ_ERROR_DETECTION)
          else if (ABS(lastRead - rtd) > 500 && PENDING(millis(), lastReadStamp + 1000)) { // if two readings within a second differ too much (~20°C), consider it a read error.
            lastFault = 0x01;
            lastRead |= 1;
            DEBUG_ECHOLNPGM("MAX31865 read error: ", rtd);
          }
        #endif
        else {
          lastRead = rtd;
          TERN_(MAX31865_USE_READ_ERROR_DETECTION, lastReadStamp = millis());
        }

        if (!(rtd & 1))   // if clearFault() was not invoked, need to clear the bias voltage and 1-shot flags
          resetFlags();

        nextEvent = SETUP_BIAS_VOLTAGE;
        nextEventStamp = millis() + MAX31865_MIN_SAMPLING_TIME_MSEC; // next step should not occur within less than MAX31865_MIN_SAMPLING_TIME_MSEC from the last one
      } break;
    }

  #endif

  return lastRead;
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
}

/**
 * Calculate and return the resistance value of the connected RTD.
 *
<<<<<<< HEAD
 * @param  refResistor The value of the matching reference resistor, usually 430 or 4300
=======
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
 * @return             The raw RTD resistance value, NOT temperature!
 */
float MAX31865::readResistance() {
  // Strip the error bit (D0) and convert to a float ratio.
<<<<<<< HEAD
  // less precise method: (readRaw() * Rref) >> 16
  return (((readRaw() >> 1) / 32768.0f) * Rref);
=======
  // less precise method: (readRaw() * refRes) >> 16
  return ((readRaw() * RECIPROCAL(65536.0f)) * refRes - wireRes);
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
}

/**
 * Read the RTD and pass it to temperature(float) for calculation.
 *
 * @return  Temperature in C
 */
float MAX31865::temperature() {
  return temperature(readResistance());
}

/**
 * Given the 15-bit ADC value, calculate the resistance and pass it to temperature(float) for calculation.
 *
 * @return  Temperature in C
 */
<<<<<<< HEAD
float MAX31865::temperature(uint16_t adcVal) {
  return temperature(((adcVal) / 32768.0f) * Rref);
=======
float MAX31865::temperature(uint16_t adc_val) {
  return temperature(((adc_val) * RECIPROCAL(32768.0f)) * refRes - wireRes);
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
}

/**
 * Calculate the temperature in C from the RTD resistance.
 * Uses the technique outlined in this PDF:
 * http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
 *
<<<<<<< HEAD
 * @param    Rrtd  the resistance value in ohms
 * @return         the temperature in degC
 */
float MAX31865::temperature(float Rrtd) {
  float temp = (RTD_Z1 + sqrt(RTD_Z2 + (RTD_Z3 * Rrtd))) / RTD_Z4;
=======
 * @param    rtd_res  the resistance value in ohms
 * @return            the temperature in degC
 */
float MAX31865::temperature(float rtd_res) {
  float temp = (RTD_Z1 + sqrt(RTD_Z2 + (RTD_Z3 * rtd_res))) * RECIPROCAL(RTD_Z4);
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5

  // From the PDF...
  //
  // The previous equation is valid only for temperatures of 0°C and above.
  // The equation for RRTD(t) that defines negative temperature behavior is a
  // fourth-order polynomial (after expanding the third term) and is quite
  // impractical to solve for a single expression of temperature as a function
  // of resistance.
  //
  if (temp < 0) {
<<<<<<< HEAD
    Rrtd = (Rrtd / Rzero) * 100; // normalize to 100 ohm
    float rpoly = Rrtd;

    temp = -242.02 + (2.2228 * rpoly);
    rpoly *= Rrtd; // square
    temp += 2.5859e-3 * rpoly;
    rpoly *= Rrtd; // ^3
    temp -= 4.8260e-6 * rpoly;
    rpoly *= Rrtd; // ^4
    temp -= 2.8183e-8 * rpoly;
    rpoly *= Rrtd; // ^5
=======
    rtd_res = (rtd_res / zeroRes) * 100; // normalize to 100 ohm
    float rpoly = rtd_res;

    temp = -242.02 + (2.2228 * rpoly);
    rpoly *= rtd_res; // square
    temp += 2.5859e-3 * rpoly;
    rpoly *= rtd_res; // ^3
    temp -= 4.8260e-6 * rpoly;
    rpoly *= rtd_res; // ^4
    temp -= 2.8183e-8 * rpoly;
    rpoly *= rtd_res; // ^5
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
    temp += 1.5243e-10 * rpoly;
  }

  return temp;
}

//
// private:
//

<<<<<<< HEAD

=======
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
/**
 * Set a value in the configuration register.
 *
 * @param config  8-bit value for the config item
 * @param enable  whether to enable or disable the value
 */
void MAX31865::setConfig(uint8_t config, bool enable) {
<<<<<<< HEAD
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (enable)
    t |= config;
  else
    t &= ~config; // disable
  writeRegister8(MAX31856_CONFIG_REG, t);
=======
  uint8_t t = stdFlags;
  if (enable) t |= config; else t &= ~config;
  writeRegister8(MAX31865_CONFIG_REG, t);
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
}

/**
 * Read a single byte from the specified register address.
 *
 * @param  addr  the register address
 * @return       the register contents
 */
uint8_t MAX31865::readRegister8(uint8_t addr) {
  uint8_t ret = 0;
  readRegisterN(addr, &ret, 1);
<<<<<<< HEAD

=======
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
  return ret;
}

/**
 * Read two bytes: 1 from the specified register address, and 1 from the next address.
 *
 * @param  addr  the first register address
 * @return       both register contents as a single 16-bit int
 */
uint16_t MAX31865::readRegister16(uint8_t addr) {
<<<<<<< HEAD
  uint8_t buffer[2] = {0, 0};
  readRegisterN(addr, buffer, 2);

  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |= buffer[1];

  return ret;
=======
  uint8_t buffer[2] = { 0 };
  readRegisterN(addr, buffer, 2);
  return uint16_t(buffer[0]) << 8 | buffer[1];
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
}

/**
 * Read +n+ bytes from a specified address into +buffer+. Set D7 to 0 to specify a read.
 *
 * @param addr    the first register address
 * @param buffer  storage for the read bytes
 * @param n       the number of bytes to read
 */
void MAX31865::readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n) {
  addr &= 0x7F; // make sure top bit is not set
<<<<<<< HEAD
  if (_sclk == TERN(LARGE_PINMAP, -1UL, -1))
    SPI.beginTransaction(spiConfig);
  else
    WRITE(_sclk, LOW);

  WRITE(_cs, LOW);
  spixfer(addr);

  while (n--) {
    buffer[0] = spixfer(0xFF);
    #ifdef MAX31865_DEBUG_SPI
      SERIAL_ECHOLNPGM("buffer read ", n, " data: ", buffer[0]);
    #endif
    buffer++;
  }

  if (_sclk == TERN(LARGE_PINMAP, -1UL, -1))
    SPI.endTransaction();

  WRITE(_cs, HIGH);
=======
  if (sclkPin == TERN(LARGE_PINMAP, -1UL, 255))
    SPI.beginTransaction(spiConfig);
  else
    digitalWrite(sclkPin, LOW);

  digitalWrite(cselPin, LOW);

  #ifdef TARGET_LPC1768
    DELAY_CYCLES(spiSpeed);
  #endif

  spiTransfer(addr);

  while (n--) {
    buffer[0] = spiTransfer(0xFF);
    buffer++;
  }

  if (sclkPin == TERN(LARGE_PINMAP, -1UL, 255))
    SPI.endTransaction();

  digitalWrite(cselPin, HIGH);
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
}

/**
 * Write an 8-bit value to a register. Set D7 to 1 to specify a write.
 *
 * @param addr  the address to write to
 * @param data  the data to write
 */
void MAX31865::writeRegister8(uint8_t addr, uint8_t data) {
<<<<<<< HEAD
  if (_sclk == TERN(LARGE_PINMAP, -1UL, -1))
    SPI.beginTransaction(spiConfig);
  else
    WRITE(_sclk, LOW);

  WRITE(_cs, LOW);

  spixfer(addr | 0x80); // make sure top bit is set
  spixfer(data);

  if (_sclk == TERN(LARGE_PINMAP, -1UL, -1))
    SPI.endTransaction();

  WRITE(_cs, HIGH);
=======
  if (sclkPin == TERN(LARGE_PINMAP, -1UL, 255))
    SPI.beginTransaction(spiConfig);
  else
    digitalWrite(sclkPin, LOW);

  digitalWrite(cselPin, LOW);

  #ifdef TARGET_LPC1768
    DELAY_CYCLES(spiSpeed);
  #endif

  spiTransfer(addr | 0x80); // make sure top bit is set
  spiTransfer(data);

  if (sclkPin == TERN(LARGE_PINMAP, -1UL, 255))
    SPI.endTransaction();

  digitalWrite(cselPin, HIGH);
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
}

/**
 * Transfer SPI data +x+ and read the response. From the datasheet...
 * Input data (SDI) is latched on the internal strobe edge and output data (SDO) is
 * shifted out on the shift edge. There is one clock for each bit transferred.
 * Address and data bits are transferred in groups of eight, MSB first.
 *
 * @param  x  an 8-bit chunk of data to write
 * @return    the 8-bit response
 */
<<<<<<< HEAD
uint8_t MAX31865::spixfer(uint8_t x) {
  if (_sclk == TERN(LARGE_PINMAP, -1UL, -1))
    return SPI.transfer(x);

  uint8_t reply = 0;
  for (int i = 7; i >= 0; i--) {
    reply <<= 1;
    WRITE(_sclk, HIGH);
    WRITE(_mosi, x & (1 << i));
    WRITE(_sclk, LOW);
    if (READ(_miso))
      reply |= 1;
  }

  return reply;
=======
uint8_t MAX31865::spiTransfer(uint8_t x) {

  if (sclkPin == TERN(LARGE_PINMAP, -1UL, 255))
    return SPI.transfer(x);

  #ifdef TARGET_LPC1768

    return swSpiTransfer(x, spiSpeed, sclkPin, misoPin, mosiPin);

  #else

    uint8_t reply = 0;
    for (int i = 7; i >= 0; i--) {
      digitalWrite(sclkPin, HIGH);       DELAY_NS_VAR(spiDelay);
      reply <<= 1;
      digitalWrite(mosiPin, x & _BV(i)); DELAY_NS_VAR(spiDelay);
      if (digitalRead(misoPin)) reply |= 1;
      digitalWrite(sclkPin, LOW);        DELAY_NS_VAR(spiDelay);
    }
    return reply;

  #endif
}

void MAX31865::softSpiBegin(const uint8_t spi_speed) {
  DEBUG_ECHOLNPGM("Initializing MAX31865 Software SPI");

  #ifdef TARGET_LPC1768
    swSpiBegin(sclkPin, misoPin, mosiPin);
    spiSpeed = swSpiInit(spi_speed, sclkPin, mosiPin);
  #else
    spiDelay = (100UL << spi_speed) / 3; // Calculate delay in ns. Top speed is ~10MHz, or 100ns delay between bits.
    pinMode(sclkPin, OUTPUT);
    digitalWrite(sclkPin, LOW);
    pinMode(mosiPin, OUTPUT);
    pinMode(misoPin, INPUT);
  #endif
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
}

#endif // HAS_MAX31865 && !USE_ADAFRUIT_MAX31865
