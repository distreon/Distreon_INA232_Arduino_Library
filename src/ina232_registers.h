#ifndef ina232_registers_h
#define ina232_registers_h

#include <stdint.h>

namespace INA232Reg {
// Registers are all 16 bits wide

/*------CONFIG REGISTER------*/
// Configuration Register. Masked fields.
namespace Config {
    const uint8_t addr = 0x00;
    const uint16_t defaultVal = 0x4127;

    // System Reset. R/W. Set this bit to '1' to generate a system reset that is the same as power-on reset.
    // Resets all registers to default values and then self-clears.
    const uint16_t rstBit = 0b1 << 15;

    // Shunt ADC Range. R/W. Enables the selection of the shunt full scale input across IN+ and IN–.
    const uint16_t adcRangeMask = 0b1 << 12;
    enum class ADCRange : uint16_t {
        pm81_92mV = 0b0 << 12, // ±81.92 mV (default)
        pm20_48mV = 0b1 << 12, // ±20.48 mV
    };

    // Average sample count. R/W. Sets the number of ADC conversion results to be averaged. The read-
    // back registers are updated after averaging is completed.
    const uint16_t avgMask = 0b111 << 9;
    enum class Avg : uint16_t {
        s1 = 0b000 << 9,    // 1 sample averaged (default)
        s4 = 0b001 << 9,    // 4 samples averaged
        s16 = 0b010 << 9,   // 16 samples averaged
        s64 = 0b011 << 9,   // 64 samples averaged
        s128 = 0b100 << 9,  // 128 samples averaged
        s256 = 0b101 << 9,  // 256 samples averaged
        s512 = 0b110 << 9,  // 512 samples averaged
        s1024 = 0b111 << 9, // 1024 samples averaged
    };

    // VBus Conversion Time. R/W. Sets the conversion time of the VBUS measurement.
    const uint16_t vBusCTMask = 0b111 << 6;
    enum class VBusCT : uint16_t {
        t140us = 0b000 << 6,  // 140us sample time
        t204us = 0b001 << 6,  // 204us sample time
        t332us = 0b010 << 6,  // 332us sample time
        t588us = 0b011 << 6,  // 588us sample time
        t1100us = 0b100 << 6, // 1100us sample time (default)
        t2116us = 0b101 << 6, // 2116us sample time
        t4156us = 0b110 << 6, // 4156us sample time
        t8244us = 0b111 << 6, // 8244us sample time
    };

    // VShunt Conversion Time. R/W. Sets the conversion time of the SHUNT measurement.
    const uint16_t vshCTMask = 0b111 << 3;
    enum class VShCT : uint16_t {
        t140us = 0b000 << 3,  // 140us sample time
        t204us = 0b001 << 3,  // 204us sample time
        t332us = 0b010 << 3,  // 332us sample time
        t588us = 0b011 << 3,  // 588us sample time
        t1100us = 0b100 << 3, // 1100us sample time (default)
        t2116us = 0b101 << 3, // 2116us sample time
        t4156us = 0b110 << 3, // 4156us sample time
        t8244us = 0b111 << 3, // 8244us sample time
    };

    // Operating Mode. R/W. Modes can be selected to operate the device either in
    // Shutdown mode, continuous mode or triggered mode.
    // The mode also allows user to select mux settings to set continuous or
    // triggered mode on bus voltage, shunt voltage measurement.
    const uint16_t modeMask = 0b111 << 0;
    enum class Mode : uint16_t {
        shutdown = 0b000 << 0,          // Shutdown
        shvTrigSingle = 0b001 << 0,     // Shunt Voltage triggered, single shot
        busvTrigSingle = 0b010 << 0,    // Bus Voltage triggered, single shot
        shvBusvTrigSingle = 0b011 << 0, // Shunt voltage and Bus voltage triggered, single shot
        shutdown2 = 0b100 << 0,         // Also shutdown
        contShV = 0b101 << 0,           // Continuous Shunt voltage
        contBusV = 0b110 << 0,          // Continuous Bus voltage
        contShvBusV = 0b111 << 0,       // Continuous Shunt and Bus voltage (default)
    };

} // namespace Config

/*------SHUNT VOLTAGE REGISTER------*/
// R. Stores current shunt voltage reading. Signed 16bit int using two's complement.
// If averaging is enabled, this register displays the averaged value.
namespace VShunt {
    const uint8_t addr = 0x01;
    const uint16_t defaultVal = 0x0000;
} // namespace VShunt

/*------VBUS VOLTAGE REGISTER------*/
// R. Stores the bus voltage reading. Bit 15 is technically reserved, as the value is
// always positive, but will read 0, allowing the register to be read as a 16bit int.
// If averaging is enabled, this register displays the averaged value.
namespace VBus {
    const uint8_t addr = 0x02;
    const uint16_t defaultVal = 0x0000;
} // namespace VBus

/*------POWER REGISTER------*/
// R. Stores the power in Watts by multiplying the decimal values of the Current
// Register with the decimal value of the Bus Voltage Register. Unsigned 16bit int.
// If averaging is enabled, this calculation is performed per sample, and this
// register displays the averaged value.
namespace Power {
    const uint8_t addr = 0x03;
    const uint16_t defaultVal = 0x0000;
} // namespace Power

/*------CURRENT REGISTER------*/
// R. Stores the result of multiplying the decimal value in the Shunt Voltage
// Register with the decimal value of the Calibration Register.
// If averaging is enabled, this register displays the averaged value.
namespace Current {
    const uint8_t addr = 0x04;
    const uint16_t defaultVal = 0x0000;
} // namespace Current

/*------SHUNT CALIBRATION REGISTER------*/
// R/W. Provides the device with the value of the shunt resistor that was present to create
// the measured differential voltage. It also sets the resolution of the Current Register.
// Programming this register sets the Current_LSB and the Power_LSB.
// Must be programmed to receive valid current and power results after
// initial power up or power cycle events.
namespace ShuntCal {
    const uint8_t addr = 0x05;
    const uint16_t defaultVal = 0x0000;
    const uint16_t mask = 0x7FFF;
} // namespace ShuntCal

/*------MASK ENABLE REGISTER------*/
// All single bit flags, some R, some R/W. All default to 0.
namespace MaskEn {
    const uint8_t addr = 0x06;
    const uint16_t defaultVal = 0x0000;

    // Shunt Over-limit. R/W. Setting this bit high configures the ALERT pin to be asserted if
    // the shunt voltage conversion result exceeds the value programmed in the LIMIT register.
    const uint16_t SOLBit = 0b1 << 15;

    // Shunt Under-limit. R/W. Setting this bit high configures the ALERT pin to be asserted if
    // the shunt voltage conversion result is below the value programmed in the LIMIT register.
    // Cannot be set if Shunt overlimit is set.
    const uint16_t SULBit = 0b1 << 14;

    // Bus Over-limit. R/W. Setting this bit high configures the ALERT pin to be asserted if
    // the bus voltage conversion result exceeds the value programmed in the LIMIT register.
    // Cannot be set if Shunt overlimit or Shunt underlimit is set.
    const uint16_t BOLBit = 0b1 << 13;

    // Bus Under-limit. R/W. Setting this bit high configures the ALERT pin to be asserted if the bus voltage
    // conversion result is below the value programmed in the LIMIT register.
    // Cannot be set if Shunt over limit, Shunt under limit or Bus over limit is set.
    const uint16_t BULBit = 0b1 << 12;

    // Power Over-limit. R/W. Setting this bit high configures the ALERT pin to be asserted if the power result
    // exceeds the value programmed in the LIMIT register.
    // Cannot be set if Shunt over limit, Shunt under limit, Bus over limit or Bus under
    // limit is set.
    const uint16_t POLBit = 0b1 << 11;

    // Conversion Ready. R/W. Setting this bit high configures the ALERT pin to be asserted when the
    // Conversion Ready Flag, Bit 3, is asserted indicating that the device is ready
    // for the next conversion.
    const uint16_t CNVRBit = 0b1 << 10;

    // MemError. R. Set on CRC or ECC error.
    const uint16_t MEMERRBit = 0b1 << 5;

    // Alert Function Flag. R. While only one Alert Function can be monitored at the
    // ALERT pin at a time, the Conversion Ready can also be enabled to assert the
    // ALERT pin. Reading the Alert Function Flag following an alert allows the user to
    // determine if the Alert Function was the source of the Alert.
    // When the Alert Latch Enable bit is set to Latch mode, the Alert Function Flag bit
    // clears only when the Mask/Enable Register is read. When the Alert Latch Enable
    // bit is set to Transparent mode, the Alert Function Flag bit is cleared following the
    // next conversion that does not result in an Alert condition.
    const uint16_t AFFBit = 0b1 << 4;

    // Conversion Ready Flag. R. Although the device can be read at any time, and the data from the last
    // conversion is available, the Conversion Ready Flag bit is provided to help
    // coordinate one-shot or triggered conversions.
    // The Conversion Ready Flag bit is set after all conversions, averaging, and
    // multiplications are complete.
    // Conversion Ready Flag bit clears under the following conditions:
    // 1.) Writing to the Configuration Register (except for Power-Down selection)
    // 2.) Reading the Mask/Enable Register
    const uint16_t CVRFBit = 0b1 << 3;

    // Math Over-flow. R. This bit is set to '1' if an arithmetic operation resulted in an overflow error. It
    // indicates that current and power data may be invalid.
    const uint16_t IVFBit = 0b1 << 2;

    // Alert Polarity. R/W. Sets the Alert pin polarity
    const uint16_t APOLMask = 0b1 << 1;
    enum class APOL : uint16_t {
        activeLow = 0b0 << 1,  // Active low - open drain (default)
        activeHigh = 0b1 << 1, // Active high
    };

    // Alert Latch Enable. R/W. When the Alert Latch Enable bit is set to Transparent mode (default), the Alert pin
    // and Alert Function Flag (AFF) bit resets to the idle states when the fault condition has been cleared.
    // When the Alert Latch Enable bit is set to Latch mode, the Alert pin and AFF bit
    // remains active following a fault until this register flag has been read.
    // This bit must be set to use the I2C Alert Response function.
    const uint16_t LENBit = 0b1 << 0;

} // namespace MaskEn

/*------ALERT LIMIT REGISTER------*/
// R/W. The Alert Limit Register contains the value used to compare to the
// register selected in the Mask/Enable Register to determine if a limit
// has been exceeded.
// A two's complement value must be used for the Shunt Over Voltage
// limit. Limit values entered should match the format of the targeted
// register.
namespace Limit {
    const uint8_t addr = 0x07;
    const uint16_t defaultVal = 0x0000;

} // namespace Limit

/*------MANUFACTURER ID REGISTER------*/
// R. Always reads "TI" in ASCII.
namespace ManID {
    const uint8_t addr = 0x3E;
    const uint16_t defaultVal = 0x5449;
} // namespace ManID
} // namespace INA232Reg

#endif