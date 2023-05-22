#ifndef ina232_i2c_h
#define ina232_i2c_h

#include <Wire.h>
#include <ina232_registers.h>
#include <stdint.h>

class INA232 {
  public:
    // I2C address options. There is a different set of 4 addresses for the "B" series
    // of the chip ("A" is much more common)
    enum A0 {
        A0_A_GND = (0b1000000),
        A0_A_VSS = (0b1000001),
        A0_A_SDA = (0b1000010),
        A0_A_SCL = (0b1000011),
        A0_B_GND = (0b1001000),
        A0_B_VSS = (0b1001001),
        A0_B_SDA = (0b1001010),
        A0_B_SCL = (0b1001011),
    };

    typedef INA232_REG::CONFIG::AVG AVG;
    typedef INA232_REG::CONFIG::VBUSCT VBUSCT;
    typedef INA232_REG::CONFIG::VSHCT VSHCT;

    // Subset of the available modes - others are singleshot, and used with trigger*() instead
    enum class MODE : uint16_t {
        SHUTDOWN = (uint16_t)INA232_REG::CONFIG::MODE::SHUTDOWN,   // Shutdown
        CONT_SHUNT = (uint16_t)INA232_REG::CONFIG::MODE::CONT_SHV, // Continuous Shunt voltage measurements
        CONT_BUSV = (uint16_t)INA232_REG::CONFIG::MODE::CONT_BUSV, // Continuous Bus voltage measurements
        CONT_SHUNT_BUSV = (uint16_t)
            INA232_REG::CONFIG::MODE::CONT_SHV_BUSV, // Continuous Shunt and Bus voltage measurements (default)
    };

    typedef INA232_REG::CONFIG::ADCRANGE ADCRANGE;

    INA232(A0 addr);

    // Attempts to start communication with the INA232 over the I2C port. If any configuration
    // has been set before this, changes will be written to the device.
    void begin(TwoWire *wirePort = &Wire);

    // Attempts to reset the INA232 device via I2C. If successful, will return all configured registers
    // to their default
    void reset();

    // Sets the measurement range of the current shunt ADC. Voltages greater than this
    // will clip to the maximum measurement range.
    void setRange(ADCRANGE range);

    // Set number of samples to average before storing result to be read
    // in the current, vbus and power registers. Results are averaged in groups, not
    // a moving window. Power is calculated per sample, then averaged, rather than
    // being calculated on the average current and vbus values later.
    //
    // If called before begin(), any change will be applied when begin() is called.
    void setAveraging(AVG averageCount);

    // Sets the time taken to do each VBus measurement. Longer times result in
    // cleaner measurement.
    void setVBusConversionTime(VBUSCT conversionTime);

    // Sets the time taken to do each VBus measurement. Longer times result in
    // cleaner measurement.
    void setShuntConversionTime(VSHCT conversionTime);

    // Automatically calculate and set ADC range and scaling value for the current and power readings to provide maximum
    // resolution given the shunt resistance, maximum expected current and maximum expected bus voltage.
    void setScaling(float shuntR, float maxCurrent, float maxVBus);

    // Allows device to be put into continuous measurement mode or shutdown (powersaving sleep).
    // Continuous conversion can be set up for either shunt or VBus measurement, or alternating
    // between both (default).
    // To use single-shot triggered mode, set this mode to SHUTDOWN and use the
    // triggerVBusMeasurement(), triggerShuntMeasurement(), and triggerShuntVBusMeasurement() methods.
    void setConversionMode(MODE conversionMode);

    // Triggers a single-shot VBus measurement.
    void triggerVBusMeasurement();
    void triggerShuntMeasurement();
    void triggerShuntVBusMeasurement();

    // Returns true if a new measurement is available to be read.
    // Note that this only returns "true" once per new measurement - subsequent calls
    // while it's still the same measurement will return "false", regardless if it's been
    // read yet or not.
    bool newMeasurementAvailable();

    // Reads out the latest bus voltage reading in Volts. If averaging is set up, this is the latest averaged result.
    // If only new measurements are wanted, use `newMeasurementAvailable()` to check first.
    float getBusVoltage();

    // Reads out the latest current shunt voltage reading in Volts. If averaging is set up, this is the latest averaged
    // result. If only new measurements are wanted, use `newMeasurementAvailable()` to check first.
    float getShuntVoltage();

    // Reads out the latest current value, as calculated from the current shunt voltage reading.
    // If averaging is set up, this is the latest averaged result.
    // If only new measurements are wanted, use `newMeasurementAvailable()` to check first.
    float getCurrent();

    // Reads out the latest power value, as calculated from the current shunt voltage and VBus readings.
    // If averaging is set up, this is the latest averaged result.
    // If only new measurements are wanted, use `newMeasurementAvailable()` to check first.
    float getPower();

    // Returns a value representing the success or failure of the last method called on this device
    bool success();

  private:
    uint8_t _devAddress;
    bool _success;
    TwoWire *_wire;
    bool _hasBegun = false;
    static constexpr float _VBUS_LSB = 0.0016; // 1.6mV Volts per bit
    float _vShuntLSB = 0.0000025;          // 2.5uV per bit (with larger ADCRange)
    float _currentLSB = 0;                 // Amps per bit
    float _minCurrentLSB = 0;
    float _shuntResistance = 0;

    uint16_t _shadowRegConfig = INA232_REG::CONFIG::DEFAULT;
    uint16_t _shadowRegShuntCal = INA232_REG::SHUNT_CAL::DEFAULT;
    uint16_t _shadowRegMaskEn = INA232_REG::MASK_EN::DEFAULT;

    static constexpr uint8_t LAST_ADDR_NOT_SET = 0xFF;
    uint8_t _lastAddress = LAST_ADDR_NOT_SET;

    void _autoScale();

    uint16_t _readWord(uint8_t address);
    void _writeWord(uint8_t address, uint16_t data);
    uint16_t _getReg(uint8_t regAddress);
    void _setReg(uint8_t regAddress, uint16_t data);
    uint16_t *_shadowPtr(uint8_t regAddress);
    uint16_t _getShadow(uint8_t regAddress);
    void _setShadow(uint8_t regAddress, uint16_t data);
};
#endif