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

    typedef INA232Reg::Config::Avg Avg;
    typedef INA232Reg::Config::VBusCT VBusCT;
    typedef INA232Reg::Config::VShCT VShCT;

    // Subset of the available modes - others are singleshot, and used with trigger*() instead
    enum class Mode : uint16_t {
        shutdown = (uint16_t)INA232Reg::Config::Mode::shutdown, // Shutdown
        contShunt = (uint16_t)INA232Reg::Config::Mode::contShV, // Continuous Shunt voltage measurements
        contBusV = (uint16_t)INA232Reg::Config::Mode::contBusV, // Continuous Bus voltage measurements
        contShuntBusV =
            (uint16_t)INA232Reg::Config::Mode::contShvBusV, // Continuous Shunt and Bus voltage measurements (default)
    };

    typedef INA232Reg::Config::ADCRange ADCRange;

    INA232(A0 addr);

    // Attempts to start communication with the INA232 over the I2C port. If any configuration
    // has been set before this, changes will be written to the device.
    void begin(TwoWire *wirePort = &Wire);

    // Attempts to reset the INA232 device via I2C. If successful, will return all configured registers
    // to their default
    void reset();

    // Sets the measurement range of the current shunt ADC. Voltages greater than this
    // will clip to the maximum measurement range.
    void setRange(ADCRange range);

    // Set number of samples to average before storing result to be read
    // in the current, vbus and power registers. Results are averaged in groups, not
    // a moving window. Power is calculated per sample, then averaged, rather than
    // being calculated on the average current and vbus values later.
    //
    // If called before begin(), any change will be applied when begin() is called.
    void setAveraging(Avg averageCount);

    // Sets the time taken to do each VBus measurement. Longer times result in
    // cleaner measurement.
    void setVBusConversionTime(VBusCT conversionTime);

    // Sets the time taken to do each VBus measurement. Longer times result in
    // cleaner measurement.
    void setShuntConversionTime(VShCT conversionTime);

    int _avgFieldToCount(INA232Reg::Config::Avg averageCount);

    float _ctFieldToTime(INA232Reg::Config::VBusCT conversionTime);
    float _ctFieldToTime(INA232Reg::Config::VShCT conversionTime);

    // Returns the approximate time it will take to get a new VBus measurement in seconds
    // (the conversion time * the number of samples to be averaged)
    float getVBusMeasurementTime();

    // Returns the approximate time it will take to get a new Shunt measurement in seconds
    // (the conversion time * the number of samples to be averaged)
    float getShuntMeasurementTime();

    // Automatically calculate and set ADC range and scaling value for the current and power readings to provide maximum
    // resolution given the shunt resistance, maximum expected current and maximum expected bus voltage.
    void setScaling(float shuntR, float maxCurrent, float maxVBus);

    // Allows device to be put into continuous measurement mode or shutdown (powersaving sleep).
    // Continuous conversion can be set up for either shunt or VBus measurement, or alternating
    // between both (default).
    // To use single-shot triggered mode, set this mode to SHUTDOWN and use the
    // triggerVBusMeasurement(), triggerShuntMeasurement(), and triggerShuntVBusMeasurement() methods.
    void setConversionMode(Mode conversionMode);

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

    // Reads out the latest current value in Amps, as calculated from the current shunt voltage reading.
    // If averaging is set up, this is the latest averaged result.
    // If only new measurements are wanted, use `newMeasurementAvailable()` to check first.
    float getCurrent();

    // Reads out the latest power value in Watts, as calculated from the current shunt voltage and VBus readings.
    // If averaging is set up, this is the latest averaged result.
    // If only new measurements are wanted, use `newMeasurementAvailable()` to check first.
    float getPower();

    // Returns the minimum increment measurable in the bus voltage (LSB)
    float getBusVoltagePrecision();

    // Returns the minimum increment measurable in the shunt voltage (LSB)
    float getShuntVoltagePrecision();

    // Returns the minimum increment measurable in the current shunt (LSB)
    float getCurrentPrecision();

    // Returns the minimum increment measurable in the power calculation (LSB)
    float getPowerPrecision();

    // Returns a value representing the success or failure of the last method called on this device
    bool success();

  private:
    uint8_t _devAddress;
    bool _success;
    TwoWire *_wire;
    bool _hasBegun = false;
    static constexpr float _vBusLSB = 0.0016; // 1.6mV Volts per bit
    float _vShuntLSB = 0.0000025;             // 2.5uV per bit (with larger ADCRange)
    float _currentLSB = 0;                    // Amps per bit
    float _minCurrentLSB = 0;
    float _shuntResistance = 0;

    uint16_t _shadowRegConfig = INA232Reg::Config::defaultVal;
    uint16_t _shadowRegShuntCal = INA232Reg::ShuntCal::defaultVal;
    uint16_t _shadowRegMaskEn = INA232Reg::MaskEn::defaultVal;

    static constexpr uint8_t lastAddrNotSet = 0xFF;
    uint8_t _lastAddress = lastAddrNotSet;

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