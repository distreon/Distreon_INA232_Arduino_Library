#include <algorithm>
#include <ina232_i2c.h>

using namespace INA232_REG;

INA232::INA232(A0 addr) { _devAddress = addr; }

void INA232::begin(TwoWire *wirePort) {
    _success = true;
    _wire = wirePort;
    // Attempt to read out manufacturer ID register, should always be 0x5449
    if (_getReg(MAN_ID::ADDR) == MAN_ID::DEFAULT) {
        _hasBegun = true;

        // Write out any shadow registers that aren't default
        if (_shadowRegConfig != CONFIG::DEFAULT) {
            _setReg(CONFIG::ADDR, _getShadow(CONFIG::ADDR));
        }
        if (_shadowRegShuntCal != SHUNT_CAL::DEFAULT) {
            _setReg(SHUNT_CAL::ADDR, _getShadow(SHUNT_CAL::ADDR));
        }
        if (_shadowRegMaskEn != MASK_EN::DEFAULT) {
            _setReg(MASK_EN::ADDR, _getShadow(MASK_EN::ADDR));
        }
    } else {
        _success = false;
    }
}

void INA232::reset() {
    _success = true;

    _setReg(CONFIG::ADDR, _getReg(CONFIG::ADDR) | CONFIG::RST_BIT);
    if (_success) {
        _setShadow(CONFIG::ADDR, CONFIG::DEFAULT);
        _setShadow(SHUNT_CAL::ADDR, SHUNT_CAL::DEFAULT);
        _setShadow(MASK_EN::ADDR, MASK_EN::DEFAULT);
        _lastAddress = LAST_ADDR_NOT_SET;
    }
}

void INA232::setRange(ADCRANGE range) {
    _success = true;

    // If the range is changing, we need to recalculate the scaling
    bool scaleNeedsUpdate = false;
    if ((_shadowRegConfig & CONFIG::ADCRANGE_MASK) != (uint16_t)range) {
        scaleNeedsUpdate = true;
    }

    uint16_t newConfig = (_getShadow(CONFIG::ADDR) & ~CONFIG::ADCRANGE_MASK) | (uint16_t)range;
    _setReg(CONFIG::ADDR, newConfig);
    if (_success && scaleNeedsUpdate) {
        _autoScale();
    }
}

void INA232::setAveraging(AVG averageCount) {
    _success = true;
    uint16_t newConfig = (_getShadow(CONFIG::ADDR) & ~CONFIG::AVG_MASK) | (uint16_t)averageCount;
    _setReg(CONFIG::ADDR, newConfig);
}

void INA232::setVBusConversionTime(VBUSCT conversionTime) {
    _success = true;
    uint16_t newConfig = (_getShadow(CONFIG::ADDR) & ~CONFIG::VBUSCT_MASK) | (uint16_t)conversionTime;
    _setReg(CONFIG::ADDR, newConfig);
}

void INA232::setShuntConversionTime(VSHCT conversionTime) {
    _success = true;
    uint16_t newConfig = (_getShadow(CONFIG::ADDR) & ~CONFIG::VSHCT_MASK) | (uint16_t)conversionTime;
    _setReg(CONFIG::ADDR, newConfig);
}

// shuntResistance in Ohms
// maxCurrent in Amps
// maxVbus in Volts
void INA232::setScaling(float shuntResistance, float maxCurrent, float maxVBus) {
    _success = true;

    // Default to lower (more accurate) ADC range, drop back to larger ADC range
    // if the low one would overflow given the max current and shunt resistance
    _shuntResistance = shuntResistance;
    if (shuntResistance * maxCurrent > 0.02048) {
        setRange(ADCRANGE::PM_81_92_MV);
    } else {
        setRange(ADCRANGE::PM_20_48_MV);
    }

    // To avoid overflowing current register, min currentLSB = maxCurrent/(2^15)
    // Power register = current register * voltage register / (32)
    // power_lsb = (2^5)*current_lsb
    // To avoid overflowing power register,  min powerLSB = maxPower/(2^16)
    //                                       min currentLSB = maxPower/(2^21)

    float maxPower = maxCurrent * maxVBus;
    _minCurrentLSB = std::min((maxCurrent / (2 ^ 15)), (maxPower / (2 ^ 21)));

    // Setting the shunt cal register and _currentLSB is broken out to allow it to be redone if
    // the ADC range is changed
    _autoScale();

    // Device is only capable of dealing with VBus up to 52.4V, and has VBus_LSB of 1.6mV
}

// If _minCurrentLSB and shuntResistance are set, calculate and set the ShuntCal register and _currentLSB
void INA232::_autoScale() {
    // If range is the lower option, need to divide shunt cal by 4
    uint16_t range = _getShadow(CONFIG::ADDR) & CONFIG::ADCRANGE_MASK;
    int shuntCalDiv = (range == (uint16_t)CONFIG::ADCRANGE::PM_20_48_MV) ? 4 : 1;
    _vShuntLSB = 0.0000025 / shuntCalDiv;
    if ((_shuntResistance != 0) && (_minCurrentLSB != 0)) {
        _setReg(SHUNT_CAL::ADDR, std::floor(0.00512 / (_minCurrentLSB * _shuntResistance * shuntCalDiv)));
        if (_success) {
            // Recalculate currentLSB to account for rounding the cal register. Rounding down the
            // cal register results in a slightly high currentLSB, but it was a minimum anyway, so
            // that's the right direction to go
            _currentLSB = 0.00512 / (_getShadow(SHUNT_CAL::ADDR) * _shuntResistance * shuntCalDiv);
        }
    }
}

void INA232::setConversionMode(MODE conversionMode) {
    _success = true;
    uint16_t newConfig = (_getShadow(CONFIG::ADDR) & ~CONFIG::MODE_MASK) | (uint16_t)conversionMode;
    _setReg(CONFIG::ADDR, newConfig);
}

void INA232::triggerVBusMeasurement() {
    _success = true;
    uint16_t newConfig = (_getShadow(CONFIG::ADDR) & ~CONFIG::MODE_MASK) | (uint16_t)CONFIG::MODE::BUSV_TRIG_SINGLE;
    _setReg(CONFIG::ADDR, newConfig);
}

void INA232::triggerShuntMeasurement() {
    _success = true;
    uint16_t newConfig = (_getShadow(CONFIG::ADDR) & ~CONFIG::MODE_MASK) | (uint16_t)CONFIG::MODE::SHV_TRIG_SINGLE;
    _setReg(CONFIG::ADDR, newConfig);
}

void INA232::triggerShuntVBusMeasurement() {
    _success = true;
    uint16_t newConfig = (_getShadow(CONFIG::ADDR) & ~CONFIG::MODE_MASK) | (uint16_t)CONFIG::MODE::SHV_BUSV_TRIG_SINGLE;
    _setReg(CONFIG::ADDR, newConfig);
}

bool INA232::newMeasurementAvailable() { return (bool)(_getReg(MASK_EN::ADDR) & MASK_EN::CVRF_BIT); }

float INA232::getBusVoltage() {
    _success = true;
    return _getReg(VBUS::ADDR) * _VBUS_LSB;
}

float INA232::getShuntVoltage() {
    _success = true;
    return (int16_t)_getReg(VSHUNT::ADDR) * _vShuntLSB;
}

float INA232::getCurrent() {
    _success = true;
    return (int16_t)_getReg(CURRENT::ADDR) * _currentLSB;
}

float INA232::getPower() {
    _success = true;
    return _getReg(POWER::ADDR) * 32 *_currentLSB;
}

// Physically read a 2 byte word from the device via I2C
uint16_t INA232::_readWord(uint8_t address) {
    // I2C reads are performed by writing a byte indicating the current register to be accessed,
    // then initiating a read.

    // If the last register to be accessed was the same as the one we want,
    // don't need to set it again. If we do, use a repeated start to hold onto the bus.
    // Underlying I2C read and write functions handle the RW bit in the
    // address packet, along with shifting it appropriately. Address provided here should be the 7 bit one.

    // No point checking result of this write, as if we use a repeated start
    // it doesn't actually get sent until after the "requestFrom()" below.
    if (_lastAddress != address) {
        _wire->beginTransmission(_devAddress);
        _wire->write(address);
        _wire->endTransmission(false);
    }

    if (_wire->requestFrom(_devAddress, 2) == 2) {
        _lastAddress = address;

        uint16_t resultWord = (uint16_t)_wire->read() << 8;
        resultWord |= _wire->read();
        return resultWord;
    } else {
        _success = false;
    }
    return 0;
}

// Physically write a 2 byte word to the device via I2C
void INA232::_writeWord(uint8_t address, uint16_t data) {
    // MSB first
    _wire->beginTransmission(_devAddress);
    _wire->write(address);
    _wire->write((uint8_t)((data & 0xFF00) >> 8));
    _wire->write((uint8_t)(data & 0x00FF));

    if (_wire->endTransmission(true) == 0) {
        _lastAddress = address;
    } else {
        _success = false;
    }
}

// Get a register value from the device. If successful, saves the result to the shadow.
uint16_t INA232::_getReg(uint8_t regAddress) {
    uint16_t result = _readWord(regAddress);
    if (_success) {
        _setShadow(regAddress, result);
    }
    return result;
}

// Set a register value on the device. If successful, update the shadow.
// If "begin()" has not been called yet, only set the shadow.
void INA232::_setReg(uint8_t regAddress, uint16_t data) {
    if (_hasBegun) {
        _writeWord(regAddress, data);
        if (_success) {
            _setShadow(regAddress, data);
        }
    } else {
        // If we haven't begun yet, update the shadow without a write
        _setShadow(regAddress, data);
    }
}

// Get a pointer to the relevant shadow register if there is one. Returns nullptr otherwise.
uint16_t *INA232::_shadowPtr(uint8_t regAddress) {
    switch (regAddress) {
    case CONFIG::ADDR:
        return &_shadowRegConfig;
    case SHUNT_CAL::ADDR:
        return &_shadowRegShuntCal;
    case MASK_EN::ADDR:
        return &_shadowRegMaskEn;
    }
    return nullptr;
}

// Get the value of a shadow register if there is one. Otherwise returns 0.
uint16_t INA232::_getShadow(uint8_t regAddress) {
    uint16_t *shadow = _shadowPtr(regAddress);
    if (shadow) {
        return *shadow;
    }
    return 0;
}

// Set the value of a shadow register if there is one, without actually writing
// the value out to the device
void INA232::_setShadow(uint8_t regAddress, uint16_t data) {
    uint16_t *shadow = _shadowPtr(regAddress);
    if (shadow) {
        *shadow = data;
    }
}