#include <Arduino.h>
#include <Wire.h>
#include <ina232_i2c.h>

const int SDAPin = 22;
const int SCLPin = 23;

INA232 ina232(INA232::A0_A_GND);

void setup() {
    delay(1000); // Convenience to allow serial monitor to start before we output anything
    Serial.begin(115200);

    Wire.begin(SDAPin, SCLPin, 400000); // Wire must be started before try and use it with the INA232

    // The INA232 library allows you to make changes to set things up before actually starting communication
    // when you call begin().
    // Defaults are 1 sample (no averaging) and a 1100us sample time.
    ina232.setAveraging(INA232::Avg::s512);
    ina232.setShuntConversionTime(INA232::VShCT::t588us);

    Serial.println("Attempting to connect to INA232...");
    ina232.begin();
    // Calling ina232.success() after any INA232 method that involved actually communicating with the device
    // will return true if no fault occurred.
    if (!ina232.success()) {
        Serial.println("Could not connct to INA232");
    }
    Serial.println("Successfully connected to INA232!");

    // This is the only one of the various setup methods that is actually required before reading
    // any data out of the INA232
    ina232.setScaling(0.003, 30, 30);

    // Setting the scaling will use the provided maximum current and bus voltage to calculate what
    // ADC range and multipliers to use to get the most precision without overflowing things.
    // The resulting precision (the step in measurement value per bit) can be queried afterward:
    Serial.print("VBus precision: ");
    Serial.print(ina232.getBusVoltagePrecision() * 1000);
    Serial.println("mV");
    Serial.print("VShunt precision: ");
    Serial.print(ina232.getShuntVoltagePrecision() * 1000000);
    Serial.println("uV");
    Serial.print("Current precision: ");
    Serial.print(ina232.getCurrentPrecision() * 1000);
    Serial.println("mA");
    Serial.print("Power precision: ");
    Serial.print(ina232.getPowerPrecision());
    Serial.println("W");

    // Using the given conversion time and averaging settings, the time taken per measurement can
    // also be queried (the power measurement is recalculated after every shunt measurement):
    Serial.print("Time per VBus measurement: ");
    Serial.print(ina232.getVBusMeasurementTime() * 1000);
    Serial.println("ms");
    Serial.print("Time per Shunt measurement: ");
    Serial.print(ina232.getShuntMeasurementTime() * 1000);
    Serial.println("ms");
}

void loop() {
    delay(1000);
    Serial.print("VBus: ");
    Serial.print(ina232.getBusVoltage());
    Serial.print("V Current: ");
    Serial.print(ina232.getCurrent() * 1000);
    Serial.print("mA Power: ");
    Serial.print(ina232.getPower());
    Serial.println("W");
}