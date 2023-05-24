#include <Arduino.h>
#include <Wire.h>
#include <ina232_i2c.h>

/*Runs a test to poll the INA232 as fast as possible for new measurements (while optionally checking to make sure
each one is new). Allows settings to be adjusted to see the effect on sample rate and how fast data can be pulled
in over the I2C bus.
*/

/* =====Speed details=====
Each read involves 5 bytes (dev address-W, write register pointer, dev address-R, register byte 1, register byte 2).
In general, these each require 9 bits, so at 400kbps this takes 112.5uS at best. With the short delay between bytes,
this is often closer to 130uS. There's usually a delay between I2C transmissions while the Arduino processes things,
and so we wind up at around 180uS per read.

Each type of measurement read needs one of these cycles, as does checking if there is a new measurement. If there
is no check for a new measurement and only one measurement type is being read successively (rather than switching
between multiple), the register pointer doesn't need to be rewritten, and so this cycle comes down to 3 bytes (including
the processing delay, about 120uS). 

The INA232 actually reads the VBus and shunt in succession, so if it's set up to read both continuously each measurement
will take the total of both the vBus and shunt conversion times.

Upshot of all of this is that _yes_, it is techinically possible to read out values as fast as the INA232 can generate them,
but when trying to get many samples per second its often not worth checking if the measurement is actually new or not.
*/


// Comment out some of these defines to allow checking of particular measurements (or all)
#define READ_VBUS
#define READ_CURRENT
#define READ_POWER

// Comment this out to read measurements out without checking if they're new or not.
#define ONLY_READ_NEW

const int groupLengthMillis = 1000;

const int SDAPin = 22;
const int SCLPin = 23;



INA232 ina232(INA232::A0_A_GND);

void setup() {
    delay(1000); // Convenience to allow serial monitor to start before we output anything
    Serial.begin(115200);

    Wire.begin(SDAPin, SCLPin, 400000); // Wire must be started before try and use it with the INA232

    // Set the number of samples to be averaged per measurement
    ina232.setAveraging(INA232::Avg::s1);

    // Set the time taken for each of the shunt and vbus conversions
    ina232.setShuntConversionTime(INA232::VShCT::t140us);
    ina232.setVBusConversionTime(INA232::VBusCT::t140us);

    // Set the INA232 to continuously take new shunt and VBus samples alternatively (which is the default anyway)
    ina232.setConversionMode(INA232::Mode::contShuntBusV);

    // Need to set scaling before we can read anything
    ina232.setScaling(0.003, 30, 30);

    Serial.println("Attempting to connect to INA232...");
    ina232.begin();
    if (!ina232.success()) {
        Serial.println("Could not connct to INA232");
    }
    Serial.println("Successfully connected to INA232!");
}



void loop() {
    unsigned long groupStartTime = millis();
    unsigned long measurementCount = 0;
    unsigned long pollCount = 0;
    bool firstMeasurement = true;
    float vBusAccumulator = 0;
    float minVBus = 0;
    float maxVBus = 0;
    float currentAccumulator = 0;
    float minCurrent = 0;
    float maxCurrent = 0;
    float powerAccumulator = 0;
    float minPower = 0;
    float maxPower = 0;

    while ((millis() - groupStartTime) < groupLengthMillis) {
#ifdef ONLY_READ_NEW
        // Spin loop until new measurement is ready
        pollCount++;
        while (!ina232.newMeasurementAvailable()) {
            pollCount++;
        }
#endif
        measurementCount++;

#ifdef READ_VBUS
        float vBus = ina232.getBusVoltage();
        vBusAccumulator += vBus;
        if (firstMeasurement || (vBus < minVBus)) {
            minVBus = vBus;
        }
        if (firstMeasurement || (vBus > maxVBus)) {
            maxVBus = vBus;
        }
#endif

#ifdef READ_CURRENT
        float current = ina232.getCurrent();
        currentAccumulator += current;
        if (firstMeasurement || (current < minCurrent)) {
            minCurrent = current;
        }
        if (firstMeasurement || (current > maxCurrent)) {
            maxCurrent = current;
        }
#endif

#ifdef READ_POWER
        float power = ina232.getPower();

        powerAccumulator += power;
        if (firstMeasurement || (power < minPower)) {
            minPower = power;
        }
        if (firstMeasurement || (power > maxPower)) {
            maxPower = power;
        }
#endif
        firstMeasurement = false;
    }

    Serial.print("\n=====Measurement group (");
    Serial.print(groupLengthMillis);
    Serial.println("ms)=====");

    Serial.print("Measurement count: ");
    Serial.print(measurementCount);
    Serial.print(" (");
    Serial.print(measurementCount / (groupLengthMillis / 1000.0));
    Serial.println(" per second)");

#ifdef ONLY_READ_NEW
    Serial.print("Average polls per measurement: ");
    Serial.println((float)pollCount / measurementCount);
#endif

#ifdef READ_VBUS
    Serial.print("VBus: ");
    Serial.print(vBusAccumulator / measurementCount);
    Serial.print("V (");
    Serial.print(minVBus);
    Serial.print("V - ");
    Serial.print(maxVBus);
    Serial.println("V)");
#endif

#ifdef READ_CURRENT
    Serial.print("Current: ");
    Serial.print(currentAccumulator / measurementCount, 3);
    Serial.print("A (");
    Serial.print(minCurrent, 3);
    Serial.print("A - ");
    Serial.print(maxCurrent, 3);
    Serial.println("A)");
#endif

#ifdef READ_POWER
    Serial.print("Power: ");
    Serial.print(powerAccumulator / measurementCount, 2);
    Serial.print("W (");
    Serial.print(minPower, 2);
    Serial.print("W - ");
    Serial.print(maxPower, 2);
    Serial.println("W)");
#endif
}