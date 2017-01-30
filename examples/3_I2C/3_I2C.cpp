#include "Particle.h"

#include "LIS3DH.h"

// Electron sample to print accelerometer samples to serial using the I2C interface
//
// Does not work on the AssetTracker (use Example 1, which uses SPI, which is how the accelerometer
// is connected on the AssetTracker.
//
// This example as is does not work on other platforms like Photon and P1 because they
// don't have Wire1. They do work with the main I2C interface, Wire, using pins D0 and D1.


SYSTEM_THREAD(ENABLED);

// This sample only uses serial, not data, so cellular is turned off to save data
SYSTEM_MODE(MANUAL);

// Print 10 samples per second to serial
const unsigned long PRINT_SAMPLE_PERIOD = 100;

// Connect the Adafruit LIS3DH breakout
// https://www.adafruit.com/products/2809
// VIN: 3V3
// GND: GND
// SCL: C5 (Electron Wire1)
// SDA: C4 (Electron Wire1)
// INT: WKP
LIS3DHI2C accel(Wire1, 0, WKP);

unsigned long lastPrintSample = 0;

void setup() {
	Serial.begin(9600);

	delay(5000);

	// Make sure you match the same Wire interface in the constructor to LIS3DHI2C to this!
	Wire1.setSpeed(CLOCK_SPEED_100KHZ);
	Wire1.begin();

	// Initialize sensor
	LIS3DHConfig config;
	config.setAccelMode(LIS3DH::RATE_100_HZ);

	bool setupSuccess = accel.setup(config);
	Serial.printlnf("setupSuccess=%d", setupSuccess);
}

void loop() {

	if (millis() - lastPrintSample >= PRINT_SAMPLE_PERIOD) {
		lastPrintSample = millis();

		LIS3DHSample sample;
		if (accel.getSample(sample)) {
			Serial.printlnf("%d,%d,%d", sample.x, sample.y, sample.z);
		}
		else {
			Serial.println("no sample");
		}
	}
}


