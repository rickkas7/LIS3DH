#include "Particle.h"

#include "LIS3DH.h"

// Example of using the movement detection feature and SPI connection, like the asset tracker.
// Instead of actually putting the device to sleep, this prints information which is handy
// for debugging purposes. The WakeOnMove example actually uses sleep mode.

SYSTEM_THREAD(ENABLED);

void movementInterruptHandler();

// LIS3DH is connected as in the AssetTracker, to the primary SPI with A2 as the CS (SS) pin, and INT connected to WKP
LIS3DHSPI accel(SPI, A2, WKP);

bool sensorsInitialized;
unsigned long lastPrintSample = 0;

volatile bool movementInterrupt = false;
uint8_t lastPos = 0;

void setup() {
	Serial.begin(9600);

	attachInterrupt(WKP, movementInterruptHandler, RISING);

	delay(5000);

	// Initialize sensors
	LIS3DHConfig config;
	config.setLowPowerWakeMode(16);

	sensorsInitialized = accel.setup(config);
	Serial.printlnf("sensorsInitialized=%d", sensorsInitialized);
}

void loop() {

	if (movementInterrupt) {
		accel.clearInterrupt();

		Serial.println("movementInterrupt");

		// Recalibrate the accelerometer for possibly being in a new orientation.
		// Wait up to 15 seconds for it to be stationary for 2 seconds.
		bool ready = accel.calibrateFilter(2000, 15000);
		Serial.printlnf("calibrateFilter ready=%d", ready);

		movementInterrupt = false;
	}
}

void movementInterruptHandler() {
	movementInterrupt = true;
}

