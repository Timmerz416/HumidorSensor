/*
 Name:		HumidorSensor.ino
 Created:	10/27/2015 9:41:07 PM
 Author:	Tim Lampman_2
*/

#include <MAX1704.h>
#include <Wire.h>
#include <HTU21D.h>
#include <JeeLib.h>
#include <XBee.h>
#include <AltSoftSerial.h>

ISR(WDT_vect) { Sleepy::watchdogEvent(); } // Setup the watchdog

const bool prefer_sleep = true;

// XBee variables
AltSoftSerial xbeeSerial;  // The software serial port for communicating with the Xbee (TX Pin 9, RX Pin 8)
XBee localRadio = XBee();  // The connection for the local coordinating radio

// XBee command codes
const uint8_t CMD_SENSOR_DATA = 4;

// XBee data codes
const uint8_t TEMPERATURE_CODE = 1;
const uint8_t LUMINOSITY_CODE = 2;
const uint8_t PRESSURE_CODE = 3;
const uint8_t HUMIDITY_CODE = 4;
const uint8_t POWER_CODE = 5;
const uint8_t LUX_CODE = 6;
const uint8_t HEATING_CODE = 7;
const uint8_t THERMOSTAT_CODE = 8;
const uint8_t TEMP_12BYTE_CODE = 9;

// Timing variables
const unsigned long SENSOR_DELAY = 60000;	// The sleep period (ms)
const unsigned long DELAY_PERIODS = 10;		// The number of sleep periods before updating sensor readings
const unsigned long STARTUP_DELAY = 3000;	// The period allowed for component warmup and initialization (ms).
const unsigned long COMM_DELAY = 1000;		// The period allowed for XBee communications to initialize/finalize (ms).

// Sensor objects
HTU21D airSensor;

// Local pins
const int LOCAL_POWER_PIN = A0;	// The analog pin that measures the power supply

const int POWER_PIN = 13;		// The digital pin for powering the sensors and XBee

// Union for conversion of numbers to byte arrays
union FloatConverter {
	float f;
	uint8_t b[sizeof(float)];
};

void SmartDelay(unsigned long delay_time, bool force_delay);

//=============================================================================
// SETUP
//=============================================================================
void setup() {
	// Initialize the pins
	pinMode(POWER_PIN, OUTPUT);		// Set the power pin to digital output
	digitalWrite(POWER_PIN, LOW);	// Turn off the power

	// Start the I2C interface
	airSensor.begin();

	// Setup the serial communications
	Serial.begin(9600);
	Serial.println("Starting service...");
}

//=============================================================================
// MAIN LOOP
//=============================================================================
void loop() {
	//-------------------------------------------------------------------------
	// POWER UP COMPONENTS
	//-------------------------------------------------------------------------
	// Turn on the power to the attached components
	digitalWrite(POWER_PIN, HIGH);
	SmartDelay(STARTUP_DELAY, false);	// Warmup delay

	// Connect to the XBee
	Serial.print("Starting XBee connection...");
	xbeeSerial.begin(9600);
	localRadio.setSerial(xbeeSerial);
	Serial.println("FINISHED");

	// Delay while components power up
	SmartDelay(COMM_DELAY, false);

	//-------------------------------------------------------------------------
	// COLLECT SENSOR DATA
	//-------------------------------------------------------------------------
	// Read the power
	FloatConverter Power;
	int powerSignal = analogRead(LOCAL_POWER_PIN);
	Power.f = 3.3*powerSignal / 1023.0;

	// Read the temperature
	FloatConverter Temperature;
	Temperature.f = airSensor.readTemperature();

	// Read the humidity
	FloatConverter Humidity;
	float raw_humidity = airSensor.readHumidity();
	Humidity.f = raw_humidity - 0.15*(25.0 - Temperature.f);	// Correction for HTU21D from spec sheet

	//-------------------------------------------------------------------------
	// SEND DATA THROUGH XBEE
	//-------------------------------------------------------------------------
	// Create the byte array to pass to the XBee
	size_t floatBytes = sizeof(float);
	uint8_t package[1 + 3*(floatBytes + 1)];
	package[0] = CMD_SENSOR_DATA;
	package[1] = TEMPERATURE_CODE;
	package[1 + (floatBytes + 1)] = HUMIDITY_CODE;
	package[1 + 2*(floatBytes + 1)] = POWER_CODE;
	for(int i = 0; i < floatBytes; i++) {
		package[i + 2] = Temperature.b[i];
		package[i + 2 + (floatBytes + 1)] = Humidity.b[i];
		package[i + 2 + 2*(floatBytes + 1)] = Power.b[i];
	}

	// Send the data package to the coordinator through the XBee
	XBeeAddress64 address = XBeeAddress64(0x00000000, 0x00000000);
	ZBTxRequest zbTX = ZBTxRequest(address, package, sizeof(package));
	localRadio.send(zbTX);

	// Print message to the serial port
	Serial.print("Sent the following message (");
	Serial.print(Temperature.f);
	Serial.print(",");
	Serial.print(Humidity.f);
	Serial.print(",");
	Serial.print(Power.f);
	Serial.print("): ");
	for(int i = 0; i < sizeof(package); i++) {
		if(i != 0) Serial.print("-");
		Serial.print(package[i], HEX);
	}
	Serial.println("");

	// Transmission delay
	SmartDelay(COMM_DELAY, false);

	//-------------------------------------------------------------------------
	// POWER DOWN COMPONENTS AND WAIT FOR NEXT CYCLE
	//-------------------------------------------------------------------------
	// Turn off the power to the components and sleep
	xbeeSerial.end();	// Turn off the serial communication with the xbee
	digitalWrite(POWER_PIN, LOW);

	// Cycle delay
	for(int i = 0; i < DELAY_PERIODS; i++) SmartDelay(SENSOR_DELAY, false);
}


//=============================================================================
// SmartDelay
//=============================================================================
void SmartDelay(unsigned long delay_time, bool force_delay) {
	if(force_delay) delay(delay_time);	// the user is selecting a delay and not sleeping
	else {
		if(prefer_sleep) Sleepy::loseSomeTime(delay_time);
		else delay(delay_time);
	}
}