/*
 * Project LoRA-Mesh-Client - basic example of a remote counter
 * Description: Will send counts each our to server to be relayed to Particle / Ubidots
 * Author: Chip McClelland
 * Date: 7-25-22
 */

// Version list 
// v1 - initial attempt - no sleep
// v2 - Rough working example - one node

// Included libraries
#include <RHMesh.h>
#include <RH_RF95.h>

// Particle Product definitions
const uint8_t firmVersion = 2;

// Mesh has much greater memory requirements, and you may need to limit the
// max message length to prevent wierd crashes
#ifndef RH_MAX_MESSAGE_LEN
#define RH_MAX_MESSAGE_LEN 255
#endif

// System Mode Calls
SYSTEM_MODE(MANUAL);								// For testing - allows us to monitor the device
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
retained uint8_t resetCount;

// For monitoring / debugging, you have some options on the next few lines
SerialLogHandler logHandler(LOG_LEVEL_INFO);

// Prototype functions
void sensorISR();
void recordCount();
void sendMessage();

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, CONNECTING_STATE, REPORTING_STATE, RESP_WAIT_STATE, FIRMWARE_UPDATE};
char stateNames[9][16] = {"Initialize", "Error", "Idle", "Sleeping", "Napping", "Connecting", "Reporting", "Response Wait", "Firmware Update"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// In this small artifical network of 4 nodes,
const byte CLIENT_ADDRESS = 1;
const byte SERVER1_ADDRESS = 2;
const byte SERVER2_ADDRESS = 3;
const byte SERVER3_ADDRESS = 4;

//Define pins for the RFM9x:
#define RFM95_CS D6
#define RFM95_RST A5
#define RFM95_INT D2
#define RF95_FREQ 915.0
const pin_t blueLED = D7;

// Singleton instance of the radio driver
// RH_RF95 driver(D6, D2);
RH_RF95 driver(RFM95_CS, RFM95_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(driver, CLIENT_ADDRESS);

// Set up pins
const pin_t intPin = D3;

// Program variables
uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
volatile bool sensorDetect = false;
uint16_t hourly = 0;
uint16_t daily = 0;
const uint16_t devID = 65534;
uint8_t alerts = 0;
uint16_t nextReportSeconds = 60;					// Default send every 1 minute until configured by the gateway
time_t lastReportSeconds = 0;						// Ensures we send right away

void setup() 
{
	// Wait for a USB serial connection for up to 15 seconds
	waitFor(Serial.isConnected, 15000);

	pinMode(blueLED,OUTPUT);						// Blue led signals sends
	pinMode(intPin, INPUT_PULLDOWN);				// Initialize sensor interrupt pin

	// Take note if we are restarting due to a pin reset - either by the user or the watchdog - could be sign of trouble
  	if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
    	resetCount++;
    	if (resetCount > 6) alerts = 13;            // Excessive resets
  	}

	if (!manager.init()) Log.info("init failed"); // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
	
	driver.setFrequency(RF95_FREQ);					// Setup ISM frequency - typically 868.0 or 915.0 in the Americas, or 433.0 in the EU

	// The default transmitter power is 13dBm, using PA_BOOST.
	// If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
	// you can set transmitter powers from 5 to 23 dBm:
	driver.setTxPower(23, false);

	attachInterrupt(intPin, sensorISR, RISING);     // Pressure Sensor interrupt from low to high

	Log.info("Startup complete - battery %4.2f%%, reporting every %u seconds clock is %s", System.batteryCharge(), nextReportSeconds, (Time.isValid()) ? "valid" : "not valid");

	Time.zone(-5.0);								// Local time in East Coast
	Time.setDSTOffset(1.0);							// DST offset of 1
	Time.beginDST();								// Summer - so daylight savings

	if (state == INITIALIZATION_STATE) state = IDLE_STATE;	// We got through setup without error
}


void loop()
{

	switch (state) {
		case IDLE_STATE: {							// State we spend time in when there is nothing else to do
			if (Time.now() - lastReportSeconds > nextReportSeconds) {	// Using the sendFrequency set above, we go to the reporing state
				sensorDetect = true;
				state = REPORTING_STATE;
			}
		}
		break;

		case REPORTING_STATE: {						// This is where we will send the message then go back to Idle
			sendMessage();
			state = IDLE_STATE;
		}
		break;

	}

	if (sensorDetect) recordCount();                // The ISR had raised the sensor flag - this will service interrupts regardless of state
}

void sensorISR() {
	sensorDetect = true;
}

void recordCount() {
	static time_t lastCount = Time.now();
	if (Time.now() - lastCount) {
		lastCount = Time.now();
		hourly++;
		daily++;
		sensorDetect = false;
	}
}

void sendMessage() {
	Log.info("Sending to manager_mesh_server1");
	digitalWrite(blueLED,HIGH);

	const uint8_t temp = 85;
	uint8_t battChg = System.batteryCharge();
	uint8_t battState = System.batteryState();
    int16_t rssi = driver.lastRssi();
	static uint8_t msgCnt = 0;
	uint8_t payload[17];

	payload[0] = 0; 								// to be replaced/updated
	payload[1] = 0; 								// to be replaced/updated
	payload[2] = highByte(devID);					// Set for device
	payload[3] = lowByte(devID);
	payload[4] = firmVersion;						// Set for code release
	payload[5] = highByte(hourly);
	payload[6] = lowByte(hourly); 
	payload[7] = highByte(daily);
	payload[8] = lowByte(daily); 
	payload[9] = temp;
	payload[10] = battChg;
	payload[11] = battState;	
	payload[12] = resetCount;
	payload[13] = alerts;
	payload[14] = highByte(rssi);
	payload[15] = lowByte(rssi); 
	payload[16] = msgCnt++;

	// Send a message to manager_server
  	// A route to the destination will be automatically discovered.
	Log.info("sending message %d", payload[16]);
	if (Particle.connected()) Particle.publish("sending","payload to server1",PRIVATE);
	if (manager.sendtoWait(payload, sizeof(payload), SERVER1_ADDRESS) == RH_ROUTER_ERROR_NONE) {
		// It has been reliably delivered to the next node.
		// Now wait for a reply from the ultimate server
		uint8_t len = sizeof(buf);
		uint8_t from;     
		Log.info("Message sent");
		if (manager.recvfromAckTimeout(buf, &len, 3000, &from)) {
			buf[len] = 0;
			char data[64];
			snprintf(data, sizeof(data),"Response: 0x%02x rssi=%d - delivery %s", from, driver.lastRssi(), (buf[0] == payload[16]) ? "successful":"unsuccessful");
			Log.info(data);
			uint32_t newTime = ((buf[1] << 24) | (buf[2] << 16) | (buf[3] << 8) | buf[4]);
			Log.info("Time is: %lu",newTime);
			Time.setTime(newTime);  // Set time based on response from gateway
			Log.info("Time set to %lu local time is %s", newTime, Time.timeStr(newTime).c_str());
			Log.info("Next report in %u seconds",((buf[5] << 8) | buf[6]));
			nextReportSeconds = ((buf[5] << 8) | buf[6]);
			if (Particle.connected()) Particle.publish("Update",data,PRIVATE);
 		}
		else {
			Log.info("No reply, is rf95_mesh_server1, rf95_mesh_server2 and rf95_mesh_server3 running?");
		}
	}
	else Log.info("sendtoWait failed. Are the intermediate mesh servers running?");
	lastReportSeconds = Time.now();
	digitalWrite(blueLED,LOW);
}

