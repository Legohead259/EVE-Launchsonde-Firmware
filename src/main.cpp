#include "helpers/pins.h"
#include <SPI.h>
#include <SD.h>
#include <MicroNMEA.h>
#include <Adafruit_SHT31.h>
#include "helpers/EVEHelper.h"

#define BATT_VOLTAGE_THRESHOLD 2.5 // V

bool isInDiagMode = true; // By default the hardware will be in FLIGHT mode. DIAG is selecting by setting D13 HIGH
State_t prevState;

// Filesystem instantiation
char filename[13];

// GPS instantiation
HardwareSerial& gps = Serial1;
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
bool ledState = LOW;
volatile bool ppsTriggered = false;
uint8_t lastSecond = 0;
float lastMillis = millis();
float curMillis = 0;
float curMSecond = 0;

// SHT instantiations
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// ===========================
// === PROTOTYPE FUNCTIONS ===
// ===========================


void initFileSystem();
void initGPS();
void initSHT31();
bool checkBattVoltage();
void pollGPS();
void pollSHT31();
void printTelemetryData();
void printBaseStationTelemetry();
void ppsHandler(void);

void setup() {
	if (isInDiagMode) {
    	Serial.begin(115200);
		while(!Serial); // Wait for serial connection
	}

    //DotStar Initialization
    strip.begin(); // Initialize pins for output
    strip.setBrightness(50);
    strip.show();  // Turn all LEDs off ASAP

    // initRadio();

    pinMode(MODE_SELECT_PIN, INPUT_PULLDOWN);
    // isInDiagMode = digitalRead(MODE_SELECT_PIN);
    if (isInDiagMode) {
        Serial.println("Launchsonde is in DIAGNOSTIC mode"); //DEBUG
        // sendDiagnosticData(WARN, "Launchsonde is in DIAGNOSTIC mode"); // Broken, needs more testing
        blinkCode(B1111, PURPLE); // Blink .-.- in PURPLE on the diagnostic RGB LED
    }
    else {
        Serial.println("Launchsonde is in FLIGHT mode"); //DEBUG
        // sendDiagnosticData(WARN, "Launchsonde is in FLIGHT mode"); // Broken, needs more testing
        blinkCode(B1111, GREEN); // Blink .-.- in GREEN on the diagnoostic RGB LED
    }

    if (isInDiagMode) while(!Serial); //Wait for serial terminal open. Only applies when mode select is HIGH
    
    setLaunchsondeState(BOOTING); // System is on and initializing


    // initFileSystem();
    initRadio();
    initGPS();
    // initBNO055();
    initBMP388();
    initSHT31();

    setLaunchsondeState(STANDBY); // System initialized successfully and is awaiting sensor calibration
}

void loop() {
    if (isInDiagMode) { // Only check if HW mode is DIAG (D13 -> HIGH).
        while (Serial.available()) { // Check for commands from the Serial terminal and execute.
            executeCommand((Command) Serial.read(), Serial.read());
        }
    }

    // if (!isInDiagMode) { // Check if launchsonde is in FLIGHT mode
    //     if (!checkBattVoltage()) { // Check battery voltage is out of flight range (2.5-4.4 V)
    //         setLaunchsondeState(LOW_BATT);
    //         while (1) { // Blocks further code execution
    //             blinkCode(LOW_BATT_ERROR_CODE, AMBER);
    //             // sendDiagnosticData(FATAL, "Low battery voltage detected"); // Broken, needs more testing
    //             if (checkBattVoltage()) { // Check if false reading and break loop if so
    //                 setLaunchsondeState(prevState);
    //                 break;
    //             } 
    //         }
    //     }
    // }
    // else checkBattVoltage(); // If in DIAGNOSTIC mode, check the battery voltage anyways, but do not block code if its low

	// Set the NeoPixel to always be green when in FLIGHT mode
	strip.setPixelColor(0, isInDiagMode ? BLUE : GREEN);
	strip.show();

	static long unsigned int _lastSample = 0;
	if (millis() > _lastSample+SAMPLE_TIME) {
		_lastSample = millis();
		pollGPS();
		// pollBNO055();
		pollBMP388();
		pollSHT31();

		// Format timestamp
		// char _buf[64];
		// sprintf(_buf, "%04d-%02d-%02dT%s", data.year, data.month, data.day, data.timestamp);

		// // Write to log file
		// File _dataFile = SD.open(filename, FILE_WRITE);
		// _dataFile.printf("%s,%0.3f,%0.3f,%0.3f,%0.3f\r\n", _buf, data.pressure, data.altitude, data.humidity, data.shtTemp);
		// _dataFile.close();
	}
    data.packetSize = sizeof(data);

    if (data.state == STANDBY && checkSensorsReady()) { // Check if sensors are calibrated
        setLaunchsondeState(READY);
    }
    else {
        setLaunchsondeState(prevState);
    }

    if (isInDiagMode) {
        printTelemetryData();
        // printBaseStationTelemetry();
        delay(500); // Delay between readings
    }
    // sendTelemetryData(); // Broken, needs more testing
}


//=======================
//===POLLING FUNCTIONS===
//=======================


void pollGPS() {
    //Parse milliseconds for timestamp
    uint8_t curSecond = nmea.getSecond();
    // Serial.println(curSecond);
    if (curSecond == lastSecond) {
        curMillis = millis();
        curMSecond = curMillis - lastMillis;
        // Serial.println((int) curMSecond); //DEBUG
    }
    else {
        lastSecond = curSecond;
        lastMillis = millis();
        curMSecond = 0;
    }

    //Parse timestamp
    sprintf(data.timestamp, "%02d:%02d:%02d:%03d", nmea.getHour(), nmea.getMinute(), nmea.getSecond(), (int) curMSecond);

    //Update NMEA string based on PPS pulse from GPS. By default refresh rate is 1Hz
    if (ppsTriggered) {
        ppsTriggered = false;
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);

        data.GPSFix = nmea.isValid();
        data.numSats = nmea.getNumSatellites();
        data.HDOP = nmea.getHDOP();
        data.latitude = nmea.getLatitude();
        data.longitude = nmea.getLongitude();
        // nmea.getAltitude(data.altitudeMSL);
        data.GPSSpeed = nmea.getSpeed();
        data.GPSCourse = nmea.getCourse();
    }

    while (!ppsTriggered && gps.available()) {
        char c = gps.read();
        nmea.process(c);
    }
}

void pollSHT31() {
    data.shtTemp = sht31.readTemperature();
    data.humidity = sht31.readHumidity();
}

bool checkBattVoltage() {
    data.voltage = analogRead(BATT_IN_PIN) * (3.3 / 65536) * 2; //Read true voltage of battery
    return data.voltage > BATT_VOLTAGE_THRESHOLD && data.voltage < 4.4; //Check if battery is above 2.5V and below 4.3V
}


//==============================
//===INITIALIZATION FUNCTIONS===
//==============================



void initFileSystem() {
	Serial.print("Initializing SD Card...");
	if (!SD.begin(SD_CS)) {
		Serial.println("failed to initialize SD card!");
		while(true) blinkCode((byte) GEN_ERROR_CODE, RED);
	}
	Serial.println("done!");

	// Find unique log name and set as filename
	Serial.print("Finding unique log name...");
	for (int x=0; x<1000; x++) {
		sprintf(filename, "log%03d.csv", x);
		if (!SD.exists(filename)) break;
	}
	Serial.println(filename);

	// Write telemetry data headers
	Serial.print("Writing data headers...");
	File _dataFile = SD.open(filename, FILE_WRITE);
	_dataFile.println("Timestamp (ISO8601),Pressure (Pa),Altitude AGL (m),Humidity (%),Temperature (°C)");
	_dataFile.close();
	Serial.println("done!");
}

// void initSDCard() {
// 	Serial.print("Initializing SD Card...");
// 	if (!SD.begin()) {
// 		Serial.println("failed to initialize filesystem!");
// 		setLaunchsondeState(FILE_SYSTEM_FAIL);
// 		while(true) blinkCode((byte) GEN_ERROR_CODE, RED); // Blink out general error code
// 	}
// }

void initGPS() {
    Serial.print("Initializing GPS...");
    gps.begin(9600);
    if (!gps) {
        Serial.println("Failed to initialize GPS"); //DEBUG
        setLaunchsondeState(GPS_FAIL);
        while(1) { // Blocks further code execution
            blinkCode((byte) GPS_ERROR_CODE, RED); // Blink out GPS error code
            // sendDiagnosticData(FATAL, "Failed to initailize GPS"); // Broken, requires further testing 
        }
    }
	// nmea.setUnknownSentenceHandler(printUnknownSentence);
	// Clear the list of messages which are sent.
	MicroNMEA::sendSentence(gps, "$PORZB");
	// Send only RMC and GGA messages.
	MicroNMEA::sendSentence(gps, "$PORZB,RMC,1,GGA,1");
	// Disable compatability mode (NV08C-CSM proprietary message) and adjust precision of time and position fields
	MicroNMEA::sendSentence(gps, "$PNVGNME,2,9,1");
	// MicroNMEA::sendSentence(gps, "$PONME,2,4,1,0");

	pinMode(GPS_PPS_INT_PIN, INPUT);
	attachInterrupt(GPS_PPS_INT_PIN, ppsHandler, RISING);
    Serial.println("done!"); //DEBUG
}

void initSHT31() {
    Serial.print("Initializing SHT31...");
    if (!sht31.begin(0x44)) { // Set to 0x45 for alternate i2c addr
        setLaunchsondeState(SHT_FAIL);
        while (1) { // Blocks Further code execution
            blinkCode((byte) SHT_ERROR_CODE, RED);
            // sendDiagnosticData(FATAL, "Failed to initialize SHT-31D"); // Broken, requires further testing 
        }
    }
    Serial.println("done!"); //DEBUG
}


//=======================
//===UTILITY FUNCTIONS===
//=======================


void printUnknownSentence(const MicroNMEA& nmea) {
	// Do nothing
}

void ppsHandler(void) {
	ppsTriggered = true;
	// Serial.println(\triggered!"); //DEBUG
}


//================================
//===DIAGNOSTIC PRINT FUNCTIONS===
//================================

void printTelemetryData() {
    Serial.println("---TELEMETRY PACKET---");
    Serial.print("Battery voltage:  "); Serial.print(data.voltage);             Serial.println(" V");
    Serial.print("GPS month:        "); Serial.print(data.month);               Serial.println();
    Serial.print("GPS day:          "); Serial.print(data.day);                 Serial.println();
    Serial.print("GPS year:         "); Serial.print(data.year);                Serial.println();
    Serial.print("Timestamp:        "); Serial.print(data.timestamp);           Serial.println(" [hh:mm:ss:mss]");
    Serial.print("GPS fix:          "); Serial.print(data.GPSFix);              Serial.println();
    Serial.print("# of sats:        "); Serial.print(data.numSats);             Serial.println();
    Serial.print("GPS HDOP:         "); Serial.print(data.HDOP/10);             Serial.println();
    Serial.print("Latitude:         "); Serial.print(data.latitude/1000000);    Serial.println("°");
    Serial.print("Longitude:        "); Serial.print(data.longitude/1000000);   Serial.println("°");
    Serial.print("GPS speed:        "); Serial.print(data.GPSSpeed/1000);       Serial.println(" kts");
    Serial.print("GPS course:       "); Serial.print(data.GPSCourse/1000);      Serial.println("°");
    Serial.print("Barometer temp:   "); Serial.print(data.baroTemp);	        Serial.println("°C");
    Serial.print("Pressure:         "); Serial.print(data.pressure);            Serial.println(" Pa");
    Serial.print("Altitude (AGL):   "); Serial.print(data.altitude);            Serial.println(" m");
    Serial.print("System calib:     "); Serial.print(data.sysCal);              Serial.println();
    Serial.print("Gyro calib:       "); Serial.print(data.gyroCal);             Serial.println();
    Serial.print("Accel calib:      "); Serial.print(data.accelCal);            Serial.println();
    Serial.print("Mag calib:        "); Serial.print(data.magCal);              Serial.println();
    Serial.print("X acceleration:   "); Serial.print(data.accelX);              Serial.println(" m/s");
    Serial.print("Y acceleration:   "); Serial.print(data.accelY);              Serial.println(" m/s");
    Serial.print("Z acceleration:   "); Serial.print(data.accelZ);              Serial.println(" m/s");
    Serial.print("X gyro:           "); Serial.print(data.gyroX);               Serial.println(" rad/s");
    Serial.print("Y gyro:           "); Serial.print(data.gyroY);               Serial.println(" rad/s");
    Serial.print("Z gyro:           "); Serial.print(data.gyroZ);               Serial.println(" rad/s");
    Serial.print("Roll:             "); Serial.print(data.roll);                Serial.println("°");
    Serial.print("Pitch:            "); Serial.print(data.pitch);               Serial.println("°");
    Serial.print("Yaw:              "); Serial.print(data.yaw);                 Serial.println("°");
    Serial.print("X linear accel:   "); Serial.print(data.linAccelX);           Serial.println(" m/s");
    Serial.print("X linear accel:   "); Serial.print(data.linAccelY);           Serial.println(" m/s");
    Serial.print("X linear accel:   "); Serial.print(data.linAccelZ);           Serial.println(" m/s");
    Serial.print("IMU temp:         "); Serial.print(data.imuTemp);             Serial.println("°C");
    Serial.print("SHT temp:         "); Serial.print(data.shtTemp);             Serial.println("°C");
    Serial.print("Humidity:         "); Serial.print(data.humidity);             Serial.println("%");
    Serial.print("State:            "); Serial.print(data.state);               Serial.println();
    Serial.print("Packet size:      "); Serial.print(data.packetSize);          Serial.println();
}

/**
 * Prints out telemetry data in the format expected by the base station. For testing purposes only!
*/
void printBaseStationTelemetry() {
    Serial.print(data.timestamp); Serial.print(',');
    Serial.print(data.voltage); Serial.print(',');
    Serial.print(data.GPSFix); Serial.print(',');
    Serial.print(data.numSats); Serial.print(',');
    Serial.print(data.HDOP); Serial.print(',');
    Serial.print(data.latitude); Serial.print(',');
    Serial.print(data.longitude); Serial.print(',');
    Serial.print(data.GPSSpeed); Serial.print(',');
    Serial.print(data.GPSCourse); Serial.print(',');
    Serial.print(data.altitude); Serial.print(',');
    Serial.print(data.sysCal); Serial.print(',');
    Serial.print(data.gyroCal); Serial.print(',');
    Serial.print(data.accelCal); Serial.print(',');
    Serial.print(data.magCal); Serial.print(',');
    Serial.print(data.accelX); Serial.print(',');
    Serial.print(data.accelY); Serial.print(',');
    Serial.print(data.accelZ); Serial.print(',');
    Serial.print(data.gyroX); Serial.print(',');
    Serial.print(data.gyroY); Serial.print(',');
    Serial.print(data.gyroZ); Serial.print(',');
    Serial.print(data.roll); Serial.print(',');
    Serial.print(data.pitch); Serial.print(',');
    Serial.print(data.yaw); Serial.print(',');
    Serial.print(data.linAccelX); Serial.print(',');
    Serial.print(data.linAccelY); Serial.print(',');
    Serial.print(data.linAccelZ); Serial.print(',');
    Serial.print(data.imuTemp); Serial.print(',');
    Serial.print(data.shtTemp); Serial.print(',');
    Serial.print(data.humidity); Serial.print(',');
    Serial.print(data.state); Serial.print(',');
    Serial.print(data.packetSize); Serial.print(',');
    Serial.print(0); Serial.print(','); // Period
    Serial.print(0); Serial.print(','); // Frequency
    Serial.print(0); Serial.print(','); // RSSI
    Serial.print(0); Serial.print(','); // DX
    Serial.print(0); Serial.print(','); // DY
    Serial.println(0);                  // Distance
}