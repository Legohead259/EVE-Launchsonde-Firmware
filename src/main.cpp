#include "EVE_Util.h"
#include <SPI.h>
#include <SD.h>
#include <MicroNMEA.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_BMP3XX.h>
#include <LoRa.h>
#include <Adafruit_DotStar.h>

#define BATT_VOLTAGE_THRESHOLD 2.5 // V
#define COUNTDOWN_TIME 10 // s
#define SAMPLE_TIME 50 // ms

#ifndef UUID_DEF
	#define UUID_DEF 0xFF
#endif 

// Hardware instantiation
const PROGMEM char UUID_FILENAME[9] = "UUID.txt";
byte UUID = 0xFF; // Default UUID is 0xFF to indicate no UUID is set yet
bool isUUIDConfig = false;
const PROGMEM byte RECEIVER_UUID = 0x00; // Default receiver UUID is 0x00
bool isInDiagMode = false; // By default the hardware will be in FLIGHT mode. DIAG is selecting by setting D13 HIGH
State prevState;

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

// IMU Instantiations
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
adafruit_bno055_offsets_t offsets; //struct for the calibration offsets to be saved into memory and loaded.
uint8_t sysCal, gyroCal, accelCal, magCal = 0;
bool calDataLoaded;

// SHT instantiations
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// BMP388 instantiation
Adafruit_BMP3XX bmp;
float initialPressure = 1013.25; //hPa
bool isInitialPressCal = false;

// DotStar instantiation
#define DASH_ON 250
#define DOT_ON 125
#define BLINK_INTERVAL 125
#define MESSAGE_INTERVAL 1000

Adafruit_DotStar strip(NUM_PIXELS, DOTSTAR_DATA_PIN, DOTSTAR_CLK_PIN, DOTSTAR_RGB);
const uint32_t OFF      =  strip.Color(0, 0, 0);       //BGR
const uint32_t WHITE    =  strip.Color(255, 255, 255);
const uint32_t BLUE     =  strip.Color(255, 0, 0);
const uint32_t RED      =  strip.Color(0, 0, 255);
const uint32_t GREEN    =  strip.Color(0, 255, 0);
const uint32_t PURPLE   =  strip.Color(255, 0, 255);
const uint32_t AMBER    =  strip.Color(0, 191, 255);
const uint32_t CYAN     =  strip.Color(255, 255, 0);
const uint32_t LIME     =  strip.Color(0, 255, 125);

// // Telemetry instantiation
// Telemetry data;

// ===========================
// === PROTOTYPE FUNCTIONS ===
// ===========================


void blinkCode(byte code, uint32_t color);
void setLaunchsondeState(State state);
void initFileSystem();
void initGPS();
void initBNO055();
void initBMP388();
void initSHT31();
void executeCommand(Command cmd, byte msg);
bool checkBattVoltage();
void pollGPS();
void pollBNO055();
void pollBMP388();
void pollSHT31();
bool checkSensorsReady(); 
void printTelemetryData();
void printBaseStationTelemetry();
void sendDiagnosticData(LogLevel, char*);
void radioCallback(int);
bool calibrateInitialPressure();
void loadIMUCalData();
void launch();
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

    initFileSystem();
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

	static long _lastSample = 0;
	if (millis() > _lastSample+SAMPLE_TIME) {
		_lastSample = millis();
		pollGPS();
		// pollBNO055();
		pollBMP388();
		pollSHT31();

		// Format timestamp
		char _buf[64];
		sprintf(_buf, "%04d-%02d-%02dT%s", data.year, data.month, data.day, data.timestamp);

		// Write to log file
		File _dataFile = SD.open(filename, FILE_WRITE);
		_dataFile.printf("%s,%0.3f,%0.3f,%0.3f,%0.3f\r\n", _buf, data.pressure, data.altitude, data.humidity, data.shtTemp);
		_dataFile.close();
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


//=====================
//===RADIO FUNCTIONS===
//=====================


/*
Packet structure:
---IMPLICIT HEADERS---
1. Sender UUID
2. Destination UUID
3. Payload type (telemetry, log, command)

---TELEMETRY PAYLOAD---
4a. Telemetry data structure
5a. End packet

---DIAGNOSTIC PAYLOAD---
4b. Log level
5b. Log message
6b. End packet

---COMMAND PAYLOAD---
4c. Command identifier
5c. Command message (if applicable)
6c. End packet
*/

void sendTelemetryData() {
    LoRa.beginPacket();
    LoRa.write(UUID);
    LoRa.write(RECEIVER_UUID);
    LoRa.write(TELEMETRY_PACKET);
    LoRa.write((uint8_t*)&data, data.packetSize);
    LoRa.endPacket();
    LoRa.receive();
}

void sendDiagnosticData(LogLevel level, char* msg) {
    LoRa.beginPacket();
    LoRa.write(UUID);
    LoRa.write(RECEIVER_UUID);
    LoRa.write(LOG_PACKET);
    LoRa.write((byte) level);
    LoRa.print(msg);
    LoRa.endPacket();
    LoRa.receive();
}

void radioCallback(int packetSize) {
    if (packetSize == 0) return;

    byte sender = LoRa.read();
    byte destination = LoRa.read();
    if (destination != UUID) return; // Ignore received messages if not the intended receiver
    byte packetType = LoRa.read();
    if (packetType != COMMAND_PACKET) return; //Ignore any packets that aren't command packets
    byte cmdIdent = LoRa.read();
    byte cmdMsg;
    if (packetSize != 0) cmdMsg = LoRa.read(); // Remaining data is the command message
    executeCommand((Command) cmdIdent, cmdMsg); // Grab command identifier from radio packet and execute appropriate command
    //Additional processing for command messages???
}


//=======================
//===COMMAND FUNCTIONS===
//=======================


void executeCommand(Command cmd, byte msg) {
    switch (cmd) {
    case ARM:
        Serial.println("ARMING"); //DEBUG
        if (data.state == READY) {
            setLaunchsondeState(ARMED);
        }
        break;
    case DISARM:
        Serial.println("DISARMING"); //DEBUG
        if (data.state == ARMED) {
            setLaunchsondeState(prevState);
        }
        break;
    case LAUNCH:
        launch();
        Serial.println("LAUNCHING"); //DEBUG
        break;
    case ZERO_ALT:
        Serial.print("Calibrating altimeter..."); //DEBUG
        while (!calibrateInitialPressure()); //Block program while altimeter recalibrates
        Serial.println("done"); //DEBUG
        break;
    case CAL_IMU:
        Serial.println("CALIBRATING IMU"); //DEBUG
        loadIMUCalData();
        break;
    case RECORD_IMU:
        Serial.println("RECORDING IMU CALIBRATION"); //DEBUG
        setLaunchsondeState(IMU_RECORDING);
        calDataLoaded = false;
        while (!bno.isFullyCalibrated()) { //Enter blocking loop until sensors are calibrated. Probably need some sort of interrupt to exit
            pollBNO055();
            // sendTelemetryData();
            Serial.printf("Calibration: SYS=%d, GYRO=%d, ACCEL=%d, MAG=%d \n", data.sysCal, data.gyroCal, data.accelCal, data.magCal); //DEBUG
        }
        bno.getSensorOffsets(offsets); // Set calibration offsets
        calDataLoaded = true;
        Serial.println("done");
        break;
    default:
        char errorMsg[64]; // Create buffer for error message
        sprintf(errorMsg, "Unknown command received: 0x%02X", (byte) cmd); //Load error message buffer
        // Keep disabled until radio testing can occur. Possibly brokwn due to code crashes with undefined UUID. Maybe try after implementing UUID?
        // sendDiagnosticData(WARN, errorMsg); //Send error message as diagnostic data to ground station
        Serial.println(errorMsg); //DEBUG
        break;
    }
}

void launch() {
    if (data.state == ARMED && checkSensorsReady()) { //Check if system is armed and sensors are ready for flight
        setLaunchsondeState(LAUNCHING);

        //Countdown sequence
        long startMillis = millis();
        while (data.state == LAUNCHING && millis() < startMillis+COUNTDOWN_TIME*1000) { //Countdown to launch while system is armed
            loop(); //Continue sending data and listening for commands. There is still the opportunity to cancel here
        }

        //Ignition
        if (data.state == LAUNCHING) { //Check if system is go for launch. This will only be true if the launch was not canceled in the countdown sequence
            // sendDiagnosticData(WARN, "LAUNCH!"); // Broken, needs more testing
            digitalWrite(IGNITE_PIN, HIGH); //LAUNCH! This will cause current to flow to ignitor hopefully catching the propellent and igniting more. Short will automatically be broken after ignition
        }

        //Verify flight condition
        if (data.state == LAUNCHING && millis() >= startMillis+COUNTDOWN_TIME*1000) { //Check if system is go for launch and countdown has ended
            while (millis() < startMillis+(COUNTDOWN_TIME+SAMPLE_TIME)*1000) { //Sample data to determine flight
                loop(); // Continue collecting data
                if (data.altitude > 1.5 && data.linAccelX > 1.5) { //Check if altitude has increased and there is a vertical linear acceleration
                    // sendDiagnosticData(INFO, "Flight confirmed"); // Broken, needs more testing
                    setLaunchsondeState(FLIGHT); //Presume that if these conditions are met, launch was successful
                    return;
                }
            }
            digitalWrite(IGNITE_PIN, LOW); // Stops shorting the ignitor to preserve power and prevent damage
            // sendDiagnosticData(FATAL, "FLIGHT NOT CONFRIMED! SAFING..."); // Broken, needs more testing
            setLaunchsondeState(IGN_FAIL); //Presume that if launch conditions aren't met during sample time, the ignitor failed
        }
    }
    // else {
        // if (data.state != ARMED)
            // sendDiagnosticData(ERROR, "Launch command received but, system is not armed"); // Broken, needs more testing
        // else 
            // sendDiagnosticData(ERROR, "Launch command received but, sensors are not ready for flight");  // Broken, needs more testing
    // }
}

void loadIMUCalData() {
    if (calDataLoaded) {
        bno.setSensorOffsets(offsets); // Load offset data to BNO sensor
        if (bno.isFullyCalibrated()) {
            // sendDiagnosticData(INFO, "Calibration data loaded successfully");
            Serial.println("Calibration data loaded successfully!"); //DEBUG
            return;
        }
    }
    // sendDiagnosticData(WARN, "Calibration data not loaded");
    Serial.println("Calibration data not loaded"); //DEBUG
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

void pollBMP388() {
    if (!bmp.performReading()) {
        Serial.println("Failed to get BMP388 reading"); // DEBUG
        // sendDiagnosticData(ERROR, "Failed to get BMP388 reading"); // Broken for now, requires further testing
    }
    data.baroTemp = bmp.temperature;
    data.pressure = bmp.pressure;
    data.altitude = bmp.readAltitude(initialPressure);
}

void pollBNO055() {
    bno.getCalibration(&data.sysCal, &data.gyroCal, &data.accelCal, &data.magCal);
    if (bno.isFullyCalibrated()) { //Don't read IMU data unless sensors are calibrated
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);    // - m/s^2
        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);         // - rad/s
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);            // - degrees
        imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);   // - m/s^2

        //Add accelerometer data to data packet            
        data.accelX = accel.x();
        data.accelY = accel.y();
        data.accelZ = accel.z();

        //Add gyroscope data to data packet
        data.gyroX = gyro.x();
        data.gyroY = gyro.y();
        data.gyroZ = gyro.z();
        
        //Add euler rotation data to data packet
        data.roll = euler.z();
        data.pitch = euler.y();
        data.yaw = euler.x();

        //Add linear accleration data to data packet
        data.linAccelX = linaccel.x();
        data.linAccelY = linaccel.y();
        data.linAccelZ = linaccel.z();
    }

    data.imuTemp = bno.getTemp();
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

void initRadio() {
    Serial.print("Initializing radio...");
    LoRa.setPins(RFM_CS_PIN, RFM_RST_PIN, RFM_IRQ_PIN);
    
    if (!LoRa.begin(915E6)) {
        Serial.println("Starting LoRa failed!");
        while(1) { // Block further code execution
            blinkCode((byte) RADIO_ERROR_CODE, RED);
        }
    }
    LoRa.setSyncWord(0xF3);
    Serial.println("done!");
}

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

void initBMP388() {
    Serial.print("Initializing BMP388...");
    if (!bmp.begin_I2C()) {   // hardware I2C mode
        Serial.println("Failed to initialize BMP388!"); // DEBUG
        setLaunchsondeState(ALT_FAIL);
        while (1) { // Blocks further code execution
            blinkCode((byte) ALTIMETER_ERROR_CODE, RED);
            // sendDiagnosticData(FATAL, "Failed to initialize BMP388"); // Broken, requires further testing 
        }
    }
    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("done!");

    Serial.print("Calibrating initial pressure...");
    while(!calibrateInitialPressure()); //Block code execution until initial pressure is calibrated
    Serial.println("done!");
}

void initBNO055() {
    Serial.print("Initializing IMU..."); //DEBUG
    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055"); //DEBUG
        setLaunchsondeState(IMU_FAIL);
        while(1) { // Blocks further code execution
            blinkCode((byte) IMU_ERROR_CODE, RED);
            // sendDiagnosticData(FATAL, "Failed to intialize BNO055"); // Broken, requires further testing 
        }
    }
    bno.setExtCrystalUse(true);
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


// ==============================
// ===DIAGNOSTIC LED FUNCTIONS===
// ==============================


void blinkCode(byte code, uint32_t color) {
    bool dash = true;
    for (int n=0; n<4; n++) {
        if (bitRead(code, n)) {
            if (dash) {
                strip.setPixelColor(0, color); strip.show();
                delay(DASH_ON);
                strip.setPixelColor(0, OFF); strip.show();
                delay(BLINK_INTERVAL);
            }
            else {
                strip.setPixelColor(0, color); strip.show();
                delay(DOT_ON);
                strip.setPixelColor(0, OFF); strip.show();
                delay(BLINK_INTERVAL);
            }
        }
        else {
            if (dash) delay(DASH_ON+BLINK_INTERVAL);
            else delay(DOT_ON+BLINK_INTERVAL);
        }
        dash = !dash;
    }
    delay(MESSAGE_INTERVAL);
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

bool calibrateInitialPressure() {
	int failzone = 1; //Looks for +/- 1m condition
    double total = 0;
    const uint8_t reps = 10;
    for (uint8_t i = 0; i < reps; i++) {
        bmp.performReading();
        total += bmp.pressure;
        // Serial.print("Pressure: "); Serial.println(bmp.pressure);
        delay(100); //Minimum sample interval. Default is 66
    }
    initialPressure = total / reps / 100; // hPa
    // Serial.print("Pressure: "); Serial.print(initialPressure/100); Serial.print("\t Altitude:"); Serial.println(bmp.readAltitude(initialPressure/100)); //DEBUG
    // Serial.printf("Pressure: %d, Altitude: %d ...", initialPressure, bmp.readAltitude(initialPressure));

    if (bmp.readAltitude(initialPressure) < failzone && bmp.readAltitude(initialPressure) > -failzone) { //Check if the altitude is truely zeroed
		isInitialPressCal = true;
        return true;
	}
    else {
        return false;
	}
}

void setLaunchsondeState(State s) {
    prevState = data.state;
    data.state = s;
    char msgBuf[64];
    char stateBuf[32];
    getStateString(stateBuf, s);
    sprintf(msgBuf, "Setting launchsonde state to: %s", stateBuf);
    Serial.println(msgBuf); // DEBUG
    // sendDiagnosticData(INFO, msgBuf); //Broken. Requires further testing
    // TODO: switch-case with the state and set LED color based off state (See RGBDiagnostics enum)
}

bool checkSensorsReady() {
    //If GPS has positive fix, altimeter is in range of ±1m, and IMU is calibrated return true
    if (data.GPSFix && (data.altitude < 1 && data.altitude > -1) && bno.isFullyCalibrated()) {
        return true;
    }
    else {
        return false;
    }
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