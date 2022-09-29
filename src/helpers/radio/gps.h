#include <MicroNMEA.h>

// GPS instantiation
HardwareSerial& gps = Serial1;
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
volatile bool ppsTriggered = false;
uint8_t lastSecond = 0;
float lastMillis = millis();
float curMillis = 0;
float curMSecond = 0;

void printUnknownSentence(const MicroNMEA& nmea) {
	// Do nothing
}

void ppsHandler(void) {
	ppsTriggered = true;
	// Serial.println(\triggered!"); //DEBUG
}

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

static void pollGPS() {
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
    data.year = nmea.getYear();
    data.month = nmea.getMonth();
    data.day = nmea.getDay();
    data.hour = nmea.getHour();
    data.minute = nmea.getMinute();
    data.second = nmea.getSecond();
    data.msecond = curMSecond;

    //Update NMEA string based on PPS pulse from GPS. By default refresh rate is 1Hz
    data.GPSFix = nmea.isValid();
    data.numSats = nmea.getNumSatellites();
    data.HDOP = nmea.getHDOP();
    data.latitude = nmea.getLatitude();
    data.longitude = nmea.getLongitude();
    // nmea.getAltitude(data.altitudeMSL);
    data.GPSSpeed = nmea.getSpeed();
    data.GPSCourse = nmea.getCourse();

    while (gps.available()) {
        char c = gps.read();
        nmea.process(c);
    }
}