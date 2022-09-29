#include "helpers/pins.h"
#include <SPI.h>
#include <SD.h>
#include "helpers/EVEHelper.h"

#define BATT_VOLTAGE_THRESHOLD 2.5 // V


// ===========================
// === PROTOTYPE FUNCTIONS ===
// ===========================


void initFileSystem();
bool checkBattVoltage();
void pollSHT31();
void printTelemetryData();
void printBaseStationTelemetry();
void ppsHandler(void);

void setup() {
	#ifdef DIAGNOSTIC
    	Serial.begin(115200);
		while(!Serial); // Wait for serial connection
	#endif

    initDotStar();

    #ifdef DIAGNOSTIC
        currentMode = DIAGNOSTIC;
        Serial.println("Launchsonde is in DIAGNOSTIC mode"); //DEBUG
        // sendDiagnosticData(WARN, "Launchsonde is in DIAGNOSTIC mode"); // Broken, needs more testing
        blinkCode(B1111, PURPLE); // Blink .-.- in PURPLE on the diagnostic RGB LED
    #endif
    if (currentMode == FLIGHT) {
        #ifdef DIAGNOSTIC
            Serial.println("Launchsonde is in FLIGHT mode"); //DEBUG
        #endif
        // sendDiagnosticData(WARN, "Launchsonde is in FLIGHT mode"); // Broken, needs more testing
        blinkCode(B1111, GREEN); // Blink .-.- in GREEN on the diagnoostic RGB LED
    }
        
    setLaunchsondeState(BOOTING); // System is on and initializing

    // initFilesystem();
    initRadio();
    initGPS();
    // initBNO055();
    initBMP388();
    initSHT31();

    setLaunchsondeState(STANDBY); // System initialized successfully and is awaiting sensor calibration
}

void loop() {
    if (currentMode == DIAGNOSTIC_MODE) { // Only check if HW mode is DIAG (D13 -> HIGH).
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
	strip.setPixelColor(0, currentMode == DIAGNOSTIC_MODE ? BLUE : GREEN);
	strip.show();

	static long unsigned int _lastSample = 0;
	if (millis() > _lastSample+SAMPLE_TIME) {
		_lastSample = millis();
		pollGPS();
		// pollBNO055();
		pollBMP388();
		pollSHT31();
	}
    data.packetSize = sizeof(data);

    if (data.state == STANDBY && checkSensorsReady()) { // Check if sensors are calibrated
        setLaunchsondeState(READY);
    }
    else {
        setLaunchsondeState(previousState);
    }

    if (currentMode == DIAGNOSTIC_MODE) {
        printTelemetryData();
        // printBaseStationTelemetry();
        delay(500); // Delay between readings
    }
    // sendTelemetryData(); // Broken, needs more testing
}