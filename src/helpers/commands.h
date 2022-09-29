// FOR KEYBOARD COMMANDS (TESTING) SET ARM=0x30!
enum Command {
    ARM = 0x30,                        // Arm the package for launch
    DISARM,                     // Disarm the package for safety
    LAUNCH,                     // Ignite the motor and launch package
    ZERO_ALT,                   // Zero the altimeter
    CAL_IMU,                    // Load calibration data into IMU
    RECORD_IMU,                 // Record calibration data from IMU and put it into EEPROM
    SET_UUID,                   // Uses aditional argument to set the UUID of the instrumentation
    ZERO_GPS                    // Zero the GPS start point (only used on ground station)
};

void getCommandString(char* outStr, Command c) {
    switch (c) {
    case ARM:
        strcpy(outStr, "ARM");
        break;
    case DISARM:
        strcpy(outStr, "DISARM");
        break;
    case LAUNCH:
        strcpy(outStr, "LAUNCH");
        break;
    case ZERO_ALT:
        strcpy(outStr, "ZERO ALTIMETER");
        break;
    case CAL_IMU:
        strcpy(outStr, "CALIBRATE IMU");
        break;
    case RECORD_IMU:
        strcpy(outStr, "RECORD IMU CALIBRATION");
        break;
    case ZERO_GPS:
        strcpy(outStr, "ZERO GPS");
        break;
    default:
        strcpy(outStr, "Unknown");
        break;
    }
}

static bool checkSensorsReady() {
    //If GPS has positive fix, altimeter is in range of ±1m, and IMU is calibrated return true
    return data.GPSFix && (data.altitude < 1 && data.altitude > -1) && bno.isFullyCalibrated();
}

static void launch() {
    if (data.state == ARMED && checkSensorsReady()) { //Check if system is armed and sensors are ready for flight
        setLaunchsondeState(LAUNCHING);

        // Countdown sequence
        long unsigned int startMillis = millis();
        while (data.state == LAUNCHING && millis() < startMillis+COUNTDOWN_TIME*1000) { //Countdown to launch while system is armed
            loop(); //Continue sending data and listening for commands. There is still the opportunity to cancel here
        }

        // Ignition
        if (data.state == LAUNCHING) { //Check if system is go for launch. This will only be true if the launch was not canceled in the countdown sequence
            // sendDiagnosticData(WARN, "LAUNCH!"); // Broken, needs more testing
            digitalWrite(IGNITE_PIN, HIGH); //LAUNCH! This will cause current to flow to ignitor hopefully catching the propellent and igniting more. Short will automatically be broken after ignition
        }

        // Verify flight condition
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

static void loadIMUCalData() {
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

static void executeCommand(Command cmd, byte msg) {
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
            setLaunchsondeState(previousState);
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