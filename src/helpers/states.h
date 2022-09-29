enum State_t {
    // Nominal states
    BOOTING,                    // Package is initializing
    STANDBY,                    // Package has initialized and is calibrating sensors
    READY,                      // Package has calibrated sensors and is ready to fly
    ARMED,                      // Package is armed and ready to ignite motor
    LAUNCHING,                  // Package is preparing for launch
    FLIGHT,                     // Package is in flight
    RECOVERY,                   // Package has landed and is in recovery mode
    
    // Additional functions
    IMU_RECORDING = 10,         // Package is recording IMU calibration data and saving it

    // Error states
    GEN_ERROR = -1,             // Package has encountered an undiagnosed error
    RADIO_FAIL = -2,            // Radio has failed to initialize
    GPS_FAIL = -3,              // GPS has failed to initialize
    ALT_FAIL = -4,              // Altimeter has failed to initialize
    IMU_FAIL = -5,              // IMU has failed to initialize
    SHT_FAIL = -6,              // SHT has failed to initialize
    IGN_FAIL = -7,              // Ignitor failed to ignite motor or has lost connection
    LOW_BATT = -8,              // Battery voltage is below threshold
    FILE_SYSTEM_FAIL = -9       // File system failed to initialize properly
};

State_t previousState;

static void getStateString(char* outStr, State_t s) {
    switch(s) {
        case BOOTING:
            strcpy(outStr, "BOOTING");
            break;
        case STANDBY:
            strcpy(outStr, "STANDBY");
            break;
        case READY:
            strcpy(outStr, "READY");
            break;
        case ARMED:
            strcpy(outStr, "ARMED");
            break;
        case LAUNCHING:
            strcpy(outStr, "LAUNCHING");
            break;
        case FLIGHT:
            strcpy(outStr, "FLIGHT");
            break;
        case RECOVERY:
            strcpy(outStr, "RECOVERY");
            break;
        case IMU_RECORDING:
            strcpy(outStr, "IMU_RECORDING");
            break;
        case RADIO_FAIL:
            strcpy(outStr, "RADIO_FAIL");
            break;
        case GPS_FAIL:
            strcpy(outStr, "GPS_FAIL");
            break;
        case ALT_FAIL:
            strcpy(outStr, "ALT_FAIL");
            break;
        case IMU_FAIL:
            strcpy(outStr, "IMU_FAIL");
            break;
        case SHT_FAIL:
            strcpy(outStr, "SHT_FAIL");
            break;
        case IGN_FAIL:
            strcpy(outStr, "IGN_FAIL");
            break;
        case LOW_BATT:
            strcpy(outStr, "LOW_BATT");
            break;
        default:
            strcpy(outStr, "DINGUS");
            break;
    }
}

static void setLaunchsondeState(State_t s) {
    previousState = (State_t) data.state;
    data.state = s;
    char msgBuf[64];
    char stateBuf[32];
    getStateString(stateBuf, s);
    sprintf(msgBuf, "Setting launchsonde state to: %s", stateBuf);

    #ifdef DIAGNOSTIC_MODE
        Serial.println(msgBuf); // DEBUG
    #endif
    // sendDiagnosticData(INFO, msgBuf); //Broken. Requires further testing
    // TODO: switch-case with the state and set LED color based off state (See RGBDiagnostics enum)
}