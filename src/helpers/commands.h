//FOR KEYBOARD COMMANDS (TESTING) SET ARM=0x30!
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