enum Mode {
    FLIGHT_MODE,        // Package will go through nominal flight, but not send diagnostic telemetry
    DIAGNOSTIC_MODE     // Package will send diagnostic telemetry over the serial port and will not begin code execution until a serial connection is opened.
};

Mode currentMode;

static void getModeString(char* outStr, Mode m)  {
    switch(m) {
        case FLIGHT_MODE:
            strcpy(outStr, "FLIGHT_MODE");
            break;
        case DIAGNOSTIC_MODE:
            strcpy(outStr, "DIAGNOSTIC_MODE");
            break;
        default:
            strcpy(outStr, "UNKNOWN MODE!");
            break;
    }
}