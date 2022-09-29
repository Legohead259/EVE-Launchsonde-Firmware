enum Mode_t {
    FLIGHT_MODE,        // Package will go through nominal flight, but not send diagnostic telemetry
    DIAGNOSTIC_MODE     // Package will send diagnostic telemetry over the serial port and will not begin code execution until a serial connection is opened.
};

Mode_t currentMode = FLIGHT_MODE;

static void getModeString(char* outStr, Mode_t m)  {
    switch(m) {
        case FLIGHT_MODE:
            strcpy(outStr, "FLIGHT");
            break;
        case DIAGNOSTIC_MODE:
            strcpy(outStr, "DIAGNOSTIC");
            break;
        default:
            strcpy(outStr, "UNKNOWN MODE!");
            break;
    }
}