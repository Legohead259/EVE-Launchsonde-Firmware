enum LogLevel {
    FATAL = 1,      //Indicates a fatal event
    ERROR,          //Inidcates a major error, but not fatal
    WARN,           //Indicates a substantial event that is not an error or fatal
    INFO,           //Indicates basic information for logging
    DEBUG,          //Indicates some parameter that is less important than basic information
    VERBOSE         //Indicates some information that is less important than debug information
};

static void getLogLevelString(char* outStr, LogLevel l) {
    switch(l) {
        case FATAL:
            strcpy(outStr, "FATAL");
            break;
        case ERROR:
            strcpy(outStr, "ERROR");
            break;
        case WARN:
            strcpy(outStr, "WARN");
            break;
        case INFO:
            strcpy(outStr, "INFO");
            break;
        case DEBUG:
            strcpy(outStr, "DEBUG");
            break;
        case VERBOSE:
            strcpy(outStr, "VERBOSE");
            break;
        default:
            strcpy(outStr, "UNKNOWN LOG LEVEL");
            break;
    }
}