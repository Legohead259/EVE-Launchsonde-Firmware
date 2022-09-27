// #include "filesystem/EVE_FS.h"
#include <Arduino.h>

// ============
// ===STATES===
// ============

// #if FIRMWARE_VERSION == 1
enum State {
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

static void getStateString(char* outStr, State s) {
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

//===============
//===TELEMETRY===
//===============

struct Telemetry {
    float voltage;              // Battery voltage in V
    uint8_t month;              // Month from GPS data 
    uint8_t day;                // Day from GPS data
    uint16_t year;              // Year from GPS data
    char timestamp[32];         // Timestamp in UTC obtained from GPS satellites
    bool GPSFix;                // If GPS has positive fix on location
    uint8_t numSats;            // Number of satellites GPS is communicating with
    uint8_t HDOP;               // Accuracy of GPS reading. Lower is better. In tenths (divide by 10. when displaying)
    long latitude;              // In millionths of a degree (divide by 1000000. when displaying)
    long longitude;             // In millionths of a degree (divide by 1000000. when displaying)
    long GPSSpeed;              // In thousandths of a knot (divide by 1000. when displaying)
    long GPSCourse;             // In thousandths of a degree (divide by 1000. when displaying)
    float baroTemp;             // °Celsius from the MPL3115A2
    float pressure;            // Pa
    float altitude;             // In meters Above Ground Level
    uint8_t sysCal = 0;         // IMU system calibration, 0-3 with 3 being fully calibrated
    uint8_t gyroCal = 0;        // IMU gyroscope calibration, 0-3 with 3 being fully calibrated
    uint8_t accelCal = 0;       // IMU accelerometer calibration, 0-3 with 3 being fully calibrated
    uint8_t magCal = 0;         // IMU magnetometer calibration, 0-3 with 3 being fully calibrated
    float accelX;               // m/s^2
    float accelY;               // m/s^2
    float accelZ;               // m/s^2
    float gyroX;                // rad/s
    float gyroY;                // rad/s
    float gyroZ;                // rad/s
    float roll;                 // degrees
    float pitch;                // degrees
    float yaw;                  // degrees
    float linAccelX;            // m/s^2
    float linAccelY;            // m/s^2
    float linAccelZ;            // m/s^2
    float imuTemp;              // °Celsius from the IMU
    float shtTemp;              // °Celsius (ambient) from the SHT31-D sensor
    float humidity;              // % from the SHT31-D sensor
    State state;                // State reported by the launchsonde.
    uint8_t packetSize;         // The size of the telemetry packet. Used as a debug tool for ground station/launchsonde comms.
} data;

enum PacketType {
    TELEMETRY_PACKET,
    LOG_PACKET,
    COMMAND_PACKET
};

//=================
//===PIN MAPPING===
//=================

#define BATT_IN_PIN         A0
#define IGNITOR_CONT_PIN    A2
#define IGNITE_PIN          A1
#define MODE_SELECT_PIN     13
#define GPS_PPS_INT_PIN     11
#define RFM_IRQ_PIN         7
#define RFM_CS_PIN          A5
#define RFM_RST_PIN         A4
#define DOTSTAR_CLK_PIN     40
#define DOTSTAR_DATA_PIN    41
#define SD_CS               A2

//=============
//===LOGGING===
//=============


enum LogLevel {
    FATAL = 1,      //Indicates a fatal event
    ERROR,          //Inidcates a major error, but not fatal
    WARN,           //Indicates a substantial event that is not an error or fatal
    INFO,           //Indicates basic information for logging
    DEBUG,          //Indicates some parameter that is less important than basic information
    VERBOSE         //Indicates some information that is less important than debug information
};

void getLogLevelString(char* outStr, LogLevel l) {
    //switch-case tree for LogLevels.
}


//==============
//===COMMANDS===
//==============

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

//===========
//===MODES===
//===========

enum Mode {
    FLIGHT_MODE,        // Package will go through nominal flight, but not send diagnostic telemetry
    DIAGNOSTIC_MODE     // Package will send diagnostic telemetry over the serial port and will not begin code execution until a serial connection is opened.
};

//======================
//===DIAGNOSTIC CODES===
//======================

#define NUM_PIXELS 1
enum RGBDiagnostics {
    LED_STANDBY_NO_RADIO,   // Solid blue
    LED_ARMED,              // Solid green
    LED_RECOVERY,           // Solid purple
    LED_LOW_BATTERY,        // Solid amber
    LED_CALIBRATING,        // Solid cyan
    LED_READY,              // Solid lime
    LED_ERROR               // Solid red
};

/*
Code Table:
Error       |  DOT  |  DASH  |  DOT  |  DASH  |  CODE  |  COLOR
------------|-------|--------|-------|--------|--------|---------
General     |   0   |   0    |   0   |    1   |  B0001 |   RED
Radio       |   0   |   0    |   1   |    0   |  B0010 |   RED
GPS         |   0   |   0    |   1   |    1   |  B0011 |   RED
Ignitor     |   0   |   1    |   0   |    0   |  B0100 |  WHITE
IMU         |   0   |   1    |   0   |    1   |  B0101 |   RED
SHT         |   0   |   1    |   1   |    0   |  B0110 |   RED
Altimeter   |   0   |   1    |   1   |    1   |  B0111 |   RED
Low Battery |   1   |   0    |   0   |    0   |  B1000 |  AMBER

A dot is 125 ms on, 125 ms off
A dash is 250 ms on, 250 ms off
Space between codes is 1 sec
*/

enum ErrorCode {
    GEN_ERROR_CODE          = B0001,
    RADIO_ERROR_CODE        = B1010,
    GPS_ERROR_CODE          = B0011,
    IGNITOR_ERROR_CODE      = B0100,
    IMU_ERROR_CODE          = B0101,
    SHT_ERROR_CODE          = B0110,
    ALTIMETER_ERROR_CODE    = B0111,
    LOW_BATT_ERROR_CODE     = B1000
};