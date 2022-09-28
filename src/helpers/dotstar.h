#include "pins.h"
#include <Adafruit_DotStar.h>

#define NUM_PIXELS 1

// ======================
// ===DIAGNOSTIC CODES===
// ======================

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

// DotStar instantiation
#define DASH_ON 250
#define DOT_ON 125
#define BLINK_INTERVAL 125
#define MESSAGE_INTERVAL 1000

Adafruit_DotStar strip(1, DOTSTAR_DATA_PIN, DOTSTAR_CLK_PIN, DOTSTAR_RBG);
const uint32_t OFF      =  strip.Color(0, 0, 0);       //BGR
const uint32_t WHITE    =  strip.Color(255, 255, 255);
const uint32_t BLUE     =  strip.Color(255, 0, 0);
const uint32_t RED      =  strip.Color(0, 0, 255);
const uint32_t GREEN    =  strip.Color(0, 255, 0);
const uint32_t PURPLE   =  strip.Color(255, 0, 255);
const uint32_t AMBER    =  strip.Color(0, 191, 255);
const uint32_t CYAN     =  strip.Color(255, 255, 0);
const uint32_t LIME     =  strip.Color(0, 255, 125);

// Functions
static void blinkCode(byte code, uint32_t color) {
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
