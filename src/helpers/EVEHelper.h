// Hardware instantiation
const PROGMEM char UUID_FILENAME[9] = "UUID.txt";
byte UUID = 0xFF; // Default UUID is 0xFF to indicate no UUID is set yet
bool isUUIDConfig = false;
const PROGMEM byte RECEIVER_UUID = 0x00; // Default receiver UUID is 0x00

#define COUNTDOWN_TIME 10 // s
#define SAMPLE_TIME 50 // ms

// Telemetry packet helper
#include "telemetry.h"

// Launchsonde states helper
#include "states.h"

// DotStar helper
#include "dotstar.h"

// Logging helper
#include "logging/logging.h"

// Sensor helpers
#include "sensors/sensors.h"

// Command helper
#include "commands.h"

// Radio helpers
#include "radio/lora.h"
#include "radio/gps.h"