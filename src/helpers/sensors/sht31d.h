#include <Adafruit_SHT31.h>

// SHT instantiations
Adafruit_SHT31 sht31 = Adafruit_SHT31();

void initSHT31() {
    Serial.print("Initializing SHT31...");
    if (!sht31.begin(0x44)) { // Set to 0x45 for alternate i2c addr
        setLaunchsondeState(SHT_FAIL);
        while (1) { // Blocks Further code execution
            blinkCode((byte) SHT_ERROR_CODE, RED);
            // sendDiagnosticData(FATAL, "Failed to initialize SHT-31D"); // Broken, requires further testing 
        }
    }
    Serial.println("done!"); //DEBUG
}

static void pollSHT31() {
    data.shtTemp = sht31.readTemperature();
    data.humidity = sht31.readHumidity();
}