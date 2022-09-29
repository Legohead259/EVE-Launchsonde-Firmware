#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// IMU Instantiations
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
adafruit_bno055_offsets_t offsets; //struct for the calibration offsets to be saved into memory and loaded.
uint8_t sysCal, gyroCal, accelCal, magCal = 0;
bool calDataLoaded;

static void initBNO055() {
    Serial.print("Initializing IMU..."); //DEBUG
    pinMode(BNO_RST_PIN, OUTPUT);
    pinMode(BNO_RST_PIN, HIGH);
    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055"); //DEBUG
        setLaunchsondeState(IMU_FAIL);
        while(1) { // Blocks further code execution
            blinkCode(IMU_ERROR_CODE, RED);
            // sendDiagnosticData(FATAL, "Failed to intialize BNO055"); // Broken, requires further testing 
        }
    }
    bno.setExtCrystalUse(true);
    Serial.println("done!"); //DEBUG
}

static void pollBNO055() {
    bno.getCalibration(&data.sysCal, &data.gyroCal, &data.accelCal, &data.magCal);
    if (bno.isFullyCalibrated()) { //Don't read IMU data unless sensors are calibrated
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);    // - m/s^2
        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);         // - rad/s
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);            // - degrees
        imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);   // - m/s^2

        //Add accelerometer data to data packet            
        data.accelX = accel.x();
        data.accelY = accel.y();
        data.accelZ = accel.z();

        //Add gyroscope data to data packet
        data.gyroX = gyro.x();
        data.gyroY = gyro.y();
        data.gyroZ = gyro.z();
        
        //Add euler rotation data to data packet
        data.roll = euler.z();
        data.pitch = euler.y();
        data.yaw = euler.x();

        //Add linear accleration data to data packet
        data.linAccelX = linaccel.x();
        data.linAccelY = linaccel.y();
        data.linAccelZ = linaccel.z();
    }

    data.imuTemp = bno.getTemp();
}