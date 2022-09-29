struct Telemetry {
    float voltage;              // Battery voltage in V
    uint8_t month;              // Month from GPS data 
    uint8_t day;                // Day from GPS data
    uint16_t year;              // Year from GPS data
    uint8_t hour;               // Hour from GPS data (UTC)
    uint8_t minute;             // Minute from GPS data
    uint8_t second;             // Second from GPS data
    uint16_t msecond;           // Milliseconds since last report
    bool GPSFix;                // If GPS has positive fix on location
    uint8_t numSats;            // Number of satellites GPS is communicating with
    uint8_t HDOP;               // Accuracy of GPS reading. Lower is better. In tenths (divide by 10. when displaying)
    long latitude;              // In millionths of a degree (divide by 1000000. when displaying)
    long longitude;             // In millionths of a degree (divide by 1000000. when displaying)
    long GPSSpeed;              // In thousandths of a knot (divide by 1000. when displaying)
    long GPSCourse;             // In thousandths of a degree (divide by 1000. when displaying)
    float baroTemp;             // °Celsius from the MPL3115A2
    float pressure;             // Pa
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
    int8_t state;                // State reported by the launchsonde.
    uint8_t packetSize;         // The size of the telemetry packet. Used as a debug tool for ground station/launchsonde comms.
} data;