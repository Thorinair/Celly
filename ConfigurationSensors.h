/********
 * Temperature Configuration
 *******/

/* Duration of a single sample in ms. */
#define SENSOR_TEMPERATURE_SAMPLE 1000
/* Total number of samples. */
#define SENSOR_TEMPERATURE_COUNT  60
/* Write debug info for the sensor. */
#define SENSOR_TEMPERATURE_DEBUG  false

/********
 * Humidity Configuration
 *******/

/* Duration of a single sample in ms. */
#define SENSOR_HUMIDITY_SAMPLE 1000
/* Total number of samples. */
#define SENSOR_HUMIDITY_COUNT  60
/* Write debug info for the sensor. */
#define SENSOR_HUMIDITY_DEBUG  false

/********
 * Pressure Configuration
 *******/

/* Duration of a single sample in ms. */
#define SENSOR_PRESSURE_SAMPLE 1000
/* Total number of samples. */
#define SENSOR_PRESSURE_COUNT  60
/* Assumed altitude of the device in meters. Required for an accurate reading. */
#define SENSOR_PRESSURE_ALTITUDE 123
/* Write debug info for the sensor. */
#define SENSOR_PRESSURE_DEBUG  false

/********
 * Air Configuration
 *******/

/* Duration of a single sample in ms. */
#define SENSOR_AIR_SAMPLE 1000
/* Total number of samples. */
#define SENSOR_AIR_COUNT  60
/* Write debug info for the sensor. */
#define SENSOR_AIR_DEBUG  false
/* Seconds between individual writes of baseline to EEPROM if enabled. */
#define SENSOR_AIR_BASE_COUNTDOWN 3600

/********
 * Light Configuration
 *******/

/* Duration of a single sample in ms. */
#define SENSOR_LIGHT_SAMPLE 100
/* Total number of samples. */
#define SENSOR_LIGHT_COUNT  600
/* Gain of the light sensor. Use one of the following: AGAIN_1X, AGAIN_4X, AGAIN_16X, AGAIN_64X */
#define SENSOR_LIGHT_GAIN   AGAIN_64X
/* Perform natural log postprocessing on the data before sending it to VariPass. */
#define SENSOR_LIGHT_LOG    true
/* Write debug info for the sensor. */
#define SENSOR_LIGHT_DEBUG  false

/********
 * Magnetic Configuration
 *******/

/* Duration of a single sample in ms. */
#define SENSOR_MAGNETIC_SAMPLE 100
/* Total number of samples. */
#define SENSOR_MAGNETIC_COUNT  600
/* Write debug info for the sensor. */
#define SENSOR_MAGNETIC_DEBUG  false
/* Magnetic calibration values. */
Vector sensorMagneticMin {-103.35, -26.70, -10.05};
Vector sensorMagneticMax {  11.25,  91.35,  87.45};

/********
 * Vibration Configuration
 *******/

/* Range of the accelerometer used for vibration: Use one of the following: */
/* ICM20948_ACCELRANGE_2G, ICM20948_ACCELRANGE_4G, ICM20948_ACCELRANGE_8G, ICM20948_ACCELRANGE_16G */
#define SENSOR_VIBRATION_RANGE ICM20948_ACCELRANGE_2G