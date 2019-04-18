/********
 * Temperature Configuration
 *******/

/* Write debug info for the sensor. */
#define SENSOR_TEMPERATURE_DEBUG false

/********
 * Humidity Configuration
 *******/

/* Write debug info for the sensor. */
#define SENSOR_HUMIDITY_DEBUG false

/********
 * Pressure Configuration
 *******/

/* Assumed altitude of the device in meters. Required for an accurate reading. */
#define SENSOR_PRESSURE_ALTITUDE 123
/* Write debug info for the sensor. */
#define SENSOR_PRESSURE_DEBUG    false

/********
 * Air Configuration
 *******/

/* Upload ticks between individual writes of baseline to EEPROM if enabled. */
#define SENSOR_AIR_BASE_COUNTDOWN 60
/* Write debug info for the sensor. */
#define SENSOR_AIR_DEBUG          false

/********
 * Light Configuration
 *******/

/* Gain of the light sensor. Use one of the following: 
 * AGAIN_1X
 * AGAIN_4X
 * AGAIN_16X
 * AGAIN_64X 
 */
#define SENSOR_LIGHT_GAIN  AGAIN_64X
/* Perform natural log postprocessing on the data before sending it to VariPass. */
#define SENSOR_LIGHT_LOG   true
/* Write debug info for the sensor. */
#define SENSOR_LIGHT_DEBUG false

/********
 * Magnetic Configuration
 *******/

/* Magnetic calibration values. */
Vector sensorMagneticMin      {-103.35, -26.70, -10.05};
Vector sensorMagneticMax      {  11.25,  91.35,  87.45};
/* Write debug info for the sensor. */
#define SENSOR_MAGNETIC_DEBUG false

/********
 * Vibration Configuration
 *******/

/* Range of the accelerometer used for vibration: Use one of the following:
 * ICM20948_ACCELRANGE_2G
 * ICM20948_ACCELRANGE_4G
 * ICM20948_ACCELRANGE_8G
 * ICM20948_ACCELRANGE_16G
 */
#define SENSOR_VIBRATION_RANGE       ICM20948_ACCELRANGE_2G
/* Range of the accelerometer used for vibration: Use one of the following:
 * ICM20948_ACCELLOWPASS_473_0_HZ
 * ICM20948_ACCELLOWPASS_246_0_HZ
 * ICM20948_ACCELLOWPASS_111_4_HZ
 * ICM20948_ACCELLOWPASS_50_4_HZ
 * ICM20948_ACCELLOWPASS_23_9_HZ
 * ICM20948_ACCELLOWPASS_11_5_HZ
 * ICM20948_ACCELLOWPASS_5_7_HZ
 */
#define SENSOR_VIBRATION_LOWPASS     ICM20948_ACCELLOWPASS_111_4_HZ
/* The frequency value to lowpass the sensor at. */
#define SENSOR_VIBRATION_HIGHPASS    0.1
/* Gain to use for better vibration detection. */
#define SENSOR_VIBRATION_GAIN        10
/* Write debug info for the sensor. */
#define SENSOR_VIBRATION_DEBUG       false