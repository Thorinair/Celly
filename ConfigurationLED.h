/********
 * LED Brightness Configuration
 *******/

/* Minimum and maximum values for the LED brightness in relation to the ambient light.
 * LED_BRIGHT_MIN and LED_BRIGHT_MAX control the minimum and maximum brightness of the LED itself.
 * LED_LIGHT_MIN and LED_LIGHT_MAX control which ambient light level will count as minimum and which as maximum.
 */
#define LED_BRIGHT_MIN 8
#define LED_BRIGHT_MAX 255
#define LED_LIGHT_MIN  1
#define LED_LIGHT_MAX  15000

/* Enable the usage of a special color when the light reaches below the LED_LIGHT_MIN value. */
#define LED_LOW_ENABLE true

/* The color to use for the low light mode. */
RGB ledLowColor {0, 0, 0};



/********
 * Power LED Configuration
 *******/

/* Micro tick count of a single gradient fade step. */
#define LED_POWER_STEP     10

/* Total duration of the gradient fade in seconds. */
#define LED_POWER_DURATION 30

/* Gradient stop definitions. */
RGB ledPowerStopA {32, 128, 255};
RGB ledPowerStopB {192, 32, 128};
RGB ledPowerStopC {32, 32, 255};
RGB ledPowerStopD {32, 160, 64};



/********
 * Notification LED Configuration
 *******/

/* Duration of a successful send pulse. */
#define LED_NOTIF_PULSE_SEND_DONE 50

/* Duration of a failed send pulse. */
#define LED_NOTIF_PULSE_SEND_FAIL 500

/* Duration of individual short sensor error pulses. */
#define LED_NOTIF_PULSE_SENSOR_ERROR 50

/* Duration of individual short button activate pulses. */
#define LED_NOTIF_PULSE_BUTTON 50

/* Notification colors. */
RGB ledNotifWiFiSearch {128, 128, 128};
RGB ledNotifWiFiFail   {192, 0, 0};

RGB ledSensorSHT       {160, 160, 24};
RGB ledSensorBMP       {32, 96, 255};
RGB ledSensorSGP       {192, 64, 0};
RGB ledSensorAPD       {128, 128, 128};
RGB ledSensorICMMag    {96, 16, 255};
RGB ledSensorICMAcc    {160, 32, 48};

RGB ledButtonUp        {96, 16, 255};
RGB ledButtonDown      {192, 64, 0};
RGB ledButtonBoth      {0, 0, 255};

/* Button fade intensities. */
#define LED_BUTTON_FADE_MIN 0.05
#define LED_BUTTON_FADE_MAX 0.6

/* Enable color debugging.
 * The notification LED will keep glowing in the color set below. 
 * No sensor measurement will take place.
 */
#define LED_COLOR_DEBUG false
RGB ledDebug {32, 160, 64};