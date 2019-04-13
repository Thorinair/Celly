# Celly
Celly is an ESP8266-powered combination of various sensors designed to analyze send environmental measurements to [VariPass](https://varipass.org/). The sensors gather various measurements related to the room and the environment the device is located in:
- Temperature
- Humidity
- Pressure
- CO2 Concentration
- VOC Concentration
- Ambient Light
- Magnetic Magnitude
- Magnetic Inclination
- Vibrations X
- Vibrations Y
- Vibrations Z

The device is able to calibrate its axes according to the gravity and north pole. Pressing and holding the upper button aligns the Z axis upwards, Y towards north and X towards east. Pressing and holding the lower button will calibrate the air quality sensor, and allow for automatic recalibration on every reboot. Holding both buttons will submit a large debug data dump to [Princess Luna](https://github.com/Thorinair/Princess-Luna).

The sensors installed in Celly are the following:
- SHT21
- BMP180
- SGP30-2.5K
- APDS9960
- ICM-20948

Celly was primarily designed to look good AND serve the functionality. Inspired by *My Little Pony: Friendship is Magic*'s character Princess Celestia, the device uses an upper RGB LED to signify the power state, flowing in the colors of her mane. The lower RGB LED signifies whether the device is sending data to VariPass. On successful sends, the LED flashes in the colors of the Mane 6. Both LEDs change their intensity depending on the ambient lighting in order to not be too obtrusive during night time.