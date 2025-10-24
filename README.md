# Celly
Celly is an ESP8266-powered combination of various sensors designed to analyze and send environmental measurements to [VariPass](https://varipass.org/). The sensors gather various measurements related to the room and the environment the device is located in:
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

## Build Instructions

Build the project using Arduino IDE using the following settings:

- **ESP8266 Package:** v2.7.4
- **Board:** Generic ESP8266 Module
- **Upload Config:**
    - Upload Speed: *115200*
    - Crystal Frequency: *26 MHz*
    - Debug port: *Disabled*
    - Flash Size: *4MB (FS:1MB OTA:~1019KB)*
    - Exceptions: *Legacy (new can return nullptr)*
    - Flash Frequency: *40MHz*
    - Flash Mode: *DIO*
    - IwIP Variant: *v2 Lower Memory*
    - Builtin Led: *13*
    - Debug Level: *None*
    - Reset Method: *dtr (aka nodemcu)*
    - Espressif FW: *nonos-sdk 2.2.1+100 (190703)*
    - SSL Support: *All SLL ciphers (most compatible)*
    - VTables: *Flash*
    - Erase Flash: *Only Sketch*
    - CPU Frequency: *80 MHz*