#include <Adafruit_NeoPixel.h>

#include <SHT21.h>
#include <SFE_BMP180.h>
#include <Adafruit_SGP30.h>
#include <SparkFun_APDS9960.h>
#include <Adafruit_Sensor.h>
#include <DPEng_ICM20948_AK09916.h>

#include "ESP8266WiFi.h"
#include <EEPROM.h>
#include <VariPass.h>

#include "Shared.h"
#include "TwiFi.h"

#include "Configuration.h"
#include "ConfigurationLED.h"
#include "ConfigurationSensors.h"
#include "ConfigurationVariPass.h"
#include "ConfigurationLuna.h"

#define max(a,b) ((a)>(b)?(a):(b))

#define PIN_PIXEL       13
#define PIN_BUTTON_UP   12
#define PIN_BUTTON_DOWN 14

#define PULSE_DONE 0
#define PULSE_FAIL 1
#define PULSE_ERRO 2
#define PULSE_BUTT 3

#define URL_RESULT_DONE 0
#define URL_RESULT_FAIL 1

#define EEPROM_BASE_ENABLED 10
#define EEPROM_BASE_CO2     11
#define EEPROM_BASE_VOC     13

#define EEPROM_ORI_ENABLED 20
#define EEPROM_ORI_X       21
#define EEPROM_ORI_Y       22
#define EEPROM_ORI_Z       23

/* LEDs */
Adafruit_NeoPixel strip = Adafruit_NeoPixel(2, PIN_PIXEL, NEO_GRB + NEO_KHZ800);
float intensity = 1.0;

struct LEDPower {
	RGB led {ledPowerStopA.r, ledPowerStopA.g, ledPowerStopA.b};
	RGB stepper {0, 0, 0};
	int step = LED_POWER_STEP;
	int counter = 0;
	int div = LED_POWER_DURATION / LED_POWER_STEP / 4;
} ledPower;

/* Sensors */
SHT21 sht;

struct SensorTemperature {
	float raw[SENSOR_TEMPERATURE_COUNT] = {0};
	float last = 0;
	int sample = 0;
	int count = 0;
} sensorTemperature;

struct SensorHumidity {
	float raw[SENSOR_HUMIDITY_COUNT] = {0};
	float last = 0;
	int sample = 0;
	int count = 0;
} sensorHumidity;

SFE_BMP180 bmp;

struct SensorPressure {
	bool available = false;
	float raw[SENSOR_PRESSURE_COUNT] = {0};
	float last = 0;
	int sample = 0;
	int count = 0;
} sensorPressure;

Adafruit_SGP30 sgp;

struct SensorAir {
	bool available = false;
	uint16_t rawCO2[SENSOR_AIR_COUNT] = {0};
	uint16_t rawVOC[SENSOR_AIR_COUNT] = {0};
	uint16_t lastCO2 = 0;
	uint16_t lastVOC = 0;
	bool baseEnabled = false;
	long baseNext = 0;
	uint16_t baseCO2 = 0;
	uint16_t baseVOC = 0;
	int sample = 0;
	int count = 0;
} sensorAir;

SparkFun_APDS9960 apd = SparkFun_APDS9960();

struct SensorLight {
	bool available = false;
	uint16_t raw[SENSOR_LIGHT_COUNT] = {0};
	uint16_t last = 0;
	int sample = 0;
	int count = 0;
} sensorLight;

DPEng_ICM20948 icm = DPEng_ICM20948(0x948A, 0x948B, 0x948C);

struct SensorICM {
	bool available = false;
	bool oriEnabled = false;
	char oriX = 'X';
	char oriY = 'Y';
	char oriZ = 'Z';
} sensorICM;

struct SensorMagnetic {
	float calcMag[SENSOR_MAGNETIC_COUNT] = {0};
	float calcInc[SENSOR_MAGNETIC_COUNT] = {0};
	float lastMag = 0;
	float lastInc = 0;
	float rawX = 0;
	float rawY = 0;
	float rawZ = 0;
	float oriX = 0;
	float oriY = 0;
	float oriZ = 0;
	int sample = 0;
	int count = 0;
} sensorMagnetic;




/* Buttons */

struct ButtonUp {
	bool pressed = false;
	int counter = 0;
} buttonUp;

struct ButtonDown {
	bool pressed = false;
	int counter = 0;
} buttonDown;

struct ButtonBoth {
	bool pressed = false;
	int counter = 0;
} buttonBoth;



void setupPins();
void setupLED();
void setupPreConnect();

void setupSensors();
void setupSensor_sht();
void setupSensor_bmp();
void setupSensor_sgp();
void setupSensor_apd();
void setupSensor_icm();

void processLEDPower();

void processSensors();
void processSensorTemperature();
void processSensorHumidity();
void processSensorPressure();
void processSensorAir();
void processSensorLight();
void processSensorMagnetic();

void processButtons();
void processButtonUp();
void processButtonDown();
void processButtonBoth();

float heading(sensors_event_t * m, sensors_event_t * a, Vector * o);
void vectorCross(Vector * a, Vector * b, Vector * out);
void vectorNormalize(Vector * a);
float vectorDot(Vector * a, Vector * b);
void assignAxes(float * inX, float * inY, float * inZ, float * outX, float * outY, float * outZ);
void calculateIntensity(uint16_t light);
uint32_t getAbsoluteHumidity(float temperature, float humidity);
void ledNotifPulse(int pulse, RGB * color);
int openURL(String url);
void eepromWriteUInt16(int pos, uint16_t val);
uint16_t eepromReadUInt16(int pos);
float maxSix(float a, float b, float c, float d, float e, float f);




void setupPins() {
    pinMode(PIN_BUTTON_UP, INPUT_PULLUP);
    pinMode(PIN_BUTTON_DOWN, INPUT_PULLUP);	
}

void setupLED() {
    strip.begin();
}

void setupPreConnect() {
	delay(PRE_CONNECT_DELAY);
    uint16_t light = 0;
    if (apd.readAmbientLight(light)) {
		calculateIntensity(light);
    } 

    strip.setPixelColor(0, strip.Color(ledPower.led.r * intensity, ledPower.led.g * intensity, ledPower.led.b * intensity));
    strip.show();
}

void setupSensors() {
	setupSensor_sht();
	setupSensor_bmp();
	setupSensor_sgp();
	setupSensor_apd();
	setupSensor_icm();
}

void setupSensor_sht() {
    sht.begin();
}

void setupSensor_bmp() {
	if (bmp.begin())
		sensorPressure.available = true;
	else
		ledNotifPulse(PULSE_ERRO, &ledSensorBMP);
}

void setupSensor_sgp() {
	if (sgp.begin()) {
    	EEPROM.begin(512);
    	sensorAir.baseEnabled = EEPROM.read(EEPROM_BASE_ENABLED);
    	EEPROM.end();

    	if (sensorAir.baseEnabled) {
			sensorAir.baseNext = SENSOR_AIR_BASE_COUNTDOWN * 1000;

    		EEPROM.begin(512);
    		sensorAir.baseCO2 = eepromReadUInt16(EEPROM_BASE_CO2);
    		sensorAir.baseVOC = eepromReadUInt16(EEPROM_BASE_VOC);
    		EEPROM.end();

    		if (!sgp.setIAQBaseline(sensorAir.baseCO2, sensorAir.baseVOC))
				ledNotifPulse(PULSE_ERRO, &ledSensorSGP);
    	}

		sensorAir.available = true;	
	}
	else
		ledNotifPulse(PULSE_ERRO, &ledSensorSGP);
}

void setupSensor_apd() {
    if (apd.init()) {
    	if (apd.enableLightSensor(false)) {
	        if (apd.setAmbientLightGain(SENSOR_LIGHT_GAIN))
				sensorLight.available = true;
		    else
				ledNotifPulse(PULSE_ERRO, &ledSensorAPD);
    	}
	    else
			ledNotifPulse(PULSE_ERRO, &ledSensorAPD);
    }
    else
		ledNotifPulse(PULSE_ERRO, &ledSensorAPD);
    
}

void setupSensor_icm() {
	if(icm.begin(SENSOR_VIBRATION_RANGE, GYRO_RANGE_250DPS)) {
    	EEPROM.begin(512);
    	sensorICM.oriEnabled = EEPROM.read(EEPROM_ORI_ENABLED);
    	EEPROM.end();

    	if (sensorICM.oriEnabled) {
    		EEPROM.begin(512);
    		sensorICM.oriX = (char) EEPROM.read(EEPROM_ORI_X);
    		sensorICM.oriY = (char) EEPROM.read(EEPROM_ORI_Y);
    		sensorICM.oriZ = (char) EEPROM.read(EEPROM_ORI_Z);
    		EEPROM.end();
    	}

		sensorICM.available = true;
	}
	else
		ledNotifPulse(PULSE_ERRO, &ledSensorICMMag);
}




void processLEDPower() {
    if (ledPower.step >= LED_POWER_STEP) {
        ledPower.step = 0;

        if (ledPower.counter == 0) {
            ledPower.stepper.r = (ledPowerStopB.r - ledPower.led.r) / ledPower.div;
            ledPower.stepper.g = (ledPowerStopB.g - ledPower.led.g) / ledPower.div;
            ledPower.stepper.b = (ledPowerStopB.b - ledPower.led.b) / ledPower.div;
        }
        else if (ledPower.counter == ledPower.div) {
            ledPower.stepper.r = (ledPowerStopC.r - ledPower.led.r) / ledPower.div;
            ledPower.stepper.g = (ledPowerStopC.g - ledPower.led.g) / ledPower.div;
            ledPower.stepper.b = (ledPowerStopC.b - ledPower.led.b) / ledPower.div;
        }
        else if (ledPower.counter == ledPower.div * 2) {
            ledPower.stepper.r = (ledPowerStopD.r - ledPower.led.r) / ledPower.div;
            ledPower.stepper.g = (ledPowerStopD.g - ledPower.led.g) / ledPower.div;
            ledPower.stepper.b = (ledPowerStopD.b - ledPower.led.b) / ledPower.div;
        }
        else if (ledPower.counter == ledPower.div * 3) {
            ledPower.stepper.r = (ledPowerStopA.r - ledPower.led.r) / ledPower.div;
            ledPower.stepper.g = (ledPowerStopA.g - ledPower.led.g) / ledPower.div;
            ledPower.stepper.b = (ledPowerStopA.b - ledPower.led.b) / ledPower.div;
        }
        
        if (ledPower.counter >= ledPower.div * 4)
            ledPower.counter = 0;
        else {
            ledPower.counter++;
            ledPower.led.r += ledPower.stepper.r;
            ledPower.led.g += ledPower.stepper.g;
            ledPower.led.b += ledPower.stepper.b;
        }       
        
        if (intensity == -1)
        	strip.setPixelColor(0, strip.Color(ledLowColor.r, ledLowColor.g, ledLowColor.b));
        else
        	strip.setPixelColor(0, strip.Color(ledPower.led.r * intensity, ledPower.led.g * intensity, ledPower.led.b * intensity));
        strip.show();
    }
    else {
        ledPower.step++;
    }
}

void processSensors() {
	processSensorTemperature();
	processSensorHumidity();
	processSensorPressure();
	processSensorAir();
	processSensorLight();
	processSensorMagnetic();
}

void processSensorTemperature() {
	if (sensorTemperature.sample >= SENSOR_TEMPERATURE_SAMPLE - 1) {
		sensorTemperature.sample = 0;
		float val = sht.getTemperature();
		if (SENSOR_TEMPERATURE_DEBUG)
			Serial.println("Temperature: " + String(val));
		sensorTemperature.raw[sensorTemperature.count] = val;
		sensorTemperature.last = val;
		sensorTemperature.count++;
	}
	else {
		sensorTemperature.sample++;
	}

	if (sensorTemperature.count >= SENSOR_TEMPERATURE_COUNT) {
		sensorTemperature.count = 0;
		float avg = 0;
		for (int i = 0; i < SENSOR_TEMPERATURE_COUNT; i++)
			avg += sensorTemperature.raw[i] / SENSOR_TEMPERATURE_COUNT;

        int result;
        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_TEMPERATURE, avg, &result);
        
        if (result == VARIPASS_RESULT_SUCCESS)
            ledNotifPulse(PULSE_DONE, &ledSensorSHT);
        else
            ledNotifPulse(PULSE_FAIL, &ledSensorSHT);
	}
}

void processSensorHumidity() {
	if (sensorHumidity.sample >= SENSOR_HUMIDITY_SAMPLE - 1) {
		sensorHumidity.sample = 0;
		float val = sht.getHumidity();
		if (SENSOR_HUMIDITY_DEBUG)
			Serial.println("Humidity: " + String(val));
		sensorHumidity.raw[sensorHumidity.count] = val;
		sensorHumidity.last = val;
		sensorHumidity.count++;
	}
	else {
		sensorHumidity.sample++;
	}

	if (sensorHumidity.count >= SENSOR_HUMIDITY_COUNT) {
		sensorHumidity.count = 0;
		float avg = 0;
		for (int i = 0; i < SENSOR_HUMIDITY_COUNT; i++)
			avg += sensorHumidity.raw[i] / SENSOR_HUMIDITY_COUNT;

        int result;
        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_HUMIDITY, avg, &result);
        
        if (result == VARIPASS_RESULT_SUCCESS)
            ledNotifPulse(PULSE_DONE, &ledSensorSHT);
        else
            ledNotifPulse(PULSE_FAIL, &ledSensorSHT);
	}
}

void processSensorPressure() {
	if (sensorPressure.available) {
		if (sensorPressure.sample >= SENSOR_PRESSURE_SAMPLE - 1) {
			sensorPressure.sample = 0;

	        char stat;
	        double temp, pres,p0,a;  
			stat = bmp.startTemperature();
	        if (stat != 0) {
	            delay(stat);            
	            stat = bmp.getTemperature(temp);
	            if (stat != 0) {                            
	                stat = bmp.startPressure(3);
	                if (stat != 0) {
	                    delay(stat);                    
	                    stat = bmp.getPressure(pres, temp);
	                    if (stat != 0) {   
	                    	float val = (float) bmp.sealevel(pres, SENSOR_PRESSURE_ALTITUDE);   
							if (SENSOR_PRESSURE_DEBUG)
								Serial.println("Pressure: " + String(val));         
	                        sensorPressure.raw[sensorPressure.count] = val;
							sensorPressure.last = val;
							sensorPressure.count++;
	                    }
	                }
	            }
	        }
		}
		else {
			sensorPressure.sample++;
		}

		if (sensorPressure.count >= SENSOR_PRESSURE_COUNT) {
			sensorPressure.count = 0;
			float avg = 0;
			for (int i = 0; i < SENSOR_PRESSURE_COUNT; i++)
				avg += sensorPressure.raw[i] / SENSOR_PRESSURE_COUNT;

	        int result;
	        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_PRESSURE, avg, &result);
	        
	        if (result == VARIPASS_RESULT_SUCCESS)
	            ledNotifPulse(PULSE_DONE, &ledSensorBMP);
	        else
	            ledNotifPulse(PULSE_FAIL, &ledSensorBMP);
		}
	}
}

void processSensorAir() {
	if (sensorAir.available) {
		if (sensorAir.sample >= SENSOR_AIR_SAMPLE - 1) {
			sensorAir.sample = 0;

	        sgp.setHumidity(getAbsoluteHumidity(sensorTemperature.last, sensorHumidity.last));
	        if (sgp.IAQmeasure()) {

	        	uint16_t val = sgp.eCO2;
				if (SENSOR_AIR_DEBUG)
					Serial.println("CO2: " + String(val));
	        	sensorAir.rawCO2[sensorAir.count] = val;
				sensorAir.lastCO2 = val;

				val = sgp.TVOC;
				if (SENSOR_AIR_DEBUG)
					Serial.println("VOC: " + String(val)); 
	        	sensorAir.rawVOC[sensorAir.count] = val; 
				sensorAir.lastVOC = val;

				sensorAir.count++;				
  			}
		}
		else {
			sensorAir.sample++;
		}

		if (sensorAir.count >= SENSOR_AIR_COUNT) {
			sensorAir.count = 0;

			float avg = 0;
			for (int i = 0; i < SENSOR_AIR_COUNT; i++)
				avg += (float) sensorAir.rawCO2[i] / SENSOR_AIR_COUNT;

	        int result;
	        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_CO2, avg, &result);
	        
	        if (result == VARIPASS_RESULT_SUCCESS)
	            ledNotifPulse(PULSE_DONE, &ledSensorSGP);
	        else
	            ledNotifPulse(PULSE_FAIL, &ledSensorSGP);
			
			avg = 0;
			for (int i = 0; i < SENSOR_AIR_COUNT; i++)
				avg += (float) sensorAir.rawVOC[i] / SENSOR_AIR_COUNT;

	        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_VOC, avg, &result);
	        
	        if (result == VARIPASS_RESULT_SUCCESS)
	            ledNotifPulse(PULSE_DONE, &ledSensorSGP);
	        else
	            ledNotifPulse(PULSE_FAIL, &ledSensorSGP);
		}

		if (sensorAir.baseEnabled) {
			if (sensorAir.baseNext <= 0) {
				sensorAir.baseNext = SENSOR_AIR_BASE_COUNTDOWN * 1000;

    			if (sgp.getIAQBaseline(&sensorAir.baseCO2, &sensorAir.baseVOC)) {
		    		EEPROM.begin(512);
		    		eepromWriteUInt16(EEPROM_BASE_CO2, sensorAir.baseCO2);
		    		eepromWriteUInt16(EEPROM_BASE_VOC, sensorAir.baseVOC);
		    		EEPROM.commit();
		    		EEPROM.end();
    			}
    			else {
					ledNotifPulse(PULSE_ERRO, &ledSensorSGP);
    			}
			}
			else {
				sensorAir.baseNext--;
			}
		}
	}
}

void processSensorLight() {
	if (sensorLight.available) {
		if (sensorLight.sample >= SENSOR_LIGHT_SAMPLE - 1) {
			sensorLight.sample = 0;

			uint16_t val = 0;
	        if (apd.readAmbientLight(val)) {
				if (SENSOR_LIGHT_DEBUG)
					Serial.println("Light: " + String(val));
				sensorLight.raw[sensorLight.count] = val;
				sensorLight.last = val;
	            calculateIntensity(val);
				sensorLight.count++;
	        }
		}
		else {
			sensorLight.sample++;
		}

		if (sensorLight.count >= SENSOR_LIGHT_COUNT) {
			sensorLight.count = 0;
			float avg = 0;
			for (int i = 0; i < SENSOR_LIGHT_COUNT; i++)
				avg += (float) sensorLight.raw[i] / SENSOR_LIGHT_COUNT;

			if (SENSOR_LIGHT_LOG) {
				avg += 1;
				avg = log(avg);
			}

	        int result;
	        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_LIGHT, avg, &result);
	        
	        if (result == VARIPASS_RESULT_SUCCESS)
	            ledNotifPulse(PULSE_DONE, &ledSensorAPD);
	        else
	            ledNotifPulse(PULSE_FAIL, &ledSensorAPD);
		}
	}
}

void processSensorMagnetic() {
	if (sensorICM.available) {
		if (sensorMagnetic.sample >= SENSOR_MAGNETIC_SAMPLE - 1) {
			sensorMagnetic.sample = 0;

			sensors_event_t aevent, gevent, mevent;
			icm.getEvent(&aevent, &gevent, &mevent);

    		sensorMagnetic.rawX = mevent.magnetic.x - (sensorMagneticMin.x + sensorMagneticMax.x) / 2;
    		sensorMagnetic.rawY = -mevent.magnetic.y - (sensorMagneticMin.y + sensorMagneticMax.y) / 2;
    		sensorMagnetic.rawZ = -mevent.magnetic.z - (sensorMagneticMin.z + sensorMagneticMax.z) / 2;

			if (sensorICM.oriEnabled) {
				assignAxes(&sensorMagnetic.rawX, &sensorMagnetic.rawY, &sensorMagnetic.rawZ, &sensorMagnetic.oriX, &sensorMagnetic.oriY, &sensorMagnetic.oriZ);
			}
			else {
				sensorMagnetic.oriX = sensorMagnetic.rawX;
				sensorMagnetic.oriY = sensorMagnetic.rawY;
				sensorMagnetic.oriZ = sensorMagnetic.rawZ;
			}

			sensorMagnetic.lastMag = sqrt(sq(sensorMagnetic.oriX) + sq(sensorMagnetic.oriY) + sq(sensorMagnetic.oriZ));
			sensorMagnetic.calcMag[sensorLight.count] = sensorMagnetic.lastMag;
			sensorMagnetic.lastInc = 90 - (acos(sensorMagnetic.oriZ / sensorMagnetic.lastMag) * (180 / PI));
			sensorMagnetic.calcInc[sensorLight.count] = sensorMagnetic.lastInc;

			sensorMagnetic.count++;
		}
		else {
			sensorMagnetic.sample++;
		}

		if (sensorMagnetic.count >= SENSOR_MAGNETIC_COUNT) {
			sensorMagnetic.count = 0;

			float avg = 0;
			for (int i = 0; i < SENSOR_MAGNETIC_COUNT; i++)
				avg += sensorMagnetic.calcMag[i] / SENSOR_MAGNETIC_COUNT;

	        int result;
	        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_MAGNITUDE, avg, &result);
	        
	        if (result == VARIPASS_RESULT_SUCCESS)
	            ledNotifPulse(PULSE_DONE, &ledSensorICMMag);
	        else
	            ledNotifPulse(PULSE_FAIL, &ledSensorICMMag);

	        avg = 0;
			for (int i = 0; i < SENSOR_MAGNETIC_COUNT; i++)
				avg += sensorMagnetic.calcInc[i] / SENSOR_MAGNETIC_COUNT;

	        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_INCLINATION, avg, &result);
	        
	        if (result == VARIPASS_RESULT_SUCCESS)
	            ledNotifPulse(PULSE_DONE, &ledSensorICMMag);
	        else
	            ledNotifPulse(PULSE_FAIL, &ledSensorICMMag);
		}
	}
}

void processButtons() {
    if (digitalRead(PIN_BUTTON_UP) == LOW && digitalRead(PIN_BUTTON_DOWN) == HIGH) {
    	buttonUp.pressed = true;
    	buttonDown.pressed = false;
		buttonBoth.pressed = false;
		if (buttonDown.counter != 0 || buttonBoth.counter != 0) {
    		buttonDown.counter = 0;
    		buttonBoth.counter = 0;
		    strip.setPixelColor(1, strip.Color(0, 0, 0));
		    strip.show();
		}
    }
    else if (digitalRead(PIN_BUTTON_UP) == HIGH && digitalRead(PIN_BUTTON_DOWN) == LOW) {
    	buttonUp.pressed = false;
    	buttonDown.pressed = true;
		buttonBoth.pressed = false;
		if (buttonUp.counter != 0 || buttonBoth.counter != 0) {
    		buttonUp.counter = 0;
    		buttonBoth.counter = 0;
		    strip.setPixelColor(1, strip.Color(0, 0, 0));
		    strip.show();
		}
    }
    else if (digitalRead(PIN_BUTTON_UP) == LOW && digitalRead(PIN_BUTTON_DOWN) == LOW) {
    	buttonUp.pressed = false;
		buttonDown.pressed = false;
    	buttonBoth.pressed = true;
		if (buttonUp.counter != 0 || buttonDown.counter != 0) {
    		buttonUp.counter = 0;
    		buttonDown.counter = 0;
		    strip.setPixelColor(1, strip.Color(0, 0, 0));
		    strip.show();
		}
    }
    else {
    	buttonUp.pressed = false;
		buttonDown.pressed = false;
		buttonBoth.pressed = false;
		if (buttonUp.counter != 0 || buttonDown.counter != 0 || buttonBoth.counter != 0) {
    		buttonUp.counter = 0;
    		buttonDown.counter = 0;
    		buttonBoth.counter = 0;
		    strip.setPixelColor(1, strip.Color(0, 0, 0));
		    strip.show();
		}
    }


    if (buttonDown.pressed) {
    	if (sensorAir.available) {
	    	if (buttonDown.counter >= BUTTON_HOLD_TIME) {
	    		buttonDown.counter = -1;

	    		processButtonDown();

		        ledNotifPulse(PULSE_BUTT, &ledButtonDown);
	    	}
	    	else if (buttonDown.counter >= 0) {
	    		buttonDown.counter++;
	    		float fade = 0;
	    		if (sensorAir.baseEnabled)
	    			fade = LED_BUTTON_FADE_MIN + (LED_BUTTON_FADE_MAX - LED_BUTTON_FADE_MIN) * ((float)(BUTTON_HOLD_TIME - buttonDown.counter) / BUTTON_HOLD_TIME);
	    		else
	    			fade = LED_BUTTON_FADE_MIN + (LED_BUTTON_FADE_MAX - LED_BUTTON_FADE_MIN) * ((float)buttonDown.counter / BUTTON_HOLD_TIME);
			    strip.setPixelColor(1, strip.Color(ledButtonDown.r * fade, ledButtonDown.g * fade, ledButtonDown.b * fade));
			    strip.show();
	    	}
	    }
    }    

    if (buttonUp.pressed) {
    	if (sensorICM.available) {
	    	if (buttonUp.counter >= BUTTON_HOLD_TIME) {
	    		buttonUp.counter = -1;

	    		processButtonUp();

		        ledNotifPulse(PULSE_BUTT, &ledButtonUp);
	    	}
	    	else if (buttonUp.counter >= 0) {
	    		buttonUp.counter++;
	    		float fade = 0;
	    		if (sensorICM.oriEnabled)
	    			fade = LED_BUTTON_FADE_MIN + (LED_BUTTON_FADE_MAX - LED_BUTTON_FADE_MIN) * ((float)(BUTTON_HOLD_TIME - buttonUp.counter) / BUTTON_HOLD_TIME);
	    		else
	    			fade = LED_BUTTON_FADE_MIN + (LED_BUTTON_FADE_MAX - LED_BUTTON_FADE_MIN) * ((float)buttonUp.counter / BUTTON_HOLD_TIME);
			    strip.setPixelColor(1, strip.Color(ledButtonUp.r * fade, ledButtonUp.g * fade, ledButtonUp.b * fade));
			    strip.show();
	    	}
	    }
    }

    else if (buttonBoth.pressed) {
    	if (buttonBoth.counter >= BUTTON_HOLD_TIME) {
    		buttonBoth.counter = -1;

    		processButtonBoth();

	        ledNotifPulse(PULSE_BUTT, &ledButtonBoth);
    	}
    	else if (buttonBoth.counter >= 0) {
    		buttonBoth.counter++;
    		float fade = LED_BUTTON_FADE_MIN + (LED_BUTTON_FADE_MAX - LED_BUTTON_FADE_MIN) * ((float)buttonBoth.counter / BUTTON_HOLD_TIME);
		    strip.setPixelColor(1, strip.Color(ledButtonBoth.r * fade, ledButtonBoth.g * fade, ledButtonBoth.b * fade));
		    strip.show();
    	}
    }

}

void processButtonUp() {
	if (sensorICM.oriEnabled) {
		sensorICM.oriEnabled = false;
    	EEPROM.begin(512);
    	EEPROM.write(EEPROM_ORI_ENABLED, sensorICM.oriEnabled);
    	EEPROM.commit();
    	EEPROM.end();
	}
	else {
		sensorICM.oriEnabled = true;

		sensors_event_t aevent, gevent, mevent;
		icm.getEvent(&aevent, &gevent, &mevent);

		float acc_X = -aevent.acceleration.x; //  X
		float acc_Y = -aevent.acceleration.y; //  Y
		float acc_Z = -aevent.acceleration.z; //  Z

		float acc_x =  aevent.acceleration.x; // -X
		float acc_y =  aevent.acceleration.y; // -Y
		float acc_z =  aevent.acceleration.z; // -Z

		float accMax = maxSix(acc_X, acc_Y, acc_Z, acc_x, acc_y, acc_z);
		float angle = 0.0;

		if (accMax == acc_X) {
			sensorICM.oriZ = 'x';

    		Vector orient {0, 0, 1};
			angle = 360 - heading(&mevent, &aevent, &orient);

			if (angle >= 315 || angle < 45) {
				sensorICM.oriY = 'z';
				sensorICM.oriX = 'Y';
			}
			else if (angle >= 45 && angle < 135) {
				sensorICM.oriY = 'y';
				sensorICM.oriX = 'z';
			}
			else if (angle >= 135 && angle < 225) {
				sensorICM.oriY = 'Z';
				sensorICM.oriX = 'y';
			}
			else if (angle >= 135 || angle < -135) {
				sensorICM.oriY = 'Y';
				sensorICM.oriX = 'Z';
			}
		}
		else if (accMax == acc_Y) {
			sensorICM.oriZ = 'y';

    		Vector orient {0, 0, 1};
			angle = heading(&mevent, &aevent, &orient);

			if (angle >= 315 || angle < 45) {
				sensorICM.oriY = 'z';
				sensorICM.oriX = 'x';
			}
			else if (angle >= 45 && angle < 135) {
				sensorICM.oriY = 'X';
				sensorICM.oriX = 'z';
			}
			else if (angle >= 135 && angle < 225) {
				sensorICM.oriY = 'Z';
				sensorICM.oriX = 'X';
			}
			else if (angle >= 135 || angle < -135) {
				sensorICM.oriY = 'x';
				sensorICM.oriX = 'Z';
			}
		}
		else if (accMax == acc_Z) {
			sensorICM.oriZ = 'z';

    		Vector orient {0, -1, 0};
			angle = heading(&mevent, &aevent, &orient);
			
			if (angle >= 315 || angle < 45) {
				sensorICM.oriY = 'Y';
				sensorICM.oriX = 'x';
			}
			else if (angle >= 45 && angle < 135) {
				sensorICM.oriY = 'X';
				sensorICM.oriX = 'Y';
			}
			else if (angle >= 135 && angle < 225) {
				sensorICM.oriY = 'y';
				sensorICM.oriX = 'X';
			}
			else if (angle >= 135 || angle < -135) {
				sensorICM.oriY = 'x';
				sensorICM.oriX = 'y';
			}
		}
		else if (accMax == acc_x) {
			sensorICM.oriZ = 'X';

    		Vector orient {0, 0, 1};
			angle = 360 - heading(&mevent, &aevent, &orient);

			if (angle >= 315 || angle < 45) {
				sensorICM.oriY = 'z';
				sensorICM.oriX = 'y';
			}
			else if (angle >= 45 && angle < 135) {
				sensorICM.oriY = 'Y';
				sensorICM.oriX = 'z';
			}
			else if (angle >= 135 && angle < 225) {
				sensorICM.oriY = 'Z';
				sensorICM.oriX = 'Y';
			}
			else if (angle >= 135 || angle < -135) {
				sensorICM.oriY = 'y';
				sensorICM.oriX = 'Z';
			}
		}
		else if (accMax == acc_y) {
			sensorICM.oriZ = 'Y';

    		Vector orient {0, 0, 1};
			angle = heading(&mevent, &aevent, &orient);

			if (angle >= 315 || angle < 45) {
				sensorICM.oriY = 'z';
				sensorICM.oriX = 'X';
			}
			else if (angle >= 45 && angle < 135) {
				sensorICM.oriY = 'x';
				sensorICM.oriX = 'z';
			}
			else if (angle >= 135 && angle < 225) {
				sensorICM.oriY = 'Z';
				sensorICM.oriX = 'x';
			}
			else if (angle >= 135 || angle < -135) {
				sensorICM.oriY = 'X';
				sensorICM.oriX = 'Z';
			}
		}
		else if (accMax == acc_z) {
			sensorICM.oriZ = 'Z';

    		Vector orient {0, -1, 0};
			angle = heading(&mevent, &aevent, &orient);

			if (angle >= 315 || angle < 45) {
				sensorICM.oriY = 'Y';
				sensorICM.oriX = 'X';
			}
			else if (angle >= 45 && angle < 135) {
				sensorICM.oriY = 'x';
				sensorICM.oriX = 'Y';
			}
			else if (angle >= 135 && angle < 225) {
				sensorICM.oriY = 'y';
				sensorICM.oriX = 'x';
			}
			else if (angle >= 135 || angle < -135) {
				sensorICM.oriY = 'X';
				sensorICM.oriX = 'y';
			}
		}

    	EEPROM.begin(512);
    	EEPROM.write(EEPROM_ORI_ENABLED, sensorICM.oriEnabled);
    	EEPROM.write(EEPROM_ORI_X, sensorICM.oriX);
    	EEPROM.write(EEPROM_ORI_Y, sensorICM.oriY);
    	EEPROM.write(EEPROM_ORI_Z, sensorICM.oriZ);
    	EEPROM.commit();
    	EEPROM.end();
	}
}

void processButtonDown() {
	if (sensorAir.baseEnabled) {
		sensorAir.baseEnabled = false;
    	EEPROM.begin(512);
    	EEPROM.write(EEPROM_BASE_ENABLED, sensorAir.baseEnabled);
    	EEPROM.commit();
    	EEPROM.end();		
	}
	else {
		sensorAir.baseEnabled = true;
    	EEPROM.begin(512);
    	EEPROM.write(EEPROM_BASE_ENABLED, sensorAir.baseEnabled);
    	EEPROM.commit();
    	EEPROM.end();

		sensorAir.baseNext = SENSOR_AIR_BASE_COUNTDOWN * 1000;

		if (sgp.getIAQBaseline(&sensorAir.baseCO2, &sensorAir.baseVOC)) {
    		EEPROM.begin(512);
    		eepromWriteUInt16(EEPROM_BASE_CO2, sensorAir.baseCO2);
    		eepromWriteUInt16(EEPROM_BASE_VOC, sensorAir.baseVOC);
    		EEPROM.commit();
    		EEPROM.end();
		}
		else {
			ledNotifPulse(PULSE_ERRO, &ledSensorSGP);
		}	
	}
}

void processButtonBoth() {
    String url = LUNA_URL;
    url += "&key=" + String(LUNA_KEY);
    url += "&temperature_raw=" + String(sensorTemperature.last);
    url += "&humidity_raw=" + String(sensorHumidity.last);
	if (sensorPressure.available) {
	    url += "&pressure_raw=" + String(sensorPressure.last);
	}
	if (sensorAir.available) {
	    url += "&co2_raw=" + String(sensorAir.lastCO2);
	    url += "&voc_raw=" + String(sensorAir.lastVOC);
	    url += "&air_base_enabled=" + String(sensorAir.baseEnabled);
	    if (sensorAir.baseEnabled) {
	    	url += "&air_base_next=" + String(sensorAir.baseNext/1000);
		    url += "&air_base_co2=0x" + String(sensorAir.baseCO2, HEX);
		    url += "&air_base_voc=0x" + String(sensorAir.baseVOC, HEX);
		}
	}
	if (sensorLight.available) {
	    url += "&light_raw=" + String(sensorLight.last);
	    url += "&light_log=" + String(log(sensorLight.last));
	}
	if (sensorICM.available) {
	    url += "&icm_ori_enabled=" + String(sensorICM.oriEnabled);
	    if (sensorICM.oriEnabled) {
	    	if (sensorICM.oriX == 'x')
	    		url += "&icm_ori_x=-X";
	    	else if (sensorICM.oriX == 'y')
	    		url += "&icm_ori_x=-Y";
	    	else if (sensorICM.oriX == 'z')
	    		url += "&icm_ori_x=-Z";
	    	else
	    		url += "&icm_ori_x=" + String(sensorICM.oriX);
	    	if (sensorICM.oriY == 'x')
	    		url += "&icm_ori_y=-X";
	    	else if (sensorICM.oriY == 'y')
	    		url += "&icm_ori_y=-Y";
	    	else if (sensorICM.oriY == 'z')
	    		url += "&icm_ori_y=-Z";
	    	else
	    		url += "&icm_ori_y=" + String(sensorICM.oriY);
	    	if (sensorICM.oriZ == 'x')
	    		url += "&icm_ori_z=-X";
	    	else if (sensorICM.oriZ == 'y')
	    		url += "&icm_ori_z=-Y";
	    	else if (sensorICM.oriZ == 'z')
	    		url += "&icm_ori_z=-Z";
	    	else
	    		url += "&icm_ori_z=" + String(sensorICM.oriZ);
	    }
	    url += "&magnetic_x_raw=" + String(sensorMagnetic.rawX);
	    url += "&magnetic_y_raw=" + String(sensorMagnetic.rawY);
	    url += "&magnetic_z_raw=" + String(sensorMagnetic.rawZ);
	    if (sensorICM.oriEnabled) {
	    	url += "&magnetic_x_ori=" + String(sensorMagnetic.oriX);
	    	url += "&magnetic_y_ori=" + String(sensorMagnetic.oriY);
	    	url += "&magnetic_z_ori=" + String(sensorMagnetic.oriZ);
	    }
	    url += "&magnetic_magnitude=" + String(sensorMagnetic.lastMag);
	    url += "&magnetic_inclination=" + String(sensorMagnetic.lastInc);

	}

    openURL(url);
}




float heading(sensors_event_t * m, sensors_event_t * a, Vector * o) {
    Vector temp {m->magnetic.x, m->magnetic.y, m->magnetic.z};
    Vector mag {m->magnetic.x, m->magnetic.y, m->magnetic.z};
    Vector acc {a->acceleration.x, a->acceleration.y, a->acceleration.z};

    temp.x -= (sensorMagneticMin.x + sensorMagneticMax.x) / 2;
    temp.y -= (sensorMagneticMin.y + sensorMagneticMax.y) / 2;
    temp.z -= (sensorMagneticMin.z + sensorMagneticMax.z) / 2;

    Vector E;
    Vector N;
    vectorCross(&temp, &acc, &E);
    vectorNormalize(&E);
    vectorCross(&acc, &E, &N);
    vectorNormalize(&N);

    float head = atan2(vectorDot(&E, o), vectorDot(&N, o)) * 180 / PI;
    if (head < 0)
    	head += 360;
    return 360 - head;
}

void vectorCross(Vector * a, Vector * b, Vector * out) {
    out->x = (a->y * b->z) - (a->z * b->y);
    out->y = (a->z * b->x) - (a->x * b->z);
    out->z = (a->x * b->y) - (a->y * b->x);
}

void vectorNormalize(Vector * a) {
    float mag = sqrt(vectorDot(a, a));
    a->x /= mag;
    a->y /= mag;
    a->z /= mag;
}

float vectorDot(Vector * a, Vector * b) {
    return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

void assignAxes(float * inX, float * inY, float * inZ, float * outX, float * outY, float * outZ) {
	if (sensorICM.oriX == 'X')
		*outX = *inX;
	else if (sensorICM.oriX == 'x')
		*outX = -*inX;
	else if (sensorICM.oriX == 'Y')
		*outX = *inY;
	else if (sensorICM.oriX == 'y')
		*outX = -*inY;
	else if (sensorICM.oriX == 'Z')
		*outX = *inZ;
	else if (sensorICM.oriX == 'z')
		*outX = -*inZ;

	if (sensorICM.oriY == 'X')
		*outY = *inX;
	else if (sensorICM.oriY == 'x')
		*outY = -*inX;
	else if (sensorICM.oriY == 'Y')
		*outY = *inY;
	else if (sensorICM.oriY == 'y')
		*outY = -*inY;
	else if (sensorICM.oriY == 'Z')
		*outY = *inZ;
	else if (sensorICM.oriY == 'z')
		*outY = -*inZ;

	if (sensorICM.oriZ == 'X')
		*outZ = *inX;
	else if (sensorICM.oriZ == 'x')
		*outZ = -*inX;
	else if (sensorICM.oriZ == 'Y')
		*outZ = *inY;
	else if (sensorICM.oriZ == 'y')
		*outZ = -*inY;
	else if (sensorICM.oriZ == 'Z')
		*outZ = *inZ;
	else if (sensorICM.oriZ == 'z')
		*outZ = -*inZ;
}

void calculateIntensity(uint16_t light) {
	if (light < LED_LIGHT_MIN) {
    	if (LED_LOW_ENABLE)
        	intensity = -1;
    	else
        	intensity = (float) LED_BRIGHT_MIN / 255;
	}
    else if (light > LED_LIGHT_MAX)
        intensity = (float) LED_BRIGHT_MAX / 255;
    else
        intensity = (LED_BRIGHT_MIN + (LED_BRIGHT_MAX - LED_BRIGHT_MIN) * ((float) (light - LED_LIGHT_MIN) / (LED_LIGHT_MAX - LED_LIGHT_MIN))) / 255;
}

uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

void ledNotifPulse(int pulse, RGB * color) {
	if (!buttonUp.pressed && !buttonDown.pressed && !buttonBoth.pressed) {
		if (pulse == PULSE_DONE || pulse == PULSE_FAIL) {
	        if (intensity == -1)
	        	strip.setPixelColor(1, strip.Color(ledLowColor.r, ledLowColor.g, ledLowColor.b));
	        else
		    	strip.setPixelColor(1, strip.Color(color->r * intensity, color->g * intensity, color->b * intensity));
		    strip.show();

			if (pulse == PULSE_DONE)
				delay(LED_NOTIF_PULSE_SEND_DONE);
			else if (pulse == PULSE_FAIL)
				delay(LED_NOTIF_PULSE_SEND_FAIL);

		    strip.setPixelColor(1, strip.Color(0, 0, 0)); 
		    strip.show();  

			delay(LED_NOTIF_PULSE_SEND_DONE);
		}
		else if (pulse == PULSE_ERRO) {
			for (int i = 0; i < 4; i++) {
			    strip.setPixelColor(1, strip.Color(color->r * intensity, color->g * intensity, color->b * intensity));
			    strip.show();
			    delay(LED_NOTIF_PULSE_SENSOR_ERROR);
		    	strip.setPixelColor(1, strip.Color(0, 0, 0)); 
		    	strip.show();  
			    delay(LED_NOTIF_PULSE_SENSOR_ERROR);
			}
		}
	}
	else {
		if (pulse == PULSE_BUTT) {
			for (int i = 0; i < 4; i++) {
			    strip.setPixelColor(1, strip.Color(color->r, color->g, color->b));
			    strip.show();
			    delay(LED_NOTIF_PULSE_BUTTON);
		    	strip.setPixelColor(1, strip.Color(0, 0, 0)); 
		    	strip.show();  
			    delay(LED_NOTIF_PULSE_BUTTON);
			}
		}
	}
}

int openURL(String url) {
    if (LUNA_DEBUG)
        Serial.println("Opening URL: " + String(LUNA_IP) + url);
        
    WiFiClient client;
    if (!client.connect(LUNA_IP, LUNA_PORT)) {  
        if (LUNA_DEBUG)
            Serial.println("Error connecting!");
        return URL_RESULT_FAIL;
    }

    client.print("GET " + url + " HTTP/1.1\r\n" +
                 "Host: " + LUNA_IP + "\r\n" + 
                 "Connection: close\r\n\r\n");
    client.stop();
    
    if (LUNA_DEBUG)
        Serial.println("Connection success.");

    return URL_RESULT_DONE;
}

void eepromWriteUInt16(int pos, uint16_t val) {
    byte* p = (byte*) &val;
    EEPROM.write(pos, *p);
    EEPROM.write(pos + 1, *(p + 1));
    EEPROM.commit();
}

uint16_t eepromReadUInt16(int pos) {
  uint16_t val;
  byte* p = (byte*) &val;
  *p       = EEPROM.read(pos);
  *(p + 1) = EEPROM.read(pos + 1);
  return val;
}

float maxSix(float a, float b, float c, float d, float e, float f) {
	float m = max(a, b);
	m = max(m, c);
	m = max(m, d);
	m = max(m, e);
	return max(m, f);
}



void setup() {
    Serial.begin(115200);

    setupLED();

	if (!LED_COLOR_DEBUG) {
    	setupPins();
    	setupSensors();
    	setupPreConnect();
	}

}

void loop() {
	processLEDPower();

	if (!LED_COLOR_DEBUG) {
		processSensors();
		processButtons();

		if (WiFi.status() != WL_CONNECTED) {
			connectWiFi(true);
	    }
	}
	else {
		strip.setPixelColor(1, strip.Color(ledDebug.r, ledDebug.g, ledDebug.b)); 
		strip.show(); 
	}
	    
	delay(1);
}

