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
#include "ConfigurationWiFi.h"

#define max(a,b) ((a) > (b) ? (a) :  (b))
#define abs(x)   ((x) >  0  ? (x) : -(x))

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
bool firstBoot = true;

struct LEDPower {
	RGB led {ledPowerStopA.r, ledPowerStopA.g, ledPowerStopA.b};
	RGB stepper {0, 0, 0};
	int step = LED_POWER_STEP;
	int counter = 0;
	int duration = LED_POWER_DURATION * 1000 / TICK_MICRO_TIME;
	int div = duration / LED_POWER_STEP / 4;
} ledPower;

/* Ticks */
struct Ticks {
	int tickMicroDelay = TICK_MICRO_TIME - TICK_MICRO_TIME_MEASURE;
	int tickSampleCounter = 0;
	int tickUploadCounter = 0;
	int buttonHoldTime = BUTTON_HOLD_TIME / TICK_MICRO_TIME;
	bool firstUpload = true;
} ticks;

/* Sensors */
SHT21 sht;

struct SensorTemperature {
	float last = 0;
} sensorTemperature;

struct SensorHumidity {
	float last = 0;
} sensorHumidity;

SFE_BMP180 bmp;

struct SensorPressure {
	bool available = false;
	float last = 0;
} sensorPressure;

Adafruit_SGP30 sgp;

struct SensorAir {
	bool available = false;
	uint16_t rawCO2[TICK_UPLOAD_COUNT] = {0};
	uint16_t rawVOC[TICK_UPLOAD_COUNT] = {0};
	uint16_t lastCO2 = 0;
	uint16_t lastVOC = 0;
	bool baseEnabled = false;
	long baseNext = 0;
	uint16_t baseCO2 = 0;
	uint16_t baseVOC = 0;
	int count = 0;
} sensorAir;

SparkFun_APDS9960 apd = SparkFun_APDS9960();

struct SensorLight {
	bool available = false;
	uint16_t raw[TICK_UPLOAD_COUNT] = {0};
	uint16_t last = 0;
	int count = 0;
} sensorLight;

DPEng_ICM20948 icm = DPEng_ICM20948(0x948A, 0x948B, 0x948C);

struct SensorICM {
	bool available = false;
	float accX[TICK_SAMPLE_COUNT] = {0};
	float accY[TICK_SAMPLE_COUNT] = {0};
	float accZ[TICK_SAMPLE_COUNT] = {0};
	bool oriEnabled = false;
	char oriX = 'X';
	char oriY = 'Y';
	char oriZ = 'Z';
	int micro = 0;
} sensorICM;

struct SensorMagnetic {
	float rawX[TICK_UPLOAD_COUNT] = {0};
	float rawY[TICK_UPLOAD_COUNT] = {0};
	float rawZ[TICK_UPLOAD_COUNT] = {0};
	float lastX = 0;
	float lastY = 0;
	float lastZ = 0;
	float avgX = 0;
	float avgY = 0;
	float avgZ = 0;
	float lastMag = 0;
	float lastInc = 0;
	int count = 0;
} sensorMagnetic;

struct SensorVibration {
	float lowCut = (2 * PI * ((float)TICK_MICRO_TIME/1000) * SENSOR_VIBRATION_HIGHPASS) / (2 * PI * ((float)TICK_MICRO_TIME/1000) * SENSOR_VIBRATION_HIGHPASS + 1);
	float rawX[TICK_UPLOAD_COUNT] = {0};
	float rawY[TICK_UPLOAD_COUNT] = {0};
	float rawZ[TICK_UPLOAD_COUNT] = {0};
	float lastX = 0;
	float lastY = 0;
	float lastZ = 0;
	float avgX = 0;
	float avgY = 0;
	float avgZ = 0;
	int count = 0;
} sensorVibration;

struct SensorGravity {
	float rawX[TICK_UPLOAD_COUNT] = {0};
	float rawY[TICK_UPLOAD_COUNT] = {0};
	float rawZ[TICK_UPLOAD_COUNT] = {0};
	float lastX = 0;
	float lastY = 0;
	float lastZ = 0;
	float avgX = 0;
	float avgY = 0;
	float avgZ = 0;
	int count = 0;	
} sensorGravity;




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

void processTicks();

void processSensorVibationMicro();

void processSensorAirSample();
void processSensorLightSample();
void processSensorMagneticSample();
void processSensorVibrationSample();
void processSensorGravitySample();

void processSensorTemperatureUpload();
void processSensorHumidityUpload();
void processSensorPressureUpload();
void processSensorAirUpload();
void processSensorLightUpload();
void processSensorMagneticUpload();
void processSensorVibrationUpload();
void processSensorGravityUpload();

void processButtons();
void processButtonUp();
void processButtonDown();
void processButtonBoth();

void processLEDPower();

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
int compare(const void* a, const void* b);



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
			sensorAir.baseNext = SENSOR_AIR_BASE_COUNTDOWN;

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
	if(icm.begin(SENSOR_VIBRATION_RANGE, GYRO_RANGE_250DPS, SENSOR_VIBRATION_LOWPASS)) {
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



void processTicks() {

	/* MICRO TICK. */
	processSensorVibationMicro();

	ticks.tickSampleCounter++;
	if (ticks.tickSampleCounter >= TICK_SAMPLE_COUNT) {
		ticks.tickSampleCounter = 0;

		/* SAMPLE TICK. */
		processSensorAirSample();
		processSensorLightSample();
		processSensorMagneticSample();
		processSensorVibrationSample();
		processSensorGravitySample();

		ticks.tickUploadCounter++;
		if (ticks.tickUploadCounter >= TICK_UPLOAD_COUNT) {
			ticks.tickUploadCounter = 0;

			/* UPLOAD TICK. */
			processSensorTemperatureUpload();
			processSensorHumidityUpload();
			processSensorPressureUpload();
			processSensorAirUpload();
			processSensorLightUpload();
			processSensorMagneticUpload();
			processSensorVibrationUpload();
			processSensorGravityUpload();

			if (ticks.firstUpload)
				ticks.firstUpload = false;
		}
	}
}



void processSensorVibationMicro() {
	if (sensorICM.available) {

		// Measure
		sensors_event_t aevent;
		icm.getEventAcc(&aevent);

		// Save
		sensorICM.accX[sensorICM.micro] = aevent.acceleration.x;
		sensorICM.accY[sensorICM.micro] = aevent.acceleration.y;
		sensorICM.accZ[sensorICM.micro] = aevent.acceleration.z;

		sensorICM.micro++;
	}
}



void processSensorAirSample() {
	// Measure
	if (!ticks.firstUpload)
		sgp.setHumidity(getAbsoluteHumidity(sensorTemperature.last, sensorHumidity.last));

	if (sgp.IAQmeasure()) {

		uint16_t val = sgp.eCO2;
		if (SENSOR_AIR_DEBUG)
			Serial.println("CO2: " + String(val));

		// Save
		sensorAir.rawCO2[sensorAir.count] = val;
		sensorAir.lastCO2 = val;

		val = sgp.TVOC;
		if (SENSOR_AIR_DEBUG)
			Serial.println("VOC: " + String(val)); 

		// Save
		sensorAir.rawVOC[sensorAir.count] = val; 
		sensorAir.lastVOC = val;

		sensorAir.count++;			
	}
}

void processSensorLightSample() {
	if (sensorLight.available) {

		// Measure
		uint16_t val = 0;
        if (apd.readAmbientLight(val)) {
			if (SENSOR_LIGHT_DEBUG)
				Serial.println("Light: " + String(val));

			// Save
			sensorLight.raw[sensorLight.count] = val;
			sensorLight.last = val;

			sensorLight.count++;

            calculateIntensity(val);
        }
	}
}

void processSensorMagneticSample() {
	if (sensorICM.available) {

		// Measure
		sensors_event_t mevent;
		icm.getEventMag(&mevent);

		float valX = mevent.magnetic.x;
		float valY = mevent.magnetic.y;
		float valZ = mevent.magnetic.z;
		if (SENSOR_MAGNETIC_DEBUG)
			Serial.println("Magnetic: " + String(valX) + ", " + String(valY) + ", " + String(valZ));

		// Save
		sensorMagnetic.rawX[sensorMagnetic.count] = valX;
		sensorMagnetic.rawY[sensorMagnetic.count] = valY;
		sensorMagnetic.rawZ[sensorMagnetic.count] = valZ;
		sensorMagnetic.lastX = valX;
		sensorMagnetic.lastY = valY;
		sensorMagnetic.lastZ = valZ;

		sensorMagnetic.count++;
	}
}

void processSensorVibrationSample() {
	if (sensorICM.available) {

		// Process
		float cutX, cutY, cutZ;
		float valX, valY, valZ;
		sensorVibration.rawX[sensorVibration.count] = 0;
		sensorVibration.rawY[sensorVibration.count] = 0;
		sensorVibration.rawZ[sensorVibration.count] = 0;

		for (int i = 0; i < sensorICM.micro; i++) {
			if (i == 0) {
				cutX = sensorICM.accX[i];
				cutY = sensorICM.accY[i];
				cutZ = sensorICM.accZ[i];
			}
			else {
				cutX = cutX * (1 - sensorVibration.lowCut) + sensorICM.accX[i] * sensorVibration.lowCut;
				cutY = cutY * (1 - sensorVibration.lowCut) + sensorICM.accY[i] * sensorVibration.lowCut;
				cutZ = cutZ * (1 - sensorVibration.lowCut) + sensorICM.accZ[i] * sensorVibration.lowCut;
			}

			valX = sensorICM.accX[i] - cutX;
			valY = sensorICM.accY[i] - cutY;
			valZ = sensorICM.accZ[i] - cutZ;

			sensorVibration.rawX[sensorVibration.count] += sq(valX * SENSOR_VIBRATION_GAIN) / sensorICM.micro;
			sensorVibration.rawY[sensorVibration.count] += sq(valY * SENSOR_VIBRATION_GAIN) / sensorICM.micro;
			sensorVibration.rawZ[sensorVibration.count] += sq(valZ * SENSOR_VIBRATION_GAIN) / sensorICM.micro;
		}

		// Save
		sensorVibration.lastX = sensorVibration.rawX[sensorVibration.count];
		sensorVibration.lastY = sensorVibration.rawY[sensorVibration.count];
		sensorVibration.lastZ = sensorVibration.rawZ[sensorVibration.count];
		if (SENSOR_VIBRATION_DEBUG)
			Serial.println("Vibration: " + String(sensorVibration.lastX) + ", " + String(sensorVibration.lastY) + ", " + String(sensorVibration.lastZ));

		sensorVibration.count++;
	}
}

void processSensorGravitySample() {
	if (sensorICM.available) {

		// Process
		float cutX, cutY, cutZ;
		float valX, valY, valZ;
		sensorGravity.rawX[sensorVibration.count] = 0;
		sensorGravity.rawY[sensorVibration.count] = 0;
		sensorGravity.rawZ[sensorVibration.count] = 0;

		for (int i = 0; i < sensorICM.micro; i++) {
			sensorGravity.rawX[sensorGravity.count] += sensorICM.accX[i] / sensorICM.micro;
			sensorGravity.rawY[sensorGravity.count] += sensorICM.accY[i] / sensorICM.micro;
			sensorGravity.rawZ[sensorGravity.count] += sensorICM.accZ[i] / sensorICM.micro;
		}

		// Save
		sensorGravity.lastX = sensorGravity.rawX[sensorGravity.count];
		sensorGravity.lastY = sensorGravity.rawY[sensorGravity.count];
		sensorGravity.lastZ = sensorGravity.rawZ[sensorGravity.count];
		if (SENSOR_GRAVITY_DEBUG)
			Serial.println("Gravity: " + String(sensorGravity.lastX) + ", " + String(sensorGravity.lastY) + ", " + String(sensorGravity.lastZ));

		sensorGravity.count++;
	}

	sensorICM.micro = 0;
}



void processSensorTemperatureUpload() {

	// Measure
	float val = sht.getTemperature();
	if (SENSOR_TEMPERATURE_DEBUG)
		Serial.println("Temperature: " + String(val));

	// Save
	sensorTemperature.last = val;

	// Upload
    int result;
    varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_TEMPERATURE, val, &result, 2);    
    if (result == VARIPASS_RESULT_SUCCESS)
        ledNotifPulse(PULSE_DONE, &ledSensorSHT);
    else
        ledNotifPulse(PULSE_FAIL, &ledSensorSHT);
}

void processSensorHumidityUpload() {

	// Measure
	float val = sht.getHumidity();
	if (SENSOR_HUMIDITY_DEBUG)
		Serial.println("Humidity: " + String(val));

	// Save
	sensorHumidity.last = val;

	// Upload
    int result;
    varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_HUMIDITY, val, &result, 2);        
    if (result == VARIPASS_RESULT_SUCCESS)
        ledNotifPulse(PULSE_DONE, &ledSensorSHT);
    else
        ledNotifPulse(PULSE_FAIL, &ledSensorSHT);
}

void processSensorPressureUpload() {
	if (sensorPressure.available) {

		// Measure
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

						// Save
						sensorPressure.last = val;

						// Upload
						int result;
				        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_PRESSURE, val, &result, 2);				        
				        if (result == VARIPASS_RESULT_SUCCESS)
				            ledNotifPulse(PULSE_DONE, &ledSensorBMP);
				        else
				            ledNotifPulse(PULSE_FAIL, &ledSensorBMP);
                    }
                }
            }
        }
	}
}

void processSensorAirUpload() {
	if (sensorAir.available) {
		if (!ticks.firstUpload) {

			// Process
			float avgCO2 = 0;
			float avgVOC = 0;

			for (int i = 0; i < sensorAir.count; i++)
				avgCO2 += (float) sensorAir.rawCO2[i] / sensorAir.count;		
			for (int i = 0; i < sensorAir.count; i++)
				avgVOC += (float) sensorAir.rawVOC[i] / sensorAir.count;

			// Upload
	        int result;
	        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_CO2, avgCO2, &result, 4);        
	        if (result == VARIPASS_RESULT_SUCCESS)
	            ledNotifPulse(PULSE_DONE, &ledSensorSGP);
	        else
	            ledNotifPulse(PULSE_FAIL, &ledSensorSGP);

	        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_VOC, avgVOC, &result, 4);        
	        if (result == VARIPASS_RESULT_SUCCESS)
	            ledNotifPulse(PULSE_DONE, &ledSensorSGP);
	        else
	            ledNotifPulse(PULSE_FAIL, &ledSensorSGP);

	        // Base
	        if (sensorAir.baseEnabled) {
		        if (sensorAir.baseNext <= 0) {
		        	sensorAir.baseNext = SENSOR_AIR_BASE_COUNTDOWN;

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

		sensorAir.count = 0;
	}
}

void processSensorLightUpload() {
	if (sensorLight.available) {

		// Process
		float avg = 0;
		for (int i = 0; i < sensorLight.count; i++)
			avg += (float) sensorLight.raw[i] / sensorLight.count;

		if (SENSOR_LIGHT_LOG) {
			avg += 1;
			avg = log(avg);
		}

		// Upload
        int result;
        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_LIGHT, avg, &result, 4);        
        if (result == VARIPASS_RESULT_SUCCESS)
            ledNotifPulse(PULSE_DONE, &ledSensorAPD);
        else
            ledNotifPulse(PULSE_FAIL, &ledSensorAPD);

		sensorLight.count = 0;
	}
}

void processSensorMagneticUpload() {
	if (sensorICM.available) {

		// Process
		float avgX = 0;
		float avgY = 0;
		float avgZ = 0;

		for (int i = 0; i < sensorMagnetic.count; i++) {
			avgX += sensorMagnetic.rawX[i] / sensorMagnetic.count;
			avgY += sensorMagnetic.rawY[i] / sensorMagnetic.count;
			avgZ += sensorMagnetic.rawZ[i] / sensorMagnetic.count;
		}

		avgX = avgX - (sensorMagneticMin.x + sensorMagneticMax.x) / 2;
		avgY = -(avgY - (sensorMagneticMin.y + sensorMagneticMax.y) / 2);
		avgZ = -(avgZ - (sensorMagneticMin.z + sensorMagneticMax.z) / 2);

		if (sensorICM.oriEnabled) {
			assignAxes(&avgX, &avgY, &avgZ, &sensorMagnetic.avgX, &sensorMagnetic.avgY, &sensorMagnetic.avgZ);
		}
		else {
			sensorMagnetic.avgX = avgX;
			sensorMagnetic.avgY = avgY;
			sensorMagnetic.avgZ = avgZ;
		}

		float valMag = sqrt(sq(sensorMagnetic.avgX) + sq(sensorMagnetic.avgY) + sq(sensorMagnetic.avgZ));
		float valInc = 90 - (acos(sensorMagnetic.avgZ / valMag) * (180 / PI));

		sensorMagnetic.lastMag = valMag;
		sensorMagnetic.lastInc = valInc;

		// Upload
        int result;
        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_MAGNITUDE, valMag, &result, 4);

		if (result == VARIPASS_RESULT_SUCCESS)
	        ledNotifPulse(PULSE_DONE, &ledSensorICMMag);
        else
            ledNotifPulse(PULSE_FAIL, &ledSensorICMMag);

        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_INCLINATION, valInc, &result, 4);
        
        if (result == VARIPASS_RESULT_SUCCESS)
            ledNotifPulse(PULSE_DONE, &ledSensorICMMag);
        else
            ledNotifPulse(PULSE_FAIL, &ledSensorICMMag);

		sensorMagnetic.count = 0;
	}
}

void processSensorVibrationUpload() {
	if (sensorICM.available) {

		// Process
		float avgX = 0;
		float avgY = 0;
		float avgZ = 0;
		for(int i = 0; i < sensorVibration.count; i++) {
			avgX += sensorVibration.rawX[i] / sensorVibration.count;
			avgY += sensorVibration.rawY[i] / sensorVibration.count;
			avgZ += sensorVibration.rawZ[i] / sensorVibration.count;
		}

		if (sensorICM.oriEnabled) {
			assignAxes(&avgX, &avgY, &avgZ, &sensorVibration.avgX, &sensorVibration.avgY, &sensorVibration.avgZ);
		}
		else {
			sensorVibration.avgX = avgX;
			sensorVibration.avgY = avgY;
			sensorVibration.avgZ = avgZ;
		}

		sensorVibration.avgX = abs(sensorVibration.avgX);
		sensorVibration.avgY = abs(sensorVibration.avgY);
		sensorVibration.avgZ = abs(sensorVibration.avgZ);

		// Upload
        int result;
        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_VIBRATION_X, sensorVibration.avgX, &result, 4);

		if (result == VARIPASS_RESULT_SUCCESS)
	        ledNotifPulse(PULSE_DONE, &ledSensorICMVib);
        else
            ledNotifPulse(PULSE_FAIL, &ledSensorICMVib);

        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_VIBRATION_Y, sensorVibration.avgY, &result, 4);

		if (result == VARIPASS_RESULT_SUCCESS)
	        ledNotifPulse(PULSE_DONE, &ledSensorICMVib);
        else
            ledNotifPulse(PULSE_FAIL, &ledSensorICMVib);

        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_VIBRATION_Z, sensorVibration.avgZ, &result, 4);

		if (result == VARIPASS_RESULT_SUCCESS)
	        ledNotifPulse(PULSE_DONE, &ledSensorICMVib);
        else
            ledNotifPulse(PULSE_FAIL, &ledSensorICMVib);

		sensorVibration.count = 0;
	}
}

void processSensorGravityUpload() {
	if (sensorICM.available) {

		// Process
		float avgX = 0;
		float avgY = 0;
		float avgZ = 0;
		for(int i = 0; i < sensorGravity.count; i++) {
			avgX += sensorGravity.rawX[i] / sensorGravity.count;
			avgY += sensorGravity.rawY[i] / sensorGravity.count;
			avgZ += sensorGravity.rawZ[i] / sensorGravity.count;
		}

		if (sensorICM.oriEnabled) {
			assignAxes(&avgX, &avgY, &avgZ, &sensorGravity.avgX, &sensorGravity.avgY, &sensorGravity.avgZ);
		}
		else {
			sensorGravity.avgX = avgX;
			sensorGravity.avgY = avgY;
			sensorGravity.avgZ = avgZ;
		}

		// Upload
        int result;
        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_GRAVITY_X, sensorGravity.avgX, &result, 4);

		if (result == VARIPASS_RESULT_SUCCESS)
	        ledNotifPulse(PULSE_DONE, &ledSensorICMGrv);
        else
            ledNotifPulse(PULSE_FAIL, &ledSensorICMGrv);

        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_GRAVITY_Y, sensorGravity.avgY, &result, 4);

		if (result == VARIPASS_RESULT_SUCCESS)
	        ledNotifPulse(PULSE_DONE, &ledSensorICMGrv);
        else
            ledNotifPulse(PULSE_FAIL, &ledSensorICMGrv);

        varipassWriteFloat(VARIPASS_KEY, VARIPASS_ID_GRAVITY_Z, sensorGravity.avgZ, &result, 4);

		if (result == VARIPASS_RESULT_SUCCESS)
	        ledNotifPulse(PULSE_DONE, &ledSensorICMGrv);
        else
            ledNotifPulse(PULSE_FAIL, &ledSensorICMGrv);

		sensorGravity.count = 0;
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
	    	if (buttonDown.counter >= ticks.buttonHoldTime) {
	    		buttonDown.counter = -1;

	    		processButtonDown();

		        ledNotifPulse(PULSE_BUTT, &ledButtonDown);
	    	}
	    	else if (buttonDown.counter >= 0) {
	    		buttonDown.counter++;
	    		float fade = 0;
	    		if (sensorAir.baseEnabled)
	    			fade = LED_BUTTON_FADE_MIN + (LED_BUTTON_FADE_MAX - LED_BUTTON_FADE_MIN) * ((float)(ticks.buttonHoldTime - buttonDown.counter) / ticks.buttonHoldTime);
	    		else
	    			fade = LED_BUTTON_FADE_MIN + (LED_BUTTON_FADE_MAX - LED_BUTTON_FADE_MIN) * ((float)buttonDown.counter / ticks.buttonHoldTime);
			    strip.setPixelColor(1, strip.Color(ledButtonDown.r * fade, ledButtonDown.g * fade, ledButtonDown.b * fade));
			    strip.show();
	    	}
	    }
    }    

    if (buttonUp.pressed) {
    	if (sensorICM.available) {
	    	if (buttonUp.counter >= ticks.buttonHoldTime) {
	    		buttonUp.counter = -1;

	    		processButtonUp();

		        ledNotifPulse(PULSE_BUTT, &ledButtonUp);
	    	}
	    	else if (buttonUp.counter >= 0) {
	    		buttonUp.counter++;
	    		float fade = 0;
	    		if (sensorICM.oriEnabled)
	    			fade = LED_BUTTON_FADE_MIN + (LED_BUTTON_FADE_MAX - LED_BUTTON_FADE_MIN) * ((float)(ticks.buttonHoldTime - buttonUp.counter) / ticks.buttonHoldTime);
	    		else
	    			fade = LED_BUTTON_FADE_MIN + (LED_BUTTON_FADE_MAX - LED_BUTTON_FADE_MIN) * ((float)buttonUp.counter / ticks.buttonHoldTime);
			    strip.setPixelColor(1, strip.Color(ledButtonUp.r * fade, ledButtonUp.g * fade, ledButtonUp.b * fade));
			    strip.show();
	    	}
	    }
    }

    else if (buttonBoth.pressed) {
    	if (buttonBoth.counter >= ticks.buttonHoldTime) {
    		buttonBoth.counter = -1;

    		processButtonBoth();

	        ledNotifPulse(PULSE_BUTT, &ledButtonBoth);
    	}
    	else if (buttonBoth.counter >= 0) {
    		buttonBoth.counter++;
    		float fade = LED_BUTTON_FADE_MIN + (LED_BUTTON_FADE_MAX - LED_BUTTON_FADE_MIN) * ((float)buttonBoth.counter / ticks.buttonHoldTime);
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
    String url = String(LUNA_URL_DEBUG) + "&key=" + String(LUNA_KEY);
    String ssid = WiFi.SSID();
    IPAddress ip = WiFi.localIP();
    for (int i = 0; i < ssid.length(); i++) {
    	if (ssid[i] == ' ')
    		ssid[i] = '+';
    }

    url += "&wifi_ssid=" + ssid;
    url += "&wifi_ip=" + String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);
    url += "&first_upload=" + String(ticks.firstUpload);
    url += "&tick_sample=" + String(ticks.tickUploadCounter);
    if (!ticks.firstUpload) {
    	url += "&temperature_raw=" + String(sensorTemperature.last, 2);
    	url += "&humidity_raw=" + String(sensorHumidity.last, 2);
    }
	if (sensorPressure.available && !ticks.firstUpload) {
	    url += "&pressure_raw=" + String(sensorPressure.last, 2);
	}
	if (sensorAir.available) {
	    url += "&co2_raw=" + String(sensorAir.lastCO2, 4);
	    url += "&voc_raw=" + String(sensorAir.lastVOC, 4);
	    url += "&air_base_enabled=" + String(sensorAir.baseEnabled);
	    if (sensorAir.baseEnabled) {
	    	url += "&air_base_next=" + String(sensorAir.baseNext);
		    url += "&air_base_co2=0x" + String(sensorAir.baseCO2, HEX);
		    url += "&air_base_voc=0x" + String(sensorAir.baseVOC, HEX);
		}
	}
	if (sensorLight.available) {
	    url += "&light_raw=" + String(sensorLight.last, 4);
	    url += "&light_log=" + String(log(sensorLight.last), 4);
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
	    url += "&magnetic_x_raw=" + String(sensorMagnetic.lastX, 4);
	    url += "&magnetic_y_raw=" + String(sensorMagnetic.lastY, 4);
	    url += "&magnetic_z_raw=" + String(sensorMagnetic.lastZ, 4);
	    if (!ticks.firstUpload) {
	    	url += "&magnetic_x_avg=" + String(sensorMagnetic.avgX, 4);
	    	url += "&magnetic_y_avg=" + String(sensorMagnetic.avgY, 4);
	    	url += "&magnetic_z_avg=" + String(sensorMagnetic.avgZ, 4);
		    url += "&magnetic_magnitude=" + String(sensorMagnetic.lastMag, 4);
		    url += "&magnetic_inclination=" + String(sensorMagnetic.lastInc, 4);
		}
	    url += "&vibration_x_raw=" + String(sensorVibration.lastX, 4);
	    url += "&vibration_y_raw=" + String(sensorVibration.lastY, 4);
	    url += "&vibration_z_raw=" + String(sensorVibration.lastZ, 4);
	    if (!ticks.firstUpload) {
	    	url += "&vibration_x_avg=" + String(sensorVibration.avgX, 4);
	    	url += "&vibration_y_avg=" + String(sensorVibration.avgY, 4);
	    	url += "&vibration_z_avg=" + String(sensorVibration.avgZ, 4);
		}
	    url += "&gravity_x_raw=" + String(sensorGravity.lastX, 4);
	    url += "&gravity_y_raw=" + String(sensorGravity.lastY, 4);
	    url += "&gravity_z_raw=" + String(sensorGravity.lastZ, 4);
	    if (!ticks.firstUpload) {
	    	url += "&gravity_x_avg=" + String(sensorGravity.avgX, 4);
	    	url += "&gravity_y_avg=" + String(sensorGravity.avgY, 4);
	    	url += "&gravity_z_avg=" + String(sensorGravity.avgZ, 4);
		}
	}

    openURL(url);
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
        
        ledPower.led.r += ledPower.stepper.r;
        ledPower.led.g += ledPower.stepper.g;
        ledPower.led.b += ledPower.stepper.b;

        ledPower.counter++;
        if (ledPower.counter >= ledPower.div * 4)
            ledPower.counter = 0;
        
        if (intensity == -1)
        	strip.setPixelColor(0, strip.Color(ledLowColor.r, ledLowColor.g, ledLowColor.b));
        else
        	strip.setPixelColor(0, strip.Color(ledPower.led.r * intensity, ledPower.led.g * intensity, ledPower.led.b * intensity));
        strip.show();
    }

    ledPower.step++;
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

int compare(const void* a, const void* b) {
     float float_a = * ((float*) a);
     float float_b = * ((float*) b);

     if (float_a == float_b) return 0;
     else if (float_a < float_b) return -1;
     else return 1;
}



void setup() {
    Serial.begin(115200);

    setupLED();

	if (!LED_COLOR_DEBUG) {
    	setupPins();
    	setupSensors();
    	setupPreConnect();
		connectWiFi(true);
		openURL(String(LUNA_URL_BOOT) + "&key=" + String(LUNA_KEY) + "&device=" + String(WIFI_HOST));
	}

}

void loop() {
	if (!LED_COLOR_DEBUG) {
		processTicks();
		processButtons();

		if (WiFi.status() != WL_CONNECTED) {
			connectWiFi(true);
	    }
	}
	else {
		strip.setPixelColor(1, strip.Color(ledDebug.r, ledDebug.g, ledDebug.b)); 
		strip.show(); 
	}

	processLEDPower();
	    
	delay(ticks.tickMicroDelay);
}

