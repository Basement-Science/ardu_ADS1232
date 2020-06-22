/*
 * ADS123x.h
 * Created: 03.12.2018 21:34:24
 */
#ifndef ADS1232_H_
#define ADS1232_H_

#include <inttypes.h>
#include <Arduino.h>

#include "GlobalMacros.h"


// ADS1234 currently not supported.

class ADS1232 {
	///////////////////////////////////////////////////
	///////////////////////////////////////////////////
	public:
	/* Constructor. Pass Pin numbers, or 255 (NO_PIN) if this pin should not be used.  
	Special care must be taken when pins are not used. Some functions in this class may malfunction. */
	ADS1232(uint8_t pin_DRDY_DOUT, uint8_t pin_SCLK, uint8_t pin_Speed, uint8_t pin_Gain0, uint8_t pin_Gain1,
	uint8_t pin_Temp, uint8_t pin_A0, uint8_t pin_PDWN_inverted);

	// public functions
	void init(int32_t RefN_Volt, int32_t RefP_Volt, bool useInterrupt);

	void setSpeed_10SPS();
	void setSpeed_80SPS();
	void setGain_x1();
	void setGain_x2();
	void setGain_x64();
	void setGain_x128();
	void selectInput_A1();
	void selectInput_A2();
	void selectInput_Temp();
	void startSleep();
	void stopSleep();

	int32_t getLastReading_binary();
	double getLastReading_voltage();
	double getLastReading_temperature_celsius();
	double getLastReading_temperature_kelvin();
	double getLastReading_temperature_fahrenheit();

	void pollADCconversion();
	bool isOverrange();
	uint8_t getNumberOfUnsettledReadings();
	uint32_t getNextConversion_Period_milliSec();

	// public properties:
	double ext_divider = 1.0f;					// set a ratio of external voltage dividers.
	int16_t offset_temperature_C = 0;			// linear offset for temperature readings in °C. Will also apply to °F and K.

	bool performCalibration = 0;				// set to run a single calibration AFTER the next Conversion.
	bool performCalibrationsContinuously = 0;	// set to run a calibration after every Conversion.
	bool skipUnsettledResults = 0;				// set to keep old Readings until a Settled result was produced.
	bool waitForSettledResults = 0;				// if results are unsettled, Stop everything else until settled.

	// DO NOT USE! FOR INTERNAL INTERRUPT-CONTROL USE ONLY.
	void retrieveResult();
	// DO NOT USE! FOR INTERNAL INTERRUPT-CONTROL USE ONLY.
	void clearInterrupt();



	///////////////////////////////////////////////////
	///////////////////////////////////////////////////
	private:

	// ADS1232�s output pin
	uint8_t pin_ADC_DRDY_DOUT = NO_PIN;

	// ADS1232�s input pins
	uint8_t pin_ADC_SCLK = NO_PIN;
	uint8_t pin_ADC_Temp = NO_PIN;
	uint8_t pin_ADC_A0 = NO_PIN;
	uint8_t pin_ADC_PDWN_inverted = NO_PIN;
	uint8_t pin_ADC_Speed = NO_PIN;
	uint8_t pin_ADC_Gain0 = NO_PIN;
	uint8_t pin_ADC_Gain1 = NO_PIN;

	// internal states:
	volatile int32_t lastReading_bin = 0;
	uint8_t readingsToDiscard = 0;
	bool isSleeping = 1;
	volatile bool Overrange = 0;

	// current settings:
	uint8_t adc_ConversionFrequency = 0;
	uint32_t adc_ConversionPeriod_microSec = 0;
	uint32_t adc_CalibrationPeriod_microSec = 0;
	uint8_t adc_gain = 0;
	uint8_t adc_input = 0;
	int32_t RefN_Voltage = 0;
	int32_t RefP_Voltage = 0;

	uint32_t NextConversion_Duration_milliSec = 0;

	#define PERIOD_10SPS_microSec 100000
	#define PERIOD_80SPS_microSec 12500
	#define PERIOD_CAL_10SPS_microSec 801030
	#define PERIOD_CAL_80SPS_microSec 101290

	#define TIMING_MARGIN 0.8
	#define TIMING_TOLERANCE_MULTIPLIER 10

	// private functions:
	void setReadingsToDiscard(uint8_t readings);
	void setNextConversionDuration();

	// mapping functions: Binary -> human readable
	double DataMap(int32_t counts, int32_t RefN_Voltage, int32_t RefP_Voltage, double ext_divider);
	double TemperatureMap_Celsius(int32_t counts);


	// Interrupt stuff
	static uint8_t num_instances;
	static ADS1232* instances[4];

	static void ISR_0();
	static void ISR_1();
	static void ISR_2();
	static void ISR_3();

	static void retrieveResult_static(uint8_t instance);
};
#endif /* ADS1232_H_ */


/////////////////////////////////////
// GARBAGE SECTION.
/////////////////////////////////////


