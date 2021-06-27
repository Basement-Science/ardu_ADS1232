#include "ADS1232.hpp"
#include <Arduino.h>

#include "GlobalMacros.h"


#if defined ADC_supportPolling && defined ADC_supportInterrupts
#pragma message "You specified support for both interrupts and polling. \
This will consume additional program memory and is usually pointless."
#elif not defined ADC_supportPolling && not defined ADC_supportInterrupts
#error Please define 'ADC_supportPolling' or 'ADC_supportInterrupts' to use this library.
#endif


#ifdef ADC_supportInterrupts
uint8_t ADS1232::num_instances = 0;
ADS1232* ADS1232::instances[ADC_max_instances] = { NULL, NULL, NULL, NULL };
#endif // ADC_supportInterrupts

ADS1232::ADS1232(uint8_t pin_DRDY_DOUT = NO_PIN, uint8_t pin_SCLK = NO_PIN, uint8_t pin_Speed = NO_PIN, uint8_t pin_Gain0 = NO_PIN, uint8_t pin_Gain1 = NO_PIN,
	uint8_t pin_Temp = NO_PIN, uint8_t pin_A0 = NO_PIN, uint8_t pin_PDWN_inverted = NO_PIN) {
	// set private variables
	pin_ADC_DRDY_DOUT = pin_DRDY_DOUT;
	pin_ADC_SCLK = pin_SCLK;
	pin_ADC_Temp = pin_Temp;
	pin_ADC_A0 = pin_A0;
	pin_ADC_PDWN_inverted = pin_PDWN_inverted;
	pin_ADC_Speed = pin_Speed;
	pin_ADC_Gain0 = pin_Gain0;
	pin_ADC_Gain1 = pin_Gain1;
}

/*	Initialize pins, apply default configuration and power up IC.*/
void ADS1232::init(int32_t RefN_Volt, int32_t RefP_Volt, bool useInterrupt) {
	// Check if the essential Pins are defined. Does NOT check if pin numbers are Valid! (because Arduino...)
	if (
		pin_ADC_DRDY_DOUT != NO_PIN &&
		pin_ADC_SCLK != NO_PIN)
	{

	#ifdef _DO_SERIAL_DEBUG_
		Serial.println("init ADC");
	#endif /* _DO_SERIAL_DEBUG_ */

		RefN_Voltage = RefN_Volt;
		RefP_Voltage = RefP_Volt;

		// initialize essential pins
		pinMode(pin_ADC_DRDY_DOUT, INPUT_PULLUP);
		pinMode(pin_ADC_SCLK, OUTPUT);

	#ifdef ADC_supportInterrupts
		// interrupt-pin handling, if specified.
		if (useInterrupt) {
			static const void(*ISRs[ADC_max_instances])() = { &ISR_0, &ISR_1, &ISR_2, &ISR_3 };

			attachInterrupt(digitalPinToInterrupt(pin_ADC_DRDY_DOUT), ISRs[num_instances], FALLING);
			instances[num_instances] = this;
			num_instances++;
		}
	#endif // ADC_supportInterrupts


		// initialize the remaining pins and functions that are available
		if (pin_ADC_Speed != NO_PIN) {
			pinMode(pin_ADC_Speed, OUTPUT);

			#if ADC_Default_SPEED == 80
				setSpeed_80SPS();
			#elif ADC_Default_SPEED == 10
				setSpeed_10SPS();
			#endif // ADC_Default_SPEED
		}

		if (pin_ADC_Gain0 != NO_PIN) {
			pinMode(pin_ADC_Gain0, OUTPUT);

			#if ADC_Default_GAIN == 1
				setGain_x1();
			#elif ADC_Default_GAIN == 2
				setGain_x2();
			#endif // ADC_Default_GAIN
		} 
		if (pin_ADC_Gain1 != NO_PIN) {
			pinMode(pin_ADC_Gain1, OUTPUT);

			#if ADC_Default_GAIN == 64
				setGain_x64();
			#elif ADC_Default_GAIN == 128
				setGain_x128();
			#endif // ADC_Default_GAIN
		} 
		
		if (pin_ADC_Temp != NO_PIN) {
			pinMode(pin_ADC_Temp, OUTPUT);
		}
		if (pin_ADC_A0 != NO_PIN) {
			pinMode(pin_ADC_A0, OUTPUT);

			#if ADC_Default_INPUT == 1
				selectInput_A1();
			#elif ADC_Default_INPUT == 2
				selectInput_A2();
			#endif // ADC_Default_SPEED
		}

		if (pin_ADC_PDWN_inverted != NO_PIN) {
			pinMode(pin_ADC_PDWN_inverted, OUTPUT);
			// allow enough time for PSU voltage to settle.
			while (millis() < 10) { /*wait*/ }
			setReadingsToDiscard(4);
			stopSleep();
		}
		

	} else {
		// invalid pins have been specified. ADC cannot work.
		#ifdef _DO_SERIAL_DEBUG_
		Serial.println("init ADC FAILED");

		// invalid pin numbers. Stop everything.
		while (1) {
			Serial.println("Fatal Error. ADS1232 was constructed with invalid pin numbers");
			delay(1000);
		}
		#endif /* _DO_SERIAL_DEBUG_ */
	}
}

/*	Sets the minimum amount of invalid readings to come. 
	If skipUnsettledResults is set, these will be skipped. 
	Otherwise this has no effect.		*/
void ADS1232::setReadingsToDiscard(uint8_t readings) {
	readingsToDiscard = max(readingsToDiscard, readings);
}

/*	Sets the frequency at which conversions will take place to 10Hz */
void ADS1232::setSpeed_10SPS() {
	digitalWrite(pin_ADC_Speed, 0);
	adc_ConversionFrequency = 10;
	adc_ConversionPeriod_microSec = PERIOD_10SPS_microSec;
	adc_CalibrationPeriod_microSec = PERIOD_CAL_10SPS_microSec;

	#ifdef ADC_supportPolling
		setNextConversionDuration();
	#endif // ADC_supportPolling

	setReadingsToDiscard(1);
}

/*	Sets the frequency at which conversions will take place to 80Hz */
void ADS1232::setSpeed_80SPS() {
	digitalWrite(pin_ADC_Speed, 1);
	adc_ConversionFrequency = 80;
	adc_ConversionPeriod_microSec = PERIOD_80SPS_microSec;
	adc_CalibrationPeriod_microSec = PERIOD_CAL_80SPS_microSec;

	#ifdef ADC_supportPolling
		setNextConversionDuration();
	#endif // ADC_supportPolling

	setReadingsToDiscard(1);
}

/*	Sets the Analog Gain to x1 */
void ADS1232::setGain_x1() {
	digitalWrite(pin_ADC_Gain0, 0);
	digitalWrite(pin_ADC_Gain1, 0);
	adc_gain = 1;

	setReadingsToDiscard(3);
}

/*	Sets the Analog Gain to x2 */
void ADS1232::setGain_x2() {
	digitalWrite(pin_ADC_Gain0, 1);
	digitalWrite(pin_ADC_Gain1, 0);
	adc_gain = 2;

	setReadingsToDiscard(3);
}

/*	Sets the Analog Gain to x64 
	WARNING: different common-mode input limitations apply. */
void ADS1232::setGain_x64() {
	digitalWrite(pin_ADC_Gain0, 0);
	digitalWrite(pin_ADC_Gain1, 1);
	adc_gain = 64;

	setReadingsToDiscard(3);
}

/*	Sets the Analog Gain to x128
	WARNING: different common-mode input limitations apply. */
void ADS1232::setGain_x128() {
	digitalWrite(pin_ADC_Gain0, 1);
	digitalWrite(pin_ADC_Gain1, 1);
	adc_gain = 128;

	setReadingsToDiscard(3);
}

/*	Selects analog Input Channel 1 */
void ADS1232::selectInput_A1() {
	digitalWrite(pin_ADC_Temp, 0);
	digitalWrite(pin_ADC_A0, 0);
	adc_input = 0;
	setReadingsToDiscard(3);
}

/*	Selects analog Input Channel 2 */
void ADS1232::selectInput_A2() {
	digitalWrite(pin_ADC_Temp, 0);
	digitalWrite(pin_ADC_A0, 1);
	adc_input = 1;
	setReadingsToDiscard(3);
}

/*	Selects the internal temperature sensor as input Channel. */
void ADS1232::selectInput_Temp() {
	digitalWrite(pin_ADC_Temp, 1);
	digitalWrite(pin_ADC_A0, 0);
	adc_input = 3;
	setReadingsToDiscard(3);

	#ifdef _DO_SERIAL_DEBUG_
	if (adc_gain > 2) {
		// gain is set too high. Warn the developer.
		Serial.println("Warning! ADC gain should be set to x1 or x2 for useful temperature readings.");
	}
	#endif /* _DO_SERIAL_DEBUG_ */
}

/*	Power down the IC. */
void ADS1232::startSleep() {
	if (pin_ADC_PDWN_inverted != NO_PIN) {
		digitalWrite(pin_ADC_PDWN_inverted, 0);
		isSleeping = 1;
	}
}

/*	Power up the IC. */
void ADS1232::stopSleep() {
	if (pin_ADC_PDWN_inverted != NO_PIN) {
		digitalWrite(pin_ADC_PDWN_inverted, 1);
		isSleeping = 0;
	}
}

/*	returns the raw binary output data from the last read conversion */
int32_t ADS1232::getLastReading_binary() {
	return lastReading_bin;
}

/*	returns the output data converted to a Voltage reading. 
	To be used with the 2 analog input channels. 
	All Scaling based on Gain, external voltage divider and reference Voltages is handled. */
double ADS1232::getLastReading_voltage() {
	return DataMap(lastReading_bin, RefN_Voltage, RefP_Voltage, ext_divider);
}

/*	returns the output data converted to a Temperature reading in °C. 
	Valid only for a gain of x1 or x2. 
	ADC should be set to TEMP signal source. Otherwise output will not make sense.*/
double ADS1232::getLastReading_temperature_celsius() {
	return TemperatureMap_Celsius(lastReading_bin);
}

/*	returns the output data converted to a Temperature reading in K.
	Valid only for a gain of x1 or x2. 
	ADC should be set to TEMP signal source. Otherwise output will not make sense.*/
double ADS1232::getLastReading_temperature_kelvin() {
	return getLastReading_temperature_celsius() + 273.15;
}

/*	returns the output data converted to a Temperature reading in °F.
	Valid only for a gain of x1 or x2. 
	ADC should be set to TEMP signal source. Otherwise output will not make sense.*/
double ADS1232::getLastReading_temperature_fahrenheit() {
	return ( getLastReading_temperature_celsius() * 1.8 ) + 32;
}

/*	returns how many more readings will be discarded if skipUnsettledResults is set. */
uint8_t ADS1232::getNumberOfUnsettledReadings() {
	return readingsToDiscard;
}

/*	returns whether the last valid conversion was hitting the positive or negative max value.*/
bool ADS1232::isOverrange() {
	return Overrange;
}

// maps binary output to a Voltage. The IC´s Reference Voltages need to be measured and input here.
// To choose the resolution of the output value (i.e. V, mV, µV),
// simply feed in RefN_Voltage and RefP_Voltage scaled accordingly.
double ADS1232::DataMap(int32_t counts, int32_t RefN_Voltage, int32_t RefP_Voltage, double ext_divider) {
	double ref = (RefP_Voltage - RefN_Voltage) / 2.0;
	double range = 8388607.0 / ref;	// would be: 0x7FFFFF IF it was an integer.
	double scale = ext_divider / (double)adc_gain;

	double value = (counts / range) * scale;

	return value;
}

// maps binary output to a Temperature in °C.
// ADC should be set to TEMP signal source. Otherwise output will not make sense.
double ADS1232::TemperatureMap_Celsius(int32_t counts) {
	double microVoltReading = ADS1232::DataMap(counts, 0, 5000000, 1.0);			// convert to voltage
	return ((microVoltReading - 111700.0) / 379.0) + 25.0 + offset_temperature_C;	// convert voltage to °C temperature
}


#ifdef ADC_supportPolling
/*	polls the ADC´s pin 'DRDY_DOUT' until it indicates that a conversion finished.
	Once ready, retrieves the reading and stores it.
	Will abort if this takes longer than expected. (hardware problems etc.) */
void ADS1232::pollADCconversion () {

	#ifdef _DO_SERIAL_DEBUG_
	if (digitalRead(pin_ADC_DRDY_DOUT) == 0) {
		// ADS1232 already has data waiting.
		// there is a chance a conversion will finish during data retrieval.
		// in that case the result will be partly from two conversions, and may be wildly incorrect.
		// This should never happen since pollADCconversion is supposed to run often enough.

		Serial.println("Warning! ADC was not polled in time!");
	} 
	#endif /* _DO_SERIAL_DEBUG_ */


	// Poll until a conversion has finished, retrieve the result.
	// if "waitForSettledResults" is set, 
	// retrieve more readings until a fully settled result exists.
	do {
		uint32_t Time_next = millis() + (getNextConversion_Period_milliSec() / TIMING_MARGIN);

		uint8_t t = 0;
		while (digitalRead(pin_ADC_DRDY_DOUT) != 0) {
			/* Poll until a conversion has finished */

			if (millis() > Time_next) {
				// conversion is taking too long. 
				#ifdef _DO_SERIAL_DEBUG_
				if (t == 0)	{
					t = 1;
					Serial.println("Error: Conversion never finished.");
					Serial.println(millis());
				}
				#endif /* _DO_SERIAL_DEBUG_ */

				break;
			}
		}

		retrieveResult();
	} while (readingsToDiscard && waitForSettledResults);
}
#endif // ADC_supportPolling

#ifdef ADC_supportPolling
/* Calculate the duration of the next conversion based on current settings */
uint32_t ADS1232::getNextConversion_Period_milliSec() {
	return NextConversion_Duration_milliSec;
}
#endif // ADC_supportPolling


/*	retrieves binary 24-bit reading from the ADC, and store it as a signed 32-bit integer. 
	To be used by internal functions, or in case finished conversions are detected via INTERRUPTs. */
void ADS1232::retrieveResult() {
	int32_t output = 0;

	// ADS1232 data transmission begins with the MSB.
	// start with a mask for the 32th bit. Shift it to correct scale later.
	for (uint32_t mask = 0x80000000; mask >= 0x00000100; mask >>= 1) {

		/* timing info:
		 ADS1232/4 can deal with fairly fast serial communication. The longest delays needed are 100ns.
		 Atmega328 @ 16MHz executes an assembler instruction every 62.5ns.
		 However reading/writing single bits with digitalWrite takes multiple
		 assembler instructions each. So Atmega328 @ 16MHz is slow enough without extra delay.
		 Therefore slower Clockspeeds will not be a problem either.
		*/

		// apply SCLK pulse
		digitalWrite(pin_ADC_SCLK, 1);
		digitalWrite(pin_ADC_SCLK, 0);

		// read DOUT pin, which contains the current data bit.
		// Add the value read to the final value.
		output += mask * digitalRead(pin_ADC_DRDY_DOUT);
	}
	output >>= 8;	// shift by 8 bit to get correct 24bit signed number.

	/*
	 apply one more SCLK pulse to make use of the "data ready" (DRDY) feature of ADS1232.
	 pin_ADC_DRDY_DOUT can then be polled to see when a conversion has finished.
	*/
	digitalWrite(pin_ADC_SCLK, 1);
	digitalWrite(pin_ADC_SCLK, 0);

	#ifdef ADC_supportPolling
		setNextConversionDuration();
	#endif // ADC_supportPolling

	if (performCalibration || performCalibrationsContinuously)	{
		/*
		 apply one more SCLK pulse to trigger ADS1232�s offset calibration.
		 this takes extra time. (101ms in fast mode, 801ms in slow mode)
		 Calibration takes effect from the next conversion onwards.
		*/
		digitalWrite(pin_ADC_SCLK, 1);
		digitalWrite(pin_ADC_SCLK, 0);

		// if performCalibrationsContinuously is used, resetting performCalibration doesnt matter
		performCalibration = 0;
	} 

	if (skipUnsettledResults && readingsToDiscard) {
		/* skip setting the reading. The previous value will be returned if requested. */
	} else {
		// set the new reading.
		lastReading_bin = output;
		Overrange = (lastReading_bin >= 8388607 || lastReading_bin <= -8388608);
	}

	// reduce readingsToDiscard if it is not 0 already.
	readingsToDiscard = readingsToDiscard > 0 ? (readingsToDiscard-1) : 0;
}


#ifdef ADC_supportPolling
/*	determines and sets the duration of the next conversion */
void ADS1232::setNextConversionDuration() {
	if (performCalibration || performCalibrationsContinuously) {
		NextConversion_Duration_milliSec = (adc_ConversionPeriod_microSec + adc_CalibrationPeriod_microSec) / 1000;
	} else {
		NextConversion_Duration_milliSec = adc_ConversionPeriod_microSec / 1000;
	}
}
#endif // ADC_supportPolling


#ifdef ADC_supportInterrupts
/*	Since the DRDY_DOUT pin is used to put out data as well,
	it will trigger the interrupt when retrieving.
	Clear the interrupt afterwards to avoid retriggering the interrupt immediately. */
void ADS1232::clearInterrupt() {
	switch (digitalPinToInterrupt(pin_ADC_DRDY_DOUT)) {
	case 0:
		EIFR = 0b00000001;		// write a 1 to INTF0 which clears Interrupt 0´s flag
		break;
	case 1:
		EIFR = 0b00000010;		// write a 1 to INTF1 which clears Interrupt 1´s flag
		break;
	default:
#ifdef _DO_SERIAL_DEBUG_
		Serial.println("Warning: interrupt clearing not supported for this MCU");
#endif /* _DO_SERIAL_DEBUG_ */
		break;
	}
}

/*	ISRs for up to 4 class instances */
static void ADS1232::ISR_0() {
	if (instances[0] != NULL) {
		instances[0]->retrieveResult();
		instances[0]->clearInterrupt();
	}
}
static void ADS1232::ISR_1() {
	if (instances[1] != NULL) {
		instances[1]->retrieveResult();
		instances[1]->clearInterrupt();
	}
}
static void ADS1232::ISR_2() {
	if (instances[2] != NULL) {
		instances[2]->retrieveResult();
		instances[2]->clearInterrupt();
	}
}
static void ADS1232::ISR_3() {
	if (instances[3] != NULL) {
		instances[3]->retrieveResult();
		instances[3]->clearInterrupt();
	}
}
#endif // ADC_supportInterrupts








/////////////////////////////////////
// GARBAGE SECTION.
/////////////////////////////////////

//#include <PinChangeInt.hpp>


//if (digitalRead(pin_ADC_DRDY_DOUT) == 0) {
//	// ADS1232 already has data waiting.
//	// there is a chance a conversion will finish during data retrieval.
//	// in that case the result will be partly from two conversions, and may be wildly incorrect.
//	// This should never happen since pollADCconversion is supposed to run often enough.
//
//#ifdef _DO_SERIAL_DEBUG_
//	Serial.println("Warning! ADC was not polled in time!");
//#endif /* _DO_SERIAL_DEBUG_ */
//
//}
//else {
//#ifdef _ADC_estimatePollRate_
//	// find out how often pollADCconversion is executed.
//	// Do this only if pollADCconversion has been executed often enough.
//	int32_t CurrentMicros = micros();
//	duration_LastIntervalBetweenPolls = CurrentMicros - timeOfLastPoll;
//	timeOfLastPoll = CurrentMicros;
//	duration_MaxIntervalBetweenPolls = max(duration_LastIntervalBetweenPolls, duration_MaxIntervalBetweenPolls);
//	if (duration_MaxIntervalBetweenPolls > adc_ConversionPeriod_microSec * TIMING_MARGIN) {
//#ifdef _DO_SERIAL_DEBUG_
//		Serial.println("Warning! ADC should be polled more often.");
//#endif /* _DO_SERIAL_DEBUG_ */
//
//	}
//
//	// TODO unfinished
//	if ((timeOfNextRetrieval_micros - micros()) < (1 - TIMING_MARGIN) * 1) {
//		/* code */
//	}
//#endif
//}