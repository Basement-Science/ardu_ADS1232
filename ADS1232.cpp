#include "ADS1232.hpp"
#include <Arduino.h>

#define NOP __asm__("nop\n\t")
#define _DO_SERIAL_DEBUG_

uint8_t ADS1232::num_instances = 0;
ADS1232* ADS1232::instances[4] = { NULL, NULL, NULL, NULL };

ADS1232::ADS1232(uint8_t pin_DRDY_DOUT, uint8_t pin_SCLK, uint8_t pin_Speed, uint8_t pin_Gain0, uint8_t pin_Gain1,
uint8_t pin_Temp, uint8_t pin_A0, uint8_t pin_PDWN_inverted) {
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
	if (
		digitalPinToPort(pin_ADC_DRDY_DOUT) != NOT_A_PIN &&
		digitalPinToPort(pin_ADC_SCLK) != NOT_A_PIN &&
		digitalPinToPort(pin_ADC_Speed) != NOT_A_PIN &&
		digitalPinToPort(pin_ADC_Gain0) != NOT_A_PIN &&
		digitalPinToPort(pin_ADC_Gain1) != NOT_A_PIN &&
		digitalPinToPort(pin_ADC_Temp) != NOT_A_PIN &&
		digitalPinToPort(pin_ADC_A0) != NOT_A_PIN &&
		digitalPinToPort(pin_ADC_PDWN_inverted) != NOT_A_PIN)
	{
		#ifdef _DO_SERIAL_DEBUG_
		Serial.println("init ADC");
		#endif /* _DO_SERIAL_DEBUG_ */
		

		// initialize pins
		pinMode(pin_ADC_DRDY_DOUT, INPUT_PULLUP);
		pinMode(pin_ADC_SCLK, OUTPUT);
		pinMode(pin_ADC_Temp, OUTPUT);
		pinMode(pin_ADC_A0, OUTPUT);
		pinMode(pin_ADC_PDWN_inverted, OUTPUT);
		pinMode(pin_ADC_Speed, OUTPUT);
		pinMode(pin_ADC_Gain0, OUTPUT);
		pinMode(pin_ADC_Gain1, OUTPUT);

		// interrupt-pin handling, if specified.
		if (useInterrupt) {
			switch (num_instances) {
			case 0:
				attachInterrupt(digitalPinToInterrupt(pin_ADC_DRDY_DOUT), ISR_0, FALLING);
				instances[0] = this;
				break;
			case 1:
				attachInterrupt(digitalPinToInterrupt(pin_ADC_DRDY_DOUT), ISR_1, FALLING);
				instances[1] = this;
				break;
			case 2:
				attachInterrupt(digitalPinToInterrupt(pin_ADC_DRDY_DOUT), ISR_2, FALLING);
				instances[2] = this;
				break;
			case 3:
				attachInterrupt(digitalPinToInterrupt(pin_ADC_DRDY_DOUT), ISR_3, FALLING);
				instances[3] = this;
				break;
			}
			num_instances++;
		}


		// default settings
		setSpeed_80SPS();
		setGain_x1();
		selectInput_A1();

		RefN_Voltage = RefN_Volt;
		RefP_Voltage = RefP_Volt;

		// allow enough time for PSU voltage to settle.
		while (millis() < 10) { NOP; }
		setReadingsToDiscard(4);
		setNextConversionDuration();
		stopSleep();
	}
	else {
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

	setNextConversionDuration();
	setReadingsToDiscard(1);
}

/*	Sets the frequency at which conversions will take place to 80Hz */
void ADS1232::setSpeed_80SPS() {
	digitalWrite(pin_ADC_Speed, 1);
	adc_ConversionFrequency = 80;
	adc_ConversionPeriod_microSec = PERIOD_80SPS_microSec;
	adc_CalibrationPeriod_microSec = PERIOD_CAL_80SPS_microSec;

	setNextConversionDuration();
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
	digitalWrite(pin_ADC_PDWN_inverted, 0);
	isSleeping = 1;
}

/*	Power up the IC. */
void ADS1232::stopSleep() {
	digitalWrite(pin_ADC_PDWN_inverted, 1);
	isSleeping = 0;
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
	double range = 8388607.0 / ref;
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

/* Calculate the duration of the next conversion based on current settings */
uint32_t ADS1232::getNextConversion_Period_milliSec() {
	return NextConversion_Duration_milliSec;
}


/*	retrieves binary 24-bit reading from the ADC, and store it as a signed 32-bit integer. 
	To be used by internal functions, or in case finished conversions are detected via INTERRUPTs. */
void ADS1232::retrieveResult() {
	digitalWrite(LED_BUILTIN, 1);

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

	setNextConversionDuration();

	if (performCalibration || performCalibrationsContinuously)	{
		/*
		 apply one more SCLK pulse to trigger ADS1232�s offset calibration.
		 this takes extra time. (101ms in fast mode, 801ms in slow mode)
		 Calibration takes effect from the next conversion onwards.
		*/
		digitalWrite(pin_ADC_SCLK, 1);
		digitalWrite(pin_ADC_SCLK, 0);

		if (!performCalibrationsContinuously) {
			performCalibration = 0;
		}
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

	digitalWrite(LED_BUILTIN, 0);
}


/*	determines and sets the duration of the next conversion */
void ADS1232::setNextConversionDuration() {
	if (performCalibration || performCalibrationsContinuously) {
		NextConversion_Duration_milliSec = (adc_ConversionPeriod_microSec + adc_CalibrationPeriod_microSec) / 1000;
	} else {
		NextConversion_Duration_milliSec = adc_ConversionPeriod_microSec / 1000;
	}
}

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