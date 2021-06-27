// This header holds settings for the ADS1232 library.

// default configurations
#define ADC_Default_SPEED 80
#define ADC_Default_GAIN 1
#define ADC_Default_INPUT 1

// toggle support for features
// choose one of these:
//	#define ADC_supportPolling
	#define ADC_supportInterrupts


#define ADC_max_instances 4

// unused so far
#define TIMING_MARGIN 0.8
#define TIMING_TOLERANCE_MULTIPLIER 10