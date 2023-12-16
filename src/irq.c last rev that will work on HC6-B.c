///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// irq.c
//
// heater controller

#include <eZ8.h>
#include <defines.h>
#include <LIMITS.H>
#include "..\\..\\common_controller\\include\\c99types.h"
#include "..\\..\\common_controller\\include\\z082a.h"
#include "..\\..\\common_controller\\include\\timer.h"
#include "..\\..\\common_controller\\include\\adc.h"
#include "..\\..\\common_controller\\include\\uart.h"
#include "..\\..\\common_controller\\include\\irq.h"
#include "config.h"
#include "error.h"
#include "gpio.h"
#define INVERT(b) { if(*b) *b = FALSE; else *b = TRUE; }


// Store big strings in ROM to conserve RData and EData space.
rom char FIRMWARE[]	= R"Aeon Laboratories HC6B2 ";
rom char VERSION[]	= R"V.20220803-0000";

#define SERNO					37

#define TC_CHANNELS				16
#define HTR_CHANNELS			6

#define AIN_TC					1		// TEMP_SENSE signal is on ANA1
#define AIN_CJ					2		// CJTEMP signal is on ANA2

#define DFS						0.99	// digital filter stability


///////////////////////////////////////////////////////
// Calibration
//
// Because it must determine the ADC's input voltage 
// from the ADC count, the code's "gain" values are 
// the reciprocals of their hardware counterparts.
//
///////////////////////////////////////////////////////
// Determining IA_ADC_OFFSET
//
// Check the ADC count with a TC input shorted.
//     This count is the IA_ADC_OFFSET
// (Use the 'z' command to check the ADC count.)
//
// Note that the IA_ADC_OFFSET combines the 
// internal ADC offset and the instrumentation
// amp offset.
//	
///////////////////////////////////////////////////////
// Determining ADC_GAIN_NEG and ADC_GAIN_POS
//
// The nominal ADC Gain (reciprocal) would be
//		2000 / 4096 = 0.4883 mV/count.
// But the true uncompensated value is variable, 
// generally higher than nominal, and somewhat
// nonlinear. A study of twelve example MCUs found
// an average of 0.5823 +/- 0.0234 mV/count. This
// variance is too great to rely on the average, so 
// MCU-specific values are required. Additionally,
// some of the nonlinearity can be corrected by
// using separate gain values for counts below vs
// above the offset.
//
// Start by estimating the ADC gain (use 0.5823 if a
// prior value isn't available).
//
// Get the thermocouple to a known, controlled temperature
// extreme (Tt). Generally, use a Type T thermocouple in
// liquid nitrogen (-195.8 C) to determine ADC_GAIN_NEG,
// and a Type K thermocouple in a furnace controlled 
// to 625 C for ADC_GAIN_POS.
//
// Observe the cold junction temperature (CJt) and the
// reported thermocouple temperature (TCt), and adjust
// the appropriate ADC_GAIN value according to the 
// following equation:
//
//                                   emf(Tt) - emf(CJt)
//   ADC_GAIN_xxx = (prior value) * -------------------
//                                  emf(TCt) - emf(CJt)
//
///////////////////////////////////////////////////////
// Determining CJOffset
//
// The nominal CJOffset value would be
//
//  ~400 / ADC_GAIN_POS = ~683 counts
//
// However, the actual offset varies significantly 
// from nominal, because of device tolerances and 
// especially the internal ADC offset.
//
// Start with an estimated value near 683.
//
// Then adjust each CJOffset by independently
// measuring each true CJ temperature and comparing
// it with the value reported by this program.
// The difference, divided by CJ_GAIN, gives the
// error between the entered CJOffset and the correct
// value
//
//	CJOffset = (prior value) + (CJt_reported - CJt_true) / CJ_GAIN
//
///////////////////////////////////////////////////////
// SERNOs 10 and below were HC6 revision A and
// calibrated differently
///////////////////////////////////////////////////////

// USGS
// the 3 USGS HC6s still need to be upgraded to the B1 revision
#if SERNO == 11
	#define CJ_OFFSET0			535
	#define IA_ADC_OFFSET		360
	#define ADC_GAIN_NEG		0.5989
	#define ADC_GAIN_POS		0.6337
#elif SERNO == 12
	#define CJ_OFFSET0			535
	#define IA_ADC_OFFSET		385
	#define ADC_GAIN_NEG		0.5981
	#define ADC_GAIN_POS		0.5866
#elif SERNO == 13
	#define CJ_OFFSET0			639
	#define CJ_OFFSET1			647
	#define IA_ADC_OFFSET		444
	#define ADC_GAIN_NEG		0.5303
	#define ADC_GAIN_POS		0.5220

// Purdue
#elif SERNO == 14
	#define CJ_OFFSET0			532
	#define IA_ADC_OFFSET		460
	#define ADC_GAIN_NEG		0.5848
	#define ADC_GAIN_POS		0.5806
#elif SERNO == 15
	#define CJ_OFFSET0			572
	#define IA_ADC_OFFSET		460
	#define ADC_GAIN_NEG		0.5949
	#define ADC_GAIN_POS		0.5908
#elif SERNO == 16
	#define CJ_OFFSET0			580
	#define CJ_OFFSET1			544
	#define IA_ADC_OFFSET		445
	#define ADC_GAIN_NEG		0.5800
	#define ADC_GAIN_POS		0.5738

// Aeon
#elif SERNO == 17
	#define CJ_OFFSET0			519
	#define IA_ADC_OFFSET		488
	#define ADC_GAIN_NEG		0.6024
	#define ADC_GAIN_POS		0.5875
#elif SERNO == 18
	#define CJ_OFFSET0			607
	#define IA_ADC_OFFSET		525
	#define ADC_GAIN_NEG		0.5805
	#define ADC_GAIN_POS		0.5656
#elif SERNO == 19
	#define CJ_OFFSET0			543
	#define CJ_OFFSET1			547
	#define IA_ADC_OFFSET		490
	#define ADC_GAIN_NEG		0.5963
	#define ADC_GAIN_POS		0.5893

// Dalhousie
#elif SERNO == 20
	#define CJ_OFFSET0			599
	#define CJ_OFFSET1			577
	#define IA_ADC_OFFSET		454
	#define ADC_GAIN_NEG		0.5871
	#define ADC_GAIN_POS		0.5817
#elif SERNO == 21
	#define CJ_OFFSET0			682
	#define CJ_OFFSET1			675
	#define IA_ADC_OFFSET		485
	#define ADC_GAIN_NEG		0.5608
	#define ADC_GAIN_POS		0.5544
#elif SERNO == 22
	#define CJ_OFFSET0			499
	#define CJ_OFFSET1			564
	#define IA_ADC_OFFSET		464
	#define ADC_GAIN_NEG		0.6015
	#define ADC_GAIN_POS		0.5932
	
// WHOI-CEGS
#elif SERNO == 23
	#define CJ_OFFSET0			673
	#define CJ_OFFSET1			673
	#define IA_ADC_OFFSET		494
	#define ADC_GAIN_NEG		0.5862
	#define ADC_GAIN_POS		0.5794
#elif SERNO == 24
	#define CJ_OFFSET0			599
	#define CJ_OFFSET1			613
	#define IA_ADC_OFFSET		490
	#define ADC_GAIN_NEG		0.5970
	#define ADC_GAIN_POS		0.5922
#elif SERNO == 25
	#define CJ_OFFSET0			650
	#define CJ_OFFSET1			666
	#define IA_ADC_OFFSET		504
	#define ADC_GAIN_NEG		0.5865
	#define ADC_GAIN_POS		0.5796
#endif


///////////////////////////////////////////////////////
// The CJ temperature sensor produces a voltage of
// approximately 400 + 19.5 mV/degC. To convert ADC
// counts to degC, we need (mV/count) / (mV/degC)
#define CJ_GAIN					(ADC_GAIN_POS / 19.5)	// degC/count

#define NO_CJ_SENSOR			ADC_OUTOFRANGE
#ifndef CJ_OFFSET0
	#define CJ_OFFSET0 NO_CJ_SENSOR
#endif
#ifndef CJ_OFFSET1
	#define CJ_OFFSET1 NO_CJ_SENSOR
#endif
	
far int CJOffset[] = {CJ_OFFSET0, CJ_OFFSET1};

///////////////////////////////////////////////////////
// The expected gain of the instrumentation amplifier
// that feeds the ADC. This figure is usually quite 
// accurate.
#define IA_GAIN				(1.0 / 35.65)	// mvIn/mvOut

far float IA_ADC_GAIN[] = 
{ 
	IA_GAIN * ADC_GAIN_NEG, 
	IA_GAIN * ADC_GAIN_POS
};


///////////////////////////////////////////////////////
// The CO_RESERVE provides time for the timer interrupt service
// routines.
// Making CO_RESERVE long enough prevents contention between
// the start and stop routines, at the cost of limiting the maximum
// duty cycle. 
// The following reserves the maximum number of T0 clock periods that 
// still generates a true 99% duty cycle output (when 100% is commanded).
// Must be at least 164 to allow the T0 isr sufficient time to compute 
// the pulse starts (see comment at isr_timer0()).
//
// These values are in units of T0 clock periods.
#define CO_RESERVE				725
#define CO_MAX					(T0_RESET - CO_RESERVE)
#define CO_MIN					0

///////////////////////////////////////////////////////
// Using a random turn-on TRIAC, the average on-time 
// per half-cycle of pulse width is w+0.5, where w is 
// also expressed in half-cycles of line power.
//
//      _           _           _
//     / \         / \         / \
//    /   \       /   \       /
//---/-----\-----/-----\-----/----
//  /       \   /       \   /
//_/ |     | \_/ |     | \_/ |
//   |     |     |     |     |
//   |     |     |     |     |
//    _______ 
// __|       |____________________  s = 0
//        _______
// ______|       |________________  s = c - w
//          _______
// ________|       |______________  s = 1
//
// This sine wave represents an AC line power signal.
// Below it are three possible TRIAC gate pulses.
//
// s is the starting time of the gate pulse, 
// referenced from a zero-crossing.
//
// w is the width of the pulse in half-cycles.
//
// c is the smallest integer > w
//
// The TRIAC turns on at the start of the gate pulse
// and turns off at the first zero-crossing after the 
// end of the pulse. The pulses are not synchronized 
// with the line frequency, so in general the start 
// point varies uniformly throughout the line power
// waveform. The pulses selected above represent 
// bounding conditions where the formula for the
// the on-time changes.
//
// For the first range of pulses, where
//      0 < s <= c - w
// the on-time of the triac is
//      ON(s) = (c - s)
//      ON(0) = c
//      ON(c-w) = (c-(c-w)) = w
// and the mean value for the range is
//      avgON_r1 = (w+c)/2
//
// Where
//      c - w < s <= 1
// the on-time is
//      ON(s) = c + (1 - s)
//      ON(c-w) = c+(1-(c-w)) = w+1
//      ON(1) = ON(0) = c
//      avgON_r2 = (w+c+1)/2
//
// When s uniformly varies over the the line waveform,
// the range sizes provide weights for computing the 
// overall average on-time.
//
// The size of the first range is
//      wt_r1 = (c - w) - 0 = c-w
// The second one is
//      wt_r2 = (1 - (c - w)) = w-c+1
//
// The weighted average is then
//   wt_r1 * avgON_r1 + wt_r2 * avgON_r2
//   -----------------------------------
//             wt_r1 + wt_r2
//
//    (c-w)(w+c)/2 + (w-c+1)(w+c+1)/2
//    -------------------------------
//           (c-w) + (w-c+1)
//
// The bottom simplifies immediately to 1,
// leaving
//
//  (c-w)(w+c) + (w-c+1)(w+c+1)
//  ---------------------------
//              2
//
// Multiplying yields
//
//  cw +c^2 -w^2 -cw +w^2 +cw +w -cw -c^2 -c +w +c +1
//  -------------------------------------------------
//                          2
//
// in which nearly every term cancels, leaving only
//
//       2w + 1
//       ------   =   w + 1/2
//         2
//
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// Effective average output period, given a randomly-timed 
// minimal-width gate pulse (1/2 of a half-cycle)
#define HC_MIN					0.5
// maximum output, also in half-cycles:
#define HC_MAX					(1.0 * HC_FREQ / CO_FREQ)
// minimum ON time as a fraction of maximum ON time
#define MIN_MAX_RATIO			(HC_MIN / HC_MAX)

///////////////////////////////////////////////////////
// The device co units are hundredths of a percent.
#define DEVICE_CO_MAX			10000	// == 100%
// Maximum gate pulse width for minimum output period,
// in device co units:
#define PW_MIN_CO				1125	// INT(DEVICE_CO_MAX * MIN_MAX_RATIO)
// gain for pulse width (converts hundredths of a percent to T0 clock periods):
#define PW_GAIN					(1.0 * CO_MAX / DEVICE_CO_MAX)

#define SP_MIN					-200	// degC
#define SP_MAX					1000	// degC

#define TEMP_MIN				-300	// degC
#define TEMP_MAX				2000	// degC
#define TEMP_ROOM				  20	// degC
#define TEMP_IMPOSSIBLE			-999.9	// degC


///////////////////////////////////////////////////////
#define PC_HTRS					(PWM1 | PWM2 | PWM3 | PWM4)
#define PA_HTRS					(PWM5 | PWM6)
#define HTR_PORT(x) 			(x < 4 ? PCOUT : PAOUT)

#define htr_on(x)	mask_clr(HTR_PORT(x), HTR_BIT[(x)])	// set -PWM low
#define htr_off(x)	mask_set(HTR_PORT(x), HTR_BIT[(x)])	// set -PWM high
#define htrs_off()	mask_set(PCOUT, PC_HTRS), mask_set(PAOUT, PA_HTRS)	// set -PWM high

#define htr_is_on(x)	(HTR_PORT(x) & HTR_BIT[(x)])


///////////////////////////////////////////////////////
typedef struct
{
	char tcType;			// thermocouple type: '~' = None, 'K' = Type K, 'T' = Type T
	float t;				// temperature, in degrees C
	uint16_t error;
} Thermocouple;

///////////////////////////////////////////////////////
typedef struct
{
	char mode;			// operating mode: '0' = off, 'a' = auto, 'm' = manual
	int8_t tc;			// thermocouple channel (-1..15; default is heater array index, -1 means none)
	int sp;				// setpoint (desired pv)
	int co;				// in hundredths of a percent (i.e., 10000 means 100.00%)
	int coLimit;
	uint16_t start;		// T0 count for pulse start
	uint8_t skip;		// number of pulses to skip between delivered pulses
	volatile uint8_t skipped;	// number of pulses skipped since last delivered pulse
	uint8_t * CorA;		// pointer to C or A element of Starts element containing this heater
	
	// PID parameters, for Auto control, tuned by device type
	// To save space, each parameter is stored in the given units, 
	// rounded to nearest integer.
	int Kc;			// Controller gain (stored as 100*Kc)
	int Ci;			// 1 / Ti (to avoid float division, stored as 100000*Ci)
	int Cd;			// Kc * Td = gain times dead time, stored as 10*Cd)
	int Cpr;		// 1 / gp (reciprocal of process gain, stored as 1000*Cpr)
	float priorPv;		// value of PV during last update
	float integral;		// accumulated error
	
	uint16_t error;
} Device;

///////////////////////////////////////////////////////
typedef struct
{
	uint8_t C;			// C-port heaters to turn on
	uint8_t A;			// A-port heaters to turn on
	uint16_t reset;		// T1 (=T0) clocks until next T1 isr
} Start_t;


///////////////////////////////////////////////////////
//
// global constants
//
///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
// Type T thermocouple emf-temperature approximation polynomial coefficients
// from the NIST standard reference table.
//
// for negative emf (mV) to temperature (degC)
far float TC_T_N[] =
{
	8.0,
	0.0, 25.949192, -0.21316967, 0.79018692, 0.42527777,
	0.13304473, 0.020241446, 0.0012668171
};

// for positive emf (mV) to temperature (degC)
far float TC_T_P[] =
{
	7.0,
	0.0, 25.928, -0.7602961, 0.04637791, -0.002165394, 
	0.00006048144, -0.0000007293422
};

// for positive temperature (degC) to emf (mV)
far float TC_T_PT[] =
{
	5.0,
	0.0, 0.38748106364e-1, 0.3329222788e-4, 0.20618243404e-6,
	-0.21882256846e-8
};


///////////////////////////////////////////////////////
// Type K thermocouple emf-temperature approximation polynomial coefficients
// from the NIST standard reference table.
//
// for negative emf to temperature
far float TC_K_N[] =
{
	9.0,
	0.0, 2.5173462e1, -1.1662878, -1.0833638, -8.9773540e-1, 
	-3.7342377e-1, -8.6632643e-2, -1.0450598e-2, -5.1920577e-4
};

// for "low" positive emf (0..20.644 mV) to temperature (0..500 degC)
far float TC_K_L[] =
{
	10.0,
	0.0, 2.5083550e1, 7.8601060e-2, -2.5031310e-1, 8.3152700e-2, 
	-1.2280340e-2, 9.8040360e-4, -4.4130300e-5, 1.0577340e-6, -1.0527550e-8
};

// for "high" positive emf (20.644..54.866 mV) to temperature (500..1372 degC)
far float TC_K_H[] =
{
	7.0,
	-1.3180580e2, 4.8302220e1, -1.6460310e0, 5.4647310e-2, 
	-9.6507150e-4, 8.8021930e-6, -3.1108100e-8
};

// for positive temperature (degC) to emf (mV)
far float TC_K_PT[] =
{
	5.0,
	-0.17600413686e-1, 0.38921204975e-1, 0.18558770032e-4,
	-0.99457592874e-7, 0.31840945719e-9
};



///////////////////////////////////////////////////////
//
// global variables
//
// Note: Some rom may be conserved by not intializing 
// global and static variables, thus using C-defined
// default values, when possible. I.e., never 
// intialize a global or static to 0, and use loops
// to initialize constant data that is repeated.
//
///////////////////////////////////////////////////////

far uint8_t HTR_BIT[HTR_CHANNELS] = { PWM1, PWM2, PWM3, PWM4, PWM5, PWM6 };

volatile uint16_t T0Ticks;					// rolls over when max uint16_t is reached
//volatile far uint16_t elapsed0;				// DEBUGGING
//volatile far uint16_t elapsed;				// DEBUGGING

volatile BOOL EnableControllerUpdate = TRUE;
volatile BOOL EnableDatalogging;

far Device Htr[HTR_CHANNELS];
far Thermocouple TC[TC_CHANNELS];
far float CJt[] = {TEMP_ROOM, TEMP_ROOM};	// thermocouple cold junction temperatures
far int ADC_TC;								// the ADC reading for TC[t_ioch]

#define MAX_STARTS 		HTR_CHANNELS
volatile Start_t Starts[MAX_STARTS];				// co pulse start times, in T0 ticks
volatile Start_t *Start = Starts;					// co pulse start indexer
volatile uint16_t FirstT1Reset;

///////////////////////////////////////////////////////
// Some reference info for confusing declarations with storage specifiers:
// char c;				// c is near (near, by default, is implicit with small memory model)
//
// far char c;			// c is far
// far char *p;			// p is near, points to a far char
//						//   equivalent to (far (char *) p),
//						//   p is a ({near} char *) into far memory
// far char *far p;		// p is far, points to a far char
//						//   equivalent to (far (char * far) p),
// 						//   p is a (char * far) into far memory
//						// "p is a pointer in far memory that points
//						// to a char in far memory"
//
// far char * far p;	// same as the immediately preceding example
// far char* far p;		// same as the immediately preceding example
//
// char *far p;			// p is far, points to a near char
//						//   equivalent to (char * far) p,
// 						//   p is a (char * far) into {near} memory
//						// "p is a pointer in far memory that points
//						// to a char in near memory"
//
// Because storage specifiers may be placed before or after
// the type in a declaration, alternative declarations are also
// possible (e.g., "char far c" is equivalent to "far char c"),
// but these alternatives are no clearer, and perhaps less clear.
///////////////////////////////////////////////////////

uint8_t h_updCh;							// current heater being updated

far uint8_t h_ioch;							// current heater for comms
far Device *far h_io = Htr;

far uint8_t t_ioch;							// current tc channel for comms
far Thermocouple *far t_io = TC;
BOOL EnableSuppression;						// suppress TC input when heater is on

uint8_t DatalogCount;						// counter for Datalogging
far uint8_t DatalogReset = 0xFF;			// report every (this many + 1) seconds
	// Note: 0xFF is used as a disable value (DatalogReset is unsigned)
	// This is convenient because the reset value needs to be one 
	// less than the desired count.	

far uint8_t selectedTCch;
far Thermocouple *far selectedTC;
far char selectedTCType;

far uint8_t selectedCJS;
far float *far selectedCJTemp;
far int selectedCJOffset;

int AdcIn;
int StabilityMeter;							// Performance metric

far float *far polynomialVariable;
far float *far polynomialCoefficients;
far float emf;


//////////////////////////////////////////////////////
//
// prototypes
//
///////////////////////////////////////////////////////
void isr_timer0();
void isr_timer1();
void isr_adc();

///////////////////////////////////////////////////////
// set defaults
void init_irq()
{
	uint8_t i;
	far Device *h = Htr;
	far Thermocouple *tc = TC;

	htrs_off();
	
	for (i = 0; i < TC_CHANNELS; ++i, ++tc)
	{
		tc->tcType = '~';
		tc->t = TEMP_IMPOSSIBLE;
		//tc->error = 0;		// default
	}
	
	for (i = 0; i < HTR_CHANNELS; ++i, ++h)
	{
		h->mode = '0';	// off
		h->tc = i;
		h->coLimit = DEVICE_CO_MAX;
	}

	SET_VECTOR(TIMER0, isr_timer0);
	SET_VECTOR(TIMER1, isr_timer1);
	SET_VECTOR(ADC, isr_adc);

	// start with CJ0 to be checked next 
	TC_select(selectedTCch = TC_CHANNELS-1);
	ADC_SELECT(AIN_CJ);
	selectedCJOffset = CJOffset[0];
	selectedCJTemp = CJt;
	adc_reset();
	
	EI_T0();
	EI_ADC();
}



///////////////////////////////////////////////////////
void preset() { }


///////////////////////////////////////////////////////
BOOL uint8CounterReset(uint8_t *count, uint8_t reset)
{
	if (*count == reset)
	{
		*count = 0;
		return TRUE;
	}
	++*count;
	return FALSE;
}


///////////////////////////////////////////////////////
BOOL uint16CounterReset(uint16_t *count, uint16_t reset)
{
	if (*count == reset)
	{
		*count = 0;
		return TRUE;
	}
	++*count;
	return FALSE;
}


///////////////////////////////////////////////////////
// digital filter
void filter(float *oldV, float *newV)
{ *newV = (DFS) * *oldV + (1.0 - DFS) * *newV; }


///////////////////////////////////////////////////////
// evaluate a polynomial in x with coefficients c[]
float evaluate_polynomial()
{
	float x = *polynomialVariable;
	int8_t i = polynomialCoefficients[0];		// #terms is first value in coefficient array
	float far *coeff = polynomialCoefficients + i;
	float sum = *coeff;
	
	while (--i > 0)
		sum = x * sum + *--coeff;
	return sum;
}


///////////////////////////////////////////////////////
// NIST Polynomial thermocouple emf approximation based on
// thermocouple type and temperature.
// Only positive temperatures are supported in this version.
// Type K deviates from NIST formula, but differs
// by less than 0.0005 mV over range of +15..+35 degC.
float CJ_emf()
{	
	float mV;
	polynomialVariable = selectedCJTemp;
	polynomialCoefficients = (selectedTCType == 'K') ? TC_K_PT : TC_T_PT;
	mV = evaluate_polynomial();
	if (selectedTCType == 'K') mV += *polynomialVariable * 0.0008359 + 0.0138456;
	return mV;
}


///////////////////////////////////////////////////////
// determine the sensed emf, in mV, based on the adc reading
float TC_emf()
{ 
	float dADC = AdcIn - IA_ADC_OFFSET;
	return dADC * IA_ADC_GAIN[dADC > 0];
}

///////////////////////////////////////////////////////
// NIST Polynomial temperature approximation based on
// thermocouple type and emf. 
float TC_temp()
{

	emf = CJ_emf();
	emf += TC_emf();

	polynomialVariable = &emf;	
	if (selectedTCType == 'K')
	{
		if (emf < 0.0)
			polynomialCoefficients = TC_K_N;
		else if (emf < 20.644 )
			polynomialCoefficients = TC_K_L;
		else
			polynomialCoefficients = TC_K_H;
	}
	else					// tcType == 'T'
	{
		if (emf < 0)
			polynomialCoefficients = TC_T_N;
		else
			polynomialCoefficients = TC_T_P;
	}
	
	return evaluate_polynomial();
}


///////////////////////////////////////////////////////
BOOL validTemp(float *t)
{ return (TEMP_MIN <= *t && *t <= TEMP_MAX); }


///////////////////////////////////////////////////////
void update_tc()
{	
	float t;
	float dt;
	float newt = TEMP_IMPOSSIBLE;
	uint16_t error = selectedTC->error;
	
	if (selectedTCch == t_ioch) ADC_TC = AdcIn;
	if (selectedTCType == '~')
		error = 0;
	else if (AdcIn == ADC_OUTOFRANGE)
		mask_set(error, ERROR_ADC);
	else
	{
		mask_clr(error, ERROR_ADC);
		t = TC_temp();
		if (validTemp(&t))
		{
			mask_clr(error, ERROR_PV);
			newt = t;
			t = selectedTC->t;			
			if (validTemp(&t))
			{
				dt = newt - t; if (dt < 0) dt = -dt;
				if (dt < 0.5)		// filter small changes
					filter(&t, &newt);
				else if (dt < 1.0)	// ignore spurious spikes
					newt = t;		// newt = oldt
				// else it's a big change, temp is moving quickly, just accept it
			}
		}
		else
			mask_set(error, ERROR_PV);
	}
	selectedTC->t = newt;
	selectedTC->error = error;
}


////////////////////////////////////////////////////////
// Check the TC mux temperature (cold-junction temperature).
// Which sensor is selected depends on the selected TC channel.
void update_cjt()
{
	float oldt, newt;
	
	if (selectedCJOffset == NO_CJ_SENSOR)
		newt = TEMP_ROOM;
	else if (AdcIn == ADC_OUTOFRANGE)
		newt = TEMP_IMPOSSIBLE;
	else
	{
		oldt = *selectedCJTemp;
		newt = CJ_GAIN * (AdcIn - selectedCJOffset);
		filter(&oldt, &newt);
	}
	*selectedCJTemp = newt;
}


///////////////////////////////////////////////////////
// Whether any heater associated with TC channel tCh
// is receiving a control output pulse.
BOOL aHeaterIsOn(uint8_t tCh)
{
	uint8_t i;
	far Device *h = Htr;
	for (i = 0; i < HTR_CHANNELS; ++i, ++h)
	{
		if (h->tc == tCh && htr_is_on(i))
			return TRUE;			
	}
	return FALSE;
}


///////////////////////////////////////////////////////
void check_adc()
{
	static BOOL cjIsSelected = TRUE;
	uint8_t nextTCch = selectedTCch;
	BOOL suppress = EnableSuppression && aHeaterIsOn(selectedTCch);
	BOOL cjWasSelected = cjIsSelected;
	uint8_t cjChannel;
	int cjOffset;
	char tcType;
	static BOOL connected;
	
	int prior_adc_in = AdcIn;
	int stabilityTest;
	static uint8_t stabilityCounter;
	
	if (AdcdSettling) return;
	AdcIn = ADCD;
	
	if (ADCD_VALID(AdcIn))
	{
		AdcIn >>= 3;
		
		if (connected)		// check for stability
		{			
			stabilityTest = AdcIn - prior_adc_in;
			if (stabilityTest < 0) stabilityTest = -stabilityTest;

			if (stabilityTest > ADC_DELTA_LIMIT)	// significantly different
			{
				stabilityCounter = 0;
			}
			else
			{
				++stabilityCounter;
				++StabilityMeter;
			}
			
			if (stabilityCounter < ADC_STABLE)		// unstable
				return;								// take another reading
			stabilityCounter = 0;
		}
	}
	else
		AdcIn = ADC_OUTOFRANGE;

	// select the next channel or cold junction to measure
	if (selectedTCch == TC_CHANNELS - 1)
	{ 
		if(cjIsSelected)
			nextTCch = 0;
		else
			cjIsSelected = TRUE;
	}
	else
	{
		if (cjIsSelected && selectedTCch == 0)
			cjIsSelected = FALSE;
		else
			++nextTCch;
	}	
	
	cjChannel = (nextTCch > 7);		// FALSE == 0, TRUE == 1
	cjOffset = CJOffset[cjChannel];
	tcType = TC[nextTCch].tcType;
	
	// Now that the next input to be measured is selected 
	// into the ADC, start a new measurement if something
	// is connected to the input.
	if (cjIsSelected)
	{
		connected = cjOffset != NO_CJ_SENSOR;
		ADC_SELECT(AIN_CJ);
		TC_select(nextTCch);
		if(connected)
			adc_reset();
	}
	else
	{
		connected = tcType != '~';
		if (connected)
		{
			TC_select(nextTCch);
			ADC_SELECT(AIN_TC);
			adc_reset();
		}		
	}
	
	// Finally, interpret the voltage just read for
	// the correct device
	if (cjWasSelected)
		update_cjt();
	else
	{
		if (!suppress)
			update_tc();
	}
	
	// Update the selectedX variables to the now-selected channel.
	selectedTCch = nextTCch;
	selectedTC = TC + selectedTCch;
	selectedTCType = tcType;
	selectedCJS = cjChannel;
	selectedCJTemp = CJt + selectedCJS;
	selectedCJOffset = cjOffset;
}


///////////////////////////////////////////////////////
void disableAllOutputs()
{
	DI_T1();
	stop_timer1();
	IRQ_CLEAR_T1();
	htrs_off();	
}


///////////////////////////////////////////////////////
// rebuilds the list of control output Starts
// As many as 507 T0 clock periods may elapse between the start of
// updateStarts() and its completion.
void updateStarts()
{
	far Device *h = Htr;
	uint8_t i, s, j, n = 0;
	Start_t *next;
	Start_t *prior;
	uint16_t reset;
	
	//elapsed0 = T0;
	// Disable all control outputs until the workload 
	// is established
	FirstT1Reset = 0;
	disableAllOutputs();
	
	for (i = 0; i < HTR_CHANNELS; ++i, ++h)
	{	
		if (h->start)			// it needs power
		{
			// find where it goes
			for (s = 0, next = Starts; s < n && next->reset < h->start; s++, next++);
				
			// add or insert a new start element if needed
			if (s == n || next->reset != h->start)
			{
				for (j = n++; j > s; j--)
				{
					next = Starts + j; 
					prior = next - 1;
					next->reset = prior->reset;
				}
				next = Starts + s;
				next->reset = h->start;
				next->C = 0;
				next->A = 0;
			}
		}			
	}

	for (i = 0, h = Htr; i < HTR_CHANNELS; ++i, ++h)
	{	
		if (h->start)
		{
			for (s = 0, next = Starts; s < n && next->reset < h->start; s++, next++);
			
			h->CorA = (uint8_t *)(Starts+s);
			if (i >= 4) h->CorA++;
		}			
	}
	
	prior = next = Starts;
	if (n > 0)
	{
		reset = prior->reset;
		for (i = 1; i < n; i++)
		{
			++next;
			prior->reset = next->reset - prior->reset;
			++prior;
		}
		next->reset = 0;
		
		// Setting FirstT1Reset to non-zero value lets T0 isr re-enable outputs
		FirstT1Reset = reset;	
	}
//	elapsed = T0 - elapsed0;
}


void setStart(far Device *h, uint16_t newStart)
{
	if (h->start != newStart)
	{
		h->start = newStart;
		updateStarts();
	}
}


///////////////////////////////////////////////////////
// Calculate skips and pulse start tick (T0 count) from the device's 
// co value.
void update_output(far Device *h)
{
	uint16_t co = h->co;
	uint8_t skip;
	float s;

	if (h->error || h->mode == '0' || co <= 0)
	{
		setStart(h, h->skip = h->skipped = 0);
	}
	else
	{
		skip = s = PW_MIN_CO / co;
		if (s > 0xFF) skip = 0xFF;

		h->skip = skip;
		setStart(h, T0_RESET - PW_GAIN * co * (skip + 1));
	}
}


///////////////////////////////////////////////////////
void setCO(far Device *h, uint16_t newco)
{
	h->co = newco;
	update_output(h);
}


///////////////////////////////////////////////////////
void pid(far Device *h, float *pv)
{
	// convert PID parameters from stored units to floats
	float Kc = 0.01 * h->Kc;		// Kc = controller gain
	float Ci = 0.00001 * h->Ci;		// Ci = 1 / Ti (note: no Kc)
	float Cd = 0.1 * h->Cd;			// Cd = Kc * Td
	float Cpr = 0.001 * h->Cpr;		// Cpr = 1 / gp  (= step test dCO/dPV)

	float ref = h->sp;				// ref is momentarily setpoint
	
	float co = Kc * (ref - *pv);	// the p term
	
	// The following line should not be executed
	// on the first pass (when entering auto mode).
	// Alternatively, h->priorPv could be set to *pv
	// before the first call to pid().
	co += Cd * (h->priorPv - *pv);	// plus the d term

	// Ideally, the integral term always equals the co that
	// corresponds to the current pv.
	// It is set to -1 when entering auto mode, 
	// to trigger a preset.
	if (co > h->coLimit || h->integral < 0.0)
	{
		if (ref < TEMP_ROOM)		// if sp < room temperature
			ref = -195.8;			// assume plant is in LN
		else
			ref = TEMP_ROOM;		// room-temperature plant
		h->integral = Cpr * (*pv - ref);
	}
	else
		h->integral += Ci * co;		// Note: Kc is in co; Ci should not include it
	
	co += h->integral;
	if (co < 0.0) co = 0.0;
	if (co > h->coLimit) co = h->coLimit;
	h->priorPv = *pv;
	setCO(h, co);
}


///////////////////////////////////////////////////////
void update_device(far Device *h)
{
	far Thermocouple *tc = TC + h->tc;	// valid only when h->tc > 0
	float t = tc->t;					// valid only when h->tc > 0
	uint16_t error = h->error;
		
	if (tc->error)
		mask_set(error, ERROR_PV);
	else
		mask_clr(error, ERROR_PV);
	
	if (h->mode == 'a')
	{
		if (h->tc < 0 || tc->tcType == '~')
			mask_set(error, ERROR_NO_TC);
		if (!error)
			pid(h, &t);
	}
	else
		mask_clr(error, ERROR_NO_TC);
	h->error = error;
}


///////////////////////////////////////////////////////
void update_controller()
{
	check_adc();	
	if (EnableControllerUpdate)
	{
		EnableControllerUpdate = FALSE;		// re-enabled later by isr_timer0
		update_device(Htr + h_updCh);
		uint8CounterReset(&h_updCh, HTR_CHANNELS-1);
	}
}


////////////////////////////////////////////////////////
void selectHeater()
{
	h_ioch = TryInput(0, HTR_CHANNELS - 1, ERROR_CHANNEL, h_ioch, 0);
	h_io = Htr + h_ioch;
}


////////////////////////////////////////////////////////
void selectThermocouple()
{
	t_ioch = TryInput(0, TC_CHANNELS-1, ERROR_TC_CH, t_ioch, 0);
	t_io = TC + t_ioch;
}


////////////////////////////////////////////////////////
void reportHeaterState(far Device *h)
{
	putc(h->mode); printSpace();
	printdec(h->co, 6, ' ', 2); printSpace();
}


////////////////////////////////////////////////////////
void reportHeater()
{
	printi(h_ioch, 1, ' '); printSpace();
	printi(h_io->sp, 4, ' '); printSpace();
	reportHeaterState(h_io);
	printdec(h_io->coLimit, 6, ' ', 2); printSpace();
	printi(h_io->tc, 2, ' '); printSpace();
	printi(h_io->Kc, 6, ' '); printSpace();
	printi(h_io->Ci, 6, ' '); printSpace();
	printi(h_io->Cd, 6, ' '); printSpace();
	printi(h_io->Cpr, 6, ' '); printSpace();
	printi(Error | h_io->error, 5, ' ');
	endMessage();
}


////////////////////////////////////////////////////////
void reportTemperature(far float *t)
{
	printdec(*t * 10.0, 6, ' ', 1); printSpace();
}


////////////////////////////////////////////////////////
void reportTC()
{
	printi(t_ioch, 2, ' '); printSpace();
	putc(t_io->tcType); printSpace();
	reportTemperature(&(t_io->t));
	printi(Error | t_io->error, 5, ' ');
	endMessage();
}


////////////////////////////////////////////////////////
void reportDataDump()
{
	uint8_t i;
	
	// report all temperatures
	for (i = 0; i < TC_CHANNELS; ++i)
		reportTemperature(&(TC[i].t));
	endLine();

	// report all heater states
	for (i = 0; i < HTR_CHANNELS; ++i)
		reportHeaterState(Htr+i);
	endLine();
	
	// report cold junction temperatures
	reportTemperature(CJt);
	reportTemperature(CJt+1);
	endLine();
	printi(StabilityMeter, 6, ' ');
	endMessage();
}


///////////////////////////////////////////////////////
void do_commands()
{
	char c, c2;
	int n;

	while (!RxbEmpty())					// process a command
	{
		mask_clr(Error, ERROR_COMMAND);
		GetInput();
		c = Command[0];					// a command
		c2 = Command[1];				// possibly a sub-command

		// the alternative switch() construction consumes ~74 bytes more codespace
		if (c == '\0')					// null command
		{
			// do nothing
		}
		else if (c == 'r')				// report
		{
			if (NargPresent)			// set Datalogging interval
			{
				// rolls under to 0xFF if DatalogReset was 0
				DatalogReset = TryInput(0, 255, ERROR_DATALOG, DatalogReset + 1, 0) - 1;
				DatalogCount = 0;
			}
			else						// one-time report
			{
				reportDataDump();
			}
		}
		else if (c == '0')				// turn heater Off
		{
			h_io->mode = c;
			update_output(h_io);
		}
		else if (c == 's')				// set setpoint
		{
			h_io->sp = TryInput(SP_MIN, SP_MAX, ERROR_SETPOINT, h_io->sp, 0);
		}
		else if (c == 'h')				// heater report
		{
			if (c2 == 't')				// associate thermcouple with this heater
			{
				h_io->tc = TryInput(-1, TC_CHANNELS-1, ERROR_TC_CH, h_io->tc, 0);
			}
			else
			{
				if (NargPresent) selectHeater();
				reportHeater();
			}
		}
		else if (c == 'n')				// select channel
		{
			selectHeater();
		}
 		else if (c == 'a')				// start auto (PID) control
		{
			h_io->mode = c;
			h_io->integral = -1;
		}
		else if (c == 'm')				// manual mode
		{
			if (NargPresent)			// manage control output manually
			{
				if (h_io->mode == 'a') h_io->mode = c;		// don't turn it on if it's off
				setCO(h_io, TryInput(0, h_io->coLimit, ERROR_CO, h_io->co, 2));
			}
			else
			{
				h_io->mode = c;
				update_output(h_io);
			}
		}
		else if (c == 't')				// thermocouple command
		{
			if (c2 == '\0')				// thermocouple report
			{
				if (NargPresent) selectThermocouple();
				reportTC();
			}
			else if (c2 == 'n')			// select thermocouple channel
			{
				selectThermocouple();
			}
			else if (c2 == '~' || c2 == 'K' || c2 == 'T')
			{
				t_io->tcType = c2;
			}
			else						// unrecognized command
			{
				mask_set(Error, ERROR_COMMAND);
			}
		}
		else if (c == 'x')				// set maximum control output
		{
			h_io->coLimit = TryInput(0, DEVICE_CO_MAX, ERROR_COMAX, h_io->coLimit, 2);
		}
		else if (c == 'c')				// configuration command
		{
			n = TryInput(0, INT_MAX, ERROR_CONFIG, 0, 0);
			if (!(Error & ERROR_CONFIG))
			{
				if (c2 == 'k')			// controller gain (Kc)
					h_io->Kc = n;
				else if (c2 == 'i')		// Ci = 1 / Ti (to avoid float division)
					h_io->Ci = n;
				else if (c2 == 'd')		// Cd = Kc * Td = gain times dead time
					h_io->Cd = n;
				else if (c2 == 'r')		// Cpr = 1 / gp (reciprocal of process gain)
					h_io->Cpr = n;
				else					// unrecognized command
					mask_set(Error, ERROR_COMMAND);
			}
		}
		else if (c == 'i')				// toggle interference suppression
		{
			INVERT(&EnableSuppression);
		}
		else if (c == 'z')				// program data
		{
			printromstr(FIRMWARE); printromstr(VERSION); endLine();
			printromstr(R"S/N:"); printi(SERNO, 4, ' '); endLine();
			printromstr(R"H:"); printi(h_ioch, 2, ' ');
			printromstr(R" T:"); printi(t_ioch, 3, ' '); endLine();
			printromstr(R"ADC:"); printi(ADC_TC, 5, ' ');
			if (EnableSuppression) putc('!');
			endMessage();
		}
		else							// unrecognized command
		{
			mask_set(Error, ERROR_COMMAND);
		}
	}
	
	if (EnableDatalogging)
	{
		EnableDatalogging = FALSE;		// re-enabled later by isr_timer0
		if (DatalogReset != 0xFF && uint8CounterReset(&DatalogCount, DatalogReset))
			reportDataDump();
	}
}


///////////////////////////////////////////////////////
// Skips have to be managed on a per-pulse basis.
void setCOStarts()
{
	far Device *h = Htr;
	uint8_t i;
	
	// Clear all C & A values
	for (i = 0; i < MAX_STARTS; ++i)
		*((int *)(Starts + i)) = 0;
	
	for (i = 0; i < HTR_CHANNELS; ++i, ++h)
	{
		if (h->start)
		{
			if (h->skipped >= h->skip)
			{
				mask_set(*(h->CorA), HTR_BIT[i]);
				h->skipped = 0;			// it's getting a pulse this time
			}
			else
				h->skipped++;
		}
	}
	Start = Starts;
}


///////////////////////////////////////////////////////
// As many as 164 T0 clock periods may elapse between 
// the start of timer0's isr and the completion of 
// setCOStarts().
void interrupt isr_timer0()
{
//elapsed0 = T0;
//	DI_T0();
//	EI_ADC();
//	EI();

	++T0Ticks;
	
#if CO_PERIOD > 1
	if (CO_INTERVAL)
#endif
	{
		disableAllOutputs();
		
		if (FirstT1Reset)
		{
			setCOStarts();
			T1R = FirstT1Reset - T0;
//elapsed = T0 - elapsed0;
			start_timer1();
			EI_T1();
		}
	}

#if CU_PERIOD > 1
	if (CU_INTERVAL)
#endif
		EnableControllerUpdate = TRUE;

if (ONE_SECOND)
	EnableDatalogging = TRUE;
	
//	DI();
//	EI_T0();
}


///////////////////////////////////////////////////////
// start CO pulses
// 11 T0 clock periods may elapse between the start of
// isr_timer1() and its completion.
void interrupt isr_timer1()
{
//elapsed0 = T0;
	mask_clr(PCOUT, Start->C);
	mask_clr(PAOUT, Start->A);

	if (Start->reset)
	{
		T1R = Start->reset;
		start_timer1();
		Start++;
	}
//elapsed = T0 - elapsed0;
}


///////////////////////////////////////////////////////
// ADC read complete...
void interrupt isr_adc()
{ 
	if (AdcdSettling)
		--AdcdSettling;
}
