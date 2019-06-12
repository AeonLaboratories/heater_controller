///////////////////////////////////////////////////////
// irq.c
//
// heater controller

#include <eZ8.h>
#include <defines.h>
#include "..\\..\\common_controller\\include\\c99types.h"
#include "..\\..\\common_controller\\include\\z082a.h"
#include "..\\..\\common_controller\\include\\timer.h"
#include "..\\..\\common_controller\\include\\adc.h"
#include "..\\..\\common_controller\\include\\uart.h"
#include "..\\..\\common_controller\\include\\irq.h"
#include "config.h"
#include "error.h"
#include "gpio.h"

// Store big strings in ROM to conserve RData and EData space.
rom char FIRMWARE[]	= R"Aeon Laboratories HC6 ";
rom char VERSION[]	= R"V.20190412-0000";

#define SERNO					20

#define TC_CHANNELS				16
#define HTR_CHANNELS			6

#define AIN_TC					1		// TEMP_SENSE signal is on ANA1
#define AIN_CJ					2		// CJTEMP signal is on ANA2

#define DFS						0.99	// digital filter stability


///////////////////////////////////////////////////////
//
// calibration
//

///////////////////////////////////////////////////////
//
// The ADC count ranges from ~0 (whatever its internal 
// offset is) to ~4096, when the input is at the 
// reference voltage (~2000 mV, using the built-in ADC 
// reference). So, the ADC sensitivity (i.e., its gain)
// is very close to 0.4883 mV per count.
//
#define ADC_GAIN				0.4883	// mV/count

// Set ADC_OFFSET to whatever the ADC reports when the 
// input is 0V.
#define ADC_OFFSET				0		// not used

#define NO_CJ_SENSOR			ADC_OUTOFRANGE

///////////////////////////////////////////////////////
//
// The CJ temperature sensor produces a voltage of
// approximately 400 + 19.5 mV/degC. Thus, the 
// CJ_GAIN (to convert ADC counts to degC) is about
//
#define CJ_GAIN					(ADC_GAIN / 19.5)
//
// The nominal CJOffset value would be
//
//  ~400 / ADC_GAIN = 819 counts
//
// However, the actual aggregate offset varies
// significantly from nominal, because of device 
// characteristics and especially the internal ADC 
// offset.
//
// Calibrate the CJOffsets by independently
// measuring each true CJ temperature and observing
// the simultaneous value reported by this program.
// The difference, divided by CJ_GAIN, gives the
// error between the entered CJOffset and the correct
// value
//
//	CJOffset = old_CJOffset + (CJt_reported - CJt_true) / CJ_GAIN
//
far int CJOffset[] = 
#if SERNO == 2
	{ 200, NO_CJ_SENSOR };
#elif SERNO == 3				// AEON-CEGS
	{ 703, 643 };
#elif SERNO == 4				// AEON-CEGS
	{ 673, NO_CJ_SENSOR };
#elif SERNO == 5				// AEON-CEGS
	{ 679, NO_CJ_SENSOR };
#elif SERNO == 6
	{ 169, 163 };
#elif SERNO == 7
	{ 165, 159 };
#elif SERNO == 8
	{ 118, NO_CJ_SENSOR };
#elif SERNO == 9
	{ 102, NO_CJ_SENSOR };
#elif SERNO == 10
	{ 200, NO_CJ_SENSOR};
#elif SERNO == 11				// USGS-CEGS TC0
	{ 535, NO_CJ_SENSOR};
#elif SERNO == 12				// USGS-CEGS TC1
	{ 535, NO_CJ_SENSOR};
#elif SERNO == 13				// USGS-CEGS TC2
	{ 639, 647 };
#elif SERNO == 14				// Purdue CEGS TC0
	{ 556, NO_CJ_SENSOR };
#elif SERNO == 15				// Purdue CEGS TC1
	{ 580, NO_CJ_SENSOR };
#elif SERNO == 16				// Purdue CEGS TC2
	{ 564, 592 };
#elif SERNO == 17				// Dalhousie
	{ 531, 523 };
#elif SERNO == 18				// Dalhousie
	{ 607, 607 };
#elif SERNO == 19				// Dalhousie
	{ 487, 567 };
#elif SERNO == 20				// Aeon CEGS 2
	{ 487, NO_CJ_SENSOR };
#endif

///////////////////////////////////////////////////////
// Pre-amp & ADC compensation calibration:
//
// Check the ADC count with a TC input shorted.
//     This count is the IA_ADC_OFFSET
// (Use the 'z' command to check the ADC count.)
//
// Note that the IA_ADC_OFFSET combines the 
// internal ADC offset and the instrumentation
// amp offset.
//	
#if SERNO == 2
	#define IA_ADC_OFFSET		134
#elif SERNO == 3
	#define IA_ADC_OFFSET		495
#elif SERNO == 4
	#define IA_ADC_OFFSET		487
#elif SERNO == 5
	#define IA_ADC_OFFSET		532
#elif SERNO == 6
	#define IA_ADC_OFFSET		141
#elif SERNO == 7
	#define IA_ADC_OFFSET		133
#elif SERNO == 8
	#define IA_ADC_OFFSET		119
#elif SERNO == 9
	#define IA_ADC_OFFSET		112
//#elif SERNO == 10						// WHERE IS THIS HC??
//	#define IA_ADC_OFFSET		133
#elif SERNO == 11
	#define IA_ADC_OFFSET		360
#elif SERNO == 12
	#define IA_ADC_OFFSET		385
#elif SERNO == 13
	#define IA_ADC_OFFSET		444
#elif SERNO == 14
	#define IA_ADC_OFFSET		398
#elif SERNO == 15
	#define IA_ADC_OFFSET		371
#elif SERNO == 16
	#define IA_ADC_OFFSET		404
#elif SERNO == 17
	#define IA_ADC_OFFSET		373
#elif SERNO == 18
	#define IA_ADC_OFFSET		406
#elif SERNO == 19
	#define IA_ADC_OFFSET		377
#elif SERNO == 20
	#define IA_ADC_OFFSET		461
#endif
//
// Get the thermocouple to a known, controlled temperature
// extreme (e.g., -195.8 in LN). Observe the cold junction
// temperature (CJt) and the ADC count with the TC at the 
// known temperature. Set IA_ADC_GAIN according to the following 
// equation:
//
//    IA_ADC_GAIN = (emf(TCt) - emf(CJt)) / (ADC - IA_ADC_OFFSET)
//
// The first value is for negative values, i.e.,
// ADC < IA_ADC_OFFSET; the second is for positive ones.
//
far float IA_ADC_GAIN[] = 
#if SERNO == 2
	{ 0.014555, 0.014555 };		// calibrated w/ Type T @ 150 C 
#elif SERNO == 3
	{ 0.015291, 0.015291 };		// calibrated w/ T @ -195.8 C
#elif SERNO == 4
	{ 0.015661, 0.016912 };		// calibrated w/ T @ -195.8 C, K @ 625 C
#elif SERNO == 5
	{ 0.015306, 0.015306 };		// calibrated w/ K @ 587 C
#elif SERNO == 6
	{ 0.012963, 0.012963 };
#elif SERNO == 7
	{ 0.013105, 0.012948 };		// calibrated w/ T @ -195.8 C, K @ 629 C
#elif SERNO == 8
	{ 0.01485 , 0.014828 };
#elif SERNO == 9
	{ 0.015328, 0.015415 };		// calibrated w/ T @ -195.8 C, K @ 629 C
//#elif SERNO == 10				// missing
//	{ 0.012963, 0.012963 };
#elif SERNO == 11
	{ 0.016800, 0.017776 };		// calibrated w/ T @ -195.8 C, K @ 625 C
#elif SERNO == 12
	{ 0.016777, 0.016455 };
#elif SERNO == 13
	{ 0.014876, 0.014643 };
#elif SERNO == 14
	{ 0.016546, 0.016419 };		// calibrated w/ T @ -195.8 C, K @ 625 C
#elif SERNO == 15
	{ 0.016799, 0.017542 };		// calibrated w/ T @ -195.8 C, K @ 625 C
#elif SERNO == 16
	{ 0.016340, 0.016278 };		// calibrated w/ T @ -195.8 C, K @ 625 C
#elif SERNO == 17
	{ 0.016898, 0.016479 };		// calibrated w/ T @ -195.8 C, K @ 625 C
#elif SERNO == 18
	{ 0.016282, 0.015865 };		// calibrated w/ T @ -195.8 C, K @ 625 C
#elif SERNO == 19
	{ 0.016726, 0.016530 };		// calibrated w/ T @ -195.8 C, K @ 625 C
#elif SERNO == 20
	{ 0.016371, 0.016294 };		// calibrated w/ T @ -195.8 C, K @ 625 C
#endif

	
///////////////////////////////////////////////////////
// The CO_RESERVE provides time for the timer interrupt service
// routines.
// Making CO_RESERVE long enough prevents contention between
// the start and stop routines, at the cost of limiting the maximum
// duty cycle. 
// The following reserves the maximum number of T0 clock periods that 
// still generates a true 99% duty cycle output (when 100% is commanded).
// Must be at least 187 to allow the T0 isr sufficient time to compute 
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
#define htr_off(x)	mask_set(HTR_PORT(x), HTR_BIT[(x)])	// set -PWM low
#define htrs_off()	mask_set(PCOUT, PC_HTRS), mask_set(PAOUT, PA_HTRS)	// set -PWM high

///////////////////////////////////////////////////////
typedef char enum { Off, Manual, Auto } Modes;

///////////////////////////////////////////////////////
typedef char enum { None, TypeK, TypeT } TC_Types;
#define TC_TYPES_MAX			2

///////////////////////////////////////////////////////
// CFF == 88 W Watlow ceramic fiber furnace
// VTC == Aeon VTC cooled by LN
// VTC_WARM == Aeon VTC without LN
// HT470 == 470 W, 6-foot heat tape
typedef char enum { CFF, VTC, VTC_WARM, HT470 } DeviceTypes;
#define DT_MAX					3


///////////////////////////////////////////////////////
typedef struct
{
	uint16_t Kc;		// Controller gain
	uint16_t Ci;		// 1 / Ti (to avoid float division)
	uint16_t Cd;		// Kc * Td = gain times dead time
	uint16_t Cpr;		// 1 / gp (reciprocal of process gain)
} PidConstants;

///////////////////////////////////////////////////////
typedef struct
{
	TC_Types tcType;
	float t;				// temperature, in degrees C
	uint16_t error;
} Thermocouple;

///////////////////////////////////////////////////////
typedef struct
{
	DeviceTypes devType;	// what kind of heater it is. selects PID constants
	Modes mode;			// operating mode
	uint8_t tc;			// thermocouple channel (0..15; default is heater array index)
	int sp;				// setpoint (desired pv)
	uint16_t co;		// in hundredths of a percent (i.e., 10000 means 100.00%)
	uint16_t coLimit;
	uint16_t start;		// T0 count for pulse start
	uint8_t skip;		// number of pulses to skip between delivered pulses
	volatile uint8_t skipped;	// number of pulses skipped since last delivered pulse
	uint8_t * CorA;		// pointer to C or A element of Starts element containing this heater
	float priorPv;		// for Auto (PID) control
	float integral;		// for Auto (PID) control
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
// PID parameters, tuned by device type
// Each parameter is stored in the given units, 
// rounded to nearest integer.
far PidConstants K[] =		//	{ 100*Kc, 100000*Ci, 10*Cd, 1000*Cpr }
{
	{ 1812, 927, 1793, 760 },		// 0 CF88 furnace (default) (set x25 (max = 25%))
	{ 2841, 2791, 1250, 10753 },	// 1 VTC cooled by Aluminum thermal conduit in LN (w/ copper wool) (set x16.67)
	{ 2381, 172, 3571, 678 },		// 2 VTC not cooled	(set x3)
	{ 6349, 2907, 1667, 8273 },		// 3 470W Heat tape
};


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
//
// Note: Some rom may be conserved by not intializing 
// global and static variables, thus using C-defined
// default values, when possible. I.e., never 
// intialize a global or static to 0 and use loops
// to initialize constant data that is repeated.
//
///////////////////////////////////////////////////////

far uint8_t HTR_BIT[HTR_CHANNELS] = { PWM1, PWM2, PWM3, PWM4, PWM5, PWM6 };

volatile uint16_t T0Ticks;					// rolls over when max uint16_t is reached
//volatile far uint16_t elapsed0;				// DEBUGGING
//volatile far uint16_t elapsed;				// DEBUGGING

volatile BOOL EnableControllerUpdate = TRUE;
volatile BOOL EnableDatalogging = FALSE;

far Device Htr[HTR_CHANNELS];
far Thermocouple TC[TC_CHANNELS];
far float CJt[] = {TEMP_ROOM, TEMP_ROOM};	// thermocouple cold junction temperatures
far int ADC_TC;								// the ADC reading for TC[t_ioch]

volatile Start_t Starts[HTR_CHANNELS];				// co pulse start times, in T0 ticks
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
// far char * far p;	// same as to the immediately preceding example
// far char* far p;		// same as to the immediately preceding example
//
// char *far p;			// p is far, points to a near char
//						//   equivalent to (char * far) p,
// 						//   p is a (char * far) into {near} memory
//						// "p is a pointer in far memory that points
//						// to a char in near memory"
//
// Because storage specifiers may be placed before or after
// the type in a declaration, alternative ceclarations are also
// possible (e.g., "char far c" is equivalent to "far char c"),
// but these alternatives are no clearer, and perhaps less clear.
///////////////////////////////////////////////////////

uint8_t h_updCh;							// current heater being updated

far uint8_t h_ioch;							// current heater for comms
far Device *far h_io = Htr;

far uint8_t t_ioch;							// current tc channel for comms
far Thermocouple *far t_io = TC;

uint8_t DatalogCount;						// counter for Datalogging
far uint8_t DatalogReset = 0xFF;			// report every (this many + 1) seconds
	// Note: 0xFF is used as a disable value (DatalogReset is unsigned)
	// This is convenient because the reset value needs to be one 
	// less than the desired count.

far uint8_t selectedTCch;
far Thermocouple *far selectedTC;
far TC_Types selectedTCType;

far uint8_t selectedCJS;
far float *far selectedCJTemp;
far int selectedCJOffset;

far int adc_in;
far float emf;

far float *far polynomialVariable;
far float *far polynomialCoefficients;

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
	
	for(i = 0; i < TC_CHANNELS; ++i, ++tc)
		tc->t = TEMP_IMPOSSIBLE;
	
	for (i = 0; i < HTR_CHANNELS; ++i, ++h)
	{
		h->tc = i;
		h->coLimit = DEVICE_CO_MAX;
	}

	SET_VECTOR(TIMER0, isr_timer0);
	SET_VECTOR(TIMER1, isr_timer1);
	SET_VECTOR(ADC, isr_adc);
	
	TC_select(TC_CHANNELS-1);
	ADC_SELECT(AIN_CJ);
	adc_reset();
	
	EI_T0();
	//EI_T1();		// dynamically enabled by T0 isr
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
float filter(float *oldV, float *newV)
{ return (DFS) * *oldV + (1.0 - DFS) * *newV; }


///////////////////////////////////////////////////////
// evaluate a polynomial in x with coefficients c[]
float evaluate_polynomial()
{
	float x = *polynomialVariable;
	uint8_t i = polynomialCoefficients[0];		// #terms is first value in coefficient array
	float sum = polynomialCoefficients[i];
	while (i > 1)
		sum = x * sum + polynomialCoefficients[--i];
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
	polynomialCoefficients = (selectedTCType == TypeK) ? TC_K_PT : TC_T_PT;
	mV = evaluate_polynomial();
	if (selectedTCType == TypeK) mV += *polynomialVariable * 0.0008359 + 0.0138456;
	return mV;
}


///////////////////////////////////////////////////////
// determine the sensed emf, in mV, based on the adc reading
float TC_emf()
{ 
	float dADC = adc_in - IA_ADC_OFFSET;
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
	
	if (selectedTCType == TypeK)
	{
		if (emf < 0)
			polynomialCoefficients = TC_K_N;
		else if (emf < 20.644 )
			polynomialCoefficients = TC_K_L;
		else
			polynomialCoefficients = TC_K_H;
	}
	else					// tcType == TypeT
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
	float oldt, newt = TEMP_IMPOSSIBLE;
	
	if (selectedTCch == t_ioch) ADC_TC = adc_in;
	if (selectedTCType == None)
		/* done */ ;
	else if (adc_in == ADC_OUTOFRANGE)
		mask_set(selectedTC->error, ERROR_ADC);
	else
	{
		mask_clr(selectedTC->error, ERROR_ADC);
		oldt = selectedTC->t;
		newt = TC_temp();
		if (validTemp(&oldt) && validTemp(&newt))
			newt = filter(&oldt, &newt);
	}
	selectedTC->t = newt;
}


////////////////////////////////////////////////////////
// Check the TC mux temperature (cold-junction temperature).
// Which sensor is selected depends on the selected TC channel.
void update_cjt()
{
	float oldt, newt;
	
	if (selectedCJOffset == NO_CJ_SENSOR)
		newt = TEMP_ROOM;
	else if (adc_in == ADC_OUTOFRANGE)
		newt = TEMP_IMPOSSIBLE;
	else
	{
		oldt = *selectedCJTemp;
		newt = CJ_GAIN * (adc_in - selectedCJOffset);
		newt = filter(&oldt, &newt);
	}
	*selectedCJTemp = newt;
}


///////////////////////////////////////////////////////
void check_adc()
{
	BOOL ainWasTC;
	uint8_t nextTCch = selectedTCch;
	
	if (AdcdSettling) return;
	adc_in = ADCD;
	
	if (ADCD_VALID(adc_in))
		adc_in >>= 3;						// adcd = (adcd >> 3) + ADC_OFFSET;
	else
		adc_in = ADC_OUTOFRANGE;

	// select the next input to be measured
	ainWasTC = SELECTED_ADC_CHANNEL == AIN_TC;
	if (ainWasTC)							// the analog input was a thermocouple
	{
		if (selectedTCch == TC_CHANNELS-1)	// if it was the last one
			ADC_SELECT(AIN_CJ);				// switch to checking the CJ sensors
		else								// else, select the next TC
			TC_select(nextTCch = selectedTCch+1);
	}
	else									// a CJ sensor is selected
	{
		if (selectedTCch)					// if selected TC is not channel 0
			TC_select(nextTCch = 0);		// check the other CJ sensor
		else
			ADC_SELECT(AIN_TC);				// switch back to the TCs
	}
	
	// Now that the next input to be measured is selected 
	// into the ADC, start a new measurement.
	adc_reset();

	// Finally, interpret the voltage just read for
	// the correct device and then update the selectedX
	// variables to the now-selected channel.
	if (ainWasTC)
		update_tc();
	else
		update_cjt();
	
	selectedTCch = nextTCch;
	selectedTC = TC + selectedTCch;
	selectedTCType = selectedTC->tcType;
	selectedCJS = selectedTCch > 7;
	selectedCJTemp = CJt + selectedCJS;
	selectedCJOffset = CJOffset[selectedCJS];
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

	if (h->error || h->mode == Off || co <= 0)
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
	far PidConstants *k = K + h->devType;
	float Kc = 0.01 * k->Kc;		// Kc = controller gain
	float Ci = 0.00001 * k->Ci;		// Ci = 1 / Ti (note: no Kc)
	float Cd = 0.1 * k->Cd;		// Cd = Kc * Td
	float Cpr = 0.001 * k->Cpr;		// Cpr = 1 / gp  (= step test dCO/dPV)

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
	far Thermocouple *tc = TC + h->tc;
	float t = tc->t;
	
	mask_clr(h->error, ERROR_NO_TC);
	mask_clr(h->error, ERROR_PV);
	
	if (h->mode == Auto)
	{
		if (tc->tcType == None)
			mask_set(h->error, ERROR_NO_TC);
		else if (tc->error || !validTemp(&t))
			mask_set(h->error, ERROR_PV);
		else
			pid(h, &t);
	}
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


///////////////////////////////////////////////////////
void report_header()
{
	printromstr(R"C DM TCT _POWER POWMAX SETP __TEMP ___CJT Error");
	endMessage();
}

////////////////////////////////////////////////////////
void report_device()
{
	uint8_t tch = h_io->tc;
	far Thermocouple *tc = TC + tch;
	
	printi(h_ioch, 1, ' '); printSpace();
	printi(h_io->devType, 1, ' ');
	printi(h_io->mode, 1, ' '); printSpace();
	printi(tch, 2, ' ');
	printi(tc->tcType, 1, ' '); printSpace();
	printdec(h_io->co, 6, ' ', 2); printSpace();
	printdec(h_io->coLimit, 6, ' ', 2); printSpace();
	printi(h_io->sp, 4, ' '); printSpace();
	printfarTenths(&(tc->t), 6, ' '); printSpace();
	printfarTenths(CJt + (tch > 7 ? 1 : 0), 6, ' '); printSpace();
	printi(Error | h_io->error, 5, ' ');
	
//	printSpace();
//	printi(elapsed, 6, ' ');
	
//	printSpace();
//	printi(h_io->skip, 2, ' ');
	
//	printSpace();
//	printi(h_io->start, 6, ' ');
	
	endMessage();
}


///////////////////////////////////////////////////////
void report_tc_header()
{
	printromstr(R"CH T __TEMP ___CJT Error");
	endMessage();
}


////////////////////////////////////////////////////////
void report_tc()
{
	printi(t_ioch, 2, ' '); printSpace();
	printi(t_io->tcType, 1, ' '); printSpace();
	printfarTenths(&(t_io->t), 6, ' '); printSpace();
	printfarTenths(CJt + (t_ioch > 7 ? 1 : 0), 6, ' '); printSpace();
	printi(Error | t_io->error, 5, ' ');
	endMessage();
}


///////////////////////////////////////////////////////
void do_commands()
{
	char c;
	BOOL tcCommand = FALSE;

	while (!RxbEmpty())				// process a command
	{
		c = getc();		
		mask_clr(Error, ERROR_COMMAND);
		
		// single-byte commands
		if (c == '\0')				// null command
		{
			// (treat as single-byte command that does nothing)
		}
		else if (c == 'z')			// program data
		{
			printromstr(FIRMWARE); printromstr(VERSION); endMessage();
			printromstr(R"S/N:"); printi(SERNO, 4, ' '); endMessage();
			printromstr(R"TC:"); printi(t_ioch, 3, ' '); printromstr(R" ADC:"); printi(ADC_TC, 5, ' '); endMessage();
			endMessage();
		}
		else if (c == 'h')			// report header
		{
			report_header();
		}
		else if (c == 'a')			// start PID control
		{
			h_io->mode = Auto;
			h_io->integral = -1;
		}
		else if (c == '0')			// turn it Off
		{
			h_io->mode = Off;
			update_output(h_io);
		}
		else						// multi-byte command
		{
			if (c == 't')			// thermocouple command
			{
				tcCommand = TRUE;
				c = getc();			// get the sub-command
			}
			getArgs();

			if (tcCommand)
			{
				if (c == 'r')		// thermocouple report
				{
					report_tc();
				}
				else if (c == 'n')	// select thermocouple channel
				{
					t_ioch = tryArg(0, TC_CHANNELS-1, ERROR_TC_CH, t_ioch, FALSE);
					t_io = TC + t_ioch;
				}
				else if (c == 't')	// set thermocouple type
				{
					t_io->tcType = tryArg(0, TC_TYPES_MAX, ERROR_TC_TYPE, t_io->tcType, FALSE);
				}
				else if (c == 'h')	// show thermocouple report header
				{
					report_tc_header();
				}
				else if (c == 'c')	// associate thermcouple channel with heater channel
				{
					// Unlike the other 't' commands, this one has nothing 
					// to do with the currently selected TC channel. Instead, it
					// sets the TC channel for the currently selected heater to
					// the supplied argument. It's really a 2-byte heater
					// command, not a tcCommand.
					h_io->tc = tryArg(0, TC_CHANNELS-1, ERROR_TC_CH, h_io->tc, FALSE);
				}
				else				// unrecognized command
				{
					mask_set(Error, ERROR_COMMAND);
				}
			} 
			else if (c == 'n')		// select channel
			{
				h_ioch = tryArg(0, HTR_CHANNELS - 1, ERROR_CHANNEL, h_ioch, FALSE);
				h_io = Htr + h_ioch;
			}
			else if (c == 'r')		// report
			{
				if (argPresent())	// set Datalogging interval
				{
					// rolls under to 0xFF if DatalogReset was 0
					DatalogReset = tryArg(0, 255, ERROR_DATALOG, DatalogReset + 1, FALSE) - 1;
					DatalogCount = 0;
				}
				else				// one-time report
					report_device();
			}
			else if (c == 'm')
			{
				h_io->mode = Manual;
				if (argPresent())	// manage control output manually
					setCO(h_io, tryArg(0, h_io->coLimit, ERROR_CO, h_io->co, TRUE));
				else
					update_output(h_io);
			}
			else if (c == 's')		// set setpoint
			{
				h_io->sp = tryArg(SP_MIN, SP_MAX, ERROR_SETPOINT, h_io->sp, FALSE);
			}
			else if (c == 'x')		// set maximum control output
			{
				h_io->coLimit = tryArg(0, DEVICE_CO_MAX, ERROR_COMAX, h_io->coLimit, TRUE);
			}
			else if (c == 'd')		// set device type
			{
				h_io->devType = tryArg(0, DT_MAX, ERROR_DEV_TYPE, h_io->devType, FALSE);
			}
			else					// unrecognized command
			{
				mask_set(Error, ERROR_COMMAND);
			}
		}
	}
	
	if (EnableDatalogging)
	{
		EnableDatalogging = FALSE;	// re-enabled later by isr_timer0
		if (DatalogReset != 0xFF && uint8CounterReset(&DatalogCount, DatalogReset))
			report_device();
	}
}


///////////////////////////////////////////////////////
// Skips have to be managed on a per-pulse basis.
void setCOStarts()
{
	far Device *h = Htr;
	uint8_t i;

	for (i = 0; i < HTR_CHANNELS; ++i, ++h)
	{
		if (h->start)
		{
			if (h->skipped < h->skip)
			{
				mask_clr(*(h->CorA), HTR_BIT[i]);
				h->skipped++;
			}
			else
			{
				mask_set(*(h->CorA), HTR_BIT[i]);
				h->skipped = 0;			// it's getting a pulse this time
			}
		}
	}
	Start = Starts;
}


///////////////////////////////////////////////////////
// As many as 187 T0 clock periods may elapse between 
// the start of timer0's isr and the completion of 
// setCOStarts().
void interrupt isr_timer0()
{
//elapsed0 = T0;
	DI_T0();
//	EI_ADC();
	EI();

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
	
	DI();
	EI_T0();
}


///////////////////////////////////////////////////////
// start CO pulses
// 11 T0 clock periods may elapse between the start of
// isr_timer1() and its completion.
void interrupt isr_timer1()
{
//	DI_ADC();	// avoid adc reads when power is on
	
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
