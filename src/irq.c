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
rom char VERSION[]	= R"V.20160130-0000";

#define SERNO					4

#define TC_CHANNELS				16
#define HTR_CHANNELS			6

#define AIN_TC					1		// TEMP_SENSE signal is on ANA1
#define AIN_CJ					2		// CJTEMP signal is on ANA2

#define DFS						0.99	// digital filter stability

// PID mode soft integrator Anti-Windup
// A value of 0 implements a clamping strategy.
// A value of 1 is the same as no anti-windup correction.
#define ANTI_WINDUP				0.05	// Select a value in {0.05 to 0.25}.


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
#elif SERNO == 3
	{ 703, 643 };
#elif SERNO == 4
	{ 673, NO_CJ_SENSOR };
#elif SERNO == 5
	{ 679, NO_CJ_SENSOR };
#elif SERNO == 6
	{ 169, 163 };
#elif SERNO == 7
	{ 655, 623 };
#elif SERNO == 8
	{ 485, NO_CJ_SENSOR };
#elif SERNO == 9
	{ 410, NO_CJ_SENSOR };
#elif SERNO == 10
	{ 200, NO_CJ_SENSOR};
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
	#define IA_ADC_OFFSET		529
#elif SERNO == 8
	#define IA_ADC_OFFSET		476
#elif SERNO == 9
	#define IA_ADC_OFFSET		446
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
	{ 0.013099, 0.024500};		// calibrated w/ T @ -195.8 C
#elif SERNO == 8
	{ 0.014989, 0.014333 };
#elif SERNO == 9
	{ 0.015519, 0.015086 };		// calibrated w/ T @ -195.8 C, K @ 629 C
#endif


// The CO_MAX_RESERVE provides time for the "stop pulse"
// interrupt service routine, plus the time at the start
// of the T0 ISR before the interrupts are re-enabled.
// Making CO_MAX_RESERVE long enough prevents contention between
// the start and stop routines, at the cost of limiting the maximum
// duty cycle.
// If the T1 ISR is minimal, and T0 starts the pulse and re-enables
// interrupts as early as possible, CO_MAX_RESERVE might need to be
// as much as 150 system clock cycles, and perhaps longer if 
// another ISR preempts or delays the T1 ISR.
// 
// The following reserves a conservative 256 system clock cycles.
// Using a power of 2 guarantees that the expression always 
// evaluates to an integer. The (T1_CLOCK_FREQ / SYS_FREQ) factor 
// converts system clocks into T1 clocks.
//
#define CO_MAX_RESERVE			(256 * T1_CLOCK_FREQ / SYS_FREQ)
//
#define CO_MAX					(CO_PERIOD * T1_CLOCK_FREQ / T0_FREQ - CO_MAX_RESERVE)
#if CO_MAX > TIMER_MAX
	#undef CO_MAX
	#define CO_MAX				TIMER_MAX
#endif

// If CO is less than CO_MIN_RESERVE, then the T1 isr can't turn it 
// off on schedule. Empirical timing tests found a minimum pulse 
// width of about 4 microseconds, which worked out to about 22 system
// clocks at a clock frequency of 5529600 Hz.
// ~4 us / 0.18 us / T1_CLOCK ~= 22
// As with CO_MAX_RESERVE, use a power of 2 and convert to T1 clocks.
//
#define CO_MIN_RESERVE			(32 * T1_CLOCK_FREQ / SYS_FREQ)
//
#define CO_MIN					CO_MIN_RESERVE
#if CO_MIN < CO_MIN_RESERVE
	#undef CO_MIN
	#define CO_MIN				CO_MIN_RESERVE
#endif

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
//       ------    = w + 1/2
//         2
//
///////////////////////////////////////////////////////
// The device co units are hundredths of a percent.
#define DEVICE_CO_MAX			10000
// channel pulses per channel update (at most):
#define CHP_PER_CU_MAX			(1.0 * CO_FREQ / CU_FREQ)
// ON time for a minimal pulse, in half-cycles of line power:
#define ON_PW_MIN				0.5
// maximum pulse width, in half-cycles:
#define PW_MAX					(1.0 * HC_FREQ / CO_FREQ)
// ON time for maximum pulse width:
#define ON_PW_MAX				(PW_MAX + ON_PW_MIN)
// minimum ON time as a fraction of maximum ON time
#define PW_MIN_MAX_RATIO		(ON_PW_MIN / ON_PW_MAX)
// minimum pulse width co:
#define PW_MIN_CO				(DEVICE_CO_MAX * PW_MIN_MAX_RATIO)
// gain for pulse count method:
#define CO_GAIN_PC				(CHP_PER_CU_MAX / PW_MIN_CO)
// gain for pulse width method:
#define CO_GAIN_PW				((CO_MAX - CO_MIN) / (DEVICE_CO_MAX - PW_MIN_CO))
//
// A CO_MIN pulse is equivalent to
//
// (PW_MIN_MAX_RATIO) of a CO_MAX pulse.
//
// E.g., An output pulse of (CO_MIN + CO_MIN_EQUIV)
// T1 ticks should deliver twice as much power as a 
// single CO_MIN pulse.
//
#define CO_MIN_EQUIV ((CO_MAX-CO_MIN) * (PW_MIN_MAX_RATIO))

#define SP_MIN					-200	// degC
#define SP_MAX					1000	// degC

#define TEMP_MIN				-300	// degC
#define TEMP_MAX				2000	// degC
#define TEMP_ROOM				25		// degC
#define TEMP_IMPOSSIBLE			-999.9	// degC


///////////////////////////////////////////////////////
#define ADDR_PAOUT				0xFD3		// locations of output ports (see ez8.h)
#define ADDR_PCOUT				0xFDB
#define PC_HTRS					(PWM1 | PWM2 | PWM3 | PWM4)
#define PA_HTRS					(PWM5 | PWM6)
#define htr_on(x)	mask_clr(*(unsigned char volatile far *)HTR_PORT[(x)], HTR_BIT[(x)])	// set -PWM low
#define htr_off(x)	mask_set(*(unsigned char volatile far *)HTR_PORT[(x)], HTR_BIT[(x)])	// set -PWM high
#define htrs_off()	mask_set(PCOUT, PC_HTRS), mask_set(PAOUT, PA_HTRS)	// set -PWM high
#define all_PC_htrs_are_off()	((PCOUT & PC_HTRS) == PC_HTRS)
#define all_PA_htrs_are_off()	((PAOUT & PA_HTRS) == PA_HTRS)
#define all_htrs_are_off()		(all_PC_htrs_are_off() && all_PA_htrs_are_off())
#define a_htr_is_on()			(!all_htrs_are_off())

typedef char enum { Off, Manual, Auto } Modes;
typedef char enum { None, TypeK, TypeT } TC_Types;
#define TC_TYPES_MAX			2
// CFF == 88 W Watlow ceramic fiber furnace
// VTT == Aeon VTT3 heating element
typedef char enum { CFF, VTT } DeviceTypes;
#define DT_MAX					1


typedef struct
{
	int p;
	int i;
	int d;
	int ipreset;
} PidConstants;

typedef struct
{
	TC_Types tcType;
	float t;				// temperature, in degrees C
	uint16_t error;
} Thermocouple;

typedef struct
{
	DeviceTypes devType;	// what kind of heater it is. selects PID constants
	Modes mode;			// operating mode
	uint8_t tc;			// thermocouple channel (0..15; default is heater array index)
	int sp;				// setpoint (desired pv)
	int co;				// in hundredths of a percent (i.e., 10000 means 100.00%)
	int coLimit;
	uint16_t ticks;		// pre-calculated CO, to avoid floating point ops in t0 isr
	uint8_t pulses;		// max pulses (of width ticks) to send between device updates
	float priorPv;		// for Auto (PID) control
	float integral;		// for Auto (PID) control
	uint16_t error;
} Device;


///////////////////////////////////////////////////////
//
// global constants
//

///////////////////////////////////////////////////////
// PID parameters, tuned by device type
far PidConstants K[] =
{
	{ -104, -195, -936, 385},		// 0 CF88 furnace (default) (CHR)
	{ -329, -1962, -1063, 3353 },	// 1 VTT in LN (metal heat-transfer medium)
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
	9.0,
	0.0, 0.38748106364e-1, 0.3329222788e-4, 0.20618243404e-6,
	-0.21882256846e-8, 0.10996880928e-10, -0.30815758772e-13,
	0.4547913529e-16, -0.27512901673e-19
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
	7.0, -1.3180580e2, 4.8302220e1, -1.6460310e0, 5.4647310e-2, 
	-9.6507150e-4, 8.8021930e-6, -3.1108100e-8
};

// for positive temperature (degC) to emf (mV)
far float TC_K_PT[] =
{
	10.0,
	-0.17600413686e-1, 0.38921204975e-1, 0.18558770032e-4,
	-0.99457592874e-7, 0.31840945719e-9, -0.56072844889e-12,
	0.56075059059e-15, -0.32020720003e-18, 0.97151147152e-22,
	-0.12104721275e-25
};



///////////////////////////////////////////////////////
//
// global variables
//
//
// Some rom may be conserved by not intializing 
// global and static variables, thus using C-defined
// default values, when possible. E.g., never 
// intialize a global or static to 0 and use loops
// to initialize constant data that is repeated.
//
uint16_t HTR_PORT[6] = { ADDR_PCOUT, ADDR_PCOUT, ADDR_PCOUT, ADDR_PCOUT, ADDR_PAOUT, ADDR_PAOUT };
uint8_t HTR_BIT[6] = { PWM1, PWM2, PWM3, PWM4,  PWM5,  PWM6 };

volatile uint16_t T0Ticks;					// rolls over when max uint16_t is reached
volatile BOOL EnableControllerUpdate = TRUE;
volatile BOOL EnableDatalogging = FALSE;

far float CJt[] = {TEMP_ROOM, TEMP_ROOM};	// thermocouple cold junction temperatures
far Thermocouple TC[TC_CHANNELS];
far Device Htr[HTR_CHANNELS];
far int ADC_TC;								// the ADC reading for TC[t_ioch]

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
//
volatile uint8_t h_opCh;					// current heater being operated
volatile far Device *h_op = Htr;				

uint8_t h_updCh;							// current heater being updated
far Device *h_upd = Htr;

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
	int i;
	far Device *h;
	
	for(i = 0; i < TC_CHANNELS; ++i)
		TC[i].t = TEMP_IMPOSSIBLE;
	
	for (i = 0; i < HTR_CHANNELS; ++i)
	{
		h = Htr + i;
		h->tc = i;
		h->coLimit = DEVICE_CO_MAX;
	}	

	htrs_off();

	TC_select(TC_CHANNELS-1);
	ADC_SELECT(AIN_CJ);
	adc_reset();

	SET_VECTOR(TIMER0, isr_timer0);
	EI_T0();

	SET_VECTOR(TIMER1, isr_timer1);
	EI_T1();
	
	SET_VECTOR(ADC, isr_adc);
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


// evaluate a polynomial in x with coefficients c[]
float evaluate_polynomial()
{
	float x = *polynomialVariable;
	int i = polynomialCoefficients[0];		// #terms is first value in coefficient array
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
	
	if (AdcdReads < ADC_SETTLING_TIME) return;
	adc_in = Adcd;
	
	if (ADCD_VALID(adc_in))
		adc_in >>= 3;						// adcd = (adcd >> 3) + ADC_OFFSET;
	else
		adc_in = ADC_OUTOFRANGE;

	ainWasTC = SELECTED_ADC_CHANNEL == AIN_TC;
	if (ainWasTC)							// the analog input was a thermocouple
	{
		if (selectedTCch == TC_CHANNELS-1)	// if it was the last one
			ADC_SELECT(AIN_CJ);				// switch to checkin the CJ sensors
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


// Calculate pulse count and width from the device's 
// co value.
void update_output(far Device *h_upd)
{
	float co;
	float p;
	uint16_t ticks;

	if (h_upd->error || h_upd->mode == Off)
		h_upd->co = h_upd->pulses = 0;
	else
	{	
		co = h_upd->co;
		p = CO_GAIN_PC * co;
		ticks = CO_MIN;
		h_upd->pulses = p;

		if (co > PW_MIN_CO)	
			ticks += CO_GAIN_PW * (co - PW_MIN_CO);
		else if (h_upd->pulses > 0)
			ticks += CO_MIN_EQUIV * (p - h_upd->pulses) / h_upd->pulses;
	
		h_upd->ticks = ticks;
	}
}


void pid(far Device *h_upd, float *pv)
{
	far PidConstants *k = K + h_upd->devType;
	float temp = h_upd->sp;
	float error = *pv - temp;
	float integral = h_upd->integral;
	float coLimit = h_upd->coLimit;
	float co = k->p * error;
	if (co > coLimit || integral < 0)
	{
		if (temp > 25.0)	// reference temperature, assumed from setpoint
			temp = -25.0;	// negative to find (*pv - reference temperature)
		else
			temp = +196.0;	// reference temperature is -196 when plant is in LN
		temp += *pv;
		integral = 0.01 * k->ipreset * temp;
	}
	else
		integral += 0.01 * k->i * error;

	co += integral + k->d * (*pv - h_upd->priorPv);
	if (co < 0.0) co = 0.0;
	if (co > coLimit) co = coLimit;
	h_upd->integral = integral;
	h_upd->priorPv = *pv;
	h_upd->co = co;
}


void update_device(far Device *h_upd)
{
	far Thermocouple *tc = TC + h_upd->tc;
	float t = tc->t;
	
	mask_clr(h_upd->error, ERROR_NO_TC);
	mask_clr(h_upd->error, ERROR_PV);
	
	if (h_upd->mode == Auto)
	{
		if (tc->tcType == None)
			mask_set(h_upd->error, ERROR_NO_TC);

		if (tc->error || !validTemp(&t))
			mask_set(h_upd->error, ERROR_PV);
		
		pid(h_upd, &t);
	}
	update_output(h_upd);
}


///////////////////////////////////////////////////////
void update_controller()
{
	check_adc();	
	if (EnableControllerUpdate)
	{
		EnableControllerUpdate = FALSE;		// re-enabled later by isr_timer0
		update_device(h_upd);
		uint8CounterReset(&h_updCh, HTR_CHANNELS-1);
		h_upd = Htr + h_updCh;
	}
}


void setMode(Modes mode)
{
	h_io->mode = mode;
	if (mode == Auto)
		h_io->integral = -1;
	else
		update_output(h_io);
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
			printromstr(R"TC"); printi(t_ioch, 3, ' '); printromstr(R" ADC:"); printi(ADC_TC, 5, ' '); endMessage();
			endMessage();
		}
		else if (c == 'h')			// report header
		{
			report_header();
		}
		else if (c == 'a')			// start PID control
		{
			setMode(Auto);
		}
		else if (c == '0')			// turn it Off
		{
			setMode(Off);
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
				if (argPresent())	// manage control output manually
				{
					h_io->co = tryArg(0, h_io->coLimit, ERROR_CO, h_io->co, TRUE);
				}
				setMode(Manual);
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
// 
#pragma interrupt
void interrupt isr_timer0()
{
	++T0Ticks;
	DI_T0();
	
	#if CO_PERIOD > 1
		if (CO_INTERVAL)
	#endif
		 {
			stop_timer1();
			IRQ_CLEAR_T1();
			EI();
			 
			htrs_off();
			if (h_op->pulses > 0)
			{
				set_timer1_mark(h_op->ticks);	// set the stop time
				htr_on(h_opCh);				// start the pulse
				start_timer1();				// timer1 ISR stops the pulse
				h_op->pulses--;
			}
			uint8CounterReset(&h_opCh, HTR_CHANNELS-1);
			h_op = Htr + h_opCh;
		 }
	#if CO_PERIOD > 1
		 else
			EI(); 
	#endif


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
// stop CO pulse
#pragma interrupt
void isr_timer1()
{
	htrs_off();
}


///////////////////////////////////////////////////////
// ADC read complete...
#pragma interrupt
void isr_adc()
{ 
	Adcd = ADCD;
	++AdcdReads;
}
