///////////////////////////////////////////////////////
// error.h
//
// heater controller

#pragma once // Include this file only once

#include "..\\..\\common_controller\\include\\c99types.h"

#define ERROR_NONE			0		// no error
#define ERROR_ADC			1		// adc out of range
#define ERROR_BUF_OVFL		2		// RSS232 input buffer overflow
#define ERROR_CRC			4		// RS232 CRC error
#define ERROR_COMMAND		8		// unrecognized command from RS232
#define ERROR_CHANNEL		16		// invalid device channel
#define ERROR_DATALOG		32		// datalogging interval out of range (0..255)

#define ERROR_SETPOINT		64		// setpoint out of range
#define ERROR_CO			128		// control output power level out of range
#define ERROR_TC_CH			256		// invalid thermocouple selected
#define ERROR_TC_TYPE		512		// invalid thermocouple type
#define ERROR_CONFIG		1024	// invalid configuration or configuration parameter
#define ERROR_COMAX			2048	// control output limit out of range
#define ERROR_NO_TC			4096	// PID commanded on heater with no thermocouple

#define ERROR_PV			8192	// PV out of range (thermocouple disconnected?)


extern volatile uint16_t Error;
