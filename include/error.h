///////////////////////////////////////////////////////
// error.h
//
// heater controller

#pragma once // Include this file only once

#include "..\\..\\common_controller\\include\\c99types.h"

#define ERROR_NONE			0		// No error
#define ERROR_PV			1		// PV out of range (thermocouple disconnected?)
#define ERROR_CHANNEL		2		// invalid heater number
#define ERROR_COMMAND		4		// unrecognized command from RS232
#define ERROR_SETPOINT		8		// setpoint out of range
#define ERROR_CO			16		// control output power level out of range
#define ERROR_DATALOG		32		// datalogging time interval out of range
#define ERROR_BUF_OVFL		64		// RS232 input buffer overflow
#define ERROR_TC_CH			128		// invalid thermocouple selected
#define ERROR_TC_TYPE		256		// invalid thermocouple type
#define ERROR_DEV_TYPE		512		// invalid device type
#define ERROR_CRC			1024	// RS232 CRC error
#define ERROR_ADC			2048	// adc out of range
#define ERROR_COMAX			4096	// control output limit out of range
#define ERROR_NO_TC			8192	// PID commanded on heater with no thermocouple

extern volatile uint16_t Error;
