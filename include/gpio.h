///////////////////////////////////////////////////////
// gpio.h
//
// heater controller

#pragma once // Include this file only once

#include <ez8.h>
#include "..\\..\\common_controller\\include\\mask.h"

///////////////////////////////////////////////////////
//
// Conventions for configuring gpio pins
// 	(Note: keep DBG & -RESET reserved for flashing/debugging)
//
// UART uses PA4/RX, PA5/TX
//
// Preferred analog inputs:
//		PB0/ANA0 .. PB3/ANA3
//
// Preferred digital inputs
// 		PC0 .. PC3
//
// Preferred digital outputs (open-drain or push-pull possible)
// 		PA0, PA1, PA2, PA3, PA6, PA7
//
// Alternate/additional analog inputs:
//		PC0/ANA4 .. PC2/ANA6
//
// Alternate/additional digital outputs (push-pull only)
//		PC0/ANA4 .. PC2/ANA6
//		PB0/ANA0 .. PB3/ANA3
//
///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
//
// NOTE: Open drain mode is only available on Port A gpio pins
// (See "Errata for Z8 Encore XP F082A Series UP0069.pdf")
//
///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
// Port A
// PA7 = OUT: -PWM5
// PA6 = OUT: -PWM6
// PA5 = OUT: TXD0 (Alt. function)
// PA4 = IN: RXD0 (Alt. function)
// PA3 = OUT: INH0
// PA2 = OUT: C (TC select)
// PA1 = OUT: B (TC select)
// PA0 = OUT: A (TC select)
//
// DD == data direction
// OC == output control
// AF == alternate function
//
// PADD		= 00010000		// 1 = IN; 0 = OUT (set unused pins to OUT)
// PAOC		= 00000000		// 1 = open drain; 0 = push-pull (the default)
// PAAF		= 00110000		// alternate functions
// PAOUT	= 11000000		// defaults
#define PA_DD					0x10
#define PA_OC					0x00
#define PA_AF					0x30
#define PA_OUT					0xC0

#define PWM5					0x80		// actually, the -PWM signal
#define PWM6					0x40

#define TC_ADDR					0x0F
#define TC_select(x)			mask_clr(PAOUT, TC_ADDR), mask_set(PAOUT, x)


///////////////////////////////////////////////////////
// Port B
// PB7 = N/A
// PB6 = N/A
// PB5 = N/A
// PB4 = N/A
// PB3 = no connect
// PB2 = IN: TCX_TEMP = ANA2 (Alt. function)
// PB1 = IN: TEMP_SENSE = ANA1 (Alt. function)
// PB0 = no connect
//
// PBDD		= 00000110		// 1 = IN; 0 = OUT (set unused pins to OUT)
// PBAF		= 00000110		// alternate functions
// PBOUT	= 00000000		// defaults
//
// NOTE: open drain mode does not work for Port B gpio pins
// (See "Errata for Z8 Encore XP F082A Series UP0069.pdf")
//
#define PB_DD					0x06
#define PB_AF					0x06
//#define PB_OUT					0x00



///////////////////////////////////////////////////////
// Port C
// PC7 = N/A
// PC6 = N/A
// PC5 = N/A
// PC4 = N/A
// PC3 = OUT: -PWM1
// PC2 = OUT: -PWM2
// PC1 = OUT: -PWM3
// PC0 = OUT: -PWM4
//
// PCDD		= 00000000		// 1 = IN; 0 = OUT (set unused pins to OUT)
// PCAF		= 00000000		// alternate functions
// PCOUT	= 00001111		// defaults
//
// NOTE: open drain mode does not work for Port C gpio pins
// (See "Errata for Z8 Encore XP F082A Series UP0069.pdf")
//
#define PC_DD					0x00
#define PC_AF					0x00
#define PC_OUT					0x0F

#define PWM1					0x08
#define PWM2					0x04
#define PWM3					0x02
#define PWM4					0x01



///////////////////////////////////////////////////////
// prototypes
//
void init_gpio(void);
