/*
 * KK board flight controller software for AVR microcontrollers
 *
 * Based on XXcontrol_KR_v1.5 by Minsoo Kim
 * Based on XXcontrol by Mike Barton
 * Based on excellent assembly code by Rolf R Bakke (kapteinkuk)
 * With ideas from Rune Hasvold (CyCrow) and OlliW on rcgroups
 * Thanks, everyone!
 *
 * NO WARRANTY EXPRESSED OR IMPLIED. USE AT YOUR OWN RISK. Always test
 * without propellers! Please do not ship derivative works without
 * source; keep this code open as Rolf first so kindly released his
 * design and code to the community.
 *
 * Should fit on 48, 88, 168, and 328. I've tested TRICOPTER mode on an
 * ATmega88A. You may wish to use avrdude -t to "dump calibration" and
 * check timings on a digital scope. Temperature and voltage shift the
 * oscillator frequency a little, and each chip responds differently.
 * See doc8271.pdf page 401.
 *
 * I notice a few microseconds of output jitter still with the internal
 * oscillator. It seems that only an external resonator or crystal will
 * solve this, but those pins are currently used for Rx and the LED, and
 * the Rx pin cannot be moved to another pin that does not share another
 * PCINT unless RESET is used for that purpose. If using all hardware
 * PPM, it may be acceptable to have a more expensive interrupt handler
 * that simply logs the interrupt time and pin states and do the rest of
 * the processing in RxGetChannels().
 *
 * See http://www.kkmulticopter.com/
 *
 * Hardware PPM supported on motor outputs M1, M2, M5 and M6; software
 * PPM on M3 and M4 (Rx interrupts can cause some jitter). M3 and M4
 * outputs will be copied to M5 and M6, when not otherwise used, to allow
 * use of full hardware PPM.
 *
 * General motor output setup:
 *
 * Single
 *             M1 CCW
 *             |
 *             |
 *
 *             M2 (Servo)
 *             |
 *             |
 *      M5 ---- ---- M3
 *     (Servo) | (Servo)
 *             |
 *             M4 (Servo)
 *
 * Dual
 *             M1 CCW
 *             |
 *             M2 CW
 *             |
 *
 *             |
 *             |
 *   M5/M3 ----+----
 *     (Servo) |
 *             |
 *          M6/M4 (Servo)
 *
 * Twin
 *          / --- \
 *        /    |    \
 *      M1 CW  |     M2 CCW
 *             |
 *      M3     |       M4
 *    (Servo)  |   (Servo)
 *             |
 *             |
 *             M5 (Tail Servo, Optional)
 *             M6 (Tail Servo Reverse, Optional)
 *
 * Tri
 *       M1 CW     M2 CCW
 *         \       /
 *          \.---./
 *           |   |
 *           `---'
 *             |
 *             |M4/M6 (Tail Servo)
 *             M3/M5 CCW
 *
 * Quad-+
 *            M1 CW
 *             |
 *             |
 *             |
 *           .---.
 * M2 CCW----|   |----M3/M5 CCW
 *           `---'
 *             |
 *             |
 *             |
 *           M4/M6 CW
 *
 * Quad-X
 *
 *        M1 CW    M2 CCW
 *         \        /
 *           \.--./
 *            |  |
 *           /`--'\
 *         /        \
 *     M4/M6 CCW   M3/M5 CW
 *
 * Hex
 *             M1 CW
 *             |
 *     M6 CCW  |     M2 CCW
 *       \     |     /
 *         \ .---. /
 *          -|   |-
 *         / `---' \
 *       /     |     \
 *     M5 CW   |     M3 CW
 *             |
 *             M4 CCW
 *
 * Y6
 *
 *      M1,4        M2,5    M1->3 = CW
 *         \       /        M4->6 = CCW
 *          \.---./
 *           |   |
 *           `---'
 *             |
 *             |
 *            M3,6
 *
 */

/*+- Configurables ---------------------------------------------------------+*/

/* Multicopter Type */

//#define SINGLE_COPTER
//#define DUAL_COPTER
//#define TWIN_COPTER
#define TRI_COPTER
//#define QUAD_COPTER
//#define QUAD_X_COPTER
//#define Y4_COPTER
//#define HEX_COPTER
//#define Y6_COPTER

/* Servo and gain pot reversing */

//#define SERVO_REVERSE
//#define GAIN_POT_REVERSE

/*
 * ESC PPM output rate -
 * Do not set lower than 122 Hz without a slower/higher t0/t1 clkdiv
 * as the period needs to fit in the 16-bit timer.
 */
//#define ESC_RATE 300	// in Hz
//#define ESC_RATE 400	// in Hz (at SINGLE_COPTER Only)
#define ESC_RATE 450	// in Hz
//#define ESC_RATE 495	// in Hz

// NOTE: Set to 50 for analog servos, 250 for digital servos.
#define SERVO_RATE 50	// in Hz

// Stick arming and throw detection (in % * 10 eg 1000 steps)
#define STICK_THROW 300

// Gyro gain shift-right (after 32-bit multiplication of GainInADC[] value).
#define GYRO_GAIN_SHIFT 5

// Stick gain shift-right (after 32-bit multiplication of GainInADC[] value).
#define STICK_GAIN_SHIFT 8

// Skip yaw gyro calculations if using external yaw gyro
//#define EXTERNAL_YAW_GYRO

// Max Collective
// limits the maximum stick collective (range 80->100  100=Off)
// this allows gyros to stabilise better when full throttle applied
#define MAX_COLLECTIVE 1000			// 95

/*+- Helper Macros ---------------------------------------------------------+*/

#define PWM_LOW_PULSE_US ((1000000 / ESC_RATE) - 2000)

#define ADC_MAX 1023

#ifdef GAIN_POT_REVERSE
#undef GAIN_POT_REVERSE
#define GAIN_POT_REVERSE ADC_MAX -
#else
#define GAIN_POT_REVERSE
#endif

#ifdef SERVO_REVERSE
#undef SERVO_REVERSE
#define SERVO_REVERSE -
#else
#define SERVO_REVERSE
#endif

/*+- Main Code -------------------------------------------------------------+*/

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "typedefs.h"
#include "io_cfg.h"

#define EEPROM_DATA_START_POS 0			// Settings save offset in eeprom

enum GyroDirection { GYRO_NORMAL = 0, GYRO_REVERSED };

enum GyroArrayIndex { ROLL = 0, PITCH, YAW };

// eeProm data structure
static struct config {
	uint8_t	setup;					// byte to identify if already setup

	uint8_t RollGyroDirection;
	uint8_t PitchGyroDirection;
	uint8_t YawGyroDirection;
} Config;				// Holds configuration (from eeProm)

bool Armed;

static uint16_t GainInADC[3];				// ADC result

#if defined(SINGLE_COPTER) || defined(DUAL_COPTER) || defined(TWIN_COPTER) || defined(TRI_COPTER)
static uint16_t servo_skip_divider;
#endif

static int16_t RxInRoll;
static int16_t RxInPitch;
static int16_t RxInCollective;
static int16_t RxInYaw;

/*
 * Careful! Making these volatile makes it spew r24 crap in the _middle_ of
 * asm volatile statements. Always check assembler output of interrupt
 * routines.
 */
static uint16_t RxChannel1;
static uint16_t RxChannel2;
static uint16_t RxChannel3;
static uint16_t RxChannel4;

register uint16_t i_tmp asm("r2");		// ISR vars
register uint16_t RxChannel1Start asm("r4");
register uint16_t RxChannel2Start asm("r6");
register uint16_t RxChannel3Start asm("r8");
register uint16_t RxChannel4Start asm("r10");
register uint8_t i_sreg asm("r12");

#ifdef TWIN_COPTER
static int16_t RxInOrgPitch;
#endif

static int16_t gyroADC[3];			// Holds Gyro ADC's
static int16_t gyroZero[3];			// used for calibrating Gyros on ground

static int16_t integral[3];				// PID integral term
static int16_t last_error[3];				// Last proportional error

static uint16_t ModeDelayCounter;

static int16_t MotorOut1;
static int16_t MotorOut2;
static int16_t MotorOut3;
static int16_t MotorOut4;
static int16_t MotorOut5;
static int16_t MotorOut6;

static int16_t MotorStartTCNT1;
#if defined(SINGLE_COPTER) || defined(DUAL_COPTER) || defined(TWIN_COPTER) || defined(TRI_COPTER)
static uint8_t servo_skip;
#endif

static void setup();

static void init_adc();
static void ReadGyros();
static void CalibrateGyros();

static void ReadGainPots();
static void RxGetChannels();
static void read_adc(uint8_t channel);

static void output_motor_ppm();

static void Initial_EEPROM_Config_Load();
static void Save_Config_to_EEPROM();
static void Set_EEPROM_Default_Config();

static void eeprom_write_byte_changed( uint8_t *  addr, uint8_t value);
static void eeprom_write_block_changes( const uint8_t * src, void * dest, size_t size);

#if 1

/*
 * Rx interrupts with inline assembler that makes them much faster.
 * Please verify that GCC does not inject anything crazy in here,
 * such as completely unused movs that clobber other registers.
 */

ISR(PCINT2_vect, ISR_NAKED)
{
	if (RX_ROLL) {				// rising
		asm volatile("lds %A0, %1" : "=r" (RxChannel1Start) : "i" (&TCNT1L));
		asm volatile("lds %B0, %1" : "=r" (RxChannel1Start) : "i" (&TCNT1H));
		asm volatile("reti");
	} else {				// falling
		asm volatile(
			"lds %A0, %3\n"
			"lds %B0, %4\n"
			"in %1, __SREG__\n"
			"sub %A0, %A2\n"
			"sbc %B0, %B2\n"
			"out __SREG__, %1\n"
				: "+r" (i_tmp), "+r" (i_sreg), "+r" (RxChannel1Start)
				: "i" (&TCNT1L), "i" (&TCNT1H));
		RxChannel1 = i_tmp;
	}
	asm volatile ("reti");
}

ISR(INT0_vect, ISR_NAKED)
{
	if (RX_PITCH) {				// rising
		asm volatile("lds %A0, %1" : "=r" (RxChannel2Start) : "i" (&TCNT1L));
		asm volatile("lds %B0, %1" : "=r" (RxChannel2Start) : "i" (&TCNT1H));
		asm volatile("reti");
	} else {				// falling
		asm volatile(
			"lds %A0, %3\n"
			"lds %B0, %4\n"
			"in %1, __SREG__\n"
			"sub %A0, %A2\n"
			"sbc %B0, %B2\n"
			"out __SREG__, %1\n"
				: "+r" (i_tmp), "+r" (i_sreg), "+r" (RxChannel2Start)
				: "i" (&TCNT1L), "i" (&TCNT1H));
		RxChannel2 = i_tmp;
	}
	asm volatile ("reti");
}

ISR(INT1_vect, ISR_NAKED)
{
	if (RX_COLL) {				// rising
		asm volatile("lds %A0, %1" : "=r" (RxChannel3Start) : "i" (&TCNT1L));
		asm volatile("lds %B0, %1" : "=r" (RxChannel3Start) : "i" (&TCNT1H));
		asm volatile("reti");
	} else {				// falling
		asm volatile(
			"lds %A0, %3\n"
			"lds %B0, %4\n"
			"in %1, __SREG__\n"
			"sub %A0, %A2\n"
			"sbc %B0, %B2\n"
			"out __SREG__, %1\n"
				: "+r" (i_tmp), "+r" (i_sreg), "+r" (RxChannel3Start)
				: "i" (&TCNT1L), "i" (&TCNT1H));
		RxChannel3 = i_tmp;
	}
	asm volatile ("reti");
}

ISR(PCINT0_vect, ISR_NAKED)
{
	if (RX_YAW) {				// rising
		asm volatile("lds %A0, %1" : "=r" (RxChannel4Start) : "i" (&TCNT1L));
		asm volatile("lds %B0, %1" : "=r" (RxChannel4Start) : "i" (&TCNT1H));
		asm volatile("reti");
	} else {				// falling
		asm volatile(
			"lds %A0, %3\n"
			"lds %B0, %4\n"
			"in %1, __SREG__\n"
			"sub %A0, %A2\n"
			"sbc %B0, %B2\n"
			"out __SREG__, %1\n"
				: "+r" (i_tmp), "+r" (i_sreg), "+r" (RxChannel4Start)
				: "i" (&TCNT1L), "i" (&TCNT1H));
		RxChannel4 = i_tmp;
	}
	asm volatile ("reti");
}

#else

/*
 * Rx interrupts without inline assembler (just a bit slower)
 */

ISR(PCINT2_vect)
{
	if (RX_ROLL) {	// rising edge
		RxChannel1Start = TCNT1;
	} else {	// falling edge
		RxChannel1 = TCNT1 - RxChannel1Start;
		i_sreg = 0;
	}
}

ISR(INT0_vect)
{
	if (RX_PITCH) {
		RxChannel2Start = TCNT1;
	} else {
		RxChannel2 = TCNT1 - RxChannel2Start;
		i_sreg = 0;
	}
}

ISR(INT1_vect)
{
	if (RX_COLL) {
		RxChannel3Start = TCNT1;
	} else {
		RxChannel3 = TCNT1 - RxChannel3Start;
		i_sreg = 0;
	}
}

ISR(PCINT0_vect)
{
	if (RX_YAW) {
		RxChannel4Start = TCNT1;
	} else {
		RxChannel4 = TCNT1 - RxChannel4Start;
		i_sreg = 0;
	}
}
#endif

static void setup()
{
	uint8_t i;

	MCUCR = _BV(PUD);	// Disable hardware pull-up
#if 0
	RX_ROLL_DIR	= INPUT;
	RX_PITCH_DIR	= INPUT;
	RX_COLL_DIR	= INPUT;
	RX_YAW_DIR	= INPUT;

	GYRO_YAW_DIR	= INPUT;
	GYRO_PITCH_DIR	= INPUT;
	GYRO_ROLL_DIR	= INPUT;
	GAIN_YAW_DIR	= INPUT;
	GAIN_PITCH_DIR	= INPUT;
	GAIN_ROLL_DIR	= INPUT;

	M1_DIR		= OUTPUT;
	M2_DIR		= OUTPUT;
	M3_DIR		= OUTPUT;
	M4_DIR		= OUTPUT;
	M5_DIR		= OUTPUT;
	M6_DIR		= OUTPUT;

	LED_DIR 	= OUTPUT;

	LED		= 0;
	RX_ROLL 	= 0;
	RX_PITCH	= 0;
	RX_COLL		= 0;
	RX_YAW		= 0;
#else
	DDRB = 0b01111111;
	DDRC = 0b11000000;
	DDRD = 0b11110001;
#endif

	/*
	 * This suits my ATmega88A: no Tx trim, output timings perfect.
	 * See doc8271.pdf page ~401; beware the step at 128. -Simon
	 */
	if (OSCCAL == 0x9d)
		OSCCAL = 0x9f;

	/*
	 * timer0 (8bit) - run at 8MHz, used to control ESC pulses
	 * We use 8Mhz instead of 1MHz (1 usec) to avoid alignment jitter.
	 */
	TCCR0B = _BV(CS00);	/* NOTE: Specified again below with FOC0x bits */

	/*
	 * timer1 (16bit) - run at 8MHz, used to measure Rx pulses
	 * and to control ESC/servo pulse
	 */
	TCCR1B = _BV(CS10);

	/*
	 * timer2 8bit - run at 8MHz / 1024 = 7812.5KHz, just used for arming
	 */
	TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);

	/*
	 * Enable Rx pin interrupts
	 */
	PCICR = _BV(PCIE0) | _BV(PCIE2);	// PCINT0..7, PCINT16..23 enable
	PCMSK0 = _BV(PCINT7);			// PB7
	PCMSK2 = _BV(PCINT17);			// PD1
	EICRA = _BV(ISC00) | _BV(ISC10);	// Any change INT0, INT1
	EIMSK = _BV(INT0) | _BV(INT1);		// External Interrupt Mask Register

#if defined(SINGLE_COPTER) || defined(DUAL_COPTER) || defined(TWIN_COPTER) || defined(TRI_COPTER)
	/*
	 * Calculate the servo rate divider (pulse loop skip count
	 * needed to avoid burning analog servos)
	 */
	for (servo_skip_divider = 1;;servo_skip_divider++)
		if (servo_skip_divider * SERVO_RATE >= ESC_RATE)
			break;
#endif

	Initial_EEPROM_Config_Load();

	init_adc();

	Armed = false;

	/*
	 * Flash the LED once at power on
	 */
	LED = 1;
	_delay_ms(150);
	LED = 0;

	sei();

	_delay_ms(1500);

	ReadGainPots();
	ReadGainPots();

	// clear config
	if (GainInADC[PITCH] < (ADC_MAX * 5) / 100 &&
	    GainInADC[ROLL]  < (ADC_MAX * 5) / 100 &&
	    GainInADC[YAW]   < (ADC_MAX * 5) / 100) {

		Set_EEPROM_Default_Config();
		while (1)
			;
	}

	// Stick Centering Test
	if (GainInADC[PITCH] < (ADC_MAX * 5) / 100) {
		while (1) {
			RxGetChannels();
			i = abs(RxInRoll) + abs(RxInPitch) + abs(RxInYaw);
			LED = 1;
			while (i) {
				LED = 0;
				i--;
			}
		}
	}

	// Gyro direction reversing
	if (GainInADC[ROLL] < (ADC_MAX * 5) / 100) {		// less than 5% (5 / 100) * 1023 = 51
		// flash LED 3 times
		for (i = 0;i < 3;i++) {
			LED = 1;
			_delay_ms(25);
			LED = 0;
			_delay_ms(25);
		}

		while (1) {
			RxGetChannels();

			if (RxInRoll < -STICK_THROW) {	// normal(left)
				Config.RollGyroDirection = GYRO_NORMAL;
				Save_Config_to_EEPROM();
				LED = 1;
			} if (RxInRoll > STICK_THROW) {	// reverse(right)
				Config.RollGyroDirection = GYRO_REVERSED;
				Save_Config_to_EEPROM();
				LED = 1;
			} else if (RxInPitch < -STICK_THROW) { // normal(up)
				Config.PitchGyroDirection = GYRO_NORMAL;
				Save_Config_to_EEPROM();
				LED = 1;
			} else if (RxInPitch > STICK_THROW) { // reverse(down)
				Config.PitchGyroDirection = GYRO_REVERSED;
				Save_Config_to_EEPROM();
				LED = 1;
			} else if (RxInYaw < -STICK_THROW) { // normal(left)
				Config.YawGyroDirection = GYRO_NORMAL;
				Save_Config_to_EEPROM();
				LED = 1;
			} else if (RxInYaw > STICK_THROW) { // reverse(right)
				Config.YawGyroDirection = GYRO_REVERSED;
				Save_Config_to_EEPROM();
				LED = 1;
			}

			_delay_ms(50);
			LED = 0;

		}
	}

	// ESC throttle calibration
	if (GainInADC[YAW] < (ADC_MAX * 5) / 100) {		// less than 5%
		// flash LED 3 times
		for (i = 0;i < 3;i++) {
			LED = 1;
			_delay_ms(25);
			LED = 0;
			_delay_ms(25);
		}

		Armed = true;
		while (1) {
			RxGetChannels();
#ifdef SINGLE_COPTER
			MotorOut1 = RxInCollective;
			MotorOut2 = 1400;		// Center: 140
			MotorOut3 = 1400;
			MotorOut4 = 1400;
			MotorOut5 = 1400;
#elif defined(DUAL_COPTER)
			MotorOut1 = RxInCollective;
			MotorOut2 = RxInCollective;
			MotorOut3 = 500;		// Center: 50
			MotorOut4 = 500;
#elif defined(TWIN_COPTER)
			MotorOut1 = RxInCollective;
			MotorOut2 = RxInCollective;
			MotorOut3 = 500;		// Center: 50
			MotorOut4 = 500;
			MotorOut5 = 500;
			MotorOut6 = 500;		// Center: 50, Reverse
#elif defined(TRI_COPTER)
			MotorOut1 = RxInCollective;
			MotorOut2 = RxInCollective;
			MotorOut3 = RxInCollective;
			MotorOut4 = 500+RxInYaw*2;		// Center: 50
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
			MotorOut1 = RxInCollective;
			MotorOut2 = RxInCollective;
			MotorOut3 = RxInCollective;
			MotorOut4 = RxInCollective;
#elif defined(HEX_COPTER) ||  defined(Y6_COPTER)
			MotorOut1 = RxInCollective;
			MotorOut2 = RxInCollective;
			MotorOut3 = RxInCollective;
			MotorOut4 = RxInCollective;
			MotorOut5 = RxInCollective;
			MotorOut6 = RxInCollective;
#else
#error No Copter configuration defined !!!!
#endif
			output_motor_ppm();	// this regulates rate at which we output signals
		}
	}
}

static inline void loop()
{
//	static uint8_t i;
	static uint16_t Change_Arming = 0;
	static uint8_t Arming_TCNT2 = 0;
	int16_t error, emax = 1023;
	int16_t imax, derivative;

	RxGetChannels();

	if (RxInCollective <= 0) {
		// Check for stick arming (Timer2 at 8MHz/1024 = 7812.5KHz)
		Change_Arming+= (uint8_t)(TCNT2 - Arming_TCNT2);
		Arming_TCNT2 = TCNT2;

		if (Armed) {
			if (RxInYaw < STICK_THROW || abs(RxInPitch) > STICK_THROW)
				Change_Arming = 0;		// re-set count
		} else {
			if (RxInYaw > -STICK_THROW || abs(RxInPitch) > STICK_THROW)
				Change_Arming = 0;		// re-set count
		}

		// 3Sec / 0.000128 = 23437 = 0x5B8D or
		// 2.5Sec / 0.000128 = 19531 = 0x4C4B
		// 0.5Sec / 0.000128 = 3906 = 0x0F42
		if (Change_Arming > 0x0F42) {
			Armed = !Armed;
			if (Armed)
				CalibrateGyros();
			ModeDelayCounter = 0;
		}
	}

	ReadGyros();

	LED = Armed;

	gyroADC[ROLL]-= gyroZero[ROLL];
	gyroADC[PITCH]-= gyroZero[PITCH];
	gyroADC[YAW]-= gyroZero[YAW];

	//--- Start mixing by setting collective to motor outputs

	RxInCollective = (RxInCollective * 10) >> 3;	// 0-800 -> 0-1000

#ifndef SINGLE_COPTER
	if (RxInCollective > MAX_COLLECTIVE)
		RxInCollective = MAX_COLLECTIVE;
#endif

#ifdef SINGLE_COPTER
	MotorOut1 = RxInCollective;
	MotorOut2 = 840;	// 840
	MotorOut3 = 840;	// 840
	MotorOut4 = 945;	// 840 + 840/8
	MotorOut5 = 945;	// 840 + 840/8
#elif defined(DUAL_COPTER)
	MotorOut1 = RxInCollective;
	MotorOut2 = RxInCollective;
	MotorOut3 = 500;
	MotorOut4 = 500;
#elif defined(TWIN_COPTER)
	MotorOut1 = RxInCollective;
	MotorOut2 = RxInCollective;
	MotorOut3 = 500;
	MotorOut4 = 500;
	MotorOut5 = 500;	// Optional
	MotorOut6 = 500;	// Optional
#elif defined(TRI_COPTER)
	MotorOut1 = RxInCollective;
	MotorOut2 = RxInCollective;
	MotorOut3 = RxInCollective;
	MotorOut4 = 500;
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER)
	MotorOut1 = RxInCollective;
	MotorOut2 = RxInCollective;
	MotorOut3 = RxInCollective;
	MotorOut4 = RxInCollective;
#elif defined(Y4_COPTER)
	MotorOut1 = RxInCollective;
	MotorOut2 = RxInCollective;
	MotorOut3 = RxInCollective * 3 / 4;		// 25% Down
	MotorOut4 = RxInCollective * 3 / 4;		// 25% Down
#elif defined(HEX_COPTER) ||  defined(Y6_COPTER)
	MotorOut1 = RxInCollective;
	MotorOut2 = RxInCollective;
	MotorOut3 = RxInCollective;
	MotorOut4 = RxInCollective;
	MotorOut5 = RxInCollective;
	MotorOut6 = RxInCollective;
#endif

	imax = RxInCollective;
	if (imax < 0)
		imax = 0;
	imax>>= 3;	/* 1000 -> 200 */

	/* Calculate roll output - Test without props!! */

	RxInRoll = ((int32_t)RxInRoll * (uint32_t)GainInADC[ROLL]) >> STICK_GAIN_SHIFT;
	gyroADC[ROLL] = ((int32_t)gyroADC[ROLL] * (uint32_t)GainInADC[ROLL]) >> GYRO_GAIN_SHIFT;
	if (Config.RollGyroDirection == GYRO_NORMAL)
		gyroADC[ROLL] = -gyroADC[ROLL];

	if (Armed) {
		if (0) {
			error = RxInRoll - gyroADC[ROLL];
			if (error > emax)
				error = emax;
			else if (error < -emax)
				error = -emax;
			integral[ROLL]+= error;
			if (integral[ROLL] > imax)
				integral[ROLL] = imax;
			else if (integral[ROLL] < -imax)
				integral[ROLL] = -imax;
			derivative = error - last_error[ROLL];
			last_error[ROLL] = error;
			RxInRoll+= error + (integral[ROLL] >> 2) + (derivative >> 2);
		} else {
			RxInRoll-= gyroADC[ROLL];
		}
	}

#ifdef SINGLE_COPTER
	MotorOut2+= RxInRoll;
	MotorOut4-= RxInRoll;
#elif defined(DUAL_COPTER)
	MotorOut4+= RxInRoll;
#elif defined(TWIN_COPTER)
	RxInRoll = (RxInRoll * 7) >> 3;	// Approximation of sin(60) without div
	MotorOut1+= RxInRoll;
	MotorOut2-= RxInRoll;
#elif defined(TRI_COPTER)
	RxInRoll = (RxInRoll * 7) >> 3;	// (.875 versus .86602540)
	MotorOut1+= RxInRoll;
	MotorOut2-= RxInRoll;
#elif defined(QUAD_COPTER)
	MotorOut2+= RxInRoll;
	MotorOut3-= RxInRoll;
#elif defined(QUAD_X_COPTER)
	RxInRoll = RxInRoll >> 1;
	MotorOut1+= RxInRoll;
	MotorOut2-= RxInRoll;
	MotorOut3-= RxInRoll;
	MotorOut4+= RxInRoll;
#elif defined(Y4_COPTER)
	RxInRoll = (RxInRoll * 7) >> 3;
	MotorOut1+= RxInRoll;
	MotorOut2-= RxInRoll;
#elif defined(HEX_COPTER)
	RxInRoll = (RxInRoll * 7) >> 3;
	MotorOut2-= RxInRoll;
	MotorOut3-= RxInRoll;
	MotorOut5+= RxInRoll;
	MotorOut6+= RxInRoll;
#elif defined(Y6_COPTER)
	RxInRoll = (RxInRoll * 7) >> 3;
	MotorOut1+= RxInRoll;
	MotorOut2+= RxInRoll;
	MotorOut3-= RxInRoll;
	MotorOut4-= RxInRoll;
#endif

	/* Calculate pitch output - Test without props!! */

	RxInPitch = ((int32_t)RxInPitch * (uint32_t)GainInADC[PITCH]) >> STICK_GAIN_SHIFT;
	gyroADC[PITCH] = ((int32_t)gyroADC[PITCH] * (uint32_t)GainInADC[PITCH]) >> GYRO_GAIN_SHIFT;
	if (Config.PitchGyroDirection == GYRO_NORMAL)
		gyroADC[PITCH] = -gyroADC[PITCH];

	if (Armed) {
		if (0) {
			error = RxInPitch - gyroADC[PITCH];
			if (error > emax)
				error = emax;
			else if (error < -emax)
				error = -emax;
			integral[PITCH]+= error;
			if (integral[PITCH] > imax)
				integral[PITCH] = imax;
			else if (integral[PITCH] < -imax)
				integral[PITCH] = -imax;
			derivative = error - last_error[PITCH];
			last_error[PITCH] = error;
			RxInPitch+= error + (integral[PITCH] >> 2) + (derivative >> 2);
		} else {
			RxInPitch-= gyroADC[PITCH];
		}
	}

#ifdef SINGLE_COPTER
	MotorOut3+= RxInPitch;
	MotorOut5-= RxInPitch;
#elif defined(DUAL_COPTER)
	MotorOut3+= RxInPitch;
#elif defined(TWIN_COPTER)
	MotorOut3-= SERVO_REVERSE RxInPitch;
	MotorOut4+= SERVO_REVERSE RxInPitch;
	// Stick Only, Optional
	RxInOrgPitch = abs(RxInOrgPitch);
	MotorOut5+= RxInOrgPitch;			// Tain Servo-Optional, Down Only
	MotorOut6-= RxInOrgPitch;			// Tain Servo-Optional, Down Only (Reverse)
#elif defined(TRI_COPTER)
	MotorOut3-= RxInPitch;
	RxInPitch = (RxInPitch >> 1);			// cosine of 60
	MotorOut1+= RxInPitch;
	MotorOut2+= RxInPitch;
#elif defined(QUAD_COPTER)
	MotorOut1+= RxInPitch;
	MotorOut4-= RxInPitch;
#elif defined(QUAD_X_COPTER)
	RxInPitch = (RxInPitch >> 1);			// cosine of 60
	MotorOut1+= RxInPitch;
	MotorOut2+= RxInPitch;
	MotorOut3-= RxInPitch;
	MotorOut4-= RxInPitch;
#elif defined(Y4_COPTER)
	MotorOut1+= RxInPitch;
	MotorOut2+= RxInPitch;
	MotorOut3-= RxInPitch;
	MotorOut4-= RxInPitch;
#elif defined(HEX_COPTER)
	MotorOut1+= RxInPitch;
	MotorOut4-= RxInPitch;
	RxInPitch = (RxInPitch >> 2);
	MotorOut2+= RxInPitch;
	MotorOut3-= RxInPitch;
	MotorOut5-= RxInPitch;
	MotorOut6+= RxInPitch;
#elif defined(Y6_COPTER)
	MotorOut5-= RxInPitch;
	MotorOut6-= RxInPitch;
	RxInPitch = (RxInPitch >> 1);			// cosine of 60
	MotorOut1+= RxInPitch;
	MotorOut2+= RxInPitch;
	MotorOut3+= RxInPitch;
	MotorOut4+= RxInPitch;
#endif

	/* Calculate yaw output - Test without props!! */

	RxInYaw = ((int32_t)RxInYaw * (uint32_t)GainInADC[YAW]) >> STICK_GAIN_SHIFT;
	gyroADC[YAW] = ((int32_t)gyroADC[YAW] * (uint32_t)GainInADC[YAW]) >> GYRO_GAIN_SHIFT;
	if (Config.YawGyroDirection == GYRO_NORMAL)
		gyroADC[YAW] = -gyroADC[YAW];

	if (Armed) {
		error = RxInYaw - gyroADC[YAW];
		if (error > emax)
			error = emax;
		else if (error < -emax)
			error = -emax;
		integral[YAW]+= error;
		if (integral[YAW] > imax)
			integral[YAW] = imax;
		else if (integral[YAW] < -imax)
			integral[YAW] = -imax;
		derivative = error - last_error[YAW];
		last_error[YAW] = error;
		RxInYaw+= error + (integral[YAW] >> 4) + (derivative >> 4);
	}

#ifdef SINGLE_COPTER
	MotorOut2+= RxInYaw;
	MotorOut3+= RxInYaw;
	MotorOut4+= RxInYaw;
	MotorOut5+= RxInYaw;
#elif defined(DUAL_COPTER)
	MotorOut1-= RxInYaw;
	MotorOut2+= RxInYaw;
#elif defined(TWIN_COPTER)
	MotorOut3+= SERVO_REVERSE(RxInYaw >> 1);
	MotorOut4+= SERVO_REVERSE(RxInYaw >> 1);
#elif defined(TRI_COPTER)
	MotorOut4+= SERVO_REVERSE RxInYaw;
#elif defined(QUAD_COPTER)
	MotorOut1-= RxInYaw;
	MotorOut2+= RxInYaw;
	MotorOut3+= RxInYaw;
	MotorOut4-= RxInYaw;
#elif defined(QUAD_X_COPTER)
	MotorOut1-= RxInYaw;
	MotorOut2+= RxInYaw;
	MotorOut3-= RxInYaw;
	MotorOut4+= RxInYaw;
#elif defined(Y4_COPTER)
	if ((MotorOut3 - RxInYaw) < 100)
		RxInYaw = MotorOut3 - 100;	// Yaw Range Limit
	if ((MotorOut3 - RxInYaw) > 1000)
		RxInYaw = MotorOut3 - 1000;	// Yaw Range Limit

	if ((MotorOut4 + RxInYaw) < 100)
		RxInYaw = 100 - MotorOut4;	// Yaw Range Limit
	if ((MotorOut4 + RxInYaw) > 1000)
		RxInYaw = 1000 - MotorOut4;	// Yaw Range Limit

	MotorOut3-= RxInYaw;
	MotorOut4+= RxInYaw;
#elif defined(HEX_COPTER)
	MotorOut1-= RxInYaw;
	MotorOut2+= RxInYaw;
	MotorOut3-= RxInYaw;
	MotorOut4+= RxInYaw;
	MotorOut5-= RxInYaw;
	MotorOut6+= RxInYaw;
#elif defined(Y6_COPTER)
	MotorOut1-= RxInYaw;
	MotorOut4-= RxInYaw;
	MotorOut5-= RxInYaw;
	MotorOut2+= RxInYaw;
	MotorOut3+= RxInYaw;
	MotorOut6+= RxInYaw;
#endif

#if defined(TRI_COPTER)
	/*
	 * Rather than clipping the motor outputs and causing instability
	 * at throttle saturation, we pull down the throttle of the other
	 * motors. This gives priority to stabilization without a fixed
	 * collective limit.
	 */
	imax = MotorOut1;
	if (MotorOut2 > imax)
		imax = MotorOut2;
	if (MotorOut3 > imax)
		imax = MotorOut3;
	imax-= 1000;
	if (imax > 0) {
		MotorOut1-= imax;
		MotorOut2-= imax;
		MotorOut3-= imax;
	}
#endif

	imax = 114;
	//--- Limit the lowest value to avoid stopping of motor if motor value is under-saturated ---
	if (MotorOut1 < imax)
		MotorOut1 = imax;	// this is the motor idle level
	if (MotorOut2 < imax)
		MotorOut2 = imax;
#if defined(TRI_COPTER) || defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
	if (MotorOut3 < imax)
		MotorOut3 = imax;
#endif
#if defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
	if (MotorOut4 < imax)
		MotorOut4 = imax;
#endif
#if defined(HEX_COPTER) || defined(Y6_COPTER)
	if (MotorOut5 < imax)
		MotorOut5 = imax;
	if (MotorOut6 < imax)
		MotorOut6 = imax;
#endif

	//--- Output to motor ESC's ---
	if (RxInCollective < 1 || !Armed) {
		/* turn off motors unless armed and collective is non-zero */
#ifdef SINGLE_COPTER
		MotorOut1 = 0;
		MotorOut2 = 840;
		MotorOut3 = 840;
		MotorOut4 = 840;
		MotorOut5 = 840;
#elif defined(DUAL_COPTER)
		MotorOut1 = 0;
		MotorOut2 = 0;
		if (!Armed) {
			MotorOut3 = 500;
			MotorOut4 = 500;
		}
#elif defined(TWIN_COPTER)
		MotorOut1 = 0;
		MotorOut2 = 0;
		if (!Armed) {
			MotorOut3 = 500;
			MotorOut4 = 500;
			MotorOut5 = 500;
			MotorOut6 = 500;
		}
#elif defined(TRI_COPTER)
		MotorOut1 = 0;
		MotorOut2 = 0;
		MotorOut3 = 0;
		if (!Armed)
			MotorOut4 = 500;
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
		MotorOut1 = 0;
		MotorOut2 = 0;
		MotorOut3 = 0;
		MotorOut4 = 0;
#elif defined(HEX_COPTER) ||  defined(Y6_COPTER)
		MotorOut1 = 0;
		MotorOut2 = 0;
		MotorOut3 = 0;
		MotorOut4 = 0;
		MotorOut5 = 0;
		MotorOut6 = 0;
#endif
	}

	LED = 0;
	output_motor_ppm();
}

int main()
{
	setup();

	while (1)
		loop();
	return 1;
}

static void init_adc()
{
	DIDR0	= 0b00111111;	// Digital Input Disable Register - ADC5..0 Digital Input Disable
	ADCSRB	= 0b00000000;	// ADC Control and Status Register B - ADTS2:0
}

static void read_adc(uint8_t channel)
{
	ADMUX	= channel;						// set channel
	ADCSRA	= _BV(ADEN) | _BV(ADSC) | _BV(ADPS1) | _BV(ADPS2);	// 0b11000110

	while (ADCSRA & _BV(ADSC))
		;	// wait to complete
}

/*
 * ADC reads 10-bit results (0-1023), so we cannot just multiply Gyro ADC
 * by Gain ADC, or we can wrap results. Full ADC range in a 16 bit value
 * is ADC shifted left by 6, so we scale the gain to 6-bit by shifting
 * right by 10 - 6 = 4 bits.
 */
static void ReadGainPots()
{
	read_adc(3);			// read roll gain ADC3
	GainInADC[ROLL] = GAIN_POT_REVERSE ADCW;

	read_adc(4);			// read pitch gain ADC4
	GainInADC[PITCH] = GAIN_POT_REVERSE ADCW;

	read_adc(5);			// read yaw gain ADC5
	GainInADC[YAW] = GAIN_POT_REVERSE ADCW;
}

static void ReadGyros()
{
	read_adc(2);			// read roll gyro ADC2
	gyroADC[ROLL] = ADCW;

	read_adc(1);			// read pitch gyro ADC1
	gyroADC[PITCH] = ADCW;

#ifdef EXTERNAL_YAW_GYRO
	gyroADC[YAW] = 0;
#else
	read_adc(0);			// read yaw gyro ADC0
	gyroADC[YAW] = ADCW;
#endif
}

static void CalibrateGyros()
{
	uint8_t i;

	ReadGainPots();	// about time we did this !

	// get/set gyro zero value (average of 16 readings)
	gyroZero[ROLL] = 0;
	gyroZero[PITCH] = 0;
	gyroZero[YAW] = 0;

	for (i = 0;i < 16;i++) {
		ReadGyros();

		gyroZero[ROLL]+= gyroADC[ROLL];
		gyroZero[PITCH]+= gyroADC[PITCH];
		gyroZero[YAW]+= gyroADC[YAW];
	}

	gyroZero[ROLL] = (gyroZero[ROLL] + 8) >> 4;
	gyroZero[PITCH] = (gyroZero[PITCH] + 8) >> 4;
	gyroZero[YAW] = (gyroZero[YAW] + 8) >> 4;
}

/*
 * This adding 7 business is to emulate exactly a signed
 * divide at the zero point (-7 through 7 will become 0).
 */
static int16_t fastdiv8(int16_t x) {
	if (x < 0)
		x+= 7;
	return x >> 3;
}

/*
 * Copy, scale, and offset the Rx inputs from the interrupt-modified
 * registers.
 *
 * If an intterupt occurs that updates an Rx variable here, SREG will be
 * copied to i_sreg. Bit 7 will not be set in that interrupt as it is
 * cleared by hardware, so i_sreg will not ever match 0xff. We use this
 * as a zero-cost flag to read again to avoid possibly-corrupted values.
 *
 * I could not find any way to avoid the optimizer killing the retry or
 * reordering it to be unsafe other than by doing the set in inline
 * assembler with a memory barrier.
 */
static void RxGetChannels()
{
	uint8_t t = 0xff;
	do {
		asm volatile("mov %0, %1":"=r" (i_sreg),"=r" (t)::"memory");
		RxInRoll = fastdiv8(RxChannel1 - 1520 * 8);
		RxInPitch = fastdiv8(RxChannel2 - 1520 * 8);
		RxInCollective = fastdiv8(RxChannel3 - 1120 * 8);
		RxInYaw = fastdiv8(RxChannel4 - 1520 * 8);
	} while (i_sreg != t);
#ifdef TWIN_COPTER
	RxInOrgPitch = RxInPitch;
#endif
}

static void output_motor_ppm()
{
	int16_t t;

	/*
	 * Bound pulse length to 1ms <= pulse <= 2ms.
	 */

	t = 1000;
	if (MotorOut1 < 0)
		MotorOut1 = 0;
	else if (MotorOut1 > t)
		MotorOut1 = t;
#ifdef SINGLE_COPTER
	t = 2000;
#endif
	if (MotorOut2 < 0)
		MotorOut2 = 0;
	else if (MotorOut2 > t)
		MotorOut2 = t;
	if (MotorOut3 < 0)
		MotorOut3 = 0;
	else if (MotorOut3 > t)
		MotorOut3 = t;
	if (MotorOut4 < 0)
		MotorOut4 = 0;
	else if (MotorOut4 > t)
		MotorOut4 = t;
	if (MotorOut5 < 0)
		MotorOut5 = 0;
	else if (MotorOut5 > t)
		MotorOut5 = t;
	if (MotorOut6 < 0)
		MotorOut6 = 0;
	else if (MotorOut6 > t)
		MotorOut6 = t;

	t = 1000;
	MotorOut1+= t;
#ifndef SINGLE_COPTER
	MotorOut2+= t;
	MotorOut3+= t;
	MotorOut4+= t;
	MotorOut5+= t;
	MotorOut6+= t;
#endif

	MotorOut1<<= 3;
	MotorOut2<<= 3;
	MotorOut3<<= 3;
	MotorOut4<<= 3;
	MotorOut5<<= 3;
	MotorOut6<<= 3;

	/*
	 * Mirror M3, M4 to M5, M6, when possible, for hardware PPM
	 * support. The compiler will throw away the above operations on
	 * M5 and M6 when it sees these.
	 */
#if defined(DUAL_COPTER) || defined(TRI_COPTER) || defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
	MotorOut5 = MotorOut3;
	MotorOut6 = MotorOut4;
#endif

	/*
	 * We can use timer compare output mode to provide jitter-free
	 * PPM output on M1, M2, M5 and M6 by using OC0A and OC0B from
	 * timer 0 (8-bit) and OC1A and OC1B from timer 1 (16-bit) to
	 * turn off the pins. Since we are counting in steps of 1us and
	 * need to wait up to 2ms, we need to delay the turn-on of the
	 * 8-bit pins to avoid early triggering.
	 *
	 * Once entering compare match output mode, we cannot directly
	 * set the pins. We can use the "force output compare" (which
	 * doesn't actually force a compare but pretends the comparison
	 * was true) to fiddle output high or low, but this would still
	 * have interrupt and instruction-timing-induced jitter. Instead,
	 * we just set the next desired switch state and set the OCRnx
	 * registers to a known time in the future. The 8-bit ones will
	 * set the pin the same way several times, so we have to make
	 * sure that we don't change the high/low mode too early.
	 *
	 * Hardware PPM (timer compare output mode) pin mapping:
	 *
	 * M1 (PB2): OCR1B (COM1B) 16-bit
	 * M2 (PB1): OCR1A (COM1A) 16-bit
	 * M3 (PB0): software only
	 * M4 (PD7): software only
	 * M5 (PD6): OCR0A (COM0A) 8-bit
	 * M6 (PD5): OCR0B (COM0B) 8-bit
	 *
	 * We must disable interrupts while setting the 16-bit registers
	 * to avoid Rx interrupts clobbering the internal temporary
	 * register for the associated 16-bit timer. 8 cycles is one
	 * microsecond at 8 MHz, so we try not to leave interrupts
	 * disabled for more than 8 cycles.
	 *
	 * We turn OFF the pins here, then wait for the ON cycle start.
	 */
	t = MotorStartTCNT1 + MotorOut1;
	asm(""::"r" (t)); /* Avoid reordering of add after cli */
	cli();
	OCR1B = t;
	sei();
	t = MotorStartTCNT1 + MotorOut2;
	asm(""::"r" (t)); /* Avoid reordering of add after cli */
	cli();
	OCR1A = t;
	sei();
	TCCR1A = _BV(COM1A1) | _BV(COM1B1);	/* Next match will clear pins */

	/*
	 * Only 8 bits will make it to the OCR0x registers, so leave the
	 * mode as setting pins ON here and then change to OFF mode after
	 * the last wrap before the actual time.
	 *
	 * We hope that TCNT0 and TCNT1 are always synchronized.
	 */
	OCR0A = MotorStartTCNT1 + MotorOut5;
	OCR0B = MotorStartTCNT1 + MotorOut6;

	do {
		cli();
		t = TCNT1;
		sei();
		t-= MotorStartTCNT1;
		if (t >= MotorOut3)
			M3 = 0;
		if (t >= MotorOut4)
			M4 = 0;
		if (t + 0xff >= MotorOut5)
			TCCR0A&= ~_BV(COM0A0);	/* Clear pin on match */
		if (t + 0xff >= MotorOut6)
			TCCR0A&= ~_BV(COM0B0);	/* Clear pin on match */
	} while (t < ((2000 + PWM_LOW_PULSE_US) << 3) - 0xff);

	/*
	 * We should now be <= 0xff ticks before the next on cycle.
	 *
	 * Set up the timer compare values, wait for the on time, then
	 * turn on software pins. We hope that we will be called again
	 * within 1ms so that we can turn them off again in time.
	 *
	 * Timer compare output mode must stay enabled, and disables
	 * regular output when enabled. The value of the COMnx0 bits set
	 * the pin high or low when the timer value matches the OCRnx
	 * value, or immediately when forced with the FOCnx bits.
	 */

	MotorStartTCNT1+= (2000 + PWM_LOW_PULSE_US) << 3;
	cli();
	t = TCNT1;
	sei();
	if (t >= MotorStartTCNT1)
		MotorStartTCNT1 = t + 0xff;
	t = MotorStartTCNT1;
	cli();
	OCR1B = t;
	sei();
	OCR0A = t;
	OCR0B = t;
	cli();
	OCR1A = t;
	sei();

#ifdef SINGLE_COPTER
	if (servo_skip == 0) {
		TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0);
//		TCCR1C = _BV(FOC1A) | _BV(FOC1B);
		TCCR0A = _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1) | _BV(COM0B0);
//		TCCR0B = _BV(CS00) | _BV(FOC0A) | _BV(FOC0B);
	} else {
		TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(COM1B0);
//		TCCR1C = _BV(FOC1A) | _BV(FOC1B);
	}
#elif defined(DUAL_COPTER) || defined(TWIN_COPTER)
	TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0);
//	TCCR1C = _BV(FOC1A) | _BV(FOC1B);
	if (servo_skip == 0) {
		TCCR0A = _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1) | _BV(COM0B0);
//		TCCR0B = _BV(CS00) | _BV(FOC0A) | _BV(FOC0B);
	}
#elif defined(TRI_COPTER)
	TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0);
//	TCCR1C = _BV(FOC1A) | _BV(FOC1B);
	if (servo_skip == 0) {
		TCCR0A = _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1) | _BV(COM0B0);
//		TCCR0B = _BV(CS00) | _BV(FOC0A) | _BV(FOC0B);
	} else {
		TCCR0A = _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1);
//		TCCR0B = _BV(CS00) | _BV(FOC0A) | _BV(FOC0B);
	}
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER) || defined(HEX_COPTER) || defined(Y6_COPTER)
	TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0);
//	TCCR1C = _BV(FOC1A) | _BV(FOC1B);
	TCCR0A = _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1) | _BV(COM0B0);
//	TCCR0B = _BV(CS00) | _BV(FOC0A) | _BV(FOC0B);
#endif

	/*
	 * Wait for the on time so we can turn on the software pins.
	 */
	do {
		cli();
		t = TCNT1;
		sei();
	} while (t < MotorStartTCNT1);

#ifdef SINGLE_COPTER
	if (servo_skip == 0) {
		M3 = 1;
		M4 = 1;
		servo_skip = servo_skip_divider;
	}
	servo_skip--;
#elif defined(DUAL_COPTER) || defined(TWIN_COPTER)
	if (servo_skip == 0) {
		M3 = 1;
		M4 = 1;
		servo_skip = servo_skip_divider;
	}
	servo_skip--;
#elif defined(TRI_COPTER)
	M3 = 1;
	if (servo_skip == 0) {
		M4 = 1;
		servo_skip = servo_skip_divider;
	}
	servo_skip--;
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER) || defined(HEX_COPTER) || defined(Y6_COPTER)
	M3 = 1;
	M4 = 1;
#endif
	/*
	 * We leave with the output pins ON.
	 */
}

static void eeprom_write_byte_changed(uint8_t *addr, uint8_t value)
{
	if (eeprom_read_byte(addr) != value)
		eeprom_write_byte(addr, value);
}

static void eeprom_write_block_changes(const uint8_t *src, void *dest, size_t size)
{
	size_t len;

	for (len = 0;len < size;len++) {
		eeprom_write_byte_changed(dest, *src);
		src++;
		dest++;
	}
}

static void Initial_EEPROM_Config_Load()
{
	// load up last settings from EEPROM
	if (eeprom_read_byte((uint8_t *)EEPROM_DATA_START_POS) != 42) {
		Config.setup = 42;
		Set_EEPROM_Default_Config();
		// write to eeProm
		Save_Config_to_EEPROM();
	} else {
		// read eeprom
		eeprom_read_block(&Config, (void *)EEPROM_DATA_START_POS, sizeof(struct config));
	}
}

static void Set_EEPROM_Default_Config()
{
	Config.RollGyroDirection	= GYRO_REVERSED;
	Config.PitchGyroDirection	= GYRO_REVERSED;
	Config.YawGyroDirection		= GYRO_NORMAL;
}

static void Save_Config_to_EEPROM()
{
	cli();
	eeprom_write_block_changes(
		(const void *)&Config, (void *)EEPROM_DATA_START_POS,
		sizeof(struct config));
	sei();
}
