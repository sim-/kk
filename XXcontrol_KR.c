// XXcopter software
// ============
// Version XXcontrol_KR_v1.5
// inspired by KKmulticopter
// based on assembly code by Rolf R Bakke
// Converted by Mike Barton
//
// Modified by Minsoo Kim
//
// Optimized for Korean Blue Board ( www.kkmulticopter.kr )
// supports 168 and 328 
//
// Copter Types*: SINGLE_COPTER, DUAL_COPTER, TWIN_COPTER, TRI_COPTER, QUAD_COPTER, QUAD_X_COPTER, Y4_COPTER, HEX_COPTER, Y6_COPTER
//
// Config Modes (at startup)
// =================
// http://www.kkmulticopter.kr/index.html?modea=manual
//
// Version History
// ==========
// _KR_v1.5	TwinCopter was added.
//			Y4Copter was added.
//			TriCopter Servo Reverse Pin was added(M5)
//			Increased Acro Stick Gain(65% to 70%)
//
// _KR_v1.4	Fixed bugs in case of yaw reversed at TriCopter
//			Default Set 3 for GAIN_MULTIPLIER
//
// _KR_v1.3	DualCopter was added.
//			Fixed bugs at ReadGainPots
//
// _KR_v1.2	SingleCopter was added.
//
// _KR_v1.1	Audible arming sound (Mike's source)
//			PWM refresh rate 495Hz (Mike's & manuLRK's source)
//			Default Set 2 for GAIN_MULTIPLIER
//			Calibrate Gyros when mode changed
//
// _KR_v1.0	Optimized for Korean Blue Board(www.kkmulticopter.kr)
//			Servo Noise Filter at Tri
//			Normal, Acro, UFO mode change by transmitter
//			Calibrate gyro when Thr: Low, Elevator: Down, Rudder: Left (Normal Mode)
//			Delay Stick Centering for Receiver Binding
//			Arming Time 0.5sec and changed to Right-Side Arming
//			User Delay Procedure
//			used XXcontrol v1.0i

/*
Single
                    M1 CCW
                     |
                     |
                     |


                    M2 (Servo)
                     |
                     |
        M5 ----    ----M3
    (Servo)      |     (Servo)
                     |
                    M4 (Servo)

Dual
                    M1 CCW
                     |
                    M2 CW
                     |

                     |
                     |
        M3 ----------
    (Servo)      |
                     |
                    M4 (Servo)

Twin
                / --- \
               /    |    \ 
         M1 CW  |     M2 CCW
                     |
            M3     |        M4
        (Servo)  |    (Servo)
                     |  
			|  
                    M5 (Tail Servo, Optional)
                    M6 (Tail Servo Reverse, Optional)

Tri   
 		  
         M1 CW          M2 CCW
              \           / 
                \ --- /
                  |   |
                   --- 
                     |  
			|  
                    M3 CCW   M4=Tail Servo or M5=Tail Servo Reverse

Quad
                    M1 CW
                     |
                     |
                     |
                 +---+
CCW M2----|   |----M3 CCW
                 +---+
                     |
                     |
                     |
                   M4 CW

Quad-X   
           M1 CW    M2 CCW
               \         / 
                \ --- /
                  |   |
                / --- \
               /         \ 
           M4  CCW   M3 CW
 
Y4
             M1 CW    M2 CCW
                \         /
                 \ --- /
                   |   |
                    --- 
                     |  
                     |
                    M3 CW/M4 CCW

Hex
                   M1 CW
                     |
          M6       |       M2 CCW
             \       |      /
              \ +---+  /
                 -|   |- 
              / +---+  \
             /      |       \
          M5       |       M3 CW
                     |
                    M4

Y6     
             M1/2     M3/4    M1,4,5 = CW
                \         /         M2,3,6 = CCW
                 \ --- /
                   |   |
                    --- 
                     |  
                     |
                    M5/6
*/

/* ----------- Configuration -----------  */
//#define SINGLE_COPTER
//#define DUAL_COPTER
//#define TWIN_COPTER
#define TRI_COPTER
//#define QUAD_COPTER
//#define QUAD_X_COPTER
//#define Y4_COPTER
//#define HEX_COPTER
//#define Y6_COPTER

//#define SERVO_REVERSE		// For TwinCopter

// defines output rate to ESC/Servo
// either define by setting ESC_RATE (Max is approx 495Hz)
// uses Timer 1 ticks to control output rate.
//#define ESC_RATE 300	// in Hz
//#define ESC_RATE 400	// in Hz (at SINGLE_COPTER Only)
#define ESC_RATE 450	// in Hz
//#define ESC_RATE 495	// in Hz

#define PWM_LOW_PULSE_INTERVAL  (((F_CPU / 8) / ESC_RATE) - 2000)/10
//#define PWM_LOW_PULSE_INTERVAL  ((1000000 / ESC_RATE) - 2000)/10
//or define by setting PWM_LOW_PULSE_INTERVAL (minimum is 1)

// Adjust these:
// 		down if you have too much gyro assistance
// 		up if you have maxxed your gyro gain 
#define ROLL_GAIN_DIVIDER 	32	/* Was about 64 */
#define PITCH_GAIN_DIVIDER 	32	/* Was about 64 */
#define YAW_GAIN_DIVIDER 	32	/* Was about 64 */

// Stick Gain
//#define NORMAL_STICK_ROLL_GAIN		100		// Stick %, Normal: 50, Acro: 60~70
//#define NORMAL_STICK_PITCH_GAIN		100		// Stick %, Normal: 50, Acro: 60~70
//#define NORMAL_STICK_YAW_GAIN		100		// Stick %, Normal: 50, Acro: 60~70

#define ACRO_STICK_ROLL_GAIN		70		// Stick %, Normal: 50, Acro: 60~70
#define ACRO_STICK_PITCH_GAIN		70		// Stick %, Normal: 50, Acro: 60~70
#define ACRO_STICK_YAW_GAIN			70		// Stick %, Normal: 50, Acro: 60~70
#define UFO_STICK_YAW_GAIN			90		// Stick %, Normal: 50, Acro: 60~70, UFO: 80~90
//#define ADC_GAIN_DIVIDER			100		// Gyro Value Range (-100~100: 150, -150~150: 225, -250~250: 375)

// Stick Arming - enable this line to enable Stick arming
#define STICK_ARMING 60

// enable this line if you don't have Yaw gyro connected
//#define EXTERNAL_YAW_GYRO

// Max Collective
// limits the maximum stick collective (range 80->100  100=Off)
// this allows gyros to stabilise better when full throttle applied
#define MAX_COLLECTIVE 90			// 95

/* ----------- Main Code -----------  */

#include <avr/io.h>  
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include <avr/interrupt.h> 
#include <avr/eeprom.h>

#include "typedefs.h"
#include "io_cfg.h"

#define EEPROM_DATA_START_POS 0		// allows moving (not really necessary)

// Max ADC
#define UC_ADC_MAX 1023				// used to invert ADC reading of uC

enum GyroDirection { GYRO_NORMAL = 0, GYRO_REVERSED };

enum GyroArrayIndex { ROLL = 0, PITCH, YAW };

// eeProm data structure
typedef struct Config_Struct CONFIG_STRUCT;
struct Config_Struct
{
	uint8_t	setup;					// byte to identify if already setup

	uint8_t RollGyroDirection;
	uint8_t PitchGyroDirection;
	uint8_t YawGyroDirection;

	// allows setting to zero
	uint16_t RxChannel1ZeroOffset;
	uint16_t RxChannel2ZeroOffset;
	uint16_t RxChannel3ZeroOffset;  // currently fixed
	uint16_t RxChannel4ZeroOffset;

};

CONFIG_STRUCT Config;				// Holds configuration (from eeProm)

bool GyroCalibrated;
//volatile BOOL RxChannelsUpdatingFlag;

bool Armed;

uint16_t GainInADC[3];				// ADC result
uint16_t GainIn[3];					// = GainInADC[3]/10

#ifdef SINGLE_COPTER
int16_t LowpassOutServo[3];			// Lowpass Out
#elif defined(DUAL_COPTER) || defined(TWIN_COPTER)
int16_t LowpassOutServo[2];
#elif defined(TRI_COPTER)
int16_t LowpassOutYaw;
#endif

#if defined(SINGLE_COPTER) || defined(DUAL_COPTER) || defined(TWIN_COPTER) || defined(TRI_COPTER)
uint16_t ServoPPMRateDivider = 0;
#endif

volatile uint16_t RxChannel1;		// ISR vars
volatile uint16_t RxChannel2;
volatile uint16_t RxChannel3;
volatile uint16_t RxChannel4;

//uint16_t RxChannel1Start;			// ISR vars
//uint16_t RxChannel2Start;
//uint16_t RxChannel3Start;
//uint16_t RxChannel4Start;

int16_t RxInRoll;					// program vars
int16_t RxInPitch;
int16_t RxInCollective;
int16_t RxInYaw;

register uint16_t RxChannel1Start asm("r2");			// ISR vars
register uint16_t RxChannel2Start asm("r4");
register uint16_t RxChannel3Start asm("r6");
register uint16_t RxChannel4Start asm("r8");

//register int16_t RxInRoll asm("r10");					// program vars
//register int16_t RxInPitch asm("r12");
//register int16_t RxInCollective asm("r14");
//register int16_t RxInYaw asm("r16");
#ifdef TWIN_COPTER
int16_t RxInOrgPitch;
#endif

int16_t gyroADC[3];					// Holds Gyro ADC's
int16_t gyroZero[3] = {0,0,0};		// used for calibrating Gyros on ground

uint16_t ModeDelayCounter;

//register uint16_t t asm("r6");

bool output_motor_high = false;
uint16_t PWM_Low_Pulse_Interval = PWM_LOW_PULSE_INTERVAL;		// non-const version of PWM_LOW_PULSE_INTERVAL

int16_t MotorOut1;
int16_t MotorOut2;
int16_t MotorOut3;
int16_t MotorOut4;
#if defined(SINGLE_COPTER) || defined(TWIN_COPTER) || defined(TRI_COPTER) || defined(HEX_COPTER) || defined(Y6_COPTER)
int16_t MotorOut5;
#endif
#if defined(TWIN_COPTER) || defined(HEX_COPTER) || defined(Y6_COPTER)
int16_t MotorOut6;
#endif

void setup(void);
void loop(void);

void Init_ADC(void);
void ReadGyros(bool calibrate);
void CalibrateGyros(void);

void ReadGainPots(void);
void RxGetChannels(void);
void read_adc(uint8_t channel);

void output_motor_ppm(void);

void Initial_EEPROM_Config_Load(void);
void Save_Config_to_EEPROM(void);
void Set_EEPROM_Default_Config(void);

void eeprom_write_byte_changed( uint8_t *  addr, uint8_t value );
void eeprom_write_block_changes( const uint8_t * src, void * dest, size_t size );

void delay_us(uint8_t time);
void delay_ms(uint16_t time);

// RX_ROLL
ISR(PCINT2_vect)
{
//	register uint8_t tisp asm("r18");
//	asm volatile("push %0" : "=r" (tisp) : "0" (tisp));
//	tisp = SREG;
	if ( RX_ROLL )			// rising
	{
		RxChannel1Start = TCNT1;
	} else {				// falling
//		RxChannelsUpdatingFlag = 1;
		RxChannel1 = TCNT1 - RxChannel1Start;
//		RxChannelsUpdatingFlag = 0;
	}
//	asm volatile("out 0x3f, r0");
//	SREG = tisp;
//	asm volatile("pop %0" : "=r" (tisp));
}

// RX_PITCH
ISR(INT0_vect)
{
	if (RX_PITCH)		
	{
		RxChannel2Start = TCNT1;
	} else {				// falling
//		RxChannelsUpdatingFlag = 1;
		RxChannel2 = TCNT1 - RxChannel2Start;
//		RxChannelsUpdatingFlag = 0;
	}
}

// RX_COLL
ISR(INT1_vect)
{
	if (RX_COLL)		
	{
		RxChannel3Start = TCNT1;

	} else {				// falling
//		RxChannelsUpdatingFlag = 1;
		RxChannel3 = TCNT1 - RxChannel3Start;
//		RxChannelsUpdatingFlag = 0;
	}
}

// RX_YAW
ISR(PCINT0_vect)
{
	
	if ( RX_YAW )			// rising
	{
		RxChannel4Start = TCNT1;

	} else {				// falling
//		RxChannelsUpdatingFlag = 1;
		RxChannel4 = TCNT1 - RxChannel4Start;
//		RxChannelsUpdatingFlag = 0;
	}
}


int main(void)
{
	setup();

	while (1)
	{
		loop();
	}

	return 1;
}

void setup(void)
{
	uint16_t i;	// nb was uint8_t, must be uint16_t for TRI
	uint16_t RxChannel1ZeroOffset, RxChannel2ZeroOffset, RxChannel4ZeroOffset;

	MCUCR |= (1<<PUD);	// Pull-up Disable

	RX_ROLL_DIR 		= INPUT;
	RX_PITCH_DIR 		= INPUT;
	RX_COLL_DIR   		= INPUT;
	RX_YAW_DIR   	 	= INPUT;

	GYRO_YAW_DIR 	 	= INPUT;
	GYRO_PITCH_DIR 	 	= INPUT;
	GYRO_ROLL_DIR  		= INPUT;
	GAIN_YAW_DIR 	 	= INPUT;
	GAIN_PITCH_DIR		= INPUT;
	GAIN_ROLL_DIR  		= INPUT;

	M1_DIR 				= OUTPUT;
	M2_DIR 				= OUTPUT;
	M3_DIR 			 	= OUTPUT;
	M4_DIR 			 	= OUTPUT;
#if defined(SINGLE_COPTER) || defined(TWIN_COPTER) || defined(TRI_COPTER) || defined(HEX_COPTER) || defined(Y6_COPTER)
	M5_DIR 				= OUTPUT;
#endif
#if defined(TWIN_COPTER) || defined(HEX_COPTER) || defined(Y6_COPTER)
	M6_DIR 				= OUTPUT;
#endif
	LED_DIR 			= OUTPUT;

	LED			= 0;
	RX_ROLL 	= 0;
	RX_PITCH 	= 0;
	RX_COLL  	= 0;
	RX_YAW   	= 0;

	// pin change interrupt enables
	PCICR |= (1 << PCIE0);			// PCINT0..7		
	PCICR |= (1 << PCIE2);			// PCINT16..23

	// pin change masks
	PCMSK0 |= (1 << PCINT7);		// PB7
	PCMSK2 |= (1 << PCINT17);		// PD1
	// external interrupts
	EICRA  = (1 << ISC00) | (1 << ISC10);	// Any change INT0, INT1
	EIMSK  = (1 << INT0) | (1 << INT1);		// External Interrupt Mask Register
	EIFR |= (1 << INTF0) | (1 << INTF1);

	// timer0 (8bit) - run @ 8MHz
	// used to control ESC/servo pulse length
	TCCR0A = 0;						// normal operation
	TCCR0B = (1 << CS00);			// clk/0
	TIMSK0 = 0; 					// no interrupts

	// timer1 (16bit) - run @ 1Mhz
	// used to measure Rx Signals & control ESC/servo output rate
	TCCR1A = 0;
	TCCR1B = (1 << CS11);

	// timer2 8bit - run @ 8MHz / 1024 = 7812.5KHz
	// and Stick-Arming
	TCCR2A = 0;	
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);	// /1024
	TIMSK2 = 0;
	TIFR2  = 0;
	TCNT2 = 0;		// reset counter

#if defined(SINGLE_COPTER) || defined(DUAL_COPTER) || defined(TWIN_COPTER) || defined(TRI_COPTER)
	// calculate Servo Rate divider
	ServoPPMRateDivider = 0;
	do {
		ServoPPMRateDivider++;
		i = ESC_RATE / ServoPPMRateDivider;
	} while (i>250);
#endif

#ifdef SINGLE_COPTER
	LowpassOutServo[ROLL]	= 84;					// Center
	LowpassOutServo[PITCH]	= 84;					// Center
	LowpassOutServo[YAW]	= 84;					// Center
#elif defined(DUAL_COPTER)
	LowpassOutServo[ROLL]	= 50;					// Center
	LowpassOutServo[PITCH]	= 50;					// Center
#elif defined(TWIN_COPTER)
	LowpassOutServo[0]		= 50;					// Pitch
	LowpassOutServo[1]		= 50;					// Yaw
#elif defined(TRI_COPTER)
	LowpassOutYaw			= 50;					// Center
#endif

	Initial_EEPROM_Config_Load();					// loads config at start-up 

	Init_ADC();

	GyroCalibrated = false;
	Armed = false;
//	RxChannelsUpdatingFlag = 0;

	RxChannel1 = Config.RxChannel1ZeroOffset;		// prime the channels 1520;
	RxChannel2 = Config.RxChannel2ZeroOffset;		// 1520;
	RxChannel3 = Config.RxChannel3ZeroOffset;		// 1120;
	RxChannel4 = Config.RxChannel4ZeroOffset;		// 1520;

	// flash LED
	LED = 1;
	delay_ms(150);
	LED = 0;

	sei();											// Global Interrupts 

	// 2 second delay
	delay_ms(1500);
	
	ReadGainPots();
	ReadGainPots();

	// clear config
	if (GainInADC[PITCH] < (UC_ADC_MAX*5)/100 && GainInADC[ROLL]  < (UC_ADC_MAX*5)/100 && GainInADC[YAW]   < (UC_ADC_MAX*5)/100 )
	{
		Set_EEPROM_Default_Config();

		while ( 1 );
	} 

	// Stick Centering
	if (GainInADC[PITCH] < (UC_ADC_MAX*5)/100)		// less than 5%
	{
	    // set offsets to zero (otherwise we affect what we want to calibrate !!)
	    Config.RxChannel1ZeroOffset  = 0;
	    Config.RxChannel2ZeroOffset  = 0;
	    Config.RxChannel4ZeroOffset  = 0;

		// flash LED 3 times
		for (i=0;i<3;i++)
		{
			LED = 1;
			delay_ms(25);
			LED = 0;
			delay_ms(25);
		}
		// 5 Seconds Delay, for binding
		delay_ms(3750);

		RxChannel1ZeroOffset = RxChannel2ZeroOffset = RxChannel4ZeroOffset = 0;
		
		for (i=0;i<4;i++)
		{
	 		RxGetChannels();

		    RxChannel1ZeroOffset += RxInRoll;
		    RxChannel2ZeroOffset += RxInPitch;
		    RxChannel4ZeroOffset += RxInYaw;

			delay_ms(100);
		}
		// nb RxGetChannels() divides RxInXXX by 4 so we won't here
	    Config.RxChannel1ZeroOffset  = RxChannel1ZeroOffset;
	    Config.RxChannel2ZeroOffset  = RxChannel2ZeroOffset;
	    Config.RxChannel3ZeroOffset  = 1120;
	    Config.RxChannel4ZeroOffset  = RxChannel4ZeroOffset;

		// Store gyro direction to EEPROM
		Save_Config_to_EEPROM();

		// flash LED, Ending Sign
		LED = 1;
		delay_ms(150);
		LED = 0;
	}


	// Gyro direction reversing
	if (GainInADC[ROLL] < (UC_ADC_MAX*5)/100)		// less than 5% (5/100) * 1023 = 51 
	{
		// flash LED 3 times
		for (i=0;i<3;i++)
		{
			LED = 1;
			delay_ms(25);
			LED = 0;
			delay_ms(25);
		}

		while(1)
		{
			RxGetChannels();

			if (RxInRoll < -30) {	// normal(left)
				Config.RollGyroDirection = GYRO_NORMAL;
				Save_Config_to_EEPROM();
				LED = 1;
			} if (RxInRoll > 30) {	// reverse(right)
				Config.RollGyroDirection = GYRO_REVERSED;
				Save_Config_to_EEPROM();
				LED = 1;
			} else if (RxInPitch < -30) { // normal(up)
				Config.PitchGyroDirection = GYRO_NORMAL;
				Save_Config_to_EEPROM();
				LED = 1;
			} else if (RxInPitch > 30) { // reverse(down)
				Config.PitchGyroDirection = GYRO_REVERSED;
				Save_Config_to_EEPROM();
				LED = 1;
			} else if (RxInYaw < -30) { // normal(left)
				Config.YawGyroDirection = GYRO_NORMAL;
				Save_Config_to_EEPROM();
				LED = 1;
			} else if (RxInYaw > 30) { // reverse(right)
				Config.YawGyroDirection = GYRO_REVERSED;
				Save_Config_to_EEPROM();
				LED = 1;
			}
	
			delay_ms(50);
			LED = 0;

		}
	}

	// ESC throttle calibration
	if (GainInADC[YAW] < (UC_ADC_MAX*5)/100)		// less than 5%
	{
		// flash LED 3 times
		for (i=0;i<3;i++)
		{
			LED = 1;
			delay_ms(25);
			LED = 0;
			delay_ms(25);
		}

		Armed = true;	// override so that output_motor_pwm() won't quit early
		PWM_Low_Pulse_Interval = ((1000000UL / 50) - 2000)/10;	// set to 50Hz
#if defined(SINGLE_COPTER) || defined(DUAL_COPTER) || defined(TWIN_COPTER) || defined(TRI_COPTER)
		ServoPPMRateDivider = 1;	// since we have already set to 50Hz
#endif
		
		while (1)	// loop forever
		{
			RxGetChannels();
#ifdef SINGLE_COPTER
			MotorOut1 = RxInCollective;
			MotorOut2 = 140;		// Center: 140
			MotorOut3 = 140;
			MotorOut4 = 140;
			MotorOut5 = 140;
#elif defined(DUAL_COPTER)
			MotorOut1 = RxInCollective;
			MotorOut2 = RxInCollective;
			MotorOut3 = 50;			// Center: 50
			MotorOut4 = 50;
#elif defined(TWIN_COPTER)
			MotorOut1 = RxInCollective;
			MotorOut2 = RxInCollective;
			MotorOut3 = 50;			// Center: 50
			MotorOut4 = 50;
			MotorOut5 = 50;
			MotorOut6 = 50;			// Center: 50, Reverse
#elif defined(TRI_COPTER)
			MotorOut1 = RxInCollective;
			MotorOut2 = RxInCollective;
			MotorOut3 = RxInCollective;
			MotorOut4 = 50;			// Center: 50
			MotorOut5 = 50;			// Center: 50, Reverse
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

void loop(void)
{
//	static uint8_t i;
	static uint16_t Change_Arming=0;
	static uint8_t Arming_TCNT2=0;

	RxGetChannels();

	if (RxInCollective < 0) {
		// check for stick arming (Timer2 @ 8MHz/1024 = 7812.5KHz)
		// arm: yaw right (>60), dis-arm: yaw left (<-60)
		Change_Arming += (uint8_t) (TCNT2 - Arming_TCNT2);
		Arming_TCNT2 = TCNT2;

		if (Armed) {		// nb to switch to Right-Side Arming: if (!Armed)
			if (RxInYaw<STICK_ARMING || abs(RxInPitch) > 30) 	Change_Arming = 0;		// re-set count
		} else {
			if (RxInYaw>-STICK_ARMING || abs(RxInPitch) > 30) 	Change_Arming = 0;		// re-set count
		}

		// 3Sec / 0.000128 = 23437 = 0x5B8D or 
		// 2.5Sec / 0.000128 = 19531 = 0x4C4B
		// 0.5Sec / 0.000128 = 3906 = 0x0F42
		if (Change_Arming>0x0F42)
		{
			Armed = ! Armed;
			LED = 0;
			ModeDelayCounter = 0;

			if (Armed) {
				CalibrateGyros();
//				output_motor_high = false;	// re-set 1st time flag
				LED = 1;

//			} else if (output_motor_high) {
//				output_motor_ppm();			// turn off
			}
			return;
		}

		// --- Calibrate gyro when collective below 1% ---
		//if ( RxInCollective < 1 && Armed && abs(RxInRoll) <20 && abs(RxInPitch) <20)
		// --- Calibrate gyro when Thr: Low, Elevator: Down, Rudder: Left ---
		if (Armed && RxInYaw < -STICK_ARMING && RxInPitch > STICK_ARMING)
		{
			if (ModeDelayCounter==0)
			{
				//ModeDelayCounter = 0xFB4F;	// 0xFFFF-FB4F=0x4B0=1200/400 = 3Seconds
				ModeDelayCounter = 0xFE6F;	// 0xFFFF-FE6F=0x190=400/400 = 1Seconds
				CalibrateGyros();
#if 0
				output_motor_high = false;	// re-set 1st time flag
				delay_ms(150);
				// Normal
				StickRollGain = NORMAL_STICK_ROLL_GAIN;
				StickPitchGain = NORMAL_STICK_PITCH_GAIN;
				StickYawGain = NORMAL_STICK_YAW_GAIN;
#endif
#if 0
				// flash LED 1 time
				for (i=0;i<1;i++)
				{
					LED = 0;
					delay_ms(25);
					LED = 1;
					delay_ms(25);
				}
#endif
			}
			ModeDelayCounter++;
		}
	}
	//--- Read gyros ---
	ReadGyros(false);

	//--- Start mixing by setting collective to motor input 1,2,3,4 and 5,6
#ifndef SINGLE_COPTER
	if (RxInCollective > MAX_COLLECTIVE) RxInCollective = MAX_COLLECTIVE;
#endif

#ifdef SINGLE_COPTER
	MotorOut1 = RxInCollective;
	MotorOut2 = 84;		// 84;
	MotorOut3 = 84;		// 84;
	MotorOut4 = 94;		// 84 + 84/8; Adjust LowPassOutValue
	MotorOut5 = 94;		// 84 + 84/8; Adjust LowPassOutValue
#elif defined(DUAL_COPTER)
	MotorOut1 = RxInCollective;
	MotorOut2 = RxInCollective;
	MotorOut3 = 50;
	MotorOut4 = 50;
#elif defined(TWIN_COPTER)
	MotorOut1 = RxInCollective;
	MotorOut2 = RxInCollective;
	MotorOut3 = 50;
	MotorOut4 = 50;
	MotorOut5 = 50;		// Optional
	MotorOut6 = 50;		// Optional
#elif defined(TRI_COPTER)
	MotorOut1 = RxInCollective;
	MotorOut2 = RxInCollective;
	MotorOut3 = RxInCollective;
	MotorOut4 = 50;
	MotorOut5 = 50;		// Reverse
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER)
	MotorOut1 = RxInCollective;
	MotorOut2 = RxInCollective;
	MotorOut3 = RxInCollective;
	MotorOut4 = RxInCollective;
#elif defined(Y4_COPTER)
	MotorOut1 = RxInCollective;
	MotorOut2 = RxInCollective;
	MotorOut3 = (RxInCollective * 3 / 4);		// 25% Down
	MotorOut4 = (RxInCollective * 3 / 4);		// 25% Down
#elif defined(HEX_COPTER) ||  defined(Y6_COPTER)
	MotorOut1 = RxInCollective;
	MotorOut2 = RxInCollective;
	MotorOut3 = RxInCollective;
	MotorOut4 = RxInCollective;
	MotorOut5 = RxInCollective;
	MotorOut6 = RxInCollective;
#endif

	//--- Calculate roll gyro output ---
	// nb IF YOU CHANGE THIS CODE, YOU MUST REMOVE PROPS BEFORE TESTING !!!
	gyroADC[ROLL] = gyroADC[ROLL] * GainIn[ROLL];
	gyroADC[ROLL] /= ROLL_GAIN_DIVIDER;
//	RxInRoll = (RxInRoll * StickRollGain / 100);	// Stick Controll %

	//--- (Add)Adjust roll gyro output to motors
	if (Config.RollGyroDirection == GYRO_NORMAL) {
		RxInRoll += gyroADC[ROLL];
	} else {
		RxInRoll -= gyroADC[ROLL];
	}

#ifdef SINGLE_COPTER
	RxInRoll += LowpassOutServo[ROLL];
	RxInRoll = (RxInRoll >> 3);
	LowpassOutServo[ROLL] -= RxInRoll;

	MotorOut2 += LowpassOutServo[ROLL];
	MotorOut4 -= LowpassOutServo[ROLL];
#elif defined(DUAL_COPTER)
	RxInRoll += LowpassOutServo[ROLL];
	RxInRoll = (RxInRoll >> 3);
	LowpassOutServo[ROLL] -= RxInRoll;

	MotorOut4 += LowpassOutServo[ROLL];
#elif defined(TWIN_COPTER)
	RxInRoll   = (RxInRoll * 20)/23;	//RxInRollSine 60= 0.866 ~ 20/23 or possibly 7/8
	MotorOut1 += RxInRoll;
	MotorOut2 -= RxInRoll;
#elif defined(TRI_COPTER)
	RxInRoll   = (RxInRoll * 20)/23;	//RxInRollSine 60= 0.866 ~ 20/23 or possibly 7/8
	MotorOut1 += RxInRoll;
	MotorOut2 -= RxInRoll;
#elif defined(QUAD_COPTER)
	MotorOut2 += RxInRoll;
	MotorOut3 -= RxInRoll;
#elif defined(QUAD_X_COPTER)
	RxInRoll   = (RxInRoll >> 1);		//was:RxInRoll	= (RxInRoll * 20)/23;
	MotorOut1 += RxInRoll;
	MotorOut2 -= RxInRoll;
	MotorOut3 -= RxInRoll;
	MotorOut4 += RxInRoll;
#elif defined(Y4_COPTER)
	RxInRoll   = (RxInRoll * 20)/23;	//RxInRollSine 60= 0.866 ~ 20/23 or possibly 7/8
	MotorOut1 += RxInRoll;
	MotorOut2 -= RxInRoll;
#elif defined(HEX_COPTER)
	RxInRoll   = (RxInRoll * 20)/23;	//RxInRoll  *= 0.866;	// Sine 60
	MotorOut2 -= RxInRoll;
	MotorOut3 -= RxInRoll;
	MotorOut5 += RxInRoll;
	MotorOut6 += RxInRoll;
#elif defined(Y6_COPTER)
	RxInRoll   = (RxInRoll * 20)/23;	//RxInRoll  *= 0.866;	// Sine 60
	MotorOut1 += RxInRoll;
	MotorOut2 += RxInRoll;
	MotorOut3 -= RxInRoll;
	MotorOut4 -= RxInRoll;
#endif

	//--- Calculate pitch gyro output ---
	// nb IF YOU CHANGE THIS CODE, YOU MUST REMOVE PROPS BEFORE TESTING !!!
	gyroADC[PITCH] = gyroADC[PITCH] * GainIn[PITCH];
	gyroADC[PITCH] /= PITCH_GAIN_DIVIDER; 
//	RxInPitch = (RxInPitch * StickPitchGain / 100);	// Stick Controll %

	//--- (Add)Adjust pitch gyro output to motors
	if (Config.PitchGyroDirection == GYRO_NORMAL) {	
		RxInPitch += gyroADC[PITCH];
	} else {
		RxInPitch -= gyroADC[PITCH];
	}

#ifdef SINGLE_COPTER
	RxInPitch += LowpassOutServo[PITCH];
	RxInPitch = (RxInPitch >> 3);
	LowpassOutServo[PITCH] -= RxInPitch;

	MotorOut3 += LowpassOutServo[PITCH];
	MotorOut5 -= LowpassOutServo[PITCH];
#elif defined(DUAL_COPTER)
	RxInPitch += LowpassOutServo[PITCH];
	RxInPitch = (RxInPitch >> 3);
	LowpassOutServo[PITCH] -= RxInPitch;

	MotorOut3 += LowpassOutServo[PITCH];
#elif defined(TWIN_COPTER)
	RxInPitch -= LowpassOutServo[0];
	RxInPitch = (RxInPitch >> 3);
	LowpassOutServo[0] += RxInPitch;

	#ifdef SERVO_REVERSE
	MotorOut3 += LowpassOutServo[0];
	MotorOut4 -= LowpassOutServo[0];
	#else
	MotorOut3 -= LowpassOutServo[0];
	MotorOut4 += LowpassOutServo[0];
	#endif

	// Stick Only, Optional
	RxInOrgPitch = abs(RxInOrgPitch * StickPitchGain / 100);	// Stick Controll %
	MotorOut5 += RxInOrgPitch;							// Tain Servo-Optional, Down Only
	MotorOut6 -= RxInOrgPitch;							// Tain Servo-Optional, Down Only (Reverse)
#elif defined(TRI_COPTER)
	MotorOut3 -= RxInPitch;
	RxInPitch = (RxInPitch >> 1);						// cosine of 60
	MotorOut1 += RxInPitch;
	MotorOut2 += RxInPitch;
#elif defined(QUAD_COPTER)
	MotorOut1 += RxInPitch;
	MotorOut4 -= RxInPitch;
#elif defined(QUAD_X_COPTER)
	RxInPitch = (RxInPitch >> 1);						// cosine of 60
	MotorOut1 += RxInPitch;
	MotorOut2 += RxInPitch;
	MotorOut3 -= RxInPitch;
	MotorOut4 -= RxInPitch;
#elif defined(Y4_COPTER)
	MotorOut1 += RxInPitch;
	MotorOut2 += RxInPitch;
	MotorOut3 -= RxInPitch;
	MotorOut4 -= RxInPitch;
#elif defined(HEX_COPTER)
	MotorOut1 += RxInPitch;
	MotorOut4 -= RxInPitch;
	RxInPitch = (RxInPitch >> 2);
	MotorOut2 += RxInPitch;	
	MotorOut3 -= RxInPitch;
	MotorOut5 -= RxInPitch;
	MotorOut6 += RxInPitch;
#elif defined(Y6_COPTER)
	MotorOut5 -= RxInPitch;
	MotorOut6 -= RxInPitch;
	RxInPitch = (RxInPitch >> 1);						// cosine of 60
	MotorOut1 += RxInPitch;
	MotorOut2 += RxInPitch;
	MotorOut3 += RxInPitch;
	MotorOut4 += RxInPitch;
#endif

	//--- Calculate yaw gyro output ---
	gyroADC[YAW] = gyroADC[YAW] * GainIn[YAW];
	gyroADC[YAW] /= YAW_GAIN_DIVIDER;
//	RxInYaw = (RxInYaw * StickYawGain / 100);			// Stick Controll %

	//--- (Add)Adjust yaw gyro output to motors
	if (Config.YawGyroDirection == GYRO_NORMAL) {		// scale gyro output
		RxInYaw += gyroADC[YAW];
	} else {
		RxInYaw -= gyroADC[YAW];
	}

#ifdef SINGLE_COPTER
	RxInYaw -= LowpassOutServo[YAW];
	RxInYaw = (RxInYaw >> 3);
	LowpassOutServo[YAW] += RxInYaw;

	MotorOut2 += LowpassOutServo[YAW];
	MotorOut3 += LowpassOutServo[YAW];
	MotorOut4 += LowpassOutServo[YAW];
	MotorOut5 += LowpassOutServo[YAW];
#elif defined(DUAL_COPTER)
	MotorOut1 -= RxInYaw;
	MotorOut2 += RxInYaw;
#elif defined(TWIN_COPTER)
	RxInYaw -= LowpassOutServo[1];
	RxInYaw = (RxInYaw >> 3);
	LowpassOutServo[1] += RxInYaw;

	#ifdef SERVO_REVERSE
	MotorOut3 -= (LowpassOutServo[1] >> 1);
	MotorOut4 -= (LowpassOutServo[1] >> 1);
	#else
	MotorOut3 += (LowpassOutServo[1] >> 1);
	MotorOut4 += (LowpassOutServo[1] >> 1);
	#endif
#elif defined(TRI_COPTER)
/*
	RxInYaw -= LowpassOutYaw;
	RxInYaw = (RxInYaw >> 3);
	LowpassOutYaw += RxInYaw;
*/
	LowpassOutYaw = RxInYaw;
	MotorOut4 += LowpassOutYaw;

	// Servo Reverse Pin
	MotorOut5 -= LowpassOutYaw;
#elif defined(QUAD_COPTER)
	MotorOut1 -= RxInYaw;
	MotorOut2 += RxInYaw;
	MotorOut3 += RxInYaw;
	MotorOut4 -= RxInYaw;
#elif defined(QUAD_X_COPTER)
	MotorOut1 -= RxInYaw;
	MotorOut2 += RxInYaw;
	MotorOut3 -= RxInYaw;
	MotorOut4 += RxInYaw;
#elif defined(Y4_COPTER)
	if (( MotorOut3 - RxInYaw ) < 10) RxInYaw = MotorOut3 - 10;		// Yaw Range Limit
	if (( MotorOut3 - RxInYaw ) > 100) RxInYaw = MotorOut3 - 100;	// Yaw Range Limit

	if (( MotorOut4 + RxInYaw ) < 10) RxInYaw = 10 - MotorOut4;		// Yaw Range Limit
	if (( MotorOut4 + RxInYaw ) > 100) RxInYaw = 100 - MotorOut4;	// Yaw Range Limit

	MotorOut3 -= RxInYaw;
	MotorOut4 += RxInYaw;
#elif defined(HEX_COPTER)
	MotorOut1 -= RxInYaw;
	MotorOut2 += RxInYaw;
	MotorOut3 -= RxInYaw;
	MotorOut4 += RxInYaw;
	MotorOut5 -= RxInYaw;
	MotorOut6 += RxInYaw;
#elif defined(Y6_COPTER)
	MotorOut1 -= RxInYaw;
	MotorOut4 -= RxInYaw;
	MotorOut5 -= RxInYaw;
	MotorOut2 += RxInYaw;
	MotorOut3 += RxInYaw;
	MotorOut6 += RxInYaw;
#endif

	//--- Limit the lowest value to avoid stopping of motor if motor value is under-saturated ---
#if defined(DUAL_COPTER)
	if ( MotorOut1 < 10 )	MotorOut1 = 10;					// this is the motor idle level
	if ( MotorOut2 < 10 )	MotorOut2 = 10;	
#elif defined(TWIN_COPTER)
	if ( MotorOut1 < 10 )	MotorOut1 = 10;					// this is the motor idle level
	if ( MotorOut2 < 10 )	MotorOut2 = 10;	
#elif defined(TRI_COPTER)
	if ( MotorOut1 < 10 )	MotorOut1 = 10;					// this is the motor idle level
	if ( MotorOut2 < 10 )	MotorOut2 = 10;	
	if ( MotorOut3 < 10 )	MotorOut3 = 10;
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
	if ( MotorOut1 < 10 )	MotorOut1 = 10;					// this is the motor idle level
	if ( MotorOut2 < 10 )	MotorOut2 = 10;	
	if ( MotorOut3 < 10 )	MotorOut3 = 10;
	if ( MotorOut4 < 10 )	MotorOut4 = 10;	
#elif defined(HEX_COPTER) ||  defined(Y6_COPTER)
	if ( MotorOut1 < 10 )	MotorOut1 = 10;					// this is the motor idle level
	if ( MotorOut2 < 10 )	MotorOut2 = 10;	
	if ( MotorOut3 < 10 )	MotorOut3 = 10;
	if ( MotorOut4 < 10 )	MotorOut4 = 10;	
	if ( MotorOut5 < 10 )	MotorOut5 = 10;	
	if ( MotorOut6 < 10 )	MotorOut6 = 10;	
#endif

	//--- Output to motor ESC's ---
	if (RxInCollective < 1 || !Armed || !GyroCalibrated)	// turn off motors if collective below 1% ???
	{														// or  if gyros not calibrated
#ifdef SINGLE_COPTER
		MotorOut1 = 0;
		MotorOut2 = 84;
		MotorOut3 = 84;
		MotorOut4 = 84;
		MotorOut5 = 84;
#elif defined(DUAL_COPTER)
		MotorOut1 = 0;
		MotorOut2 = 0;
		MotorOut3 = 50;
		MotorOut4 = 50;
#elif defined(TWIN_COPTER)
		MotorOut1 = 0;
		MotorOut2 = 0;
		MotorOut3 = 50;
		MotorOut4 = 50;
		MotorOut5 = 50;
		MotorOut6 = 50;
#elif defined(TRI_COPTER)
		MotorOut1 = 0;
		MotorOut2 = 0;
		MotorOut3 = 0;
		if (!Armed) {
			MotorOut4 = 50;
			MotorOut5 = 50;
		}
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

	// Disable if Armed here so we always send signal to avoid Plush beeping: if (armed)
	output_motor_ppm();		// output ESC signal
}

void Init_ADC(void)
{
	DIDR0 	= 0b00111111;	// Digital Input Disable Register - ADC5..0 Digital Input Disable
	ADCSRB 	= 0b00000000; 	// ADC Control and Status Register B - ADTS2:0
}

void ReadGainPots(void)
{
	read_adc( 3 );			// read roll gain ADC3
	GainInADC[ROLL] = ADCL;
	GainInADC[ROLL] += ((uint16_t) ADCH <<8);
	GainIn[ROLL] = GainInADC[ROLL] / 10;

	read_adc( 4 );			// read pitch gain ADC4
	GainInADC[PITCH] = ADCL;
	GainInADC[PITCH] += ((uint16_t) ADCH <<8);
	GainIn[PITCH] = GainInADC[PITCH] / 10;

	read_adc( 5 );			// read yaw gain ADC5
	GainInADC[YAW] = ADCL;
	GainInADC[YAW] += ((uint16_t) ADCH <<8);
	GainIn[YAW] = GainInADC[YAW] / 10;
}

void read_adc(uint8_t channel)
{
	ADMUX 	= channel;						// set channel
	ADCSRA 	= 0b11000110;					// ADEN, ADSC, ADPS1,2

	while (ADCSRA & (1 << ADSC));	// wait to complete
}

void ReadGyros(bool calibrate)
{
	read_adc( 2 );			// read roll gyro ADC2
	gyroADC[ROLL] = ADCL;
	gyroADC[ROLL] += ((uint16_t) ADCH <<8);
	if (!calibrate) gyroADC[ROLL] 	-= gyroZero[ROLL];				//remove offset from gyro output

	read_adc( 1 );			// read pitch gyro ADC1
	gyroADC[PITCH] = ADCL;
	gyroADC[PITCH] += ((uint16_t) ADCH <<8);
	if (!calibrate) gyroADC[PITCH] -= gyroZero[PITCH];				//remove offset from gyro output

#ifdef EXTERNAL_YAW_GYRO
	gyroADC[YAW] = 0;
#else
	read_adc( 0 );			// read yaw gyro ADC0
	gyroADC[YAW] = ADCL;
	gyroADC[YAW] += ((uint16_t) ADCH <<8);
	if (!calibrate) gyroADC[YAW]	-= gyroZero[YAW];				//remove offset from gyro output
#endif
}

void CalibrateGyros(void)
{
	uint8_t i;

	ReadGainPots();	// about time we did this !

	// get/set gyro zero value (average of 32 readings)
	gyroZero[ROLL] 	= 0;						
	gyroZero[PITCH] = 0;	
	gyroZero[YAW] 	= 0;

	for (i=0;i<32;i++)
	{
		ReadGyros(true);
	
		gyroZero[ROLL] 	+= gyroADC[ROLL];						
		gyroZero[PITCH] += gyroADC[PITCH];	
		gyroZero[YAW] 	+= gyroADC[YAW];
	}

	gyroZero[ROLL] 	= (gyroZero[ROLL] >> 5);						
	gyroZero[PITCH] = (gyroZero[PITCH] >> 5);
	gyroZero[YAW] 	= (gyroZero[YAW] >> 5);

	GyroCalibrated = true;
#ifdef SINGLE_COPTER
	LowpassOutServo[ROLL]	= 84;					// Center
	LowpassOutServo[PITCH]	= 84;					// Center
	LowpassOutServo[YAW]	= 84;					// Center
#elif defined(DUAL_COPTER)
	LowpassOutServo[ROLL]	= 50;					// Center
	LowpassOutServo[PITCH]	= 50;					// Center
#elif defined(TWIN_COPTER)
	LowpassOutServo[0]		= 50;					// Center
	LowpassOutServo[1]		= 50;					// Center
#elif defined(TRI_COPTER)
	LowpassOutYaw			= 50;					// Center
#endif
}

//--- Get and scale RX channel inputs ---
void RxGetChannels(void)
{
	static int16_t RxChannel;

//	while ( RxChannelsUpdatingFlag );

	cli();
	RxChannel = RxChannel1;
	sei();
	RxChannel -= Config.RxChannel1ZeroOffset;				// normalise
	RxInRoll = (RxChannel >> 2);                    //     "

//	while ( RxChannelsUpdatingFlag );

	cli();
	RxChannel = RxChannel2;
	sei();
	RxChannel -= Config.RxChannel2ZeroOffset;				// normalise
	RxInPitch = (RxChannel >> 2);                   //     "

//	while ( RxChannelsUpdatingFlag );

	cli();
	RxChannel = RxChannel3;
	sei();
	RxChannel -= Config.RxChannel3ZeroOffset;				// scale 0->100
	RxInCollective = (RxChannel >> 3);              // 

//	while ( RxChannelsUpdatingFlag );

	cli();
	RxChannel = RxChannel4;
	sei();
	RxChannel -= Config.RxChannel4ZeroOffset;				// normalise
	RxInYaw = (RxChannel >> 2);                     //     "

#ifdef TWIN_COPTER
	RxInOrgPitch = RxInPitch;
#endif
}

void output_motor_ppm(void)
{
	static uint8_t i;
	static uint16_t MotorStartTCNT1, ElapsedTCNT1;
	static int16_t MotorAdjust;
	static uint16_t PWM_Low_Count;
	static int8_t num_of_10uS;	
#if defined(SINGLE_COPTER) || defined(DUAL_COPTER) || defined(TWIN_COPTER) || defined(TRI_COPTER)
	static uint8_t ServoPPMRateCount;
#endif

	// if ESC's are high, we need to turn them off
	if (output_motor_high)
	{
#ifdef SINGLE_COPTER
		// set motor limits (0 -> 100)
		// set servo limits (0 -> 200)
		if ( MotorOut1 < 0 ) MotorOut1 = 0;
		else if ( MotorOut1 > 100 ) MotorOut1 = 100;
		if ( MotorOut2 < 0 ) MotorOut2 = 0;
		else if ( MotorOut2 > 200 ) MotorOut2 = 200;
		if ( MotorOut3 < 0 ) MotorOut3 = 0;
		else if ( MotorOut3 > 200 ) MotorOut3 = 200;
		if ( MotorOut4 < 0 ) MotorOut4 = 0;
		else if ( MotorOut4 > 200 ) MotorOut4 = 200;
		if ( MotorOut5 < 0 ) MotorOut5 = 0;
		else if ( MotorOut5 > 200 ) MotorOut5 = 200;
#else
		// set motor limits (0 -> 100)
		if ( MotorOut1 < 0 ) MotorOut1 = 0;
		else if ( MotorOut1 > 100 ) MotorOut1 = 100;
		if ( MotorOut2 < 0 ) MotorOut2 = 0;
		else if ( MotorOut2 > 100 ) MotorOut2 = 100;
		if ( MotorOut3 < 0 ) MotorOut3 = 0;
		else if ( MotorOut3 > 100 ) MotorOut3 = 100;
		if ( MotorOut4 < 0 ) MotorOut4 = 0;
		else if ( MotorOut4 > 100 ) MotorOut4 = 100;
	#if defined(TWIN_COPTER) || defined(TRI_COPTER) || defined(HEX_COPTER) || defined(Y6_COPTER)
		if ( MotorOut5 < 0 ) MotorOut5 = 0;
		else if ( MotorOut5 > 100 ) MotorOut5 = 100;
	#endif
	#if defined(TWIN_COPTER) || defined(HEX_COPTER) || defined(Y6_COPTER)
		if ( MotorOut6 < 0 ) MotorOut6 = 0;
		else if ( MotorOut6 > 100 ) MotorOut6 = 100;
	#endif
#endif

		// now calculate the time already passed that Motors were HIGH
		ElapsedTCNT1 = (TCNT1 - MotorStartTCNT1);

		// start output timer
		TIFR0 &= ~(1 << TOV0);			// clr overflow
		TCNT0 = 0;						// reset counter

		// convert into 10uS intervals
		num_of_10uS = (ElapsedTCNT1 / 10) + 1;
		MotorAdjust = 100 - num_of_10uS;

#ifdef SINGLE_COPTER
		// add adjustment (1mS - time already gone) to 1 channel
		MotorOut1 += MotorAdjust;
#else
		// add adjustment (1mS - time already gone) to all channels
		MotorOut1 += MotorAdjust;
		MotorOut2 += MotorAdjust;
		MotorOut3 += MotorAdjust;
		MotorOut4 += MotorAdjust;
	#if defined(TWIN_COPTER) || defined(TRI_COPTER) || defined(HEX_COPTER) || defined(Y6_COPTER)
		MotorOut5 += MotorAdjust;
	#endif
	#if defined(TWIN_COPTER) || defined(HEX_COPTER) || defined(Y6_COPTER)
		MotorOut6 += MotorAdjust;
	#endif
#endif

		// keep signal on for correct time
		// MotorOutX = 100 -> 200
		// Pulse len = 1   -> 2    mS

		// Servo = 0 - 200
		// Pulse len = 0 -> 2.3ms

		TIFR0 &= ~(1 << TOV0);			// clr overflow
		TCNT0 = 0;						// reset counter

#ifdef SINGLE_COPTER
		for (i=0;i<200;i++)	
		{
			while (TCNT0 < 80);			// 10uS @ 8MHz = 80 // 10 @ 1MHz = 10uS
			TCNT0 -= 80;

			if (MotorOut1)
			{
				MotorOut1--;
				if (MotorOut1==0) M1 = 0;
			}
			if (MotorOut2) 
			{
				MotorOut2--;
				if (MotorOut2==0) M2 = 0;
			}
			if (MotorOut3) 
			{
				MotorOut3--;
				if (MotorOut3==0) M3 = 0;
			}
			if (MotorOut4) 
			{
				MotorOut4--;
				if (MotorOut4==0) M4 = 0;
			}
			if (MotorOut5) 
			{
				MotorOut5--;
				if (MotorOut5==0) M5 = 0;
			}
		}
#else
		for (i=num_of_10uS;i<200;i++)	
		{
			while (TCNT0 < 80);			// 10uS @ 8MHz = 80 // 10 @ 1MHz = 10uS
			TCNT0 -= 80;

			if (MotorOut1) 
			{
				MotorOut1--;
				if (MotorOut1==0) M1 = 0;
			}
			if (MotorOut2) 
			{
				MotorOut2--;
				if (MotorOut2==0) M2 = 0;
			}
			if (MotorOut3) 
			{
				MotorOut3--;
				if (MotorOut3==0) M3 = 0;
			}
			if (MotorOut4) 
			{
				MotorOut4--;
				if (MotorOut4==0) M4 = 0;
			}
	#if defined(TWIN_COPTER) || defined(TRI_COPTER) || defined(HEX_COPTER) || defined(Y6_COPTER)
			if (MotorOut5) 
			{
				MotorOut5--;
				if (MotorOut5==0) M5 = 0;
			}
	#endif
	#if defined(TWIN_COPTER) || defined(HEX_COPTER) || defined(Y6_COPTER)
			if (MotorOut6) 
			{
				MotorOut6--;
				if (MotorOut6==0) M6 = 0;
			}
	#endif
		}
#endif

		//Now wait low signal interval
		PWM_Low_Count = PWM_Low_Pulse_Interval - 1;

		TIFR0 &= ~(1 << TOV0);		// clr overflow
		TCNT0 = 0;					// reset counter

		while (PWM_Low_Count--)
		{
			while (TCNT0 < 80);		// 20 @ 2MHz = 10uS
			TCNT0 -= 80;
		}
	}

	// Disable this to always output signal, even when not armed, to avoid Plush beeping
	// if (! Armed) return;

	// Log PWM signal HIGH	
	MotorStartTCNT1 = TCNT1;
	output_motor_high = true;

	// turn on pins
#ifdef SINGLE_COPTER
	M1 = 1;
	if(ServoPPMRateCount==ServoPPMRateDivider)
	{
		M2 = 1;
		M3 = 1;
		M4 = 1;
		M5 = 1;
		ServoPPMRateCount = 1;
	} else {
		ServoPPMRateCount++;
	}
#elif defined(DUAL_COPTER)
	M1 = 1;
	M2 = 1;
	if(ServoPPMRateCount==ServoPPMRateDivider)
	{
		M3 = 1;
		M4 = 1;
		ServoPPMRateCount = 1;
	} else {
		ServoPPMRateCount++;
	}
#elif defined(TWIN_COPTER)
	M1 = 1;
	M2 = 1;
	if(ServoPPMRateCount==ServoPPMRateDivider)
	{
		M3 = 1;
		M4 = 1;
		M5 = 1;
		M6 = 1;
		ServoPPMRateCount = 1;
	} else {
		ServoPPMRateCount++;
	}
#elif defined(TRI_COPTER)
	M1 = 1;
	M2 = 1;
	M3 = 1;
	if(ServoPPMRateCount==ServoPPMRateDivider)
	{
		M4 = 1;
		M5 = 1;
		ServoPPMRateCount = 1;
	} else {
		ServoPPMRateCount++;
	}
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
	M1 = 1;
	M2 = 1;
	M3 = 1;
	M4 = 1;
#elif defined(HEX_COPTER) ||  defined(Y6_COPTER)
	M1 = 1;
	M2 = 1;
	M3 = 1;
	M4 = 1;
	M5 = 1;
	M6 = 1;
#endif
}

void eeprom_write_byte_changed( uint8_t * addr, uint8_t value )
{ 
	if(eeprom_read_byte(addr) != value)
	{
		eeprom_write_byte( addr, value );
	}
}

void eeprom_write_block_changes( const uint8_t * src, void * dest, size_t size )
{ 
	size_t len;

	for(len=0;len<size;len++)
	{
		eeprom_write_byte_changed( dest,  *src );

		src++;
		dest++;
	}
}

void Initial_EEPROM_Config_Load(void)
{
	// load up last settings from EEPROM
	if(eeprom_read_byte((uint8_t*) EEPROM_DATA_START_POS )!=0x47)
	{
		Config.setup = 0x47;
		Set_EEPROM_Default_Config();
		// write to eeProm
		Save_Config_to_EEPROM();
	} else {
		// read eeprom
		eeprom_read_block(&Config, (void*) EEPROM_DATA_START_POS, sizeof(CONFIG_STRUCT)); 
	}
}

void Set_EEPROM_Default_Config(void)
{
	Config.RollGyroDirection 	= GYRO_REVERSED;
	Config.PitchGyroDirection	= GYRO_REVERSED;
	Config.YawGyroDirection		= GYRO_NORMAL;

	Config.RxChannel1ZeroOffset	= 1520;
	Config.RxChannel2ZeroOffset	= 1520;
	Config.RxChannel3ZeroOffset	= 1120;
	Config.RxChannel4ZeroOffset	= 1520;
}

void Save_Config_to_EEPROM(void)
{
	// write to eeProm
	cli();
	eeprom_write_block_changes( (const void*) &Config, (void*) EEPROM_DATA_START_POS, sizeof(CONFIG_STRUCT));	//current_config CONFIG_STRUCT
	sei();
}

void delay_us(uint8_t time)            /* time delay for us */
{ 
 while(time--)
 {
	asm volatile ("NOP"); asm volatile ("NOP"); 
	asm volatile ("NOP"); asm volatile ("NOP"); 
	asm volatile ("NOP"); asm volatile ("NOP"); 
	asm volatile ("NOP"); 
 }
}

void delay_ms(uint16_t time)
{
	uint8_t i;
	while(time--)
	{
		for(i=0;i<10;i++) delay_us(100);
	}
}

