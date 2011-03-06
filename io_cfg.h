/*********************************************************************
 *
 *********************************************************************
 * FileName:        io_cfg.h
 ********************************************************************/

#ifndef IO_CFG_H
#define IO_CFG_H

/** I N C L U D E S *************************************************/
#include "typedefs.h"

#define RX_ROLL    			REGISTER_BIT(PIND,1)
#define RX_PITCH    		REGISTER_BIT(PIND,2)	// INT0
#define RX_COLL    			REGISTER_BIT(PIND,3)	// INT1
#define RX_YAW    			REGISTER_BIT(PINB,7)
#define RX_ROLL_DIR   	REGISTER_BIT(DDRD,1)
#define RX_PITCH_DIR   	REGISTER_BIT(DDRD,2)
#define RX_COLL_DIR   	REGISTER_BIT(DDRD,3)
#define RX_YAW_DIR   		REGISTER_BIT(DDRB,7)

#define	GYRO_ROLL 			REGISTER_BIT(PINC,2)
#define GYRO_PITCH 			REGISTER_BIT(PINC,1)
#define GYRO_YAW 				REGISTER_BIT(PINC,0)
#define	GYRO_ROLL_DIR 	REGISTER_BIT(DDRC,2)
#define GYRO_PITCH_DIR 	REGISTER_BIT(DDRC,1)
#define GYRO_YAW_DIR 		REGISTER_BIT(DDRC,0)

#define	GAIN_ROLL 			REGISTER_BIT(PINC,3)
#define GAIN_PITCH 			REGISTER_BIT(PINC,4)
#define GAIN_YAW 				REGISTER_BIT(PINC,5)
#define	GAIN_ROLL_DIR 	REGISTER_BIT(DDRC,3)
#define GAIN_PITCH_DIR 	REGISTER_BIT(DDRC,4)
#define GAIN_YAW_DIR 		REGISTER_BIT(DDRC,5)

#define M1		  				REGISTER_BIT(PORTB,2)
#define M2		  				REGISTER_BIT(PORTB,1)
#define M3		  				REGISTER_BIT(PORTB,0)
#define M4		  				REGISTER_BIT(PORTD,7)
#define M5		  				REGISTER_BIT(PORTD,6)
#define M6		  				REGISTER_BIT(PORTD,5)
#define M1_DIR 					REGISTER_BIT(DDRB,2)
#define M2_DIR 					REGISTER_BIT(DDRB,1)
#define M3_DIR 					REGISTER_BIT(DDRB,0)
#define M4_DIR 					REGISTER_BIT(DDRD,7)
#define M5_DIR 					REGISTER_BIT(DDRD,6)
#define M6_DIR 					REGISTER_BIT(DDRD,5)

#define LED 						REGISTER_BIT(PORTB,6)
#define LED_DIR 				REGISTER_BIT(DDRB,6)


#endif //IO_CFG_H
