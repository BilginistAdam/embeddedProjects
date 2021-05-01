/*
 * L298N.h
 *
 *  Created on: Jan 12, 2021
 *      Author: bilgi
 */

#ifndef __L298N_H_
#define __L298N_H_

/* INCLUDES **********************************************************/
#include "main.h"

/* DEFINES ***********************************************************/
//@L298N_MOTOR_SPEED
#define L298N_MOTOR_SPEED_MIN 		0UL						//This is min speed for each motors.
#define L298N_MOTOR_SPEED_MAX		999UL					//This is max speed for each motors.
#define L298N_MOTOR_SPEED_STOP 		0UL						//This is stop speed for each motors.

//@L298N_MOTOR_DIRECTION
#define L298N_MOTOR_DIRECTION_FORWARD   1		//This is forward direction to motor rotation.
#define L298N_MOTOR_DIRECTION_STOP		0		//This is stop position to motor rotation.
#define L298N_MOTOR_DIRECTION_BACKWARD -1		//This is backward direction to motor rotation.

/* TYPEDEFS **********************************************************/
typedef struct{
	TIM_HandleTypeDef 	* Timer;		/* It is the address of the timer. */
	uint32_t 			  Channel;		/* It is the channel of the timer. */

	GPIO_TypeDef		* port[2];		/* This parameter holds GPIO ports. port[0] -> 0. pin port; port[1] -> 1. pin port. */
	uint16_t			  pin[2];		/* This parameter holds GPIO pins. pin[0] -> 0. pin; pin[1] -> 1. pin. */
}L298N_Config_t;

typedef struct{
	L298N_Config_t 		  Config;		/* It is the address of the config. */
	uint32_t 			  speed;		/* This parameter holds motor speed. It is able to range @L298N_MOTOR_SPEED_MIN - @L298N_MOTOR_SPEED_MAX. */
	int8_t 				  direction;	/* This parameter holds motor direction. @L298N_MOTOR_DIRECTION */
}L298N_Handler_t;

/* Function Definitions **********************************************/
/* Init */
HAL_StatusTypeDef L298N_MotorInit(L298N_Handler_t * hmotor);

/* Get */
uint32_t L298N_getMotorSpeed(L298N_Handler_t* hmotor);
int8_t L298N_getMotorDirection(L298N_Handler_t* hmotor);


//set
HAL_StatusTypeDef L298N_setMotorSpeed(L298N_Handler_t* hmotor, uint32_t speed);
HAL_StatusTypeDef L298N_setMotorDirection(L298N_Handler_t* hmotor, int8_t direction);


#endif /* __L298N_H_ */
