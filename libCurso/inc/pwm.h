/*
 * pwm.h
 *
 *  Created on: 2 Jun 2017
 *      Author: mbaudino
 */

#ifndef PWM_H_
#define PWM_H_

#include "stdint.h"
#include "chip.h"

#define SCT_PWM            LPC_SCT

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define SCT_PWM_PIN_OUT    4        /* COUT4 Generate square wave */
#define SCT_PWM_PIN_LED    2        /* COUT2 [index 2] Controls LED */

#define SCT_PWM_OUT        1        /* Index of OUT PWM */
#define SCT_PWM_LED        2        /* Index of LED PWM */
#define SCT_PWM_RATE   10000        /* PWM frequency 10 KHz */

/* Systick timer tick rate, to change duty cycle */
#define TICKRATE_HZ     1000        /* 1 ms Tick rate */

#define LED_STEP_CNT      20        /* Change LED duty cycle every 20ms */
#define OUT_STEP_CNT    1000        /* Change duty cycle every 1 second */


static void app_setup_pin(void);

#endif /* PWM_H_ */
