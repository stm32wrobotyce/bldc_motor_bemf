/*
 * bldc_motor.h
 *
 *  Created on: 28 maj 2021
 *      Author: piotr
 */

#ifndef INC_BLDC_MOTOR_H_
#define INC_BLDC_MOTOR_H_

#include "pid_controller.h"

typedef enum
{
	CCW = 0,
	CW = 1
}direction;

typedef enum
{
	ALIGNMENT = 0,
	START = 1,
	STARTED = 2
}bldc_status;

struct bldc_control
{
	TIM_HandleTypeDef	*tim_pwm;
	TIM_HandleTypeDef	*tim_com;

	TIM_OC_InitTypeDef sConfigOC;

	volatile uint8_t step_number;
	volatile uint8_t actual_step_number;
	uint32_t speed_pulse;
	volatile uint8_t dir;
	volatile uint32_t flaga_next_step;

	uint32_t bemf_adc_data[3];
	volatile uint32_t state;

	pid_str pid_controller;
};

void bldc_motor_init(TIM_HandleTypeDef *_tim_pwm, TIM_HandleTypeDef *_tim_com);
void bldc_motor_six_step_algorithm(void);
void bldc_motor_set_new_ARR_value(void);
void bldc_motor_set_speed(uint32_t speed, direction dir);

void bldc_motor_Config_Channel_Init(void);
void bldc_motor_PWM_Config_Channel(uint32_t pulse, uint32_t channel);
void bldc_motor_OC_Config_Channel(uint32_t mode, uint32_t channel);

void bldc_motor_bemf_start_adc_conversion(void);
void bldc_motor_bemf_calculation(void);
void bldc_motor_ARR_calculate(void);

void bldc_motor_start_pid_calculate(void);
void bldc_motor_alignment(void);

void bldc_motor_process(void);

#endif /* INC_BLDC_MOTOR_H_ */
