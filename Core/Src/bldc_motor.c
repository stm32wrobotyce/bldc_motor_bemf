/*
 * bldc_motor.c
 *
 *  Created on: 28 maj 2021
 *      Author: piotr
 */

#include "main.h"
#include "adc.h"
#include "pid_controller.h"

#define ARR_TIM3_VALUE			20000
#define ARR_TIM3_INIT_VALUE		1000
#define BLDC_MOTOR_MAX_SPEED	100-1

#define BEMF_NBR_OF_CONVERSION	3

#define BEMF_U_PHASE			0
#define BEMF_V_PHASE			1
#define BEMF_W_PHASE			2

#define BEMF_THRESHOLD_UP		200
#define BEMF_THRESHOLD_DOWN		200

#define PROCESS_MAX_TIME		1

struct bldc_control bldc;

uint32_t table_of_ms_time_to_count_in_process[2] = {200, 3000};
uint32_t time_counter = 0;
uint32_t time_to_count = 0;

void bldc_motor_init(TIM_HandleTypeDef *_tim_pwm, TIM_HandleTypeDef *_tim_com)
{
	bldc.tim_pwm = _tim_pwm;
	bldc.tim_com = _tim_com;

	bldc.step_number = 6;
	bldc.speed_pulse = 0;
	bldc.dir = CW;
	bldc.flaga_next_step = 0;

	bldc.state = ALIGNMENT;
	time_to_count = table_of_ms_time_to_count_in_process[ALIGNMENT];

	pid_init(&bldc.pid_controller, 0.05, 0, 0.01, 10);

	bldc_motor_Config_Channel_Init();

	__HAL_TIM_SET_AUTORELOAD(bldc.tim_com, ARR_TIM3_VALUE);

	HAL_TIM_Base_Start_IT(bldc.tim_pwm);
	HAL_TIM_Base_Start(bldc.tim_com);
	HAL_TIMEx_ConfigCommutationEvent_IT(bldc.tim_pwm, TIM_TS_ITR2, TIM_COMMUTATION_TRGI);
}

void bldc_motor_six_step_algorithm(void)
{
	if(ALIGNMENT != bldc.state)
	{
		bldc.actual_step_number = bldc.step_number;

		if(CW == bldc.dir)
		{
			bldc.step_number++;

			if(bldc.step_number > 6)
				bldc.step_number = 1;
		}
		else if(CCW == bldc.dir)
		{
			bldc.step_number--;

			if(bldc.step_number < 1)
				bldc.step_number = 6;
		}

		bldc.flaga_next_step = 1;
	}

	switch (bldc.step_number)
	{
		case 1:
		{
			bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_1);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_2);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_3);
		}
		break;

		case 2:
		{
			bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_1);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_2);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_3);
		}
		break;

		case 3:
		{
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_1);
			bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_2);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_3);
		}
		break;

		case 4:
		{
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_1);
			bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_2);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_3);
		}
		break;

		case 5:
		{
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_1);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_2);
			bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_3);
		}
		break;

		case 6:
		{
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_1);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_2);
			bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_3);
		}
		break;

	}


}

void bldc_motor_set_speed(uint32_t speed, direction dir)
{
	if(speed > BLDC_MOTOR_MAX_SPEED)
	{
		bldc.speed_pulse = BLDC_MOTOR_MAX_SPEED;
	}
	else
	{
		bldc.speed_pulse = speed;
	}

	bldc.dir = dir;
}

void bldc_motor_Config_Channel_Init(void)
{
	bldc.sConfigOC.OCMode = TIM_OCMODE_PWM1;
	bldc.sConfigOC.Pulse = 0;
	bldc.sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	bldc.sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	bldc.sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	bldc.sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	bldc.sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
}

void bldc_motor_PWM_Config_Channel(uint32_t pulse, uint32_t channel)
{
	bldc.sConfigOC.OCMode = TIM_OCMODE_PWM1;
	bldc.sConfigOC.Pulse = pulse;
	HAL_TIM_PWM_ConfigChannel(bldc.tim_pwm, &bldc.sConfigOC, channel);

	HAL_TIM_PWM_Start(bldc.tim_pwm, channel);
	HAL_TIMEx_PWMN_Start(bldc.tim_pwm, channel);
}

void bldc_motor_OC_Config_Channel(uint32_t mode, uint32_t channel)
{
	bldc.sConfigOC.OCMode = mode;
	HAL_TIM_OC_ConfigChannel(bldc.tim_pwm, &bldc.sConfigOC, channel);

	HAL_TIM_OC_Stop(bldc.tim_pwm, channel);
	HAL_TIMEx_OCN_Start(bldc.tim_pwm, channel) ;
}

void bldc_motor_bemf_start_adc_conversion(void)
{
	HAL_ADC_Start_DMA(&hadc1, bldc.bemf_adc_data, BEMF_NBR_OF_CONVERSION);
}

void bldc_motor_bemf_calculation(void)
{
	if(bldc.flaga_next_step == 0)
		return;

	//demagnetization
	if(__HAL_TIM_GetCounter(bldc.tim_com) < (__HAL_TIM_GetAutoreload(bldc.tim_com)/10))
		return;

	switch (bldc.actual_step_number)
	{
		case 1:
		{
			if(CW == bldc.dir)
			{
				if(bldc.bemf_adc_data[BEMF_W_PHASE] < BEMF_THRESHOLD_DOWN)
				{
					bldc_motor_ARR_calculate();
				}
			}
			else if(CCW == bldc.dir)
			{
				if(bldc.bemf_adc_data[BEMF_W_PHASE] > BEMF_THRESHOLD_UP)
				{
					bldc_motor_ARR_calculate();
				}
			}
		}
		break;

		case 2:
		{
			if(CW == bldc.dir)
			{
				if(bldc.bemf_adc_data[BEMF_V_PHASE] > BEMF_THRESHOLD_UP)
				{
					bldc_motor_ARR_calculate();
				}
			}
			else if(CCW == bldc.dir)
			{
				if(bldc.bemf_adc_data[BEMF_V_PHASE] < BEMF_THRESHOLD_DOWN)
				{
					bldc_motor_ARR_calculate();
				}
			}
		}
		break;

		case 3:
		{
			if(CW == bldc.dir)
			{
				if(bldc.bemf_adc_data[BEMF_U_PHASE] < BEMF_THRESHOLD_DOWN)
				{
					bldc_motor_ARR_calculate();
				}
			}
			else if(CCW == bldc.dir)
			{
				if(bldc.bemf_adc_data[BEMF_U_PHASE] > BEMF_THRESHOLD_UP)
				{
					bldc_motor_ARR_calculate();
				}
			}
		}
		break;

		case 4:
		{
			if(CW == bldc.dir)
			{
				if(bldc.bemf_adc_data[BEMF_W_PHASE] > BEMF_THRESHOLD_UP)
				{
					bldc_motor_ARR_calculate();
				}
			}
			else if(CCW == bldc.dir)
			{
				if(bldc.bemf_adc_data[BEMF_W_PHASE] < BEMF_THRESHOLD_DOWN)
				{
					bldc_motor_ARR_calculate();
				}
			}
		}
		break;

		case 5:
		{
			if(CW == bldc.dir)
			{
				if(bldc.bemf_adc_data[BEMF_V_PHASE] < BEMF_THRESHOLD_DOWN)
				{
					bldc_motor_ARR_calculate();
				}
			}
			else if(CCW == bldc.dir)
			{
				if(bldc.bemf_adc_data[BEMF_V_PHASE] > BEMF_THRESHOLD_UP)
				{
					bldc_motor_ARR_calculate();
				}
			}
		}
		break;

		case 6:
		{
			if(CW == bldc.dir)
			{
				if(bldc.bemf_adc_data[BEMF_U_PHASE] > BEMF_THRESHOLD_UP)
				{
					bldc_motor_ARR_calculate();
				}
			}
			else if(CCW == bldc.dir)
			{
				if(bldc.bemf_adc_data[BEMF_U_PHASE] < BEMF_THRESHOLD_DOWN)
				{
					bldc_motor_ARR_calculate();
				}
			}
		}
		break;
	}
}

void bldc_motor_ARR_calculate(void)
{
	uint32_t new_ARR_Value;
	uint32_t Bemf_Zero_Cross_Time;
	uint32_t old_ARR_Value = ARR_TIM3_VALUE;
	int32_t diff;

	Bemf_Zero_Cross_Time = __HAL_TIM_GetCounter(bldc.tim_com);
	old_ARR_Value = __HAL_TIM_GetAutoreload(bldc.tim_com);

	new_ARR_Value = Bemf_Zero_Cross_Time + old_ARR_Value/2;

	if(STARTED == bldc.state)
	{
		diff = pid_calculate(&bldc.pid_controller, new_ARR_Value, old_ARR_Value);
		__HAL_TIM_SetAutoreload(bldc.tim_com, old_ARR_Value + diff);
	}

	bldc.flaga_next_step = 0;
}

void bldc_motor_start_pid_calculate(void)
{
	int32_t diff;
	uint32_t measure = __HAL_TIM_GetAutoreload(bldc.tim_com);

	diff = pid_calculate(&bldc.pid_controller, ARR_TIM3_INIT_VALUE, measure);
	__HAL_TIM_SetAutoreload(bldc.tim_com, measure + diff);
}

void bldc_motor_alignment(void)
{
	bldc.step_number = 6;
	__HAL_TIM_SetAutoreload(bldc.tim_com, ARR_TIM3_VALUE);
}

void bldc_motor_process(void)
{
	static uint32_t time = 0;

	if(0 == time)
		time = HAL_GetTick();

	if((HAL_GetTick() - time) > PROCESS_MAX_TIME)
	{
		time = HAL_GetTick();

		time_counter += PROCESS_MAX_TIME;

		if(time_counter >= time_to_count)
		{
			time_counter = 0;
			if(STARTED != bldc.state)
			{
				bldc.state++;
				time_to_count = table_of_ms_time_to_count_in_process[bldc.state];
			}
		}

		switch (bldc.state)
		{
			case ALIGNMENT:
			{
				bldc_motor_alignment();
				break;
			}

			case START:
			{
				bldc_motor_start_pid_calculate();
				break;
			}

			case STARTED:
			{
				break;
			}
			default:
				break;
		}
	}
}
