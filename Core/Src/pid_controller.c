/*
 * pid_controller.c
 *
 *  Created on: 11 lut 2021
 *      Author: piotr
 */

#include "pid_controller.h"

void pid_init(pid_str *pid_data, float kp_init, float ki_init, float kd_init, int anti_windup_limit_init)
{
	pid_data->previous_error = 0;
	pid_data->total_error = 0;

	pid_data->Kp = kp_init;
	pid_data->Ki = ki_init;
	pid_data->Kd = kd_init;

	pid_data->anti_windup_limit = anti_windup_limit_init;
}

void pid_reset(pid_str *pid_data)
{
	pid_data->total_error = 0;
	pid_data->previous_error = 0;
}

int pid_calculate(pid_str *pid_data, int setpoint, int process_variable)
{
	int error;
	float p_term, i_term, d_term;

	error = setpoint - process_variable;													//obliczenie uchybu
	pid_data->total_error += error;															//sumowanie uchybu

	p_term = (float)(pid_data->Kp * error);													//odpowiedź członu proporcjonalnego
	i_term = (float)(pid_data->Ki * pid_data->total_error);									//odpowiedź członu całkującego
	d_term = (float)(pid_data->Kd * (error - pid_data->previous_error));		//odpowiedź członu różniczkującego

	if(i_term >= pid_data->anti_windup_limit) i_term = pid_data->anti_windup_limit;			//Anti-Windup - ograniczenie odpowiedzi członu całkującego
	else if(i_term <= -pid_data->anti_windup_limit) i_term = -pid_data->anti_windup_limit;

	pid_data->previous_error = error;											//aktualizacja zmiennej z poprzednią wartością błędu

	return (int)(p_term + i_term + d_term);														//odpowiedź regulatora
}
