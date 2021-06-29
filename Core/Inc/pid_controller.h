/*
 * pid_controller.h
 *
 *  Created on: 11 lut 2021
 *      Author: piotr
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

typedef struct
{
	int previous_error; 		//Poprzedni błąd dla członu różniczkującego
	int total_error;			//Suma uchybów dla członu całkującego
	float Kp;					//Wzmocnienie członu proporcjonalnego
	float Ki;					//Wzmocnienie członu całkującego*/
	float Kd;					//Wzmocnienie członu różniczkującego*/
	int anti_windup_limit;		//Anti-Windup - ograniczenie członu całkującego*/
}pid_str;

void pid_init(pid_str *, float, float, float, int);
void pid_reset(pid_str *);
int pid_calculate(pid_str *, int, int);

#endif /* INC_PID_CONTROLLER_H_ */
