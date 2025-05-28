
#include "./inc/motor_commands.h"


/*
void Motor_Init();
void Motor_Forward(uint16_t left_duty_cycle, uint16_t right_duty_cycle);
void Motor_Backward(uint16_t left_duty_cycle, uint16_t right_duty_cycle);
void Motor_Left(uint16_t left_duty_cycle, uint16_t right_duty_cycle);
void Motor_Right(uint16_t left_duty_cycle, uint16_t right_duty_cycle);
void Motor_Stop();
*/

void Motor_Turn(int16_t duty_cycle) {
	uint16_t abs_duty_cycle = (duty_cycle < 0) ? -duty_cycle : duty_cycle;

	if (duty_cycle < 0) {
		
		// Motor_Left(abs_duty_cycle, abs_duty_cycle);

	} else {

		// Motor_Right(abs_duty_cycle, abs_duty_cycle);
	}
}


void Motor_Move(int16_t duty_cycle) {
	uint16_t abs_duty_cycle = (duty_cycle < 0) ? -duty_cycle : duty_cycle;

	if (duty_cycle < 0) {
		
		Motor_Backward(abs_duty_cycle, abs_duty_cycle);

	} else {
		
		Motor_Forward(abs_duty_cycle, abs_duty_cycle);
	}
}



// void 
