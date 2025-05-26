#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f1xx_hal.h"

void Init_Pins(GPIO_TypeDef *GPIO_Enable, uint16_t Pin_Enable,
		GPIO_TypeDef *GPIO_Step, uint16_t Pin_Step,
		GPIO_TypeDef *GPIO_Direction, uint16_t Pin_Direction);

void Motor_On(void);

void Motor_Off(void);

void Steps(uint32_t steps_);

void MOTOR_Direction(uint8_t);

void DelayMotor(uint32_t us);

#endif
