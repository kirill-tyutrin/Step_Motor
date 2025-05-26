#include "motor.h"
#include "LCD1602.h"
#include "math.h"
static uint16_t Motor_Pin_Enable;
static GPIO_TypeDef *Motor_Enable;

static uint16_t Motor_Pin_Direction;
static GPIO_TypeDef *Motor_Direction;

static uint16_t Motor_Pin_Step;
static GPIO_TypeDef *Motor_Step;
// const int FULL_ANGLE = 360;
#define timer htim4
//
//
extern TIM_HandleTypeDef timer;
extern volatile uint8_t STOP_MOTOR;
extern volatile uint8_t KN;
extern volatile uint8_t flag_irq;
extern volatile uint8_t FLAG;
//extern volatile uint9_t FLAG1;
extern volatile uint8_t flag_usk;
volatile uint8_t flag_dec = 0;
extern volatile uint8_t flag_test;

const int steps = 1600;
float c0 = 1;
//const char *type[] = { "ob/m", "ob/s" };
uint32_t Delta_Time;
uint32_t buffer;
uint8_t buffer_dir = 1;
uint8_t dir__ = 1;
void DelayMotor(uint32_t us) {

	__HAL_TIM_SET_COUNTER(&timer, 0);
	while (__HAL_TIM_GET_COUNTER(&timer) < us)
		;

}

void Init_Pins(GPIO_TypeDef *GPIO_Enable, uint16_t Pin_Enable,
		GPIO_TypeDef *GPIO_Step, uint16_t Pin_Step,
		GPIO_TypeDef *GPIO_Direction, uint16_t Pin_Direction) {

	Motor_Pin_Enable = Pin_Enable;
	Motor_Enable = GPIO_Enable;

	Motor_Pin_Step = Pin_Step;
	Motor_Step = GPIO_Step;

	Motor_Pin_Direction = Pin_Direction;
	Motor_Direction = GPIO_Direction;

}

void Speed(int speed_) {
	float USK = (float) speed_ / 0.5f * 0.104f;
	Delta_Time = (int) (60 * 1000 * 1000 / (2 * steps * speed_));
	c0 = 1.0f * 1000.0f * 1000.0f / (Delta_Time * 2.0f)
			* pow(2.0f * 2.0f * 3.14f / steps / USK, 0.5f);

}
void Motor_On(void) {
	HAL_GPIO_WritePin(Motor_Enable, Motor_Pin_Enable, GPIO_PIN_RESET);
}
void Motor_Off(void) {
	HAL_GPIO_WritePin(Motor_Enable, Motor_Pin_Enable, GPIO_PIN_SET);
}

float Acceleration(uint32_t n, float a) {
	return a * (1.0f - 2.0f / (4.0f * n + 1.0f));
}

float Deceleration(uint32_t n, float a) {
	return a / (1.0f - 2.0f / (4.0f * n + 1.0f));
}

void ACC() { // функция ускорения или торможения
	//Ускорение:
	float C = 1.0;
	C = c0;
	uint32_t i = 1;
	if (flag_usk) {
		while (C >= 1) {
			HAL_GPIO_WritePin(Motor_Step, Motor_Pin_Step, GPIO_PIN_SET);
			DelayMotor((int) Delta_Time * Acceleration(i, C));
			HAL_GPIO_WritePin(Motor_Step, Motor_Pin_Step, GPIO_PIN_RESET);
			DelayMotor((int) Delta_Time * Acceleration(i, C));
			C = Acceleration(i, C);
			i++;
		}
		buffer = i;
		flag_usk = 0;
	}

	if (flag_dec) {
		C = 1.0;
		for (size_t i = buffer; i > 1; i--) {
			HAL_GPIO_WritePin(Motor_Step, Motor_Pin_Step, GPIO_PIN_SET);
			DelayMotor((int) Delta_Time * Deceleration(i, C));
			HAL_GPIO_WritePin(Motor_Step, Motor_Pin_Step, GPIO_PIN_RESET);
			DelayMotor((int) Delta_Time * Deceleration(i, C));
			C = Deceleration(i, C);
		}

	}
}
void MOTOR_Direction(uint8_t stable) {
	if (stable >= 2) {
		return;
	}
	HAL_GPIO_WritePin(Motor_Direction, Motor_Pin_Direction, stable);
}
void Steps(uint32_t steps_) {
	ACC();
	for (size_t i = 0; i < steps_; i++) { //razgon
//			if(buffer_dir != dir__ && flag_usk == 0){
//					flag_dec = 1;
//					ACC();
//					flag_dec = 0;
//					flag_usk = 1;
//					HAL_GPIO_WritePin(Motor_Direction, Motor_Pin_Direction, dir__);
//					ACC();
//					flag_dec = 0;
//					flag_usk = 0;
//					buffer_dir = dir__;
//
//			}
		HAL_GPIO_WritePin(Motor_Step, Motor_Pin_Step, GPIO_PIN_SET);
		DelayMotor(Delta_Time);
		HAL_GPIO_WritePin(Motor_Step, Motor_Pin_Step, GPIO_PIN_RESET);
		DelayMotor(Delta_Time);
		if (STOP_MOTOR == 0) {
			flag_dec = 1;
			ACC();
			flag_dec = 0;
			flag_usk = 1;
			flag_test = 1;
			Motor_Off();
			break;
		}
	}
}

