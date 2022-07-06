
#include <stdint.h>
#include "Arduino.h"

extern TIM_HandleTypeDef htim3, htim2;

void Motor_Init(void);

void Motor_Run(int speed);

void InterruptHandler(void);

int Motor_GetPulses();