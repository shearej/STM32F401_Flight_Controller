/**
  ******************************************************************************
  * @file           : driver.c
  * @brief          : Motor driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define MOTOR1 TIM_CHANNEL_1
#define MOTOR2 TIM_CHANNEL_2
#define MOTOR3 TIM_CHANNEL_3
#define MOTOR4 TIM_CHANNEL_4

#define MOTOR_MAX_PULSE 2000
#define MOTOR_MIN_PULSE 1100
#define MOTOR_STOP_PULSE 0

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static uint8_t motorsArmed;

static uint32_t motor1;
static uint32_t motor2;
static uint32_t motor3;
static uint32_t motor4;

/* Private function prototypes -----------------------------------------------*/

void Start_Motor(uint32_t);
void Stop_Motor(uint32_t);

/* Private user code ---------------------------------------------------------*/

void ArmMotor(void *argument)
{
  /* USER CODE BEGIN 5 */

  uint32_t channel = *(uint32_t *) argument;
  uint8_t motorArmed = 0;

  /* Infinite loop */
  for(;;)
  {
	if (0 == motorArmed)
	{
	  Set_PWM(1915, channel);
	  osDelay(pdMS_TO_TICKS(1000));
	  Set_PWM(1000, channel);
	  osDelay(pdMS_TO_TICKS( 6000 ));

	  //Start_Motor(channel);

	  motorArmed = 1;
	}
  }

  osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

void Run_Motors()
{
  if (0 == motorsArmed)
	{
	  osThreadNew(ArmMotor, &motor1, NULL);
	  osThreadNew(ArmMotor, &motor2, NULL);
	  osThreadNew(ArmMotor, &motor3, NULL);
	  osThreadNew(ArmMotor, &motor4, NULL);

	  motorsArmed = 1;
	}

    Start_Motor(motor1);
    Start_Motor(motor2);
    Start_Motor(motor3);
    Start_Motor(motor4);
}

void Start_Motor(uint32_t channel)
{
	Set_PWM(MOTOR_MIN_PULSE, channel);
}

void Stop_Motors()
{
	Stop_Motor(motor1);
	Stop_Motor(motor2);
	Stop_Motor(motor3);
	Stop_Motor(motor4);
}

void Stop_Motor(uint32_t channel)
{
	Set_PWM(MOTOR_STOP_PULSE, channel);
}

void Innit_System(void)
{
	motorsArmed = 0;

	motor1 = MOTOR1;
    motor2 = MOTOR2;
    motor3 = MOTOR3;
    motor4 = MOTOR4;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
