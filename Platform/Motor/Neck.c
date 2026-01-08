/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Neck.c
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
 * @attention
  *
  * Belongs to neck test for ZEHO AI ROBOT Demo V1.0.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* USER CODE BEGIN */
#ifndef __MOTOR_H
#define __MOTOR_H

#include <string.h>
#include "Neck.h"
#include "tim.h"
#include "servo_angles.h"
/**
* @brief  Twist The Neck
* @param  Angle
*         This parameter can be one of the following values:
*            @arg Angle: Servo_R_MAX(LEFT MAX)
*            @arg Angle: Servo_R_MIN(RIGHT MAX)
*            @arg Angle: Servo_R_MID(Start position)
* @retval cmd status
*	 Position ------------------------------------------------------------------
*																		 back
*																		Servo_3
*																		
*																		Servo_R
*																		
*														Servo_1						Servo_2
*0:0.5ms-50   45:1ms-100	90:1.5ms-150		135:2ms-200			180:2.5ms-250
 */
/*
void NeckTwist(uint8_t Angle)
{
	if((Angle>Servo_R_MAX) || (Angle<Servo_R_MIN))
	{
		return;
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Angle);
	}
}*/

/**
  * @brief  Twist The Neck
  * @param  Angle
  *         This parameter can be one of the following values:
  *            @arg Angle: Servo_R_MAX(LEFT MAX)
  *            @arg Angle: Servo_R_MIN(RIGHT MAX)
  *            @arg Angle: Servo_R_MID(Start position)
  * @retval cmd status

  */
	// 计算数组长度
    uint16_t len = ARRAY_LENGTH(servo1_angles);
void NeckNod()
{
	for(uint8_t i=0;i<=30;i++){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 100+i);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 100-i);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 100+i);
		HAL_Delay(20);
	}

	for(uint8_t i=0;i<=30;i++){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 130-i);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 70+i);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 130-i);
		HAL_Delay(20);
	}
}
void NeckUpDown()
{
	for(uint8_t i=0;i<=30;i++){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 100-i);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 100-i);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 100-i);
		HAL_Delay(20);
	}
	for(int i=0;i<=30;i++){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 70+i);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 70+i);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 70+i);
		HAL_Delay(20);
	}
}
void NeckShow(){
	for(uint8_t i=0;i<len;i++){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 50+(servo1_angles[i]-30)/0.9);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 50+(servo2_angles[i]-30)/0.9);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 50+(servo3_angles[i]-30)/0.9);
		HAL_Delay(20);
	}
}
/* USER CODE END */
#endif

