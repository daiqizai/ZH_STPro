/*
 * drv8825.h
 *
 *  Created on: Nov 26, 2025
 *      Author: ZEHO
 */

#ifndef INC_DRV8825_H_
#define INC_DRV8825_H_

#ifndef __DRV8825_H
#define __DRV8825_H

#include "stm32f4xx_hal.h"

// 引脚定义（根据实际硬件修改）
#define DRV8825_DIR_GPIO_Port  GPIOF
#define DRV8825_DIR_Pin        GPIO_PIN_0
#define DRV8825_STEP_GPIO_Port   GPIOF
#define DRV8825_STEP_Pin         GPIO_PIN_1
#define DRV8825_EN_GPIO_Port    GPIOF
#define DRV8825_EN_Pin          GPIO_PIN_2
#define DRV8825_MS0_GPIO_Port   GPIOF
#define DRV8825_MS0_Pin         GPIO_PIN_3
#define DRV8825_MS1_GPIO_Port   GPIOF
#define DRV8825_MS1_Pin         GPIO_PIN_4
#define DRV8825_MS2_GPIO_Port   GPIOF
#define DRV8825_MS2_Pin         GPIO_PIN_5
#define DRV8825_RST_GPIO_Port   GPIOF
#define DRV8825_RST_Pin         GPIO_PIN_6

// 宏定义：引脚操作
#define STEP_HIGH  HAL_GPIO_WritePin(DRV8825_STEP_GPIO_Port, DRV8825_STEP_Pin, GPIO_PIN_SET)
#define STEP_LOW   HAL_GPIO_WritePin(DRV8825_STEP_GPIO_Port, DRV8825_STEP_Pin, GPIO_PIN_RESET)
#define DIR_CW     HAL_GPIO_WritePin(DRV8825_DIR_GPIO_Port, DRV8825_DIR_Pin, GPIO_PIN_RESET)  // 顺时针
#define DIR_CCW    HAL_GPIO_WritePin(DRV8825_DIR_GPIO_Port, DRV8825_DIR_Pin, GPIO_PIN_SET)    // 逆时针
#define ENABLE_8825     HAL_GPIO_WritePin(DRV8825_EN_GPIO_Port, DRV8825_EN_Pin, GPIO_PIN_RESET)    // 使能（低电平）
#define DISABLE_8825    HAL_GPIO_WritePin(DRV8825_EN_GPIO_Port, DRV8825_EN_Pin, GPIO_PIN_SET)      // 禁用（高电平）

// 细分模式定义（MS1、MS2、MS3组合，参考DRV8825数据手册）
typedef enum {
    MICROSTEP_1 = 0,    // 1/1步（全步）
    MICROSTEP_2,        // 1/2步
    MICROSTEP_4,        // 1/4步
    MICROSTEP_8,        // 1/8步
    MICROSTEP_16,       // 1/16步
    MICROSTEP_32        // 1/32步（部分版本支持）
} DRV8825_MicroStepTypeDef;

// 函数声明
void DRV8825_Init(DRV8825_MicroStepTypeDef microstep);
void DRV8825_SetMicroStep(DRV8825_MicroStepTypeDef microstep);
void DRV8825_Rotate(uint8_t dir, uint32_t steps, uint16_t step_delay_us);
void DRV8825_Stop(void);

#endif



#endif /* INC_DRV8825_H_ */
