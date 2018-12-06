/**
  ******************************************************************************
  * @file    hal.h
  * @author  Moon
  * @version V0.1
  * @date    19-Jun-2016
  * @brief   This file contains the headers of the mcu configuration and function
  define

  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __hal_H
#define __hal_H

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_gpio.h"
#include "stm8s_tim1.h"
#include "TW_40W.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
    
#define WDG_RLOAD IWDG->KR = (uint8_t)0xAA 
#define DIM_MAX_PERCENT   (1000)
#define DIM_MIN_PERCENT   (100)        
#define DIMMING_FLAG       0X4000
#define POWER_COUNT        0X4003
#define DIMMING_PERCENT    0X4002
#define MODE_UPDATE GPIO_ReadInputPin(GPIOA,GPIO_PIN_2)
#define MODE_LEAD GPIO_ReadInputPin(GPIOA,GPIO_PIN_1)
#define CURRENT_UPDATE_DUTY(a)   TIM1_SetCompare3(a)
//3  
#define VOLTAGE_UPDATE_DUTY(a)   TIM1_SetCompare4(a)//4

#define UPDATE_DUTY(a)   TIM1_SetCompare2(a)//4

/* 00 ----> DW*/ 
/* 01 ---->Flick short*/
/* 10 ---->Flick 3 times*/
/* 11 ---->constant CCT set*/

#define DW_MODE 0X00
#define FLICK_SHORT 0X01
#define FLICK_3     0X02
#define CONSTANT_CCT 0X03
/* Exported functions ------------------------------------------------------- */
void CLK_Config(void);
void GPIO_Config(void);
void ADC_Config(void);
void MCU_Ini(void);
uint16_t get_adc_result(uint8_t adc_channel_no);
void IWDG_Config(void);
void Time1_Config(void);
void Time2_Config(void);
void V_Sample(void);
void delay(uint16_t n);
void one_ten_handle(void);
void PWM1_update(void);
void identify(void);
void pc_period_on_time(void);
void Fast_tune(void);
void Color_data_save(void);
void power_count(void);
void power_down(void);
void power_reset(void);
void Fast_tune();
void dimming_judge();
void DW_handle(void);
int abs1(int i);
extern uint8_t g_sys_flag,g_sys_flag1,identify_flag;
extern uint8_t MotorIndex;
extern uint8_t g_box_s_flag,g_led_handle_id;
extern uint16_t g_period,g_s_s_duty;
extern uint16_t g_on_time;
extern int32_t  g_s_duty;
extern int32_t  g_a_duty,Get_time;
//extern uint16_t Get_time;
#endif /* __STM8S_IT_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
