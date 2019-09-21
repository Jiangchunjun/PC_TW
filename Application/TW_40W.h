/*
***************************************************************************************************
*                            configration for 250W
*
* File   : power_config_40w.h
* Author : Moon Jiang
* Date   : 2018.12.04
***************************************************************************************************
* Copyright (C) 2017 OSRAM Asia Pacific Management Company.  All rights reserved.
***************************************************************************************************
*/

#if defined(OT_TW_40W)

#ifndef _POWER_CONFIG_40W
#define _POWER_CONFIG_40W
#define SYSTICK_TIME 150   //10ms systick
#define START_UP_DELAY 0 //300 ms
#define PWM_SLOW_FLAG  0  // no PWM update
#ifdef PMW_2K5
#define SHIFT_BIT1  1
#define TIM1_PERIOD   (12800*1)//(3200*4-1)  //2k5Khz
#else
#define TIM1_PERIOD   799//(3200*4-1)  //40Khz
#endif

#define CURVE_A 6222//(5815)
#define CURVE_B  21//(211) //(*128)
#define DELAY_TIME  2//20ms  
#define ONE_TEN_LOW (143*8)//143
#define ONE_TEN_HIGH (283*8)//283
#define SHIFT_BIT 3  //0// this is for duty value calculation
#define CURRENT_DUTY_INI 200//3200
#define FREQUENCY_20KH     0// 5khz to 20Khz value shift 2 bit 1/4
#define PWM_BIG_STEP      0
#define PWM_SMALL_STEP    0
#define VOLTAGE_INI     0//3200//5Khz
#endif /* _POWER_CONFIG_250W_H */
#endif

/**************** (C) COPYRIGHT OSRAM Asia Pacific Management Company *********END OF FILE*********/
