/**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    30-September-2014
  * @brief   Main program body
   ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "hal_digital.h"
#include "TW_40W.h"
#include <math.h>
/* Private defines -----------------------------------------------------------*/
#define OPTION_ADDRESS 0X4803
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
uint16_t g_period=0, g_on_time=0,g_test_duty;
uint8_t g_test_io=0;
uint32_t g_time_base=32;
extern uint8_t g_dimming_flag, g_mode,flag,g_adc_flag,g_interrupt_flag;
extern uint16_t data;
void main(void)
{  
  static uint16_t count=0; 
  static uint8_t i=0,test_num=0,test_cct=0;
  extern const uint16_t duty_step[51];
  /* Infinite loop */
  __disable_interrupt();
  MCU_Ini();
  __enable_interrupt();
  while (1)
  {   
    if(i++>250)
    {
      WDG_RLOAD;
      i=0; 
    }
    if(g_adc_flag)
    {
      V_Sample();
      g_adc_flag=0;
    }
    if(g_sys_flag)//period 10ms           
    {
      g_sys_flag=0;
      
      TIM1->ARRH = (uint8_t)((TIM1_PERIOD/g_time_base*10) >> 8);
      TIM1->ARRL = (uint8_t)(TIM1_PERIOD/g_time_base*10);
      
      Color_data_save();  
#ifdef TW_SHORT
      Short_protect();
#endif
#ifdef TEST_CCT
      
      g_s_duty=duty_step[test_cct];
#endif
#ifdef TEST_FW
      if(g_test_io>0)
      {
        if(g_test_io++>1)
        {
          g_test_io=0;
          GPIO_WriteLow(GPIOA,GPIO_PIN_1);
        }
      }
#endif      
      //cct_get_data();
      /****this is for 200ms 1 1 delay*/
    }
#ifdef TW_10W
    if(count++>1800)//1800//
#else
     if(count++>1800)//1800// 
#endif
    {
      count=0;
      if(g_a_duty!=g_s_duty)
      {
        if(g_s_duty>g_a_duty)
        {   
#ifdef PMW_2K5
          if(g_s_duty>g_a_duty+(30*SHIFT_BIT1)&&g_a_duty>(30*SHIFT_BIT1))
            g_a_duty+=(26*SHIFT_BIT1);//3
#else     
          if(g_s_duty>g_a_duty+50&&g_a_duty>50)
            g_a_duty+=3;
#endif
          else                     
            g_a_duty++; 
#ifdef    PMW_2K5
          if(g_a_duty>(12799*SHIFT_BIT1))
            g_a_duty=(12799*SHIFT_BIT1);
#else
          if(g_a_duty>799)
            g_a_duty=799;
#endif
        }
        else
        {
          if((g_s_duty<g_a_duty))
          {   
#ifdef PMW_2K5
            if(g_a_duty>g_s_duty+(30*SHIFT_BIT1)&&g_a_duty<(12770*SHIFT_BIT1))
              g_a_duty-=(26*SHIFT_BIT1);//3
#else
            if(g_a_duty>g_s_duty+50&&g_a_duty<50)
              g_a_duty-=3;
#endif
            else
              g_a_duty--;
            if(g_a_duty<0)g_a_duty=0;
          }
        }
      if(flag==0)
      {
//        if(g_a_duty>795)
//        {
//          CURRENT_UPDATE_DUTY((799-g_a_duty));//update duty
//          VOLTAGE_UPDATE_DUTY((0)); 
//        }
//        else
//          if(g_a_duty<3)
//          {
//            CURRENT_UPDATE_DUTY((800));//update duty
//            VOLTAGE_UPDATE_DUTY((799-g_a_duty)); 
//          }
//        else
        {
#ifdef PMW_2K5
#ifdef TW_18W
          CURRENT_UPDATE_DUTY((g_a_duty/g_time_base*10)<<0);//update duty
          VOLTAGE_UPDATE_DUTY((g_a_duty/g_time_base*10)<<0); 
          
#else
          CURRENT_UPDATE_DUTY((12799-g_a_duty)*SHIFT_BIT1);//update duty
          VOLTAGE_UPDATE_DUTY((12799-g_a_duty)*SHIFT_BIT1); 
#endif
          
#else
          CURRENT_UPDATE_DUTY((799-g_a_duty)<<0);//update duty
          VOLTAGE_UPDATE_DUTY((799-g_a_duty)<<0); 
#endif 
        }
      }
//        CURRENT_UPDATE_DUTY((3200-g_a_duty)>>3);//update duty
//        VOLTAGE_UPDATE_DUTY((3200-g_a_duty)>>3); 
        //g_a_duty=200;

      }
    }    
  }
}


#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
