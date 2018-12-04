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
uint16_t g_period=0, g_on_time=0;
extern uint8_t g_dimming_flag, g_mode;
extern uint16_t data;
void main(void)
{ 
    // static uint8_t delay_done=0;   
    static uint16_t count=0; 
    static uint8_t s_flag_delay=0,i=0;
    
    /* Infinite loop */
    __disable_interrupt();
    MCU_Ini();
    dimming_judge();
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_FALL_ONLY);//ÉÏÉýÑØÖÐ¶Ï
    __enable_interrupt();
    while (1)
    {   
        if(i++>250)
        {
            WDG_RLOAD;
            i=0;
        }
        if(g_sys_flag)//period 10ms           
        {    
          g_sys_flag=0;
          Color_data_save();
          /****this is for 200ms 1 1 delay*/
          if(!s_flag_delay)
          {
            if(count++>=1)
            {
              count=0;
              s_flag_delay=1;
              VOLTAGE_UPDATE_DUTY(g_a_duty);  
              CURRENT_UPDATE_DUTY(g_a_duty);
            }
          }
        }
        if(s_flag_delay)
        {
            if(count++>1800)//1800
            {
                count=0;
                if(g_s_duty>g_a_duty)
                {   
                  if(g_s_duty>g_a_duty+200&&g_a_duty>200)
                    g_a_duty+=20;
                  else                     
                    g_a_duty++; 
                  if(g_a_duty>3200)g_a_duty=3200;
                  {
                    // CURRENT_UPDATE_DUTY(g_a_duty);
                    // VOLTAGE_UPDATE_DUTY(g_a_duty);  
                  }
                }
                else
                {
                  if((g_s_duty<g_a_duty))
                  {   
                      if(g_a_duty>g_s_duty+200&&g_a_duty<3000)
                        g_a_duty-=20;
                      else
                      g_a_duty--;
                    if(g_a_duty<0)g_a_duty=0;
                    {
                      //  CURRENT_UPDATE_DUTY(g_a_duty);
                      //  VOLTAGE_UPDATE_DUTY(g_a_duty);  
                    } 
                  }
                }
                CURRENT_UPDATE_DUTY(g_a_duty);
                VOLTAGE_UPDATE_DUTY(g_a_duty);  
            }
        }
         //wfi();        
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
