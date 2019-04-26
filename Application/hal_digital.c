/**
  ******************************************************************************
  * @file    hal.c
  * @author  Moon
  * @version V0.1
  * @date    1-Jun-2016
  * @brief   Mcu hardware configure
  *          Timeout case actions 
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "hal_digital.h"
#include "math.h"
#include <math.h>
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t g_sys_flag=1,g_adc_flag=0;
uint8_t g_sys_flag1=0;
uint16_t g_v_arr[1];
int32_t g_data;
uint16_t voltage_duty=VOLTAGE_INI;//160;//20%
uint16_t current_duty=CURRENT_DUTY_INI; //1% to 10%
uint16_t transformer_duty=40;
uint16_t  g_current_duty_s=320;
uint16_t  g_current_duty_a=CURRENT_DUTY_INI;
int32_t  g_s_duty=0;
uint8_t  g_record_color_data=0;
uint8_t  g_s_color_data=0;
uint16_t  g_s_s_duty=0;
int32_t  g_a_duty=CURRENT_DUTY_INI;
int32_t Get_time=0;
uint8_t g_dimming_flag=0;
uint16_t ad_arr[60]={0,0,0};
uint32_t ad_fast=0;
uint32_t ad_sum=0;
uint8_t count1=0;
uint8_t identify_flag=0;
uint16_t off_count=0;
uint32_t temp2_pre=0;
uint16_t ad_ac_low=0;
uint8_t flick_handler=0;
uint8_t mode_update_flag=0;
uint8_t flick_judge=1;
uint8_t g_mode=0;
uint32_t g_duty_index=0;
uint8_t g_flag_uart=0;
uint8_t g_flag_uart1=0;
uint8_t g_flag_cct=0;
uint16_t g_pluse_count=0;
uint8_t g_save_flag=0;
uint8_t flag=0;
uint8_t short_flag=0,short_count=0;
uint16_t wirte_count=0;
#ifdef COMP_40KHZ
const uint16_t duty_step[51]={0	,
3	,
4	,
6	,
7	,
9	,
10	,
13	,
15	,
19	,
31	,
45	,
56	,
66	,
76	,
86	,
97	,
108	,
118	,
129	,
140	,
151	,
163	,
174	,
185	,
196	,
207	,
218	,
229	,
241	,
252	,
263	,
274	,
285	,
296	,
306	,
317	,
328	,
338	,
349	,
359	,
376	,
384	,
387	,
390	,
392	,
393	,
395	,
396	,
397	,
399	};
    
#else
const uint16_t duty_step[51]={4	,
5	,
9	,
30	,
60	,
85	,
115	,
140	,
165	,
195	,
220	,
245	,
270	,
295	,
325	,
345	,
370	,
390	,
415	,
435	,
455	,
475	,
495	,
515	,
530	,
550	,
565	,
582	,
598	,
610	,
626	,
642	,
655	,
666	,
678	,
693	,
704	,
716	,
726	,
734	,
745	,
757	,
765	,
780	,
785	,
789	,
791	,
792	,
793	,
794	, //794
795	   //795
  };
#endif
extern uint16_t  test[100];
extern uint8_t index,g_run_flag,flag_pulse;
extern uint32_t g_pulse_time1;
//uint16_t g_on_time=0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/*************************************************************************************/
/***clock configure***********************************************************/
void CLK_Config(void)
{
  CLK_DeInit();
  /* Clock divider to HSI/1 */
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
  
}
/********************************************************************************/
/***IO ports configure***********************************************************/
void GPIO_Config(void)
{
  //GPIO_DeInit(GPIOA);
  //GPIO_DeInit(GPIOB);
  //GPIO_DeInit(GPIOC);
  //GPIO_DeInit(GPIOD);
#ifdef EXTERN_INTERRUPT
    //GPIO_Init(GPIOD,GPIO_PIN_3,GPIO_MODE_IN_FL_NO_IT);  //1-10V input detection
    //GPIO_Init(GPIOC,GPIO_PIN_4,GPIO_MODE_IN_FL_NO_IT);  //1-10V input detection
    //GPIO_Init(GPIOD,GPIO_PIN_6,GPIO_MODE_IN_FL_NO_IT);
  GPIOD->CR2 &= (uint8_t)(~(GPIO_PIN_6));
  GPIOD->DDR &= (uint8_t)(~(GPIO_PIN_6));
  GPIOD->CR1 &= (uint8_t)(~(GPIO_PIN_6));
  GPIOD->CR2 &= (uint8_t)(~(GPIO_PIN_6));
#else
    //GPIO_Init(GPIOD,GPIO_PIN_3,GPIO_MODE_IN_FL_IT);  //1-10V input detection
    //GPIO_Init(GPIOC,GPIO_PIN_4,GPIO_MODE_IN_FL_IT);  //1-10V input detection
#endif
#ifdef TIME_CAPTURE
    //GPIO_Init(GPIOD,GPIO_PIN_3,GPIO_MODE_IN_FL_NO_IT);  //1-10V input detection
    //GPIO_Init(GPIOC,GPIO_PIN_4,GPIO_MODE_IN_FL_NO_IT);  //1-10V input detection
#endif
  //GPIO_WriteLow(GPIOC,GPIO_PIN_6);
  //GPIO_Init(GPIOD,GPIO_PIN_2,GPIO_MODE_OUT_PP_HIGH_SLOW);  //PWM for 1-10 transformer
  //GPIO_Init(GPIOC,GPIO_PIN_4,GPIO_MODE_OUT_PP_LOW_FAST);  //PWM for voltage loop reference
  //GPIO_Init(GPIOA,GPIO_PIN_3,GPIO_MODE_IN_FL_NO_IT);  //OVP event detection
    
    //GPIO_Init(GPIOA,GPIO_PIN_1,GPIO_MODE_IN_FL_NO_IT);  //Voltage detection
    
    GPIOA->CR2 &= (uint8_t)(~(GPIO_PIN_1));
    GPIOA->DDR &= (uint8_t)(~(GPIO_PIN_1));
    GPIOA->CR1 &= (uint8_t)(~(GPIO_PIN_1));
    GPIOA->CR2 &= (uint8_t)(~(GPIO_PIN_1));
    
    //GPIO_Init(GPIOA,GPIO_PIN_2,GPIO_MODE_IN_FL_NO_IT);  //Voltage detection
    
    GPIOA->CR2 &= (uint8_t)(~(GPIO_PIN_2));
    GPIOA->DDR &= (uint8_t)(~(GPIO_PIN_2));
    GPIOA->CR1 &= (uint8_t)(~(GPIO_PIN_2));
    GPIOA->CR2 &= (uint8_t)(~(GPIO_PIN_2));
    
    //GPIO_Init(GPIOD,GPIO_PIN_3,GPIO_MODE_IN_FL_IT);  //AC pulse detection
    
    GPIOD->CR2 &= (uint8_t)(~(GPIO_PIN_3));
    GPIOD->DDR &= (uint8_t)(~(GPIO_PIN_3));
    GPIOD->CR1 &= (uint8_t)(~(GPIO_PIN_3));
    GPIOD->CR2 |= (uint8_t)GPIO_PIN_3;
    
    GPIO_Init(GPIOC,GPIO_PIN_4,GPIO_MODE_OUT_PP_LOW_FAST);  //PWM for voltage loop reference
    
    GPIO_Init(GPIOC,GPIO_PIN_7,GPIO_MODE_OUT_PP_LOW_FAST);  //PWM for voltage loop reference
    
    GPIO_Init(GPIOC,GPIO_PIN_7,GPIO_MODE_OUT_PP_HIGH_FAST);  // IO for test
    
    GPIO_Init(GPIOD,GPIO_PIN_2,GPIO_MODE_IN_FL_NO_IT);  // IO for test
     
    //GPIO_Init(GPIOC,GPIO_PIN_3,GPIO_MODE_OUT_PP_HIGH_FAST);
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_FALL_ONLY);//falling edge
}
/********************************************************************************/

/********************************************************************************/
void MCU_Ini(void)
{
  CLK_Config();
  GPIO_Config();
  dimming_judge();
  Time1_Config();
  Time2_Config();
  UART1_Init(115200,UART1_WORDLENGTH_8D,UART1_STOPBITS_1,UART1_PARITY_NO,UART1_SYNCMODE_CLOCK_DISABLE,UART1_MODE_TXRX_ENABLE);
  UART1_Cmd(ENABLE);
  ADC_Config();
  IWDG_Config(); 
}
/******************************************************************************/
void one_ten_handle(void)
{
 
//    uint32_t temp=0;
//    static uint32_t temp_1=0;
//    static uint8_t count=50,count_1=0,flag_pwm=0;
//    static uint32_t temp_pre=0, temp_diff;
//    //static uint8_t temp_2=0;
//    if(count++>DELAY_TIME)
//    {
//        count=0; 
//        if(count_1++>=70)
//        {
//            //g_v_arr[0];//
//            count_1=70;
//            temp=g_data;
//        }  
//        else
//        {
//            temp=g_v_arr[0];//g_data;//
//        }
//        //temp=g_v_arr[0];
//        if(temp>ONE_TEN_HIGH)//208
//            temp=ONE_TEN_HIGH;
//        if(temp<ONE_TEN_LOW)
//            temp=ONE_TEN_LOW;
//        if(temp_pre>temp)
//        {
//            temp_diff=(temp_pre-temp);  
//        }
//        else
//        {
//            temp_diff=(temp-temp_pre);
//        }
//        if((temp_diff*1000)>(3*temp_pre))
//        {
//            temp_1=(CURVE_A-((CURVE_B*temp)>>SHIFT_BIT));
//            temp_pre=temp;
//        }
//        {
//            if(flag_pwm==PWM_SLOW_FLAG)// no PWM update 0
//            {
//                CURRENT_UPDATE_DUTY(temp_1);//149 //596 for tenny    
//            }
//            else
//                g_current_duty_s=temp_1;
//        }
//        //temp_pre=temp_1;
//    } 
}
/******************************************************************************/
void Pwm_update(void)
{
//    uint16_t s_current_duty=0;
//    
//    if(g_current_duty_a!=g_current_duty_s)
//    {
//        if(g_current_duty_s>g_current_duty_a)
//        {
//            if(g_current_duty_s>g_current_duty_a+100)//100 //2500
//            {
//                g_current_duty_a+=PWM_BIG_STEP;//10 for 75W
//            }
//            else
//            {
//                g_current_duty_a+=PWM_SMALL_STEP;
//            }
//        }
//        else
//        {
//            if(g_current_duty_a>g_current_duty_s+50)
//            {
//                g_current_duty_a-=PWM_BIG_STEP;
//            }
//            else
//            {
//                g_current_duty_a-=PWM_SMALL_STEP;
//            }
//            
//        }
//        
//        s_current_duty=(g_current_duty_a>>FREQUENCY_20KH);//left shift 2bit (5Khz to 20Khz) 
//        CURRENT_UPDATE_DUTY(s_current_duty);
//    }
    
}

/******************************************************************************/
void ADC_Config(void)
{  
 /* De-Init ADC peripheral*/
  ADC1_DeInit();
  /* Init ADC2 peripheral */
  
  ADC1->CR2 &= (uint8_t)(~ADC1_CR2_ALIGN);
  /* Configure the data alignment */
  ADC1->CR2 |= (uint8_t)(ADC1_ALIGN_RIGHT);
  ADC1->CR1 |= ADC1_CR1_CONT; 
  
    /* Clear the ADC1 channels */
  ADC1->CSR &= (uint8_t)(~ADC1_CSR_CH);
  /* Select the ADC1 channel */
  ADC1->CSR |= (uint8_t)(ADC1_CHANNEL_3);
  
  ADC1->CR1 &= (uint8_t)(~ADC1_CR1_SPSEL);
  ADC1->CR1 |= (uint8_t)(ADC1_PRESSEL_FCPU_D18);
  
  ADC1->CR2 &= (uint8_t)(~ADC1_CR2_EXTSEL);
  ADC1->CR2 &= (uint8_t)(~ADC1_CR2_EXTTRIG);
  ADC1->CR2 |= (uint8_t)(ADC1_EXTTRIG_TIM);
  
  ADC1->TDRL |= (uint8_t)((uint8_t)0x01 << (uint8_t)ADC1_SCHMITTTRIG_CHANNEL3);
  
  ADC1->CR1 |= ADC1_CR1_ADON;
  
  //ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS, (ADC1_CHANNEL_6), ADC1_PRESSEL_FCPU_D18, \
            ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, (ADC1_SCHMITTTRIG_CHANNEL5),DISABLE);
  /* Enable ADC1 */
  ADC1->CR1 |= ADC1_CR1_ADON;
  //ADC1_Cmd(ENABLE); 
  /*Start Conversion */
   ADC1->CR1 |= ADC1_CR1_ADON;
  //ADC1_StartConversion();
}
/******************************************************************************/
/* -------------------------------------------------------------------------- */
/* ROUTINE NAME: uint16_t get_adc_result(uint_8 No_channel)                   */
/* INPUT/OUTPUT: None.                                                        */
/* DESCRIPTION:  according the selected channel then get the adc value        */
/* -------------------------------------------------------------------------- */
uint16_t get_adc_result(uint8_t adc_channel_no)
{
	uint8_t temp_channel;	
	uint16_t temp_data_h = 0;
	uint8_t temp_data_l = 0;
	temp_channel = adc_channel_no;
	ADC1->CSR &= (uint8_t)(~ADC1_CSR_CH);            /* Clear the ADC1 channels */	
	ADC1->CSR |= (uint8_t)(temp_channel);            /* Select the ADC1 channel */
	ADC1->CR1 |= (uint8_t)0x01; //
	while(!(ADC1->CSR & 0x80));                                 /* wait adc end */
	ADC1->CSR &= (uint8_t)(~0x80);
	ADC1->CR1 |= (uint8_t)0x01; //
	while(!(ADC1->CSR & 0x80));                                 /* wait adc end */
	ADC1->CSR &= (uint8_t)(~0x80);                        /* clear adc end flag */
	temp_data_l = ADC1->DRL;                                  /* Read LSB first */
	temp_data_h = ADC1->DRH;                                   /* Then read MSB */
	temp_data_h = (uint16_t)(temp_data_l | (uint16_t)(temp_data_h << (uint8_t)8));
	ADC1-> CR1 &= (uint8_t)(~0x01);		//disable ADC
	return(temp_data_h);
}
/******************************************************************************/
void Time1_Config(void)
{
    
  /* Set the Autoreload value */
  TIM1->ARRH = (uint8_t)(TIM1_PERIOD >> 8);
  TIM1->ARRL = (uint8_t)(TIM1_PERIOD);
  
  /* Set the Prescaler value */
  TIM1->PSCRH = (uint8_t)(0x0 >> 8);
  TIM1->PSCRL = (uint8_t)(0x0);
  
  /* Select the Counter Mode */
  TIM1->CR1 = (uint8_t)((uint8_t)(TIM1->CR1 & (uint8_t)(~(TIM1_CR1_CMS | TIM1_CR1_DIR)))
                        | (uint8_t)(TIM1_COUNTERMODE_UP));
  
  /* Set the Repetition Counter value */
  TIM1->RCR = 0;
#ifndef DEAD_TIME
    //TIM1_TimeBaseInit(0x1,TIM1_COUNTERMODE_UP,TIM1_PERIOD,0);
#ifdef TIME_CAPTURE
  TIM1->IER|=0X01;
#endif
  /* Disable the Channel 1: Reset the CCE Bit, Set the Output State , 
  the Output N State, the Output Polarity & the Output N Polarity*/
  TIM1->CCER2 &= (uint8_t)(~( TIM1_CCER2_CC3E | TIM1_CCER2_CC3NE | 
                             TIM1_CCER2_CC3P | TIM1_CCER2_CC3NP));
  /* Set the Output State & Set the Output N State & Set the Output Polarity &
  Set the Output N Polarity */
  TIM1->CCER2 |= (uint8_t)((uint8_t)((uint8_t)(TIM1_OUTPUTSTATE_ENABLE  & TIM1_CCER2_CC3E   ) |
                                     (uint8_t)(TIM1_OUTPUTNSTATE_DISABLE & TIM1_CCER2_CC3NE  )) | 
                           (uint8_t)((uint8_t)(TIM1_OCPOLARITY_LOW   & TIM1_CCER2_CC3P   ) | 
                                     (uint8_t)(TIM1_OCNPOLARITY_LOW  & TIM1_CCER2_CC3NP  )));
  
  /* Reset the Output Compare Bits & Set the Output Compare Mode */
  TIM1->CCMR3 = (uint8_t)((uint8_t)(TIM1->CCMR3 & (uint8_t)(~TIM1_CCMR_OCM)) | 
                          (uint8_t)TIM1_OCMODE_PWM1);//TIM1_OCMODE_PWM1
  
  /* Reset the Output Idle state & the Output N Idle state bits */
  TIM1->OISR &= (uint8_t)(~(TIM1_OISR_OIS3 | TIM1_OISR_OIS3N));
  /* Set the Output Idle state & the Output N Idle state configuration */
  TIM1->OISR |= (uint8_t)((uint8_t)(TIM1_OISR_OIS3 & TIM1_OCIDLESTATE_SET) | 
                          (uint8_t)(TIM1_OISR_OIS3N & TIM1_OCNIDLESTATE_RESET));
  
  /* Set the Pulse value */
#ifdef PMW_2K5
  TIM1->CCR3H = (uint8_t)(6400);//g_s_duty
  TIM1->CCR3L = (uint8_t)(6400);  
#else
  TIM1->CCR3H = (uint8_t)(400);//g_s_duty
  TIM1->CCR3L = (uint8_t)(400);  
#endif 
  //TIM1_OC3Init(TIM1_OCMODE_PWM1,TIM1_OUTPUTSTATE_ENABLE,TIM1_OUTPUTNSTATE_DISABLE,3200-g_s_duty,TIM1_OCPOLARITY_LOW,TIM1_OCNPOLARITY_LOW,TIM1_OCIDLESTATE_SET,TIM1_OCNIDLESTATE_RESET);//HIGH
  
  /* Disable the Channel 4: Reset the CCE Bit */
  TIM1->CCER2 &= (uint8_t)(~(TIM1_CCER2_CC4E | TIM1_CCER2_CC4P));
  /* Set the Output State  &  the Output Polarity */
  TIM1->CCER2 |= (uint8_t)((uint8_t)(TIM1_OUTPUTSTATE_ENABLE & TIM1_CCER2_CC4E ) |  
                           (uint8_t)(TIM1_OCPOLARITY_HIGH  & TIM1_CCER2_CC4P ));
  
  /* Reset the Output Compare Bit  and Set the Output Compare Mode */
  TIM1->CCMR4 = (uint8_t)((uint8_t)(TIM1->CCMR4 & (uint8_t)(~TIM1_CCMR_OCM)) | 
                          TIM1_OCMODE_PWM1);//TIM1_OCMODE_PWM1
  /* Set the Output Idle state */
  TIM1->OISR |= (uint8_t)(~TIM1_CCER2_CC4P);
  /* Set the Pulse value */
#ifdef PMW_2K5
  TIM1->CCR4H = (uint8_t)(6400);//g_s_duty
  TIM1->CCR4L = (uint8_t)(6400);
#else
  TIM1->CCR4H = (uint8_t)(400);//g_s_duty
  TIM1->CCR4L = (uint8_t)(400);
#endif
  //TIM1_OC4Init(TIM1_OCMODE_PWM1,TIM1_OUTPUTSTATE_ENABLE,current_duty,TIM1_OCPOLARITY_LOW,TIM1_OCIDLESTATE_SET);//HIGH
#else
   TIM1_OC2Init(TIM1_OCMODE_PWM1,TIM1_OUTPUTSTATE_ENABLE,TIM1_OUTPUTNSTATE_ENABLE,voltage_duty,TIM1_OCPOLARITY_HIGH,TIM1_OCNPOLARITY_HIGH,TIM1_OCIDLESTATE_SET,TIM1_OCNIDLESTATE_RESET);//HIGH
   TIM1->DTR=0X55;
#endif  
  TIM1->CR1 |= TIM1_CR1_CEN;
  //TIM1_Cmd(ENABLE); 
  TIM1->BKR |= TIM1_BKR_MOE;  
#ifdef PMW_2K5  
  CURRENT_UPDATE_DUTY((400));//update duty
  VOLTAGE_UPDATE_DUTY((400)); 
#else
  CURRENT_UPDATE_DUTY((6400));//update duty
  VOLTAGE_UPDATE_DUTY((6400)); 
#endif
  //g_a_duty=g_s_duty; //test
  //TIM1_CtrlPWMOutputs(ENABLE);  
}
/******************************************************************************/
/******************************************************************************/
void Time2_Config(void)
{
    __disable_interrupt();
    
#ifdef EXTERN_INTERRUPT  
    TIM2->IER|=0X01;
    
    TIM2->PSCR = (uint8_t)(TIM2_PRESCALER_4);
    /* Set the Autoreload value */
    TIM2->ARRH = (uint8_t)(0X10A >> 8);
    TIM2->ARRL = (uint8_t)(0X10A);
    //TIM2_TimeBaseInit(TIM2_PRESCALER_4,0X10A);
#else
    TIM2_TimeBaseInit(TIM2_PRESCALER_64,0XFFFF);
#endif
  //TIM2_OC3Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE,transformer_duty, TIM2_OCPOLARITY_HIGH);
  
  //TIM2_ICInit(TIM2_CHANNEL_2,TIM2_ICPOLARITY_RISING,TIM2_ICSELECTION_DIRECTTI,TIM2_ICPSC_DIV1,0); // capture ini
  //TIM2_OC3PreloadConfig(ENABLE); 
   TIM2->CR1 |= (uint8_t)TIM2_CR1_ARPE;
  //TIM2_ARRPreloadConfig(ENABLE);
#ifdef TIME_CAPTURE
  TIM2_ITConfig( TIM2_IT_CC2 , ENABLE); 
  TIM2_ClearFlag(TIM2_FLAG_CC2);
#endif
  TIM2->CR1 |= (uint8_t)TIM2_CR1_CEN;
  //TIM2_Cmd(ENABLE);  
}
/******************************************************************************/
void V_Sample(void)
{
  static int32_t temp1,temp2;
  static uint16_t s_data_1,s_data_2,s_data_3,s_data_4,s_data_5,s_data_6,s_data_7,s_data_8;
  s_data_1=s_data_2;
  s_data_2=s_data_3;
  s_data_3=s_data_4;
  s_data_4=s_data_5;
  s_data_5=s_data_6;
  s_data_6=s_data_7;
  s_data_7=s_data_8;
  s_data_8=get_adc_result(3); // Voltage for 1-10V voltage
  if(s_data_8<130&&g_data>1500)//130 10W and 40W 0.63V
  {
    short_count++;
  }
  s_data_8&=0X3FF;
  g_v_arr[0]=((s_data_1+s_data_2+s_data_3+s_data_4+s_data_5+s_data_6+s_data_7+s_data_8)>>0);//3
  //g_v_arr[0]=340;
  temp1 =g_v_arr[0];//
  temp2 += (((temp1<<10)- temp2)>>1);
  g_data = temp2>>10;
 
}
/* -------------------------------------------------------------------------- */
/* ROUTINE NAME: void IWDG_Config(void)                                       */
/* INPUT/OUTPUT: None.                                                        */
/* DESCRIPTION:  Initialize the IWDG for 31.25ms                              */
void IWDG_Config(void)
{
  /* IWDG timeout equal to 32 ms (the timeout may varies due to LSI frequency
     dispersion) */
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  //IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG->KR = (uint8_t)0x55;//	
  /* IWDG counter clock: LSI/16 */
  IWDG_SetPrescaler(IWDG_Prescaler_256);
  //IWDG->PR = (uint8_t)0x05;//20140228		
  /* Set counter reload value to obtain 31.25ms IWDG TimeOut.
    Counter Reload Value = 32ms/IWDG counter clock period
                         = 32ms / (LSI/32)
                         = 0.032s / (LsiFreq/32)
                         = LsiFreq/(32 * 31.25)
                         = LsiFreq/1000												
  LsiFreq = 128000Hz ,LsiFreq/1000=128 
   */
  IWDG_SetReload(255);
  //IWDG->RLR = 255;//	
  /* Reload IWDG counter */
  //IWDG_ReloadCounter();
  IWDG->KR = (uint8_t)0xAA;//	
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  //IWDG_Enable();
  IWDG->KR = (uint8_t)0xCC;//20140228	
}
/*******************************************************************************
  * @}
  */
/*******************************************************************************/

void delay(uint16_t n)
{
    while (n-- > 0);
}
/*******************************************************************************/
void identify(void)
{
    static uint16_t count=0, flag=0;
    if(identify_flag==1)
    {   
        if(flag==0)           
        {
            CURRENT_UPDATE_DUTY(60*count);
            VOLTAGE_UPDATE_DUTY(60*count); 
            count++;
            if(count>50)
            {
                flag=1;
                count=0;
            }            
        }
        if(flag==1)
        {
            CURRENT_UPDATE_DUTY(3000-(60*count));
            VOLTAGE_UPDATE_DUTY(3000-(60*count)); 
            count++;
            if(count>50)
            {
                flag=2;
                count=0;
            }   
        }
        if(flag==2)
        {
            CURRENT_UPDATE_DUTY(60*count);
            VOLTAGE_UPDATE_DUTY(60*count); 
            count++;
            if(count>50)
            {
                flag=3;
                count=0;
            }   
        }
        if(flag==3)
        {
            CURRENT_UPDATE_DUTY(3000-(60*count));
            VOLTAGE_UPDATE_DUTY(3000-(60*count)); 
            count++;
            if(count>50)
            {
                flag=0;
                count=0;
                g_a_duty=0;
                identify_flag=0;
            }   
        }
    }
    else
    {
        if(identify_flag==2)
        {
            if(flag==0)
            {                
                CURRENT_UPDATE_DUTY(3200);
                VOLTAGE_UPDATE_DUTY(0);                
                count++;
                if(count>1)
                {
                  flag=1;
                  CURRENT_UPDATE_DUTY(g_a_duty);
                  VOLTAGE_UPDATE_DUTY(g_a_duty);  
                }   
            }
            else
            {
              if(flag==1)
              {
                if(count++>20)
                {
                  flag=0;
                  count=0;
                  identify_flag=0;
                }
              }
            }
            
        }
    }
}
/*******************************************************************************/
void PWM1_update(void)
{
    //static uint16_t g_p_time[8]={0,0,0,0};
    const uint8_t level[11]={0,25,50,75,100,125,150,175,200,225,255};
    static uint32_t g_ton_time[8]={0,0,0,0};
    static uint8_t i=0;
    static int32_t temp, temp1,temp3=0,temp4=0,temp5=0,temp6=0,temp2=0;
    if(1)
    {
#ifndef AVERAGE_SET
        if(Get_time>250)  Get_time=250;
        if(Get_time<=0)   Get_time=250;
        g_ton_time[i]=Get_time;
        if(i++>=7)
        {
            i=0;
        }
        
        //temp=g_p_time[0]+g_p_time[1]+g_p_time[2]+g_p_time[3]+g_p_time[4]+g_p_time[5]+g_p_time[6]+g_p_time[7];
        temp1=g_ton_time[0]+g_ton_time[1]+g_ton_time[2]+g_ton_time[3]+g_ton_time[4]+g_ton_time[5]+g_ton_time[6]+g_ton_time[7];
        temp1*=1;
        if(temp1<500)
            temp1=500;
        if(temp1>1800)
            temp1=1800;
        if(temp1>temp2)
        {
            temp=temp1-temp2;
            if(temp>10)
            {
                //            i=temp/50;
                //            g_s_duty=g_a_duty+100*i;
                g_s_duty=315*temp1/128-1230;
                //            temp=((g_s_duty+1600)*g_s_duty)/8000;
                //            g_s_duty=temp;
                //g_s_duty=g_on_time*8;
                if(g_s_duty>3200)g_s_duty=3200;
                temp2=temp1;
            }
        }
        else
        {
            temp=temp2-temp1;
            if(temp>10)
            {
                //            i=temp/50;
                //            g_s_duty=g_a_duty-100*i;
                //        if(g_s_duty<0)g_s_duty=0;
                //        temp2=temp1;
                 g_s_duty=315*temp1/128-1230;
                //            temp=((g_s_duty+1600)*g_s_duty)/8000;
                //            g_s_duty=temp;
                //g_s_duty=g_on_time*8;
                if(g_s_duty>3200)g_s_duty=3200;
                temp2=temp1;
            }
        }
#else   
        if(ad_ac_low>0)//60
        {
            if(ad_ac_low>60)//60
            {
                g_ton_time[i]=ad_sum;
                if(i++>=7)
                {
                    i=0;
                } 
                
                temp1=g_ton_time[0]+g_ton_time[1]+g_ton_time[2]+g_ton_time[3]+g_ton_time[4]+g_ton_time[5]+g_ton_time[6]+g_ton_time[7];
                temp1/=2;
                //ad_fast=ad_sum*4;
                temp3=temp1;
                temp4+=(((temp3<<10)-temp4)>>7);//7
                

                temp5=temp1;
                temp6+=(((temp5<<10)-temp6)>>4);//4
                temp1=temp4>>10;
                ad_fast=temp6>>10;
                g_duty_index=temp1;
            }
            //     if(ad_fast>temp1+500||temp1>ad_fast+500)
            //     {
            //         
            //     }
            //temp1=ad_fast;
            
            if(abs1(ad_fast-temp1)>=100)
            {
                temp1=ad_fast;
            }
            if(g_dimming_flag==1)//&&g_mode!=CONSTANT_CCT)
            {
                if(temp1<9000)//9000
                    temp1=9000;
                if(temp1>21000)//21000
                    temp1=21000;
                if(temp1>temp2_pre)
                {
                    temp=temp1-temp2_pre;
                    if(temp>150)//740
                    {
                        if(temp1<11500)//10000
                            temp1=11500;
                        if(temp1>19000)//20000
                            temp1=19000;
                        //            i=temp/50;
                        //            g_s_duty=g_a_duty+100*i;
                        g_s_duty=(437*temp1>>10)-4907;//linear calculation
                        //temp2_pre=level[((temp1-11500)/750)];
                        //g_s_duty=(3200*temp2_pre>>8);
                        //            temp=((g_s_duty+1600)*g_s_duty)/8000;
                        //            g_s_duty=temp;
                        //g_s_duty=g_on_time*8;
                        if(g_s_duty>3200)g_s_duty=3200;
                        temp2_pre=temp1;
                    }
                }
                else
                {
                    temp=temp2_pre-temp1;
                    if(temp>150)//
                    {
                        if(temp1<11500)//10000
                            temp1=11500;
                        if(temp1>19000)//20000
                            temp1=19000;
                        //            i=temp/50;
                        //            g_s_duty=g_a_duty-100*i;
                        //        if(g_s_duty<0)g_s_duty=0;
                        //        temp2=temp1;
                        g_s_duty=(437*temp1>>10)-4907;
                        //temp2_pre=level[((temp1-11500)/750)];
                        //g_s_duty=(3200*temp2_pre>>8);
                        //            temp=((g_s_duty+1600)*g_s_duty)/8000;
                        //            g_s_duty=temp;
                        //g_s_duty=g_on_time*8;
                        if(g_s_duty>3200)g_s_duty=3200;
                        temp2_pre=temp1;
                    }
                }
            }
        }
#endif        
    }   
}

void dimming_judge(void)
{
  static uint8_t temp=0;
  uint16_t temp1=0;
  
  temp=FLASH_ReadByte(DIMMING_PERCENT);       
  //temp1=temp;
  g_record_color_data=temp;
  g_s_duty=duty_step[temp];
  if(g_s_duty>799)
    g_s_duty=799;
  temp=FLASH_ReadByte(POWER_COUNT);  
  
  wirte_count=temp;
  
  temp=FLASH_ReadByte(POWER_COUNT+1);
  
  temp1=temp;
  
  wirte_count+=(temp1<<8);
  
  if(wirte_count==0XFFFF)
    wirte_count=0;
}
/******************************************************************************/
void power_down(void)
{
    static uint8_t temp=0,flag=0;
    static uint8_t i=0;
    static uint16_t time_count=0;
    
    uint32_t temp1;
    if(identify_flag!=2)//(!MODE_LEAD)
    {
        if(i<2)
        {
            if(i==0)
            {   
                if(time_count++>40)//400ms delay
                {
                    if(ad_sum>600)// power on condition 600
                    {
                        i=1; 
                        time_count=0;
                    }                                       
                }
            }
            
            if(i==1)//1s
            {
                if(ad_sum<30)// power off //500
                {
                    off_count++; //start count   
                }
                else
                {
                    if(ad_sum>600) //power on 600
                    {
                        if(off_count>1&&off_count<=100)//3s to 10s //effctive time //70 to 100
                        {
                            identify_flag=1;
                            temp=FLASH_ReadByte(DIMMING_FLAG);  
                            if(temp==0X5C)//5C DW
                            {
                                FLASH_Unlock(FLASH_MEMTYPE_DATA);
                                //FLASH_ProgramByte(DIMMING_FLAG,0X5D); // change to 5D TW
                                //WDG_RLOAD;
                                g_dimming_flag=2;
                                {
                                    temp=FLASH_ReadByte(DIMMING_PERCENT);                       
                                    temp1=temp;
                                    g_s_duty=(uint32_t)(temp1*3200/255);
                                }
                                FLASH_Lock(FLASH_MEMTYPE_DATA);
                            }
                            else
                            {   
                               // if(temp==0X5D)//5D TW
                                {
                                    FLASH_Unlock(FLASH_MEMTYPE_DATA);
                                    //FLASH_ProgramByte(DIMMING_FLAG,0X5C);  // change to 5C DW
                                    //WDG_RLOAD;
                                    g_dimming_flag=1;
                                    temp2_pre=0;
                                    FLASH_Lock(FLASH_MEMTYPE_DATA);
                                    mode_update_flag=1;
                                }
//                                else
//                                {                                          //single mode change to DW
//                                    FLASH_Unlock(FLASH_MEMTYPE_DATA);
//                                    FLASH_ProgramByte(DIMMING_FLAG,0X5C);  // change to 5C DW
//                                    //WDG_RLOAD;
//                                    g_dimming_flag=1;
//                                    temp2_pre=0;
//                                    FLASH_Lock(FLASH_MEMTYPE_DATA);
//                                }
                            }
                            off_count=0;
                            i=0; //back to fisrt condition
                            //mode change
                        }
                        else
                        {
                            if(off_count>80)
                            {
                                off_count=0;
                                i=0; //back to fisrt condition
                            }
                        }
                    }
                    
                }
                
            }                      
        }
    }
}
/******************************************************************************/
void power_count(void)
{
//    static uint8_t temp=0;
//    static uint8_t i=0;
//    static uint16_t time_count=0;
//    uint32_t temp1;
//    if(1)//(!MODE_LEAD)
//    {
//        if(i<3)
//        {
//            if(i==0)
//            {   
//                if(time_count>40)//800ms
//                {
//                    i=1;
//                    //FLASH_Unlock(FLASH_MEMTYPE_DATA);
//                    temp=FLASH_ReadByte(POWER_COUNT);
//                    //FLASH_Lock(FLASH_MEMTYPE_DATA);
//                    if(temp==255||temp<100)
//                    {
//                        temp=100;
//                    }
//                    if(temp>106)
//                    {
//                        temp=106; 
//                    }
//                }
//            }
//            time_count++;
//            if(time_count>80&&i==1)//1s
//            {
//                FLASH_Unlock(FLASH_MEMTYPE_DATA);
//                //WDG_RLOAD;
//                FLASH_ProgramByte(POWER_COUNT,(temp+1));
//                //WDG_RLOAD;
//                FLASH_Lock(FLASH_MEMTYPE_DATA);
//                i=2;
//            }
//            
//            if(time_count>300&&i==2)//5s
//            {  
//                //FLASH_Unlock(FLASH_MEMTYPE_DATA);
//                temp=FLASH_ReadByte(POWER_COUNT);
//                //FLASH_Lock(FLASH_MEMTYPE_DATA);
//                if(temp==255||temp<100)
//                {
//                    temp=100;
//                }
//                if(temp>106)
//                {
//                    temp=106; 
//                }
//                if((temp-100)>=6)
//                {
//                    FLASH_Unlock(FLASH_MEMTYPE_DATA);
//                    //WDG_RLOAD;
//                    FLASH_ProgramByte(DIMMING_FLAG,0X5B);  // single mode default mode
//                    //WDG_RLOAD;
//                    FLASH_Lock(FLASH_MEMTYPE_DATA);
//                    g_dimming_flag=0;
//                    g_s_duty=3200;
//                }
//                else
//                {
//                    if((temp-100)>=3)
//                    {
//                        temp=FLASH_ReadByte(DIMMING_FLAG);  
//                        if(temp==0X5C)//5C DW
//                        {
//                            FLASH_Unlock(FLASH_MEMTYPE_DATA);
//                            FLASH_ProgramByte(DIMMING_FLAG,0X5D); // change to 5D TW
//                            //WDG_RLOAD;
//                            g_dimming_flag=2;
//                            temp=FLASH_ReadByte(DIMMING_PERCENT);                       
//                            temp1=temp;
//                            g_s_duty=(uint32_t)(temp1*3200/255);
//                            FLASH_Lock(FLASH_MEMTYPE_DATA);
//                        }
//                        else
//                        {   
//                            if(temp==0X5D)//5D TW
//                            {
//                                FLASH_Unlock(FLASH_MEMTYPE_DATA);
//                                FLASH_ProgramByte(DIMMING_FLAG,0X5C);  // change to 5C DW
//                                //WDG_RLOAD;
//                                g_dimming_flag=1;
//                                FLASH_Lock(FLASH_MEMTYPE_DATA);
//                            }
//                            else
//                            {                                          //single mode change to DW
//                                FLASH_Unlock(FLASH_MEMTYPE_DATA);
//                                FLASH_ProgramByte(DIMMING_FLAG,0X5C);  // change to 5C DW
//                                //WDG_RLOAD;
//                                g_dimming_flag=1;
//                                FLASH_Lock(FLASH_MEMTYPE_DATA);
//                            }
//                        }
//                    }
//                } 
//                i=3;
//                FLASH_Unlock(FLASH_MEMTYPE_DATA);
//                FLASH_ProgramByte(POWER_COUNT,100);
//                //            while(FLASH_WaitForLastOperation(FLASH_MEMTYPE_DATA)==FLASH_STATUS_SUCCESSFUL_OPERATION)
//                //            {}
//                FLASH_Lock(FLASH_MEMTYPE_DATA);
//            }
//        }
//    }
}
    
/******************************************************************************/
void power_reset()
{
    static uint8_t i=0;
    static uint16_t time_count=0;
    static uint16_t reset_count=0;
    if(identify_flag!=2)//(!MODE_LEAD)
    {

            if(i==0)
            {   
                if(time_count++>10)//40 100ms delay
                {
                    if(ad_sum>600)// power on condition //600
                    {
                        i=1; 
                        reset_count++;
                        time_count=0;
                    }                                       
                }
            }
            
            if(i==1)//power is on 
            {
                time_count++;// record time
                if(time_count>70)//400 more than 0.7s then clear count 
                {
                    time_count=0;
                    reset_count=0;
                    //i=0;
                }
                if(ad_sum<500)//500
                {
                    if(time_count>10&&time_count<=70)// 60 400 1 to 4 s is effective
                    {
                    }
                    else
                    {
                       reset_count=0; 
                    }
                    time_count=0;
                    i=0;
                }
            }
            if(reset_count>=2)//3 times to reset mode
            {
                off_count=0;
                reset_count=0;
                FLASH_Unlock(FLASH_MEMTYPE_DATA);
                //WDG_RLOAD;
                //FLASH_ProgramByte(DIMMING_FLAG,0X5B);  // single mode default mode
                //WDG_RLOAD;
                FLASH_Lock(FLASH_MEMTYPE_DATA);
                g_dimming_flag=0;
                g_s_duty=0;
                identify_flag=1;
            }
    }
}
/******************************************************************************/


void pc_period_on_time(void)
{
//    static uint16_t temp=0,temp1=0,temp2=0,temp3=0,temp4=0;
//    static uint32_t count1,count2,count3;
//    static uint8_t flag1=0,flag2=0;    
//    
//    temp1=temp2;
//    //temp2=temp3;
//    
//    //temp3=temp4;
//    temp2=get_adc_result(4);
//    
//    temp=(temp1+temp2)>>1;
//    if(temp>204&&flag1==0)
//    {   
//        flag1=1;
//        //count3=count2;
//        //if(count3==0) count3=1;
//        //CURRENT_UPDATE_DUTY(count3);
//        //VOLTAGE_UPDATE_DUTY(count3);
//        g_period=count2;
//        g_on_time=count1;
//        count2=0;
//        count1=0;
//        //GPIO_WriteReverse(GPIOD,GPIO_PIN_2);
//        //g_s_duty=g_on_time*8;
//    }
//    if(flag1==1&&temp2>204)
//    {
//        count1++;
//        //GPIO_WriteReverse(GPIOD,GPIO_PIN_2);
//        // count2++;
//    }  
//    if(flag1==1)
//    {
//        if(temp<180)
//            flag1=0;
////        flag2=1;
//    }
//    if(flag2==1||flag1==1)
////    {
//        count2++;
////    }
//    if(flag1==0&&count1==0||count2>500)
//    {
//        g_on_time=0;
//        count1=0;
//        count2=0;
//    }

}
/******************************************************************************/
void DW_handle(void)
{   
    static uint8_t delay_done=100;
    static uint16_t s_pre_duty=0;
    static uint8_t save_count=0;
    static uint8_t temp=0;
    static int32_t temp2;
    static uint8_t temp3=4;
    static uint8_t flag=0;
    if(flick_handler!=1)//&&g_mode!=CONSTANT_CCT)
    {
    if(delay_done++>=40&&(g_dimming_flag==0X01)&&(ad_ac_low>60))//every 100ms 600
    {
        delay_done=0;
        if(s_pre_duty>g_s_duty)
        {
            temp=s_pre_duty-g_s_duty;
        }
        else
        {
            temp=g_s_duty-s_pre_duty;
        }
        s_pre_duty=g_s_duty;
        if(temp>80&&flag)
        {
            flag=0;
            save_count=0;// big gap
            count1=0;
        }
        else
        {
          if(flag==1&&temp<40)
          {
            count1++;
          }
          else
          {
            count1=0;
          }
        }
        if(count1>=13)
        {
          count1=0;
          FLASH_Unlock(FLASH_MEMTYPE_DATA);
          //FLASH_ProgramByte(DIMMING_FLAG,0X5D); // change to 5D TW
          //WDG_RLOAD;
          identify_flag=1;
          g_dimming_flag=2;
          FLASH_Lock(FLASH_MEMTYPE_DATA);          
        }
        if(temp<40&&flag==0)//
        {
            save_count++;
        }
        else
        {
            save_count=0;
        }
        if(save_count>=temp3)
        {
            save_count=0;
            flag=1;
            FLASH_Unlock(FLASH_MEMTYPE_DATA);
            temp=FLASH_ReadByte(DIMMING_PERCENT);
            temp2=temp;
            temp2=(temp2*3200/255);
            if(temp2>g_s_duty)
            {
                if(temp2-g_s_duty>40)//20
                {
                    //FLASH_ProgramByte(DIMMING_PERCENT,(uint8_t)(g_s_duty*255/3200));
                }
            }
            else
            {
                if(g_s_duty-temp2>40)//20
                {
                    //FLASH_ProgramByte(DIMMING_PERCENT,(uint8_t)(g_s_duty*255/3200));    
                }
            }
//            if(MODE_UPDATE)
//            {
//                g_dimming_flag=2;
//                FLASH_ProgramByte(DIMMING_FLAG,0X5D);//back to TW mode
//            }
            g_s_s_duty=g_s_duty;
            FLASH_Lock(FLASH_MEMTYPE_DATA);
        }

    }
    }
}
/******************************************************************************/
void Fast_tune(void)
{
//    static uint16_t count=0;
//    static uint8_t flag_delay=0;
//    static uint8_t flag_mode=0;// 0 means start 
//   
//    static uint8_t s_flag=0;
//    
//    static uint16_t count2=0;
//    if(!flag_delay)
//    {
//        if(count++>=400)//4s
//        {
//            count=0;
//            flag_delay=1;
//        }
//    }
//    
//    if(flag_delay&&MODE_LEAD)
//    {
//        if(flag_mode==0)
//        {
//            if(ad_fast<=8000)
//            {
//                flag_mode=4;// small angle start 
//            }
//            else
//            {
//                if(ad_fast>=16000)
//                {
//                    flag_mode=5;//big angle start
//                }
//            }
//        }
//        if(flag_mode==4)
//        {
//            if(ad_fast>=16000)
//            {
//                flag_mode=2;//start to change at big 
//            }
//        }
//        if(flag_mode==5)
//        {
//            if(ad_fast<=8000)
//            {
//                flag_mode=1;//start to change at small 
//            }
//        }
//        if(flag_mode==1)
//        {
//            if(ad_fast>=16000)
//            {
//                count1++;
//                flag_mode=2;
//                count2=0;
//            }
//        }
//        if(flag_mode==2)
//        {
//            if(ad_fast<=8000)
//            {
//                count1++;
//                flag_mode=1;
//                count2=0;
//            }
//        }
//        
//       if(flag_mode==1||flag_mode==2)
//       {
//           count2++;
//       }
//       if(count2>=300)
//       {
//           count2=0;
//           flag_mode=0;
//           if(count1>=2)
//           {
//               //                        count=0;
//               //                        count1=0;
//               //                        flag_mode=0;
//               if(g_dimming_flag==2||g_dimming_flag==0)
//               {
//                   g_dimming_flag=1;
//                   FLASH_Unlock(FLASH_MEMTYPE_DATA);
//                   FLASH_ProgramByte(DIMMING_FLAG,0X5C);
//                   FLASH_Lock(FLASH_MEMTYPE_DATA);
//               }
//               else
//               {
//                   g_dimming_flag=2;
//                   g_s_duty=g_s_s_duty;
//                   CURRENT_UPDATE_DUTY(g_s_duty);
//                   VOLTAGE_UPDATE_DUTY(g_s_duty); 
//                   FLASH_Unlock(FLASH_MEMTYPE_DATA);
//                   FLASH_ProgramByte(DIMMING_FLAG,0X5D);
//                   FLASH_Lock(FLASH_MEMTYPE_DATA);
//               }
//           }
//           count1=0;
//           count=0;
//       }
//        if(count1>=5)
//        {
//            if(count<=1000)
//            {
//                s_flag=1;
//            }
//        }
//        if(flag_mode==1||flag_mode==2)
//        {
//            if(count++>=800||s_flag==1)//10s
//            {  
//                if(count1>=5)
//                {
//                    FLASH_ProgramByte(DIMMING_FLAG,0X5B);  // single mode default mode
//                    g_dimming_flag=0;
//                    g_s_duty=3200;  
//                }
//                else
//                {
//                    if(count1>=2)
//                    {
////                        count=0;
////                        count1=0;
////                        flag_mode=0;
//                        if(g_dimming_flag==2||g_dimming_flag==0)
//                        {
//                            g_dimming_flag=1;
//                            FLASH_Unlock(FLASH_MEMTYPE_DATA);
//                            FLASH_ProgramByte(DIMMING_FLAG,0X5C);
//                            FLASH_Lock(FLASH_MEMTYPE_DATA);
//                        }
//                        else
//                        {
//                            g_dimming_flag=2;
//                            g_s_duty=g_s_s_duty;
//                            CURRENT_UPDATE_DUTY(g_s_duty);
//                            VOLTAGE_UPDATE_DUTY(g_s_duty); 
//                            FLASH_Unlock(FLASH_MEMTYPE_DATA);
//                            FLASH_ProgramByte(DIMMING_FLAG,0X5D);
//                            FLASH_Lock(FLASH_MEMTYPE_DATA);
//                        }
//                    }
//                }
//                count=0;
//                count1=0;
//                flag_mode=0;
//                s_flag=0;
//            }
//            
//        }       
//    }
}
/******************************************************************************/
void Color_data_save(void)
{
   static uint8_t save_count=0,error_count=0,count=0;
   uint8_t data_1=0;
   extern uint16_t period_time;
   if(save_count++>150)
   {
     save_count=150;
     if(g_data<=2000)//1V power off
     {   
       if(g_s_color_data!=g_record_color_data&&g_save_flag==1&&flag_pulse==0)
       {
         g_record_color_data=g_s_color_data;
         save_count=0;
         g_save_flag=0;
         
         wirte_count++;
         
         FLASH_Unlock(FLASH_MEMTYPE_DATA);
         FLASH_ProgramByte(DIMMING_PERCENT,(uint8_t)g_record_color_data);
         
         FLASH_ProgramByte(POWER_COUNT,(uint8_t)wirte_count);
         
         FLASH_ProgramByte(POWER_COUNT+1,(uint8_t)(wirte_count>>8));
         FLASH_Lock(FLASH_MEMTYPE_DATA);
       }
     }
   }
   //g_pluse_count++;
   if(g_flag_uart==1)
   {
     g_flag_uart=0;
     UART1_SendData8(g_s_color_data+1);
   }
   if(g_flag_uart1)
   {
     if(g_flag_uart1==1)
     {
       if(count==0)
       {
          data_1=(test[index-1]>>8);
          data_1=(g_pluse_count>>8);
         UART1_SendData8(data_1);
       }
       else
       {
         //UART1_SendData8(test[index-1]&0XFF);
         UART1_SendData8(g_pluse_count&0XFF);
       }
     }
     else
     {
       if(g_flag_uart1==3)
       {
         
       }
       //UART1_SendData8(error_count++);
       //UART1_SendData8(period_time-240);
     }
     if(count++==1)
     {
     g_flag_uart1=0;
     count=0;
     }
   }
}
/******************************************************************************/
void Flicker_handle(void)
{
    static uint16_t flicker_count=0;    
    static uint8_t delay_flag=0,flicker_count1=0;;
    static uint16_t delay_time=0;
    static uint16_t pre_ad_fast, pre_ad_low=0; 
    static uint8_t count=0;
    if(delay_flag==0)
    {   
        if( flicker_count++>150)//delay 1s
        {
            delay_flag=1;
            flicker_count=0;
        }
    }
    if(off_count>0)
    {
        mode_update_flag=1;
        delay_time=0;
    }
    if(g_dimming_flag==0X01&&delay_flag==1&&mode_update_flag==0)//
    {
        if(abs1(ad_fast-pre_ad_fast)>30)
        {
            pre_ad_fast=ad_fast;
        }
        else
            ad_fast=pre_ad_fast;
        
        if(flick_judge)
        {
            if(ad_fast<11000&&ad_fast>2000)//(ad_fast<10500)//(ad_ac_low<262&&ad_ac_low>50)//9000 ad_fast
            {
                flick_handler=1;
                flick_judge=0;
            }
        }
        
        if(g_mode==FLICK_3)
        {
            if(flick_handler==2)//==2 three times
            {
                if(ad_fast>11500)// <20)
                {
                    flick_handler=0;
                    flick_judge=1;
                    delay_time=0;
                    mode_update_flag=0;
                }
            }
        }
        else
        {
            if(flick_handler==1)//==2 three times
            {
                if(ad_fast>11500)// <20)
                {
                    flick_handler=0;
                    flick_judge=1;
                    delay_time=0;
                    mode_update_flag=0;
                }
            }
        }

    }
    
    if(mode_update_flag)
    {
        if(delay_time++>650)
        {
            mode_update_flag=0;
            delay_time=0;
        }
    }
}
/******************************************************************************/
  int abs1(int i)
  {      /* compute absolute value of int argument */
    return (i < 0 ? -i : i);
  }
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/******************************************************************************/
void cct_get_data(void)
{ 
  static uint16_t data=0;
  uint8_t i=0;
  if(g_flag_cct==1)
  {
    g_flag_cct=0;
    data=0;
    for(i=0;i<(index-1);i++)
    {
      
      if(300<test[i]&&test[i]<600)//450
      {
        data|=(1<<9);
      }
      else
        if(700<test[i]&&test[i]<1100)//900
        {
          data|=(1<<8);
        }
        else
          if(1150<test[i]&&test[i]<1550)//1350
          {
            data|=(1<<7);
          }
          else
            if(1600<test[i]&&test[i]<2000)//1800
            {
              data|=(1<<6);
            }
            else
              if(2050<test[i]&&test[i]<2450)//2250
              {
                data|=(1<<5);
              }
              else      
                if(2500<test[i]&&test[i]<2900)//2700
                {
                  data|=(1<<4);
                }
                else
                  if(2950<test[i]&&test[i]<3350)//3150
                  {
                    data|=(1<<3);
                  }
                  else
                    if(3400<test[i]&&test[i]<3800)//3600
                    {
                      data|=(1<<2);
                    }
      
                    else      
                      if(3850<test[i]&&test[i]<4250)//4050
                      {
                        data|=(1<<1);
                      }
                      else
                        if(4300<test[i]&&test[i]<4700)//4500
                        {
                          data|=(1<<0);
                        }
      
    }
//    g_s_color_data=((data>>4)-1);
//    if(g_s_color_data>50)
//      g_s_color_data=50;
//    uint32_t temp=0; 
//    temp=g_s_color_data;
//    g_s_duty=3200*(temp);
//    g_s_duty/=50;
//    g_flag_uart=1;
//    g_save_flag=1;
   //UART1_SendData8((data>>4));
  }
}

void Short_protect(void)
{
  static uint16_t delay=0; 
  static uint16_t count=0,short_time=0;
  
  {
    if(delay++>200)
    { 
      if(delay==202)
      {
        short_count=0;
      }
      delay=203;
      
      if(short_time++>200)
      {
        short_time=0;
        
        if(short_count>10&&short_flag==0)
        {
          short_flag=1;
        }
        short_count=0;
      }
      if(short_flag==1)//g_data<2000&&flag==0||
      {
        flag=1; 
      }
      
      if(flag==1)
      {
        CURRENT_UPDATE_DUTY((799));//update duty
        VOLTAGE_UPDATE_DUTY((0));
        flag=2;
      }
      
      if(flag==2)
      {
        if(count++>50)//close 2s
        {
          count=0;
          flag=3;
          CURRENT_UPDATE_DUTY((g_a_duty));//update duty
          VOLTAGE_UPDATE_DUTY((g_a_duty));
        }
      }
      if(flag==3)
      {
        if(count++>200)
        {
          count=0;
          flag=0;
          short_flag=0;
        }
      }
    }
  
  }

}
/******************************************************************************/