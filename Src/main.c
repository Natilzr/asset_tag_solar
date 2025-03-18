/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BC7161.h"
#include "ble_adv.h"
#include "string.h"
#include "reg_parameter.h"
#include "max31726.h"
#include "sht3x.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_USER_START_ADDR   0x08003C00   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     0x08003F00   /* End @ of user Flash area */

#define         SM32F0xx_EL_SIGN_ADDRESS 0x1FFFF7AC // RM0091 p.933
#define BD_ADDR_SIZE  (6)

#define FLOAT_TO_INT(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

uint8_t g_BDdaddr[BD_ADDR_SIZE];
uint8_t TagID[4];
  uint32_t Address = 0, PageError = 0;
  
//#define IBtemp 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//                            --------------------------
//                     BOOT---|1                PA14 20|--swclk
//                     RFSCL--|2 PF0            PA13 19|--swdio
//                     RFSDA--|3 PF1            PA10 18|--SDA,RX
//                     NRST --|4                PA9  17|--SCL,TX
//                     VDDA --|5                VDD  16|--VDD
//                   INTLIS --|6 PA0            VSS  15|--VSS
//                   BATT ADC-|7 PA1            PB1  14|--GPIO3
//                   RX free -|8 PA2            PA7  13|--IN1
//                   TX free -|9 PA3            PA6  12|--IN2
//                      free -|10 PA4           PA5  11|--LED
//                            --------------------------  
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void BC7161_inital(void);
void BeaconSend(void);
void RTC_Init(void);
static void RTC_AlarmConfig(void);
void   GPIO_SLEEP(void);
void BC7161_wakeupAndSamp(void);
void GetMac(void);
void PduTimeUpdate(uint32_t *count,uint32_t *time);
void UartTest(void);
uint32_t _atoi(uint8_t *s);
void GetMM(void);
void LedOn(void);
void LedOff(void);
uint8_t itoa(uint8_t i);
void hextoc(uint8_t hex,uint8_t* msb,uint8_t* lsb);
//uint16_t Temp;
//uint16_t Humi;
//float Temp;
//float Humi;
uint8_t Temp[2];
uint8_t Humi[2];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  uint8_t		timeout_cnt;
static adv_pdu_struct AdvPayload;
  uint8_t		pdu_length;

  uint8_t       ERROR_F;
#ifndef IBtemp
  //for TLM
  uint32_t      PDU_Count;
  uint32_t      PDU_time;
  
  uint8_t ble_adv_pdu_TLM[]=
{
0x02,0x01,0x06,
0x03,// Length
0x03,// Param: Service List
0xAA, 0xFE,  // Eddystone ID
0X17,//0x11,  // Length
0x16,  // Service Data
0xAA, 0xFE, // Eddystone ID
0x20,  // TLM flag
0x00, // TLM version
  /* [13] */ 0x00, 0x00,  // Battery voltage
  /* [15] */ 0x80, 0x00,  // Beacon temperature
  /* [17] */ 0x00, 0x00, 0x00, 0x00, // Advertising PDU count
  /* [21] */ 0x00, 0x00, 0x00, 0x00 ,// Time since reboot
     /* [25] */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // MAC ADDED
};

#if 0
uint8_t ble_adv_pdu_IBEACON[] =
{
0x02,0x01,0x06,0x1A,0xFF,//0-4
0x4C,0x00,0x02,0x15,//VENDOR 5-8
0xFD,0xA5,0x06,0x93,//UUID 9-12
0xA4,0xE2,//temp13-14
0x4F,0xB1,0xAF,0xCF,0xC6,0xEB,0x07,0x64,0x78,0x25,//15-24
0x00,0x00,//major//25-26
0x00,0x00,//minor27-28
0xD8
}; 
#endif
#if 0
//CBBD5EA473E0
uint8_t ble_adv_pdu_IBEACON[] =
{
/*0x02,0x01,0x06,*/0x1E,0xFF,//0-4
0x4C,0x00,0x12,0x19,//VENDOR 5-8
0x24,0x7C,0xCF,0x44,
0x3B,0x08,
0xAC,0x38,0xC1,0x06,0x8D,0xFE,0x28,0x18,0xD9,0xCE,
0xA1,0x02,
0x46,0x17,
0x8E,0x2C,0xB5,0x03,0x00
};

#endif
//F1F8DB12C46D
uint8_t ble_adv_pdu_IBEACON[] =
{
/*0x02,0x01,0x06,*/0x1E,0xFF,//0-4
0x4C,0x00,0x12,0x19,//VENDOR 5-8
0x24,0x14,0x70,0x24,
0xBE,0xDB,
0x11,0xC3,0xDB,0x4E,0xD6,0x4F,0x4D,0xCE,0xC7,0x1A,
0x33,0x9C,
0x23,0x5B,
0xD7,0x10,0x42,0x01,0x00
}; //24147024BEDB11C3DB4ED64F4DCEC71A339C235BD710420100

#else
uint8_t ble_adv_pdu_HYUMI[] =
{
0x02,0x01,0x06,0x1A,0xFF,//0-4
0x59,0x00,0x02,0x15,//VENDOR 5-8
//0x4C,0x00,0x02,0x15,//VENDOR 5-8
0xFD,0xA5,0x06,0x93,//UUID 9-12
0x00,0x00,//major  13-14
0x00,0x00,//minor  15-16
0xd8,//1M TX POWER    17
0xE2,//BATTERY%       18
0x00,0x00,0x00,0x00,0x00,0x00,//X,Y,Z 19-24
0x00,0x00,//TEMPERATURE 25-26
0x00,0x00,//HYUMIDTY//27-28
0xb0    // battery 29

};
#endif
    uint8_t MAC[6];
    uint32_t ADC_VAL;
    uint16_t ADC_VAL16;
    uint8_t SENSOR;
   // int16_t val;
      uint8_t val[2];
      uint8_t MN[2];
      uint8_t MJ[2];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
  int main(void)
{
  /* USER CODE BEGIN 1 */
  uint16_t stat;
    GPIO_InitTypeDef GPIO_InitStruct;
#ifndef IBtemp
    PDU_Count = 0;//0x1f40;//0;
    PDU_time = 0;//0xfa00;//0;
#endif
    SENSOR = 0;
    ERROR_F = 0;
    stat = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_ADC_Init();
  MX_TIM14_Init();
  MX_TIM3_Init();
  SENSOR = 0;
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc);
  HAL_TIM_Base_Start(&htim3);
  //GetMac();
  #ifndef SHIPREK
  memcpy(MAC, (void*)SM32F0xx_EL_SIGN_ADDRESS,BD_ADDR_SIZE);
#else
#if 0
  MAC[5] = 0xCB;
  MAC[4] = 0xBD;
  MAC[3] = 0x5E;
  MAC[2] = 0xA4;
  MAC[1] = 0x73;
  MAC[0] = 0xE0;
#endif
    MAC[5] = 0xF1;
  MAC[4] = 0xF8;
  MAC[3] = 0xDB;
  MAC[2] = 0x12;
  MAC[1] = 0xC4;
  MAC[0] = 0x6D;
  
#endif
  UartTest();
  GetMM();
#ifndef SHIPREK
#ifndef IBtemp
  ble_adv_pdu_IBEACON[25] = MJ[1];
  ble_adv_pdu_IBEACON[26] = MJ[0];
  ble_adv_pdu_IBEACON[27] = MN[1];
  ble_adv_pdu_IBEACON[28] = MN[0];
#else
  ble_adv_pdu_HYUMI[13] = MJ[1];
  ble_adv_pdu_HYUMI[14] = MJ[0];
  ble_adv_pdu_HYUMI[15] = MN[1];
  ble_adv_pdu_HYUMI[16] = MN[0];
#endif
#endif SHIPREK
  HAL_UART_MspDeInit(&huart1);
  MX_I2C1_Init();
  
  //test
#ifdef IBtemp

   SHT3X_ClearAllAlertFlags();
   SHT3X_ReadStatus(&stat);
   SHT3X_GetTempAndHumiPollingDirect(&Temp,&Humi,
                                    REPEATAB_LOW,
                                    10);
#endif  
   
  //test end
//#if 0
   

   
  #ifndef IBtemp
  if(Max317267_WriteRegister(ConfigurationReg,2,0x81) == HAL_OK)
  {
    SENSOR = 1;//sensor is MAX report external only
      if(Max317267_ReadRegister(TempReg,2,val) != HAL_OK)
      {
        ERROR_F = 1;
        //error reading
      }
  }  
  else if(STDS75_WriteRegister(ConfigurationReg,2,0x40) == HAL_OK)
  {
    SENSOR = 2;//senor is STDS75 internal 
      if(STDS75_ReadRegister(TempReg,2,val)== HAL_TIMEOUT)
      {
        ERROR_F = 1;
        //error reading
      }     
      if(STDS75_WriteRegister(ConfigurationReg,2,0x41) != HAL_OK)
      {
        ERROR_F = 1;
         //error reading       
      }

  }
  
  else
  {
    HAL_I2C_MspDeInit(&hi2c1);
  }
  #endif 
#if 0
    if(SENSOR == 1) 
    {
      if(Max317267_ReadRegister(TempReg,2,val) != HAL_OK)
      {
        ERROR_F = 1;
        //error reading
      }
    }
    else if(SENSOR == 2)
    {
      if(STDS75_ReadRegister(TempReg,2,val)== HAL_TIMEOUT)
      {
        ERROR_F = 1;
        //error reading
      }     
      if(STDS75_WriteRegister(ConfigurationReg,2,0x41) != HAL_OK)
      {
        ERROR_F = 1;
         //error reading       
      }
    
  }
#endif
#if 0
  if(SENSOR==0)
  {
      HAL_I2C_MspDeInit(&hi2c1);
  }
#endif
 // #endif
  
  if(ERROR_F == 1)
    while(1){}
  RTC_Init();
  BC7161_inital();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
#ifndef IBtemp
#if defined(IBEACON)
    if(SENSOR!=0)
#endif

    {
#ifndef SHIPREK
        PduTimeUpdate(&PDU_Count,&PDU_time);
#endif  //SHIPREK
    }
#endif
        BeaconSend();
        LED_GPIO_Port->BSRR = LED_Pin;
        GPIO_SLEEP();

        HAL_SuspendTick();
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
        /* Clear PWR wake up Flag */    
        HAL_ResumeTick();
        __HAL_RCC_GPIOF_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_TIM14_CLK_ENABLE();
        __HAL_RCC_ADC1_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1) == GPIO_PIN_SET)
    {
       BC7161_inital();
    }
   

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*---------------------------------------------------------------------------*/
/*	 */
/*---------------------------------------------------------------------------*/
void BC7161_inital()
{
  uint8_t i;
 	BC7161_InterfaceConfigure();
	//DelayXmSec(50*4);		//50ms	
        //HAL_Delay(200);
 
        delay_us_Sleep(25000);	
      
	BC7161_wakeup();
        BC7161_page_read_register(CHIPID_REG,&AdvPayload.adv_data[0],3);	
        BC7161_register_setup();
	BC7161_LIRC_calibration();
	AdvPayload.header.value = ADV_NONCONN_IND;					/* adv type = ADV_NONCONN_IND */
for(i=0;i<6;i++)//0-5
{
        AdvPayload.adv_addr[i] = MAC[i];
}
#ifndef IBtemp
#if defined(IBEACON)
        if(SENSOR == 0)
        {
          pdu_length = sizeof(ble_adv_pdu_IBEACON)+ADV_ADDR_SIZE;       
         AdvPayload.header.bits.length = pdu_length;
          memcpy(AdvPayload.adv_data,ble_adv_pdu_IBEACON,sizeof(ble_adv_pdu_IBEACON));
        }
        else 
        {
#endif
          pdu_length = sizeof(ble_adv_pdu_TLM)+ADV_ADDR_SIZE;
          AdvPayload.header.bits.length = pdu_length;
          memcpy(AdvPayload.adv_data,ble_adv_pdu_TLM,sizeof(ble_adv_pdu_TLM));
          /*ADDED FOR ios TO READ mac*/
 for(i=0;i<6;i++)//0-5
 {
   AdvPayload.adv_data[30-i] = MAC[i];
 }
#if 0
        AdvPayload.adv_data[25] = MAC[5];
        AdvPayload.adv_data[26] = MAC[4];
        AdvPayload.adv_data[27] = MAC[3];
        AdvPayload.adv_data[28] = MAC[2];
        AdvPayload.adv_data[29] = MAC[1];
        AdvPayload.adv_data[30] = MAC[0];
#endif
#if defined(IBEACON)
        }	
#endif
#else
         pdu_length = sizeof(ble_adv_pdu_HYUMI)+ADV_ADDR_SIZE;       
         AdvPayload.header.bits.length = pdu_length;
         memcpy(AdvPayload.adv_data,ble_adv_pdu_HYUMI,sizeof(ble_adv_pdu_HYUMI)); 
        
#endif
}


void BeaconSend(void)
{
  uint8_t ret = 0;
  uint16_t Timout = 200;
 // GPIO_InitTypeDef GPIO_InitStruct;
  /* wake up BC7161 */
  //BC7161_wakeup();
  BC7161_wakeupAndSamp();
  /* write adv data to BC7161 FIFO */
  LED_GPIO_Port->BRR =LED_Pin;//LedOn();
  if(BC7161_write_pdu_fifo((uint8_t *)&AdvPayload,pdu_length+ADV_HEADER_SIZE) == 0x03)
  {
    /* trigger TX start,0x81(CH37),0x83(CH37,CH38),0x87(CH37,CH38,CH39) */
    ret = BC7161_TriggerAdvStart(ADV_CHANNEL);
    if(ret == 0)
    {

      while(1)
      {
        LED_GPIO_Port->BRR =LED_Pin;//LedOn();
        LED_GPIO_Port->BSRR = LED_Pin;
      }
    }
    while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1) == GPIO_PIN_RESET)
    {
      if(Timout-- == 0)
      {
       BC7161_inital();
       break;
      }
    }

  }
            
  timeout_cnt=TIMEOUT_COUNTER;	//Tony	 20190919
  
}



void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
#if 0
 //  
     if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
  //
#endif
RTC->WPR = 0xCA; /* (1) */
RTC->WPR = 0x53; /* (1) */
RTC->ISR |= RTC_ISR_INIT; /* (2) */
while ((RTC->ISR & RTC_ISR_INITF) != RTC_ISR_INITF) /* (3) */
{
/* add time out here for a robust application */
}
RTC->TR = 0; /* (5) */
RTC->ISR &=~ RTC_ISR_INIT; /* (6) */
RTC->WPR = 0xFE; /* (7) */
RTC->WPR = 0x64; /* (7) */


  //HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_1);
   
}

void   GPIO_SLEEP(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin =  GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;//|GPIO_PIN_9|GPIO_PIN_10;//|GPIO_PIN_13|GPIO_PIN_14;//|GPIO_PIN_9|GPIO_PIN_10;//|GPIO_PIN_13|GPIO_PIN_14;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
/* 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin =  GPIO_PIN_0|GPIO_PIN_1;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1|GPIO_PIN_0,GPIO_PIN_RESET);
*/
 //   HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);


  /* Disable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOF_CLK_DISABLE();
  __HAL_RCC_I2C1_CLK_DISABLE();
  __HAL_RCC_ADC1_CLK_DISABLE();
  __HAL_RCC_TIM14_CLK_DISABLE();
}
#if 0

uint8_t GPIO_Test(void)
{
      GPIO_InitTypeDef GPIO_InitStruct;
      GPIO_InitStruct.Pin = GPIO_PIN_3;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3))return 1;
      return 0;      
}

#endif
#if 0
void GetMac(void)
{
      memcpy(MAC, (void*)SM32F0xx_EL_SIGN_ADDRESS,BD_ADDR_SIZE);

}
#endif
void BC7161_wakeupAndSamp(void)
{       
        uint16_t    hyumi = 0;
        HAL_ADC_Start(&hadc);
        uint32_t h;
        uint16_t                pres;
#ifndef IBtemp
        if(SENSOR==1)
        {
          if(Max317267_ReadRegister(TempReg,2,val) != HAL_OK)
          {
          }

        }
        if(SENSOR==2)
        {
           if(STDS75_ReadRegister(TempReg,2,val) != HAL_OK)
            {
                      ERROR_F = 1;
            }
           if(STDS75_WriteRegister(ConfigurationReg,2,0x40) != HAL_OK) //turn on sensor
            {
                      ERROR_F = 1;
            }
        }
#else
        //hyumi sensor 
        SHT3X_GetTempAndHumiPollingDirect(&Temp,&Humi,
                                    REPEATAB_LOW,
                                    10);
        
#endif
        h = (uint32_t)Humi[1] + (uint32_t)(Humi[0] << 8); 
        h = h * 100;
      //  hyumi  = Humi[0];
       // hyumi +=  (Humi[1] << 8); 
       // hyumi = hyumi / 655;
        h = h / 65535;
        hyumi = (uint16_t)h ;
       
      
        //hyumi = hyumi / 65535;
	SCL_RESET();				/* SCL = low */
	//delay_us(800);			/* delay 2ms */	
        delay_us_Sleep(200);

        HAL_ADC_PollForConversion(&hadc,10);
 
        ADC_VAL = HAL_ADC_GetValue(&hadc);
        ADC_VAL16 = ADC_VAL & 0xffff;
        ADC_VAL16 = ADC_VAL16 * 1.46;
        //ADC_VAL16 += 70;
#ifndef IBtemp
#if defined(IBEACON)
        if(SENSOR==0)
        {
                if(ADC_VAL16 > 4100) pres = 100;
                else if(ADC_VAL16 > 4100) pres = 97;
                else if(ADC_VAL16 > 4050) pres = 95;
                else if(ADC_VAL16 > 4000) pres = 92;
                else if(ADC_VAL16 > 3950) pres = 90;
                else if(ADC_VAL16 > 3900) pres = 85;
 //               else if(ADC_VAL16 > 3800) pres = 82;
                else if(ADC_VAL16 > 3750) pres = 80;
//                else if(ADC_VAL16 > 3700) pres = 75;
                else if(ADC_VAL16 > 3600) pres = 70;
 //               else if(ADC_VAL16 > 3500) pres = 65;
                else if(ADC_VAL16 > 3300) pres = 60;
                else if(ADC_VAL16 > 3000) pres = 50;
                else if(ADC_VAL16 > 2800) pres = 30;
#ifndef SHIPREK
                AdvPayload.adv_data[29]=0;
#endif  //SHIPREK
        }
        else
        {  
#endif
          
          AdvPayload.adv_data[14]=ADC_VAL16 & 0xff;//battery
          AdvPayload.adv_data[13]=ADC_VAL16 >> 8;  
          AdvPayload.adv_data[16]=val[1];         //temperature
          AdvPayload.adv_data[15]=val[0];
              //    tests
          if(SENSOR==0)
          {     
            AdvPayload.adv_data[15]=0xd3;//val[0]; fixed
            AdvPayload.adv_data[16]=0x00;//val[0]; fixed
          }
#if defined(IBEACON)
        }
#endif
        if(SENSOR==2)
        {
          if(STDS75_WriteRegister(ConfigurationReg,2,0x41) != HAL_OK) //turn off sensor shdn when finish converting 
          {
          }
        }
        if(SENSOR==1)
        {
          if(Max317267_WriteRegister(ConfigurationReg,2,0x81) != HAL_OK)
          {
          }
          
        }
#else
        if(ADC_VAL16 > 4100) pres = 100;
        else if(ADC_VAL16 > 4100) pres = 97;
        else if(ADC_VAL16 > 4050) pres = 95;
        else if(ADC_VAL16 > 4000) pres = 92;
        else if(ADC_VAL16 > 3950) pres = 90;
        else if(ADC_VAL16 > 3900) pres = 85;
        else if(ADC_VAL16 > 3800) pres = 82;
        else if(ADC_VAL16 > 3750) pres = 80;
        else if(ADC_VAL16 > 3700) pres = 75;
        else if(ADC_VAL16 > 3600) pres = 70;
        else if(ADC_VAL16 > 3500) pres = 65;
        else if(ADC_VAL16 > 3300) pres = 60;
        else if(ADC_VAL16 > 3000) pres = 50;
        else if(ADC_VAL16 > 2800) pres = 30;
        AdvPayload.adv_data[18]=pres;
        //hyumi sensor
          AdvPayload.adv_data[25]=Temp[0];         //temperature
          AdvPayload.adv_data[26]=Temp[1];
 //         AdvPayload.adv_data[27]=Humi[0];         //humi
 //         AdvPayload.adv_data[28]=Humi[1];
           AdvPayload.adv_data[27]=hyumi & 0xff;        //humi
          AdvPayload.adv_data[28]=hyumi >> 8;         
          AdvPayload.adv_data[29]=0;
#endif
        SCL_SET();

}
#ifndef IBtemp
void PduTimeUpdate(uint32_t *count,uint32_t *time)
{
      uint8_t *vp;
        *count = *count+1;
        *time = *time + 8;
        vp = (uint8_t *)&PDU_Count;
        AdvPayload.adv_data[17] = vp[3];
        AdvPayload.adv_data[18] = vp[2];
        AdvPayload.adv_data[19] = vp[1];
        AdvPayload.adv_data[20] = vp[0];
        vp = (uint8_t *)&PDU_time;
        AdvPayload.adv_data[21] = vp[3];
        AdvPayload.adv_data[22] = vp[2];
        AdvPayload.adv_data[23] = vp[1];
        AdvPayload.adv_data[24] = vp[0];
}
#endif

void UartTest(void)
{
    /*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;
//uint8_t i;
    uint8_t TxBuf[20];
    uint8_t RxBuf[15];
    uint32_t TempInt2=0;
    uint8_t i = 0;
    HAL_StatusTypeDef ret;
    uint8_t MM[8];
    TxBuf[0] = 'M';
    TxBuf[1] = ':';
    for(i=0;i<6;i++)
    {
      hextoc(MAC[i],&TxBuf[12-(i*2)],&TxBuf[13-(i*2)]);
    }
#if 0
  hextoc(MAC[0],&TxBuf[12],&TxBuf[13]);
   hextoc(MAC[1],&TxBuf[10],&TxBuf[11]);
    hextoc(MAC[2],&TxBuf[8],&TxBuf[9]);
     hextoc(MAC[3],&TxBuf[6],&TxBuf[7]);
      hextoc(MAC[4],&TxBuf[4],&TxBuf[5]);
       hextoc(MAC[5],&TxBuf[2],&TxBuf[3]);
#endif
  TxBuf[14] = ',';
  //TxBuf[14] = 0x0d;
  //TxBuf[15] = 0x0a;
  HAL_UART_Transmit(&huart1,TxBuf,15,100);
  memcpy(MM, (void*)FLASH_USER_START_ADDR,8);
  for(i=0;i<4;i++)
  {
    hextoc(MM[i],&TxBuf[6-(i*2)],&TxBuf[7-(i*2)]);
  }        
#if 0
      hextoc(MM[0],&TxBuf[6],&TxBuf[7]); 
      hextoc(MM[1],&TxBuf[4],&TxBuf[5]);
      hextoc(MM[2],&TxBuf[2],&TxBuf[3]);
      hextoc(MM[3],&TxBuf[0],&TxBuf[1]);
#endif
   
      TxBuf[8] = ',';
      TxBuf[9] = 'V';
      TxBuf[10] = '3';
      TxBuf[11] = 0x0d;
      TxBuf[12] = 0x0a;
      HAL_UART_Transmit(&huart1,TxBuf,13,100);
//  while(ret != HAL_OK)
 // {
    ret = HAL_UART_Receive(&huart1,RxBuf,12,3000);
  //}
  if(RxBuf[0] == 'I' && RxBuf[1] == 'D' && RxBuf[11] == 0x0d)
  {
      TempInt2 = _atoi(&RxBuf[3]);     
      HAL_FLASH_Unlock();
      /* Fill EraseInit structure*/
      EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;    //1k            
      EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
      EraseInitStruct.NbPages = 1;//(FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
      
        if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
        {
          while(1){}
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_USER_START_ADDR, TempInt2) != HAL_OK)
        {
          while(1){}
        }
        HAL_FLASH_Lock();
      
      
      
      TxBuf[0] = 'O'; 
      TxBuf[1] = 'K';
      TxBuf[2] = '=';
      memcpy(MM, (void*)FLASH_USER_START_ADDR,8);
      for(i=0;i<4;i++)
      {
        hextoc(MAC[i],&TxBuf[9-(i*2)],&TxBuf[10-(i*2)]);
      } 
#if 0
      hextoc(MM[0],&TxBuf[9],&TxBuf[10]);
      hextoc(MM[1],&TxBuf[7],&TxBuf[8]);
      hextoc(MM[2],&TxBuf[5],&TxBuf[6]);
      hextoc(MM[3],&TxBuf[3],&TxBuf[4]);
#endif
      TxBuf[11] = 0x0d;
      TxBuf[12] = 0x0a;
      HAL_UART_Transmit(&huart1,TxBuf,13,100);
  }
  else 
  {
      TxBuf[0] = 'T'; 
      TxBuf[1] = 'I';
      TxBuf[2] = 'M';
      TxBuf[3] = 'E';      
      TxBuf[4] = 0x0d;
      TxBuf[5] = 0x0a;
      HAL_UART_Transmit(&huart1,TxBuf,6,100);
  }
    
  }
  
  
  /**
 * _atoi - Converts a string to an integer.
 * @s: The string to be converted.
 *
 * Return: The integer value of the converted string.
 */
uint32_t _atoi(uint8_t *s)
{
    
    uint32_t num = 0;
/*
    do {

        if (*s >= '0' && *s <= '9')
            num = (num * 10) + (*s - '0');

        else if (num > 0)
            break;
    } while (*s++);
    */
        do {

        if (*s >= '0' && *s <= '9')
            num = (num * 16) + (*s - '0');
        else if (*s >= 'A' && *s <= 'F')
            num = (num * 16) + (*s - 'A' + 10);        
        else if (num > 0)
            break;
    } while (*s++);

    return (num);
}

void GetMM(void)
{
      uint32_t data32=0 ;
        Address = FLASH_USER_START_ADDR;
        data32 = *((__IO uint32_t *)Address);
        MN[0] = (uint8_t)((data32 ) & 0xff);
        MN[1] = (uint8_t)((data32 >> 8) & 0xff);
        MJ[0] = (uint8_t)(( data32 >> 16) & 0xff);
        MJ[1] = (uint8_t)((data32 >>  24) & 0xff);
        if(MN[0] == 0xff)
        {
          MN[0] = 0;
          MN[1] = 0;
          MJ[0] = 0;
          MJ[1] = 0;    
        }
}

#if 0
void LedOn(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = LED_Pin;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

    //HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);

    LED_GPIO_Port->BRR =LED_Pin;
}
#endif
#if 0
void LedOff(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = LED_Pin;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

    //HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
    LED_GPIO_Port->BSRR = LED_Pin;
}
#endif
void hextoc(uint8_t hex,uint8_t* msb,uint8_t* lsb)
{
  uint8_t u;
  uint8_t l;
  u = (hex & 0xf0)>>4;
  l = (hex & 0xf);
  *msb = itoa(u);
   *lsb = itoa(l);
 

}

uint8_t itoa(uint8_t i)
{
    if (i <= 9)
    return i + '0';
  if (i == 0xa)
    return 'A';
  if (i == 0xb)
    return 'B';
  if (i == 0x0c)
    return 'C';
  if (i == 0x0d)
    return 'D';
  if (i == 0x0e)
    return 'E';
  if (i == 0x0f)
    return 'F';
  return 0;
}
#if 0
void hex_to_string(uint8_t* msg, size_t msg_sz, uint8_t* hex, size_t hex_sz)
{
   memset(msg, '\0', msg_sz);
   if (hex_sz % 2 != 0 || hex_sz/2 >= msg_sz)
      return;

   for (int i = 0; i < hex_sz; i+=2)
   {
      uint8_t msb = (hex[i+0] <= '9' ? hex[i+0] - '0' : (hex[i+0] & 0x5F) - 'A' + 10);
      uint8_t lsb = (hex[i+1] <= '9' ? hex[i+1] - '0' : (hex[i+1] & 0x5F) - 'A' + 10);
      msg[i / 2] = (msb << 4) | lsb;
   }
}
#endif



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
