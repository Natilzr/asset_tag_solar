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

#define SAMPLING_TIME           10*60
#define TICK_TIME       10
#define ADC_TIME        SAMPLING_TIME/TICK_TIME

#define         SM32F0xx_EL_SIGN_ADDRESS 0x1FFFF7AC // RM0091 p.933
#define BD_ADDR_SIZE  (6)

#define FLOAT_TO_INT(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

uint8_t g_BDdaddr[BD_ADDR_SIZE];
uint8_t TagID[4];
uint32_t Address = 0, PageError = 0;

//#define IBtemp 1
#define INPUT_SIZE 76
#define OUTPUT_SIZE 37
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
void GPIO_SLEEP(void);
void BC7161_wakeupAndSamp(void);
void GetMac(void);
void PduTimeUpdate(uint32_t* count, uint32_t* time);
void UartTest(void);
uint32_t _atoi(uint8_t* s);
void GetMM(void);
void LedOn(void);
void LedOff(void);
uint8_t itoa(uint8_t i);
void hextoc(uint8_t hex, uint8_t* msb, uint8_t* lsb);
void hex_string_to_bytes(const char input[INPUT_SIZE], uint8_t output[OUTPUT_SIZE]);
//uint16_t Temp;
//uint16_t Humi;
//float Temp;
//float Humi;
uint8_t Temp[2];
uint8_t Humi[2];

//#define ROLING_TIME     450//15min
//if sending every 6 sec  10  sending is 1 min 
#define MEASURED_INTERVAL       6//sec
#define PING_PER_MIN    60/MEASURED_INTERVAL
#define ROLING_HOURS    7//hours
#define ROLING_TIME    ROLING_HOURS*60*PING_PER_MIN   //4200
//#define ROLING_TIME     12600//7hours  for 2 sec
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t UUID_num;
uint32_t elapsed_time;
uint8_t timeout_cnt;
static adv_pdu_struct AdvPayload;
uint8_t pdu_length;

uint8_t BatLvl;



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


uint8_t MAC[6];
uint32_t ADC_VAL;
uint16_t ADC_VAL16;
uint16_t samp_time;
// int16_t val;
uint8_t val[2];
uint8_t MN[2];
uint8_t MJ[2];
uint8_t M;
/* USER CODE END 0 */
//
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    uint16_t stat;
    GPIO_InitTypeDef GPIO_InitStruct;

    stat = 0;
    elapsed_time = 0;
    UUID_num = 0;
    uint8_t* e;
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
    /* USER CODE BEGIN 2 */
    HAL_ADCEx_Calibration_Start(&hadc);
    HAL_TIM_Base_Start(&htim3);
    UartTest();// config service
    HAL_UART_MspDeInit(&huart1);
    MX_I2C1_Init();
    samp_time = 2;//sample battery after 2 advertizing
    RTC_Init();
    BC7161_inital();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        if (elapsed_time++ > ROLING_TIME)
        {
            elapsed_time = 0;
            BC7161_inital();
            if (UUID_num++ > 22)
            {
                UUID_num = 0;
            }
        }


        BeaconSend();
#ifdef LED
        LED_GPIO_Port->BSRR = LED_Pin;
#endif
        
#ifdef  PA
        //delay_us_Sleep(20000);
        delay_us_Sleep(4000);
        GPIOA->BRR = GPIO_PIN_0;
        delay_us_Sleep(9100);
        GPIOA->BSRR = GPIO_PIN_0;
        delay_us_Sleep(4500);
        GPIOA->BRR = GPIO_PIN_0;
#endif
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

        while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET)
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
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

    /** Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14
                                | RCC_OSCILLATORTYPE_LSI;
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1
                                | RCC_PERIPHCLK_RTC;
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
    for (int8_t a = 0; a < 7; a++)
    {
        M = *((uint8_t*)((UUID_num * 40) + FLASH_USER_START_ADDR + 5 - a));
        MAC[a] = M;
    }
    BC7161_InterfaceConfigure();
    //DelayXmSec(50*4);		//50ms	
    //HAL_Delay(200);

    delay_us_Sleep(25000);
    BC7161_wakeup();
    BC7161_page_read_register(CHIPID_REG, &AdvPayload.adv_data[0], 3);
    BC7161_register_setup();
    BC7161_LIRC_calibration();
    AdvPayload.header.value = ADV_NONCONN_IND;                  /* adv type = ADV_NONCONN_IND */
    for (i = 0; i < 6; i++)//0-5
    {
        AdvPayload.adv_addr[i] = MAC[i];
    }

        
    pdu_length = sizeof(ble_adv_pdu_IBEACON)+ADV_ADDR_SIZE;       
    AdvPayload.header.bits.length = pdu_length;
          //memcpy(AdvPayload.adv_data,ble_adv_pdu_IBEACON,sizeof(ble_adv_pdu_IBEACON));
    memcpy(AdvPayload.adv_data,(uint8_t*)(FLASH_USER_START_ADDR+6+(UUID_num*40)),31);
        

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
#ifdef LED    
    LED_GPIO_Port->BRR = LED_Pin;//LedOn();
#endif
    if (BC7161_write_pdu_fifo((uint8_t*)&AdvPayload, pdu_length + ADV_HEADER_SIZE) == 0x03)
    {
        /* trigger TX start,0x81(CH37),0x83(CH37,CH38),0x87(CH37,CH38,CH39) */
        ret = BC7161_TriggerAdvStart(ADV_CHANNEL);
        if (ret == 0)
        {

            while (1)
            {
  
                LED_GPIO_Port->BRR = LED_Pin;//LedOn();
                LED_GPIO_Port->BSRR = LED_Pin;

            }
        }
        while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET)
        {
            if (Timout-- == 0)
            {
                BC7161_inital();
                break;
            }
        }
#ifdef PA
    GPIOA->BSRR = GPIO_PIN_0;
#endif
    }
    else
    {
      while(1)
      {}
    }

    timeout_cnt = TIMEOUT_COUNTER;  //Tony	 20190919

}



void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef* hrtc)
{

    RTC->WPR = 0xCA; /* (1) */
    RTC->WPR = 0x53; /* (1) */
    RTC->ISR |= RTC_ISR_INIT; /* (2) */
    while ((RTC->ISR & RTC_ISR_INITF) != RTC_ISR_INITF) /* (3) */
    {
        /* add time out here for a robust application */
    }
    RTC->TR = 0; /* (5) */
    RTC->ISR &= ~RTC_ISR_INIT; /* (6) */
    RTC->WPR = 0xFE; /* (7) */
    RTC->WPR = 0x64; /* (7) */


    //HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_1);

}

void GPIO_SLEEP(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;//|GPIO_PIN_9|GPIO_PIN_10;//|GPIO_PIN_13|GPIO_PIN_14;//|GPIO_PIN_9|GPIO_PIN_10;//|GPIO_PIN_13|GPIO_PIN_14;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    //   HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);


    /* Disable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOF_CLK_DISABLE();
    __HAL_RCC_I2C1_CLK_DISABLE();
    __HAL_RCC_ADC1_CLK_DISABLE();
    __HAL_RCC_TIM14_CLK_DISABLE();
}

void BC7161_wakeupAndSamp(void)
{
#define ADC_FULL_SCALE     4095U
#define VREFINT_MV         1200U   // Typical internal reference = 1.200 V

    uint16_t i;
    //hyumi = hyumi / 65535;

                                //delay_us(800);			/* delay 2ms */	

    if(samp_time-- == 0)
    {
#ifdef COIN
      // Enable VREFINT
      ADC->CCR |= ADC_CCR_VREFEN;
      delay_us_Sleep(800);
      HAL_ADC_Start(&hadc);
      SCL_RESET();                /* SCL = low */
      delay_us_Sleep(800);
      HAL_ADC_PollForConversion(&hadc, 10);
      ADC->CCR &= ~ADC_CCR_VREFEN;
      
#else
      HAL_ADC_Start(&hadc);
      SCL_RESET();                /* SCL = low */
      delay_us_Sleep(800);
      HAL_ADC_PollForConversion(&hadc, 10);
#endif
      ADC_VAL = HAL_ADC_GetValue(&hadc);
      
#ifdef COIN
      ADC_VAL = ADC_VAL - 50;
      ADC_VAL16 = (uint16_t)((VREFINT_MV * ADC_FULL_SCALE) / ADC_VAL );
#else
      
        ADC_VAL16 = ADC_VAL & 0xffff;
        ADC_VAL16 = ADC_VAL16 * 1.46;
#endif
#ifdef  COIN
        if(ADC_VAL16 > 3000) BatLvl = 0x00;//high
        else if(ADC_VAL16 > 2900) BatLvl = 0x40;
        else if(ADC_VAL16 > 2700) BatLvl = 0x80;
        else BatLvl = 0xc0;
#else
          ///battery handling
        if(ADC_VAL16 > 3800) BatLvl = 0x00;//high
        else if(ADC_VAL16 > 3700) BatLvl = 0x40;
        else if(ADC_VAL16 > 3500) BatLvl = 0x80;
        else BatLvl = 0xc0;
#endif
        AdvPayload.adv_data[6] &= 0x3F;
        AdvPayload.adv_data[6] |= BatLvl;
        samp_time = ADC_TIME;
    }
    else
    {
        SCL_RESET();                /* SCL = low */
delay_us_Sleep(800);
   
    }
        SCL_SET();
        i = 80;
        while (i--)
        {
        }
}


void UartTest(void)
{
    /*Variable used for Erase procedure*/
    static FLASH_EraseInitTypeDef EraseInitStruct;
    //uint8_t i;
    uint8_t TxBuf[20];
    uint8_t RxBuf[78];
    uint8_t OUT[37];
    uint32_t TempInt2 = 0;
    uint8_t* Ptx;
    uint8_t i = 0;
    uint8_t eraised = 0;
    HAL_StatusTypeDef ret;
    uint8_t MM[8];
    TxBuf[0] = 'M';
    TxBuf[1] = ':';
    Ptx = (void*)(SM32F0xx_EL_SIGN_ADDRESS + 5);
    //Ptx = (uint8_t*)FLASH_USER_START_ADDR;
    HAL_UART_Transmit(&huart1, TxBuf, 2, 100);
    HAL_Delay(2);
    for (i = 0; i < 12; i += 2)
    {
        hextoc(*Ptx, &TxBuf[0], &TxBuf[1]);
        Ptx--;
        HAL_UART_Transmit(&huart1, TxBuf, 2, 100);
        HAL_Delay(2);
    }
    Ptx = (uint8_t*)FLASH_USER_START_ADDR;
    TxBuf[0] = ',';
    TxBuf[1] = 'U';
    TxBuf[2] = ':';
    HAL_UART_Transmit(&huart1, TxBuf, 3, 100);
    HAL_Delay(2);
    for (i = 0; i < 74; i += 2)
    {
        hextoc(*Ptx, &TxBuf[0], &TxBuf[1]);
        Ptx++;
        HAL_UART_Transmit(&huart1, TxBuf, 2, 100);
        HAL_Delay(2);
    }
    TxBuf[0] = '\r';
    HAL_UART_Transmit(&huart1, TxBuf, 1, 100);
    HAL_Delay(1);
    //  while(ret != HAL_OK)
    // {
    for (int8_t R = 0; R < 22; R++)//24 numbers in memmory
    {
        ret = HAL_UART_Receive(&huart1, RxBuf, 77, 4000);
        //}
        if (RxBuf[0] == 'I' && RxBuf[1] == 'D' && RxBuf[76] == 0x0d)
        {
            uint8_t output[OUTPUT_SIZE] = { 0 };
            hex_string_to_bytes(RxBuf + 2, OUT);
            //TempInt2 = _atoi(&RxBuf[3]);     
            HAL_FLASH_Unlock();
            /* Fill EraseInit structure*/
            if (eraised == 0)
            {
                EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;    //1k            
                EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
                EraseInitStruct.NbPages = 1;//(FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
                if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
                {
                    while (1) { }
                }
                eraised = 1;
            }
            for (uint32_t i = 0; i < 40; i += 8)
            {
                uint64_t temp = 0;
                // Pack 8 bytes into a 64-bit value (or fill remaining with 0xFF)
                for (uint8_t j = 0; j < 8; j++)
                {
                    if (i + j < 40)
                    {
                        temp |= ((uint64_t)OUT[i + j] << (8 * j));
                    }
                    else
                    {
                        temp |= ((uint64_t)0xFF << (8 * j));  // Fill remaining bytes with 0xFF
                    }
                }
                // Program Flash memory with the 64-bit data
                if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_USER_START_ADDR + i + (R * 40), temp) != HAL_OK)
                {
                    // Handle error
                    break;
                }

            }
            HAL_FLASH_Lock();


            memset(RxBuf, 0, 77);
            TxBuf[0] = 'O';
            TxBuf[1] = 'K';
            TxBuf[2] = '\r';
            HAL_UART_Transmit(&huart1, TxBuf, 3, 100);
            HAL_Delay(2);

        }
        else
        {
            TxBuf[0] = 'T';
            TxBuf[1] = 'I';
            TxBuf[2] = 'M';
            TxBuf[3] = 'E';
            TxBuf[4] = 0x0d;
            TxBuf[5] = 0x0a;
            HAL_UART_Transmit(&huart1, TxBuf, 6, 100);
            return;
        }

    }//end of loop
    TxBuf[0] = 'D';
    TxBuf[1] = 'O';
    TxBuf[2] = 'N';
    TxBuf[3] = 'E';
    TxBuf[4] = 0x0d;
    TxBuf[5] = 0x0a;
    HAL_UART_Transmit(&huart1, TxBuf, 6, 100);
}

/**
* _atoi - Converts a string to an integer.
* @s: The string to be converted.
*
* Return: The integer value of the converted string.
*/
uint32_t _atoi(uint8_t* s)
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
    do
    {

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
    uint32_t data32 = 0;
    Address = FLASH_USER_START_ADDR;
    data32 = *((__IO uint32_t *)Address);
    MN[0] = (uint8_t)((data32) & 0xff);
    MN[1] = (uint8_t)((data32 >> 8) & 0xff);
    MJ[0] = (uint8_t)((data32 >> 16) & 0xff);
    MJ[1] = (uint8_t)((data32 >> 24) & 0xff);
    if (MN[0] == 0xff)
    {
        MN[0] = 0;
        MN[1] = 0;
        MJ[0] = 0;
        MJ[1] = 0;
    }
}


void hextoc(uint8_t hex, uint8_t* msb, uint8_t* lsb)
{
    uint8_t u;
    uint8_t l;
    u = (hex & 0xf0) >> 4;
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




// Function to convert a single hex character to a numeric value
uint8_t hex_char_to_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';         // Convert '0'-'9' to 0-9
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;    // Convert 'A'-'F' to 10-15
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;    // Convert 'a'-'f' to 10-15
    return 0; // Should never reach here if input is valid
}

// Function to convert 76-character hex string to 37-byte array
void hex_string_to_bytes(const char input[INPUT_SIZE], uint8_t output[OUTPUT_SIZE])
{
    for (int i = 0; i < OUTPUT_SIZE; i++)
    {
        uint8_t high = hex_char_to_value(input[2 * i]);       // Convert first hex digit
        uint8_t low = hex_char_to_value(input[2 * i + 1]);    // Convert second hex digit
        output[i] = (high << 4) | low;  // Combine into one byte
    }
}


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
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
