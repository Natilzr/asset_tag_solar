/*----------------------------------------------------------------------------
 * Name     : main.c
 * Purpose  : BC5602 test 
 * Note(s)  : 
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2012 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include "bc7161.h"
#include "main.h"
#include "typedef.h"
#include "tim.h"
#include "reg_parameter.h"
//#include	"timer.h"


#define Ts5     1
#define  IIC_WRITE_DADDR   		(BC7161_DEVICE_ADDR << 1)
#define  IIC_READ_DADDR    		((BC7161_DEVICE_ADDR << 1) | 0x01)

unsigned char	BC7161SetupTable[][2] = 
{
#if 0
	{0x33,0x01},						/* reset 1.2V register */
	{0x25,0x00},						/* GPIO input mode */
	{0x26,0x04},                  /* GPIO2 pull-high,GPIO3 no pull-high */
	{0x27,0x40},
	{0x07,0x95},
	{0x0C,0xAF},
	{0x0D,0xD7},
	{0x1E,0x3E},						/* 2019/08/09 update */
	{0x2A,0x47},                  /* 2019/08/09 update */
	{0x33,0x10},
#endif   
        

	/* reset 1.2V register */	
	{RESET_CTL_REG,0x01},
	/* setup internal capacitor load for the CRYSTAL */
	{XO_TRIM_REG,CRYSTAL_TRIM_VALUE},
	/* Fixed */
	{0x07,0x95},
	/* setup TX power value */	
	{TXPWR_CTL_REG,TX_POWER_VALUE1},
	{TXPWR_CTL_REG+1,TX_POWER_VALUE2},
	/* setup Adv event resend counter */	
	{PKT_RESEND_REG,ADVE_RESEND_COUNTER},
	/* setup Adv channel delay time */		
	{APRD_CTL_REG,APRD_VALUE},
	/* setup Adv event period time */
	{PKT_PERIOD_REG,ADVE_PERIOD_VLOW},
	{PKT_PERIOD_REG+1,ADVE_PERIOD_VHIGH},
	/* Fixed (2019/08/09 update )*/
	{VCO_ACAL_REG,0x3E},
	/* setup GIO function & pull_high */
	{GIO_CTL_REG,GIO_FUN_VALUE},
	{GIOPU_CTL_REG,GIO_PULL_HIGH},
	/* setup light sleep threshold */
	{SLUM_CTL_REG,SLEEP_HOLD_VALUE},
	/* Fixed (2019/08/09 update)*/
	{0x2A,0x47},
        
};





unsigned char BC7161powertable[][2] = 
{
	{0xAF,0x71},{0xAF,0x77},{0xAF,0xD7},{0xA2,0x67},{0xA1,0xAF}
};

void delay_us (uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim3,0);  // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim3) < us);  // wait for the counter to reach the us input in the parameter
}

void delay_us_Sleep (uint16_t us)
{
  //8mHZ / 16 = 500KHZ 2usec
       //    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    htim14.Instance = TIM14;
    htim14.Init.Prescaler = 16;
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = us;
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_TIM_CLEAR_IT(&htim14, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim14);
    HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_ResumeTick();
    HAL_TIM_Base_DeInit(&htim14);
    //__HAL_TIM_SET_COUNTER(&htim14,0);  // set the counter value a 0
	//while (__HAL_TIM_GET_COUNTER(&htim14) < us);  // wait for the counter to reach the us input in the parameter
  //  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

/*----------------------------------------------------------------------------*/
/*	 IIC serial start bit                                                      */
/*----------------------------------------------------------------------------*/
static void IIC_interface_start(void)
{   
   SCL_SET();
   delay_us(Ts5);
 //  SDA_DIRO();
   SDA_RESET();
   delay_us(Ts5);
   SCL_RESET();
}
/*----------------------------------------------------------------------------*/
/*	 IIC serial stop bit                                                       */
/*----------------------------------------------------------------------------*/
static void IIC_interface_stop(void)
{  
   SCL_RESET();      
   //SDA_DIRO();
   SDA_RESET();
   delay_us(Ts5);
   SCL_SET();
   delay_us(Ts5);   
   SDA_SET();
}
/*----------------------------------------------------------------------------*/
/*	 IIC serial output                                                         */
/*----------------------------------------------------------------------------*/
static uint8_t IIC_interface_ouput(uint8_t sd)
{
   uint8_t i;
   uint8_t ack;
   
  // SDA_DIRO();
   for(i=0;i<8;i++)
   {
      (sd & 0x80) ? SDA_SET():SDA_RESET();
      delay_us(Ts5);      
      SCL_SET();      
      sd <<= 1;
      delay_us(Ts5);      
      SCL_RESET();
      delay_us(1);
   }
   SDA_SET();
 //  SDA_DIRI();
   delay_us(Ts5);         
   SCL_SET();
   delay_us(Ts5);   
   ack = SDA_IN() ? FALSE:TRUE;//condition ? ref consequent : ref alternative
   SCL_RESET();
   delay_us(Ts5);      
  // SDA_DIRO();
   return(ack);
}
/*----------------------------------------------------------------------------*/
/*	 IIC serial input                                                          */
/*----------------------------------------------------------------------------*/
static uint8_t IIC_interface_input(uint8_t ack)
{
   uint8_t i,sd;
   
   sd = 0;
 //  SDA_DIRI();
   SDA_SET();
   for(i=0;i<8;i++)
   {
      delay_us(Ts5);         
      sd <<= 1;
      SCL_SET();
      delay_us(Ts5);         
      sd |= SDA_IN() ? 0x01:0x00;
      SCL_RESET();
   }
 //  SDA_DIRO();
   ack ? SDA_RESET() : SDA_SET();
   delay_us(Ts5);
   SCL_SET();
   delay_us(Ts5);      
   SCL_RESET();
   delay_us(Ts5);      
   return(sd);
}
/*----------------------------------------------------------------------------*/
/* @brief 	: MCU & BC7161 interface Configure 											*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_InterfaceConfigure(void)
{

	IIC_interface_stop();
}
/*----------------------------------------------------------------------------*/
/* @brief 	: BC7161 register define setup												*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_get_chip_id(uint8_t *pb)
{
	while(!BC7161_page_read_register(CHIPID_REG,pb,3));
}
/*----------------------------------------------------------------------------*/
/* @brief 	: BC7161 register define setup												*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_gpio_setup(uint8_t gfn2,uint8_t gfn3,uint8_t gpu)
{
	BC7161_write_register(GIO_CTL_REG,(gfn2 & 0x07) | ((gfn3 & 0x07) << 4));
	BC7161_write_register(GIOPU_CTL_REG,gpu & 0x0F);
}
/*----------------------------------------------------------------------------*/
/* @brief 	: BC7161 register define setup												*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
uint8_t BC7161_get_gpio_state(void)
{
	uint8_t rval;
	rval = 0;
	rval |= GIO3_IN() ? 0x02:0x00;
	return(rval);
}
/*----------------------------------------------------------------------------*/
/* @brief 	: BC7161 register define setup												*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_register_setup(void)
{
	uint8_t	x;
	for(x=0;x<(sizeof(BC7161SetupTable)/2);x++)
	{
		BC7161_write_register(BC7161SetupTable[x][0],BC7161SetupTable[x][1]);
	}
}
/*----------------------------------------------------------------------------*/
/* @brief 	: BC7161 register define setup												*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_set_crystal_cap(uint8_t cap)
{
	BC7161_write_register(XO_TRIM_REG,cap);
}
/*----------------------------------------------------------------------------*/
/* @brief 	: BC7161 register define setup												*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_set_sleep_mode(uint8_t mode)
{
	slum_ctl_t reg;
	BC7161_read_register(SLUM_CTL_REG,&reg.value);
	reg.bit.lstom = mode;
	BC7161_write_register(SLUM_CTL_REG,reg.value);
}
/*----------------------------------------------------------------------------*/
/* @brief 	: BC7161 register define setup												*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_set_tx_power(uint8_t tpw)
{
	drtxp_ctl_t	reg;
	
	BC7161_read_register(DRTXP_CTL_REG,&reg.value);
	reg.bit.tx_power = tpw;
	BC7161_write_register(DRTXP_CTL_REG,reg.value);
	if(tpw > 4) tpw = 4;
	BC7161_write_register(TXPWR_CTL_REG,BC7161powertable[tpw][0]);
	BC7161_write_register(TXPWR_CTL_REG+1,BC7161powertable[tpw][1]);
}
/*----------------------------------------------------------------------------*/
/* @brief 	: BC7161 register define setup												*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_set_resend_counter(uint8_t cnt)
{
	BC7161_write_register(PKT_RESEND_REG,cnt);
}
/*----------------------------------------------------------------------------*/
/* @brief 	: BC7161 register define setup												*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_set_period_delay(uint8_t pdth,uint8_t aprd)
{
	aprd_ctl_t	reg;
	reg.bit.aprd_pdth = pdth;
	reg.bit.pkt_aprd = aprd;
	BC7161_write_register(APRD_CTL_REG,reg.value);
}
/*----------------------------------------------------------------------------*/
/* @brief 	: BC7161 register define setup												*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_set_period_timer(uint16_t tm)
{
	tm /= 10;
	if(tm) tm -= 1;
	BC7161_page_write_register(PKT_PERIOD_REG,(uint8_t *)&tm,2);
}
/*----------------------------------------------------------------------------*/
/* @brief 	: BC7161 register define setup												*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
uint8_t BC7161_write_pdu_fifo(uint8_t *pd,uint8_t len)
{
  uint8_t ret = 0,result = 0;
	if(BC7161_write_register(PDU_PTR_REG,00))/* setup PDU data pointer */
        {
          ret++;
        }
	if(BC7161_write_register(PDU_LEN_REG,len | _FLUSH_FIFO_))	/* write PDU length & flush PDU FIFO */
        {
          ret++;
        }
	if(BC7161_page_write_register(PDU_DATA_REG,pd,len))
        {
          ret++;
        }
        return ret;
}
/*----------------------------------------------------------------------------*/
/* @brief 	: BC7161 register define setup												*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_trigger_tx_start(uint8_t chn)
{
	txsfno_ctl_t reg;
	
//	BC7161_write_register(TXSFNO_CTL_REG,0x00);	
	reg.value = (chn & 0x07);
	reg.bit.tx_start = 1;
	BC7161_write_register(TXSFNO_CTL_REG,reg.value);	
}
/*----------------------------------------------------------------------------*/
/* @brief 	: BC7161 register define setup												*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_stop_transmit(void)
{
	txsfno_ctl_t reg;

	BC7161_read_register(TXSFNO_CTL_REG,&reg.value);
	reg.bit.tx_start = 0;
	BC7161_write_register(TXSFNO_CTL_REG,reg.value);
}
/*----------------------------------------------------------------------------*/
/* @brief 	: BC7161 register define setup												*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_LIRC_calibration(void)
{
#if 0
	lirc_ctl_t reg;
	
	reg.value = 0x80;
	BC7161_write_register(LIRC_CTL_REG,reg.value);
	do
	{
		BC7161_read_register(LIRC_CTL_REG,&reg.value);
	}while(reg.bit.cal);
#endif   
        uint8_t	reg[2];
	
	BC7161_page_read_register(LIRC_CTL_REG,reg,2);
	reg[0] |= 0x80;
	reg[1] |= 0x01;
	BC7161_page_write_register(LIRC_CTL_REG,reg,2);
	do
	{
		BC7161_page_read_register(LIRC_CTL_REG,reg,2);
	}while((reg[0] & 0x80) || (reg[1] & 0x01));
        
        
}
/*----------------------------------------------------------------------------*/
/* @brief 	: BC7161 register define setup												*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_vco_calibration(void)
{	
	uint8_t	reg;
	
	BC7161_read_register(VCO_ACAL_REG,&reg);
	reg |= 0x01;
	BC7161_write_register(VCO_ACAL_REG,reg);
	do
	{
		BC7161_read_register(VCO_ACAL_REG,&reg);		
	}while(reg & 0x01);	
}
/*----------------------------------------------------------------------------*/
/* @brief 	: wake up BC7161																	*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_set_dnk_register(void)
{
	uint32_t	dk;
	uint8_t		dn;
	double	fn;
	
	BC7161_read_register(0x35,&dn);
	fn = ((dn & 0x7F) + 2400.0) * 1000000;
	fn /= (32000000 * 2);
	dn = (uint8_t)fn;
	fn -= (double)dn;
	fn *= (double)1048576.0;
	dk = (uint32_t)fn;
	BC7161_write_register(0x40,0x80);
	BC7161_write_register(0x49,dn);
	BC7161_page_write_register(0x4A,(uint8_t *)&dk,3);
	BC7161_write_register(0x40,0x00);	
}
/*----------------------------------------------------------------------------*/
/* @brief 	: wake up BC7161																	*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_carrier_enable(void)
{	
	BC7161_write_register(0x2B,0x00);	/* FSCALE = 0 */
	BC7161_write_register(0x2C,0x80);	
	BC7161_write_register(0x40,0x80);	/* enable 0x40~0x6F register W/R */		
	BC7161_write_register(0x59,0x08);	/* DR_SEL=1 */	
	BC7161_vco_calibration();	
	BC7161_write_register(0x42,0x81);	/* enable DIR & DTXD=1 */	
	BC7161_write_register(0x44,0x02);	/* SX_EN=1 */
	delay_us(100);
	BC7161_write_register(0x44,0x06);	/* TX_EN=1,SX_EN=1 */
	BC7161_write_register(0x40,0x00);	/* disable 0x40~0x6F register W/R */
}
/*----------------------------------------------------------------------------*/
/* @brief 	: wake up BC7161																	*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_pn9_enable(void)
{
	BC7161_write_register(0x2B,0xFF);	/* FSCALE = 0x7FF */
	BC7161_write_register(0x2C,0x87);
	BC7161_write_register(0x40,0x80);	/* enable 0x40~0x6F register W/R */	
	BC7161_write_register(0x59,0x10);	/* TXPN9_EN=1 */
	BC7161_vco_calibration();	
	BC7161_write_register(0x42,0x81);	/* enable DIR & DTXD=1 */	
	BC7161_write_register(0x44,0x02);	/* SX_EN=1 */
	delay_us(100);
	BC7161_write_register(0x44,0x06);	/* TX_EN=1,SX_EN=1 */
	BC7161_write_register(0x40,0x00);	/* disable 0x40~0x6F register W/R */
}
/*----------------------------------------------------------------------------*/
/* @brief 	: wake up BC7161																	*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_carrier_disable(void)
{
	BC7161_write_register(0x40,0x80);	/* enable 0x40~0x6F register W/R */	
	BC7161_write_register(0x44,0x02);	/* TX_EN=0,SX_EN=1 */
	delay_us(50);
	BC7161_write_register(0x44,0x00);	/* TX_EN=0,SX_EN=0 */
	BC7161_write_register(0x42,0x00);	/* enable DIR & DTXD=1 */
	BC7161_write_register(0x40,0x00);	/* disable 0x40~0x6F register W/R */	
}
/*----------------------------------------------------------------------------*/
/* @brief 	: wake up BC7161																	*
 *	@param	:																						*
 *	@retval  :																						*/
/*----------------------------------------------------------------------------*/
void BC7161_wakeup(void)
{
	SCL_RESET();				/* SCL = low */
	//delay_us(2000);			/* delay 2ms */
        delay_us_Sleep (1000);
	SCL_SET();					/* SCL = high */
}
/*----------------------------------------------------------------------------*/
/* @brief 	: read BC7161 register 														 	*
 *	@param	: adr->register address															*
 *	@retval  : bit[7..0] = register value													*
 *				  bit[8] = ack status															*/
/*----------------------------------------------------------------------------*/
uint8_t BC7161_read_register(uint8_t adr,uint8_t *rval)
{
   uint8_t ack;
	
   IIC_interface_start();           /* IIC start bit */
   ack = IIC_interface_ouput(IIC_WRITE_DADDR);	/* write device address */
   if(ack)
   {
      IIC_interface_ouput(adr);    	/* write register address */
      IIC_interface_stop();			/* IIC stop */
      IIC_interface_start();        /* IIC restart */
      ack = IIC_interface_ouput(IIC_READ_DADDR);	/* write device address */
      if(ack)
      {
         *rval = IIC_interface_input(FALSE);	/*read current register */
      }
   }
   IIC_interface_stop();				/* IIC stop */
   return(ack);
}
/*----------------------------------------------------------------------------*/
/* @brief 	: write data to BC7161 register 												*
 *	@param	: register address & write value												*
 *	@retval  : ack status																		*/
/*----------------------------------------------------------------------------*/
uint8_t BC7161_write_register(uint8_t adr,uint8_t da)
{
   uint8_t ack;
   IIC_interface_start();           /* IIC start bit */
   ack = IIC_interface_ouput(IIC_WRITE_DADDR);
   if(ack)
   {
      IIC_interface_ouput(adr);    /* write register address */
      IIC_interface_ouput(da);
   }
	IIC_interface_stop();
   return(ack);
}
/*----------------------------------------------------------------------------*/
/*	 BC7161 page read regiseter                                                */
/*----------------------------------------------------------------------------*/
uint8_t BC7161_page_read_register(uint8_t sra,uint8_t *pb,uint8_t size)
{
   uint8_t ack;
   IIC_interface_start();           /* IIC start bit */
   ack = IIC_interface_ouput(IIC_WRITE_DADDR);
   if(ack)
   {
      IIC_interface_ouput(sra);     /* write register address */
      IIC_interface_stop();
      IIC_interface_start();        /* IIC start bit */
      ack = IIC_interface_ouput(IIC_READ_DADDR);
      if(ack)
      {
         for(;size > 1;size--)
         {
            *pb = IIC_interface_input(TRUE);
            pb++;
         }
         *pb = IIC_interface_input(FALSE);
      }
   }
   IIC_interface_stop();
   return(ack);
}
/*----------------------------------------------------------------------------*/
/*	 BC7161 page write regiseter                                               */
/*----------------------------------------------------------------------------*/
uint8_t BC7161_page_write_register(uint8_t sra,uint8_t *pb,uint8_t size)
{
   uint8_t ack;
   IIC_interface_start();           /* IIC start bit */
   ack = IIC_interface_ouput(IIC_WRITE_DADDR);
   if(ack)
   {
      ack = IIC_interface_ouput(sra);    /* write register address */
      while( ack && ( size > 0))
      {
         ack = IIC_interface_ouput(*pb);
         pb++;
         size --;
      }
   }
   IIC_interface_stop();
   return(ack);
}
#if 0

/*----------------------------------------------------------------------------*/
/*	 BC7161 write multibyte register                                 				*/
/*----------------------------------------------------------------------------*/
uint8_tv BC7161_WriteMultiRegister(uint8_t adr,uint8_t *val,u8 leng)
{
	uint8_t	ack;
	
	IIC_StartBit();
//	ack = IIC_DataOutput(IIC_WRITE_DADDR);
	IIC_DataOutput(IIC_WRITE_DADDR);
//	asm volatile("mov %0,A"::"m"(ack));	
	if(_acc)
	{
//		ack = IIC_DataOutput(adr);				/* write register address */
//		asm volatile("mov A,%0"::"m"(adr));	
//		asm volatile("call _IIC_DataOutput");
//		asm volatile("mov %0,A"::"m"(ack));
		_mp1 = ((u16)val & 0xFF);
		_bp = ((u16)val >> 8);						
		while((ack==TRUE) && (leng > 0))
		{
//			ack=IIC_DataOutput(*val);
//			asm volatile("mov A,%0"::"m"(_iar1));	
//			asm volatile("call _IIC_DataOutput");
//			asm volatile("mov %0,A"::"m"(ack));
			_mp1++;
			leng--;
		}
	}
	IIC_StopBit();
	return(ack);
}

/*----------------------------------------------------------------------------*/
/*	 BC7161 read multibyte register                                  				*/
/*----------------------------------------------------------------------------*/
uint8_t BC7161_ReadMultiRegister(uint8_t adr,uint8_t *rval,u8 leng)
{
	uint8_t	ack;
	
	IIC_StartBit();
//	ack=IIC_DataOutput(IIC_WRITE_DADDR);
	IIC_DataOutput(IIC_WRITE_DADDR);
//	asm volatile("mov %0,A"::"m"(ack));	
//	if(ack)
	if(_acc)
	{
//		IIC_DataOutput(adr);				/* write register address */		
//		asm volatile("mov A,%0"::"m"(adr));	
//		asm volatile("call _IIC_DataOutput");	
		IIC_StopBit();						
		IIC_StartBit();					/* restart bit */
//		ack=IIC_DataOutput(IIC_READ_DADDR);
		IIC_DataOutput(IIC_READ_DADDR);
//		asm volatile("mov %0,A"::"m"(ack));
//		if(ack)
		if(_acc)
		{
			_mp1 = ((u16)rval & 0xFF);
			_bp = ((u16)rval >> 8);			
			while(leng)
			{	
//				*rval=IIC_DataInput((leng > 1)?TRUE:FALSE);
//				asm volatile("deca %0"::"m"(leng));	/* if(leng==1) acc=FALSE else acc=TRUE */
//				asm volatile("call _IIC_DataInput");
//				asm volatile("mov %0,A"::"m"(_iar1));
//				*rval=adr;
//				rval++;
				_mp1++;
				leng--;
			}
		}
	}
	IIC_StopBit();
	return(ack);	
}
#endif