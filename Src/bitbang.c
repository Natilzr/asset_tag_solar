

#define	_SCL_PORT_			GPIOF
#define	_SCL_			        GPIO_PIN_0 
#define	_SCL_AFIO_PORT_			GPIO_PA
#define	_SCL_AFIO_FUN_			AFIO_FUN_GPIO

#define	_SDA_PORT_			GPIOF
#define	_SDA_				GPIO_PIN_1
#define	_SDA_AFIO_PORT_			GPIO_PA
#define	_SDA_AFIO_FUN_			AFIO_FUN_GPIO

#define	SCL_SET()					(_SCL_PORT_->BSRR = _SCL_)
#define	SCL_RESET()					(_SCL_PORT_->BRR = _SCL_)
#define	SDA_DIRO()					(_SDA_PORT_->MODER |= _SDA_)
#define	SDA_DIRI()					(_SDA_PORT_->MODER &= ~_SDA_)
#define	SDA_SET()					(_SDA_PORT_->BSRR = _SDA_)
#define	SDA_RESET()					(_SDA_PORT_->BRR = _SDA_)
#define	SDA_TOGGLE()				((_SDA_PORT_->ODR & _SDA_)? SDA_RESET():SDA_SET())
#define	SDA_IN()						(_SDA_PORT_->IDR & _SDA_)

#define	_GIO3_PORT_					GPIOA
#define	_GIO3_						GPIO_PIN_4
#define	_GIO3_AFIO_PORT_			GPIO_PA
#define	_GIO3_AFIO_FUN_			AFIO_FUN_GPIO

#define	GIO3_DIRO()					(_GIO3_PORT_->DIRCR |= _GIO3_)
#define	GIO3_DIRI()					(_GIO3_PORT_->DIRCR &= ~_GIO3_)
#define	GIO3_SET()					(_GIO3_PORT_->SRR = _GIO3_)
#define	GIO3_RESET()				(_GIO3_PORT_->RR = _GIO3_)
#define	GIO3_TOGGLE()				((_GIO3_PORT_->DOUTR & _GIO3_)? GIO3_RESET():GIO3_SET())
#define	GIO3_IN()					(_GIO3_PORT_->IDR & _GIO3_)


void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim14,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim14) < us);  // wait for the counter to reach the us input in the parameter
}

/*----------------------------------------------------------------------------*/
/*	 IIC serial start bit                                                      */
/*----------------------------------------------------------------------------*/
static void IIC_interface_start(void)
{   
   SCL_SET();
   delay_us(5);
 //  SDA_DIRO();
   SDA_RESET();
   delay_us(5);
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
   delay_us(5);
   SCL_SET();
   delay_us(5);   
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
      delay_us(5);      
      SCL_SET();      
      sd <<= 1;
      delay_us(5);      
      SCL_RESET();
      delay_us(1);
   }
   SDA_SET();
 //  SDA_DIRI();
   delay_us(5);         
   SCL_SET();
   delay_us(5);   
   ack = SDA_IN() ? FALSE:TRUE;
   SCL_RESET();
   delay_us(5);      
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
      delay_us(5);         
      sd <<= 1;
      SCL_SET();
      delay_us(5);         
      sd |= SDA_IN() ? 0x01:0x00;
      SCL_RESET();
   }
 //  SDA_DIRO();
   ack ? SDA_RESET() : SDA_SET();
   delay_us(5);
   SCL_SET();
   delay_us(5);      
   SCL_RESET();
   delay_us(5);      
   return(sd);
}


/*----------------------------------------------------------------------------*/
/* @brief 	: read register 														 	*
 *	@param	: adr->register address															*
 *	@retval  : bit[7..0] = register value													*
 *				  bit[8] = ack status															*/
/*----------------------------------------------------------------------------*/
uint8_t read_register(uint8_t adr,uint8_t *rval)
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
/* @brief 	: write data register 												*
 *	@param	: register address & write value												*
 *	@retval  : ack status																		*/
/*----------------------------------------------------------------------------*/
uint8_t write_register(uint8_t adr,uint8_t da)
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
/*	 read regiseter                                                */
/*----------------------------------------------------------------------------*/
uint8_t page_read_register(uint8_t sra,uint8_t *pb,uint8_t size)
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
/*	 page write regiseter                                               */
/*----------------------------------------------------------------------------*/
uint8_t page_write_register(uint8_t sra,uint8_t *pb,uint8_t size)
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