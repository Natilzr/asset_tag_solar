
#ifndef _BC7161_H_
#define _BC7161_H_

#include	"bc7161reg.h"
#include        "main.h" 


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





#define BIT_0 	0x01 		/* The value of bit 0 */
#define BIT_1 	0x02 		/* The value of bit 1 */
#define BIT_2 	0x04 		/* The value of bit 2 */
#define BIT_3 	0x08 		/* The value of bit 3 */
#define BIT_4 	0x10 		/* The value of bit 4 */
#define BIT_5 	0x20 		/* The value of bit 5 */
#define BIT_6 	0x40 		/* The value of bit 6 */
#define BIT_7 	0x80 		/* The value of bit 7 */
#define BIT_8 	0x0100 	/* The value of bit 8 */
#define BIT_9 	0x0200 	/* The value of bit 9 */
#define BIT_10 0x0400 	/* The value of bit 10 */
#define BIT_11 0x0800 	/* The value of bit 11 */
#define BIT_12 0x1000 	/* The value of bit 12 */
#define BIT_13 0x2000 	/* The value of bit 13 */
#define BIT_14 0x4000 	/* The value of bit 14 */
#define BIT_15 0x8000 	/* The value of bit 15 */

/*----------------- XO Control Register -----------------*/
typedef union 
{
   unsigned char	value;
   struct 
   {
      unsigned char  xo_il : 2;
      unsigned char  xshift: 2;
      unsigned char  xodiv2: 1;
      unsigned char  xosel : 1;
      unsigned char  xrdy_dly : 2;
   }bit;
}xo_ctl_t;

/*----------------- PDU data length Register -----------------*/
typedef union 
{
   unsigned char	value;
   struct 
   {
      unsigned char  pdulen : 7;		
      unsigned char  flush_ff : 1;
   }bit;
}pdu_len_t;

#define	_FLUSH_FIFO_		0x80

/*----------------- packet format Control Register -----------------*/
typedef union 
{
   unsigned char	value;
   struct 
   {
      unsigned char  pmlaa_en : 1;
		unsigned char  crc_en : 1;
		unsigned char  wht_en : 1;	
		unsigned char  pmlen : 1;
      const unsigned char : 4;
   }bit;
}pkt_ctl_t;

/*----------------- whitening Control Register -----------------*/
typedef union 
{
   unsigned char	value;
   struct 
   {
      unsigned char  whtsd : 7;
		unsigned char  whtsds : 1;
   }bit;
}wht_ctl_t;

/*----------------- Auto period retransmit delay Control Register -----------------*/
typedef union 
{
   unsigned char	value;
   struct 
   {
      unsigned char  pkt_aprd : 5;
		const unsigned char : 1;		
		unsigned char  aprd_pdth : 2;
   }bit;
}aprd_ctl_t;

/*----------------- data rate & Tx power Control Register -----------------*/
typedef union 
{
   unsigned char	value;
   struct 
   {
      unsigned char  data_rate : 3;
		const unsigned char : 1;
      unsigned char  tx_power : 3;
		const unsigned char : 1;		
   }bit;
}drtxp_ctl_t;

enum
{
	DR500KBPS = 0,
	DR1MBPS,
	DR2MBPS,
	DR4MBPS
};

enum
{
	N10DBM = 0,
	N05DBM,
	P00DBM,
	P05DBM,
	P07DBM
};

/*----- ADV Channel Enable & TX start Control Register -----------------*/
#define	ADV_CH37		0x01//0x01
#define	ADV_CH38		0x01//0x02
#define	ADV_CH39		0x01//0x04
#define	ADV_CHANNEL	((ADV_CH39 << 2)+(ADV_CH38 << 1)+ADV_CH37)

typedef union 
{
   unsigned char	value;
   struct 
   {
      unsigned char  ch37en : 1;
		unsigned char  ch38en : 1;
		unsigned char  ch39en : 1;
		const unsigned char : 4;
      unsigned char  tx_start : 1;
   }bit;
}txsfno_ctl_t;

/*----------------- LIRC Control Register -----------------*/
typedef union 
{
   unsigned char	value;
   struct 
   {
      unsigned char  en : 1;
		unsigned char  op : 5;
		unsigned char  ow : 1;
      unsigned char  cal : 1;
   }bit;
}lirc_ctl_t;

/*------------------ GPIO Control Register -------------------------*/
typedef union 
{
   unsigned char value;
	struct 
   {
	   unsigned char gio2 : 3;
		const unsigned char : 1;
	   unsigned char gio3 : 3;
		const unsigned char : 1;
	} bit;
} gio_ctl_t;

enum 
{
   INPUT_MODE = 0,         /* GPIO input mode */
   FCLK_OUTPUT,            /* Frequency clock output */
   TBCLK_OUTPUT,       		/* TX bit clock output */
   TXD_OUTPUT,             /* TX data output */
   TXD_INPUT,             	/* TX data input */
	TXS_OUTPUT,					/* TX start status output */
   LOSC_INOUT,            	/* LOSC input or output */
   TXACT_OUTPUT,           /* TX active status output */
};

/*------------------ GPIO pull-up Control Register -------------------------*/
typedef union 
{
   unsigned char value;
	struct 
   {
	   unsigned char sdapu : 1;
	   unsigned char sclpu : 1;
	   unsigned char gio2pu : 1;
	   unsigned char gio3pu : 1;
		const unsigned char : 4;
	} bit;
} giopu_ctl_t;

/*------------------ CFG27 Control Register -------------------------*/
#define	SLEEP_MODE1		0		/* I2C timeout on with 2/10mS delay, TX time out on with 0S delay. */
#define	SLEEP_MODE2		1		/* I2C timeout on with 2/10mS delay, TX time out on with 2/10mS delay. */
#define	SLEEP_NO			3     /* time outoff, always on light sleep mode */

typedef union 
{
   unsigned char value;
	struct 
   {
	   unsigned char lirccks : 1;
	   unsigned char lircss : 1;
	   unsigned char wottu : 1;
	   unsigned char rftxm : 1;
	   unsigned char lstom : 2;
	   unsigned char lstos : 1;
	   unsigned char pwron : 1;
	} bit;
} slum_ctl_t;

void    BC7161_InterfaceConfigure(void);
void 	BC7161InterfaceConfigure(void);
void 	BC7161_get_chip_id(uint8_t *pb);
void 	BC7161_gpio_setup(uint8_t gfn2,uint8_t gfn3,uint8_t gpu);
uint8_t 	BC7161_get_gpio_state(void);
void 	BC7161_register_setup(void);
void 	BC7161_set_crystal_cap(uint8_t cap);
void 	BC7161_set_sleep_mode(uint8_t mode);
void 	BC7161_set_tx_power(uint8_t tpw);
void 	BC7161_set_resend_counter(uint8_t cnt);
void 	BC7161_set_period_delay(uint8_t pdth,uint8_t aprd);
void 	BC7161_set_period_timer(uint16_t tm);
uint8_t 	BC7161_write_pdu_fifo(uint8_t *pd,uint8_t len);
void 	BC7161_trigger_tx_start(uint8_t chn);
void 	BC7161_stop_transmit(void);
void 	BC7161_LIRC_calibration(void);
void 	BC7161_vco_calibration(void);
void 	BC7161_set_dnk_register(void);
void 	BC7161_carrier_enable(void);
void 	BC7161_pn9_enable(void);
void 	BC7161_carrier_disable(void);
void 	BC7161_wakeup(void);
uint8_t 	BC7161_read_register(uint8_t adr,uint8_t *rv);
uint8_t 	BC7161_write_register(uint8_t adr,uint8_t da);
uint8_t 	BC7161_page_read_register(uint8_t sra,uint8_t *pb,uint8_t size);
uint8_t 	BC7161_page_write_register(uint8_t sra,uint8_t *pb,uint8_t size);
void delay_us (uint16_t us);
void delay_us_Sleep (uint16_t us);
#define	BC7161_TriggerAdvStart(advch)	BC7161_write_register(TXSFNO_CTL_REG,advch | 0x80)	
#define BC7161_Timeout(advch)           BC7161_write_register(TXSFNO_CTL_REG,advch & 0x7F)  

#endif   /* _BC7161_H_ */
