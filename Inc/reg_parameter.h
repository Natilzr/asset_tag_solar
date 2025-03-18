
#ifndef _REG_PARAMETER_H_
#define _REG_PARAMETER_H_

#include		"bc7161reg.h"
#include		"bc7161.h"

#define	LPM_5_DBM			1										
#define	LPM_2_DBM			2		
#define	LPM_0_DBM			3		
#define	LPM_m5_DBM			4		
#define	LPM_m10_DBM			5
				
#define	HPM_8_DBM			6					
#define	HPM_5_DBM			7							
#define	HPM_m2_DBM			8		
#define	HPM_m5_DBM			9		
#define	HPM_m10_DBM			10		
			    
#define	CRYSTAL_TRIM_VALUE		0x21		/*Trim value for the internal capacitor load for the crystal.*/
#define	TX_POWER_LEVEL			4           /* 0:-10dBm,1:-5dBm,2:0dBm,3:+2dBm,4:+5dBm */
#define	ADVE_RESEND_COUNTER		1//2//5 	 	    /* Adev Event resend counter 0~254,255:Always repeat */
#define	ADVE_PERIOD_TIMER		30	    	/* Adev Event period timer 10ms~10240ms */
#define	ADVE_CH_DELAY			2000 //250		    /* Adev channel delay 250us~8000us */
#define	ADVE_CH_THRESHOLD		0			/* 0:1ms,1:1.5ms,2:2ms,3:3ms */
#define	GIO2_FUNCTION			INPUT_MODE
#define	GIO3_FUNCTION			TXS_OUTPUT
#define	I2C_SDA_PULL_HIGH		0			/* I2C SDA pull high 0:disable,1:enable */
#define	I2C_SCL_PULL_HIGH		0			/* I2C SCL pull high 0:disable,1:enable */
#define	GIO2_PULL_HIGH			1			/* GIO2 pull high 0:disable,1:enable */
#define	GIO3_PULL_HIGH			0			/* GIO3 pull high 0:disable,1:enable */
#define	SLEEP_THRESHOLD			0//1			/* light sleep to deep sleep hold time,0:2ms,1:10ms */
  

#if	(TX_POWER_LEVEL == 0)
#define	TX_POWER_VALUE1				0xAF		/* -10dBm */
#define	TX_POWER_VALUE2				0x71
#elif	(TX_POWER_LEVEL == 1)
#define	TX_POWER_VALUE1				0xAF		/* -5dBm */
#define	TX_POWER_VALUE2				0x73
#elif	(TX_POWER_LEVEL == 3)
#define	TX_POWER_VALUE1				0xA2		/* +2dBm */
#define	TX_POWER_VALUE2				0xA2
#elif	(TX_POWER_LEVEL == 4)
#define	TX_POWER_VALUE1				0xA2		/* +5dBm */
#define	TX_POWER_VALUE2				0x67
#else	
#define	TX_POWER_VALUE1				0xAF		/* 0dBm */
#define	TX_POWER_VALUE2				0xD7
#endif

#define	APRD_VALUE						((ADVE_CH_THRESHOLD << 6)+((ADVE_CH_DELAY/250)-1))
#define	ADVE_PERIOD_VALUE				((ADVE_PERIOD_TIMER/10)-1)
#define	ADVE_PERIOD_VLOW				(ADVE_PERIOD_VALUE & 0xFF)
#define	ADVE_PERIOD_VHIGH				((ADVE_PERIOD_VALUE/256) & 0x03)
#define	GIO_FUN_VALUE					((GIO3_FUNCTION << 4)+GIO2_FUNCTION)
#define	GIO_PULL_HIGH					((GIO3_PULL_HIGH << 3)+(GIO2_PULL_HIGH << 2)+(I2C_SCL_PULL_HIGH << 1)+I2C_SDA_PULL_HIGH)
#define	SLEEP_HOLD_VALUE			    (SLEEP_THRESHOLD << 6)

#define TIMEOUT_COUNTER		         ((((ADVE_PERIOD_TIMER*ADVE_RESEND_COUNTER)+5)*105+199)/200)

#if	TIMEOUT_COUNTER > 255
	Error 'TIMER_COUNTER'
#endif	

#endif   /* _REG_PARAMETER_H_ */