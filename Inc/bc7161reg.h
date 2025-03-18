
#ifndef _BC7161_REG_H_
#define _BC7161_REG_H_

#define	BC7161_DEVICE_ADDR			0x71
/*------------- BC7161 register memory map ------------------*/
#define	XO_TRIM_REG						0x00
#define	XO_CTL_REG						0x01
#define	TXPWR_CTL_REG					0x0C
#define	PDU_DATA_REG					0x10
#define	PDU_LEN_REG						0x11
#define	PDU_PTR_REG						0x12
#define	PKT_CTL_REG						0x13
#define	WHT_CTL_REG						0x14
#define	PKT_RESEND_REG					0x15
#define	APRD_CTL_REG					0x16
#define	PKT_PERIOD_REG					0x17
#define	DRTXP_CTL_REG					0x19
#define	TXSFNO_CTL_REG					0x1A
#define	LIRC_CTL_REG					0x1D
#define	VCO_ACAL_REG					0x1E
#define	GIO_CTL_REG						0x25
#define	GIOPU_CTL_REG					0x26
#define	SLUM_CTL_REG					0x27
#define	CHIPID_REG						0x30
#define	CHIPVER_REG						0x32
#define	RESET_CTL_REG					0x33

#endif	/* _BC7161_REG_H_ */
