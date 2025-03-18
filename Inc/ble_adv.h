
#ifndef _BLE_ADV_H_
#define _BLE_ADV_H_

/*------------------ BLE advertising PDU -------------------------*/

typedef union 
{
   unsigned short value;
	struct 
   {
	    unsigned short  type 	: 4;
		const unsigned  short  	: 2;
		unsigned short  txadd 	: 1;
		unsigned short  rxadd	: 1;
		unsigned short  length	: 6;
		const unsigned short		: 2;
	}bits;
} adv_header_struct;

#define	ADV_HEADER_SIZE		    sizeof(adv_header_struct)
#define	ADV_ADDR_SIZE			6
#define	ADV_DATA_MAX_SIZE		31

typedef struct 
{
	adv_header_struct	header;
    unsigned char adv_addr[ADV_ADDR_SIZE];
	unsigned char adv_data[ADV_DATA_MAX_SIZE];
} adv_pdu_struct;

#define	ADV_PDU_SIZE			sizeof(adv_pdu_struct)

enum
{
	ADV_IND = 0,
	ADV_DIRECT_IND,
	ADV_NONCONN_IND,
	SCAN_REQ,
	SCAN_RSP,
	CONNECT_REQ,
	ADV_SCAN_IND
};

	
typedef struct
{
	unsigned char length;
	unsigned char type;
	unsigned char data[];
} ad_struct;


#endif   /* _BLE_ADV_H_ */
