
#ifndef MAX_31
#define MAX_31


#define MAX_31_IIC_ADD 0x98
#define STDS75_IIC_ADD 0x90

#define SHT30_IIC_ADD  0x88

#define TempReg                 0x00
#define ConfigurationReg        0x01
#define ThystReg                0x02
#define TosReg                  0x03

uint8_t Max317267_WriteRegister(uint8_t adr,uint8_t size,uint8_t val);


uint8_t Max317267_ReadRegister(uint8_t adr,uint8_t size,uint8_t *rval);

uint8_t STDS75_WriteRegister(uint8_t adr,uint8_t size,uint8_t val);
uint8_t STDS75_ReadRegister(uint8_t adr,uint8_t size,uint8_t *rval);


uint8_t Sht_WriteRegister(uint8_t adr,uint8_t size,uint8_t val);
uint8_t Sht_ReadRegister(uint8_t adr,uint8_t size,uint8_t *rval);


#endif
    
    
    
    