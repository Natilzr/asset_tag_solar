#include "main.h"
#include "max31726.h"
#include        "string.h"
#include "bc7161.h"


extern I2C_HandleTypeDef hi2c1;


uint8_t Max317267_WriteRegister(uint8_t adr,uint8_t size,uint8_t val)
{
  HAL_StatusTypeDef ret;

    uint8_t buf[3];
    buf[0] = adr;
    buf[1] = val;
    ret = HAL_I2C_Master_Transmit(&hi2c1,MAX_31_IIC_ADD, buf, size, I2C_DELAY);
        return ret;
}

uint8_t Max317267_ReadRegister(uint8_t adr,uint8_t size,uint8_t *rval)
{
    HAL_StatusTypeDef ret;

    uint8_t buf[40];
    buf[0] = adr;
    ret = HAL_I2C_Master_Transmit(&hi2c1,MAX_31_IIC_ADD | 0x01, buf, 1, I2C_DELAY);

    HAL_I2C_Master_Receive(&hi2c1,MAX_31_IIC_ADD, buf, size, 100);
    *rval = buf[0];
    *(rval+1) = buf[1];
    return ret;
}
    
    
uint8_t Sht_WriteRegister(uint8_t adr,uint8_t size,uint8_t val)
{
  HAL_StatusTypeDef ret;

    uint8_t buf[3];
    buf[0] = adr;
    buf[1] = val;
    ret = HAL_I2C_Master_Transmit(&hi2c1,STDS75_IIC_ADD, buf, size, I2C_DELAY);
        return ret;
}

uint8_t Sht_ReadRegister(uint8_t adr,uint8_t size,uint8_t *rval)
{
    HAL_StatusTypeDef ret;

    uint8_t buf[40];
    buf[0] = adr;
    ret = HAL_I2C_Master_Transmit(&hi2c1,STDS75_IIC_ADD , buf, 1, I2C_DELAY);
   // delay_us (1000);
    ret = HAL_I2C_Master_Receive(&hi2c1,STDS75_IIC_ADD, buf, size, 100);
    *rval = buf[0];
    *(rval+1) = buf[1];
    return ret;
}    
    



uint8_t STDS75_WriteRegister(uint8_t adr,uint8_t size,uint8_t val)
{
  HAL_StatusTypeDef ret;

    uint8_t buf[3];
    buf[0] = adr;
    buf[1] = val;
    ret = HAL_I2C_Master_Transmit(&hi2c1,STDS75_IIC_ADD, buf, size, I2C_DELAY);
        return ret;
}

uint8_t STDS75_ReadRegister(uint8_t adr,uint8_t size,uint8_t *rval)
{
    HAL_StatusTypeDef ret;
 
    uint8_t buf[40];
    buf[0] = adr;
    ret = HAL_I2C_Master_Transmit(&hi2c1,STDS75_IIC_ADD , buf, 1, I2C_DELAY);
   // delay_us (1000);
    ret = HAL_I2C_Master_Receive(&hi2c1,STDS75_IIC_ADD, buf, size, 100);
    *rval = buf[0];
    *(rval+1) = buf[1];
    return ret;
}