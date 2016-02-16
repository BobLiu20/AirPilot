#ifndef __AP_I2C_H
#define __AP_I2C_H
#include "includes.h"

extern uint16_t I2C_ErrorCount;						//I2C错误累计

void I2C2_Init(void);
uint8_t I2C2_WriteReg(u8 SlaveAddr,uint16_t Addr, uint8_t Data);
//uint8_t I2C2_ReadReg(u8 SlaveAddr,uint16_t Addr);
//uint8_t I2C2_ReadReg_Buf(u8 SlaveAddr,uint16_t Addr,u8 len,u8 *buf);
uint8_t I2C2_TiReadReg_Buf(u8 SlaveAddr,uint16_t Addr,u8 len,u8 *buf);	//适用于OS下，使用中断

#endif
