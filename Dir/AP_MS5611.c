/**
	******************************************************************************
	* @file    AP_MS5611.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   高精度气压计
	******************************************************************************
**/
#include "AP_MS5611.h"
#ifdef MS5611
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
// MS5611, Standard address 0x77
#define MS5611_ADDR                 (0x77<<1)

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8

/* variables ---------------------------------------------------------*/
static uint32_t MS5611_Ut;  // static result of temperature measurement
static uint32_t MS5611_Up;  // static result of pressure measurement
static uint16_t MS5611_C[PROM_NB];  // on-chip ROM
static uint8_t MS5611_Osr = CMD_ADC_4096;

/* function prototypes -----------------------------------------------*/
static void MS5611_Reset(void);
static uint16_t MS5611_Prom(int8_t coef_num);
static int8_t MS5611_Crc(uint16_t *prom);
static uint32_t MS5611_Read_Adc(void);
static void MS5611_Start_Ut(void);
static void MS5611_Get_Ut(void);
static void MS5611_Start_Up(void);
static void MS5611_Get_Up(void);
static int32_t MS5611_Calculate(void);

/**
  * @brief  检测是否存在MS5611
  *         
  * @param  baro:气压函数结构体
  * @retval 存在时返回true
  */
bool MS5611_Detect(baro_t *baro)
{
	OS_ERR err;
	bool ack = false;
	uint8_t sig;
	int i;
	OSTimeDly(10,OS_OPT_TIME_DLY,&err);// 大概延时个10Ms,让气压计上电热身下

	ack = I2C2_TiReadReg_Buf(MS5611_ADDR, CMD_PROM_RD, 1, &sig);
	if (!ack)
		return false;

	MS5611_Reset();
	// read all coefficients
	for (i = 0; i < PROM_NB; i++)
			MS5611_C[i] = MS5611_Prom(i);
	// check crc, bail out if wrong - we are probably talking to BMP085 w/o XCLR line!
	if (MS5611_Crc(MS5611_C) != 0)
			return false;

	// TODO prom + CRC
	baro->ut_delay = 10000;
	baro->up_delay = 10000;
	baro->repeat_delay = 4000;
	baro->start_ut = MS5611_Start_Ut;
	baro->get_ut = MS5611_Get_Ut;
	baro->start_up = MS5611_Start_Up;
	baro->get_up = MS5611_Get_Up;
	baro->calculate = MS5611_Calculate;

  return true;
}

static void MS5611_Reset(void)
{
	OS_ERR err;
  I2C2_WriteReg(MS5611_ADDR, CMD_RESET, 1);
  OSTimeDly(3,OS_OPT_TIME_DLY,&err);
}

static uint16_t MS5611_Prom(int8_t coef_num)
{
    uint8_t rxbuf[2] = { 0, 0 };
    I2C2_TiReadReg_Buf(MS5611_ADDR, CMD_PROM_RD + coef_num * 2, 2, rxbuf); // send PROM READ command
    return rxbuf[0] << 8 | rxbuf[1];
}

static int8_t MS5611_Crc(uint16_t *prom)
{
    int32_t i, j;
    uint32_t res = 0;
    uint8_t zero = 1;
    uint8_t crc = prom[7] & 0xF;
    prom[7] &= 0xFF00;

    // if eeprom is all zeros, we're probably fucked - BUT this will return valid CRC lol
    for (i = 0; i < 8; i++) {
        if (prom[i] != 0)
            zero = 0;
    }
    if (zero)
        return -1;

    for (i = 0; i < 16; i++) {
        if (i & 1) 
            res ^= ((prom[i >> 1]) & 0x00FF);
        else 
            res ^= (prom[i >> 1] >> 8);
        for (j = 8; j > 0; j--) {
            if (res & 0x8000) 
                res ^= 0x1800;
            res <<= 1;
        }
    }
    prom[7] |= crc;
    if (crc == ((res >> 12) & 0xF)) 
        return 0;

    return -1;
}

static uint32_t MS5611_Read_Adc(void)
{
    uint8_t rxbuf[3];
    I2C2_TiReadReg_Buf(MS5611_ADDR, CMD_ADC_READ, 3, rxbuf); // read ADC
    return (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}

static void MS5611_Start_Ut(void)
{
    I2C2_WriteReg(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + MS5611_Osr, 1); // D2 (temperature) conversion start!
}

static void MS5611_Get_Ut(void)
{
    MS5611_Ut = MS5611_Read_Adc();
}

static void MS5611_Start_Up(void)
{
    I2C2_WriteReg(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + MS5611_Osr, 1); // D1 (pressure) conversion start!
}

static void MS5611_Get_Up(void)
{
    MS5611_Up = MS5611_Read_Adc();
}

int32_t BaroTemperature;//校准后的温度
static int32_t MS5611_Calculate(void)
{
    int32_t temperature, off2 = 0, sens2 = 0, delt;
    int32_t pressure;

    int32_t dT = MS5611_Ut - ((uint32_t)MS5611_C[5] << 8);
    int64_t off = ((uint32_t)MS5611_C[2] << 16) + (((int64_t)dT * MS5611_C[4]) >> 7);
    int64_t sens = ((uint32_t)MS5611_C[1] << 15) + (((int64_t)dT * MS5611_C[3]) >> 8);
    temperature = 2000 + (((int64_t)dT * MS5611_C[6]) >> 23);
		BaroTemperature=temperature;
    if (temperature < 2000) { // temperature lower than 20degC 
        delt = temperature - 2000;
        delt = delt * delt;
        off2 = (5 * delt) >> 1;
        sens2 = (5 * delt) >> 2;
        if (temperature < -1500) { // temperature lower than -15degC
            delt = temperature + 1500;
            delt = delt * delt;
            off2  += 7 * delt;
            sens2 += (11 * delt) >> 1;
        }
    }
    off  -= off2; 
    sens -= sens2;
    pressure = (((MS5611_Up * sens ) >> 21) - off) >> 15;
    return pressure;
}

uint32_t BaroPressureSum;	//用来保存BARO_TAB_SIZE次的总和
void Baro_Common(int32_t p) 
{
	static int32_t baroHistTab[BARO_TAB_SIZE];
	static int8_t baroHistIdx;
	
	uint8_t indexplus1 = (baroHistIdx + 1)%BARO_TAB_SIZE;
	baroHistTab[baroHistIdx] = p;
	BaroPressureSum += baroHistTab[baroHistIdx];
	BaroPressureSum -= baroHistTab[indexplus1];
	baroHistIdx = indexplus1;  
}


#endif
