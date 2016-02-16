/**
	******************************************************************************
	* @file    AP_Serial.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   与上位机调参软件通讯协议实现与应用
	******************************************************************************
**/
#include "AP_Serial.h"

flags_t f;

int16_t debug[4];
uint8_t vbat;                   // battery voltage in 0.1V steps

/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
#define  VERSION  210//版本号

#define MSP_VERSION              0
#define PLATFORM_32BIT           0x80000000

#define MSP_IDENT                100    //out message         multitype + version
#define MSP_STATUS               101    //out message         cycletime & errors_count & sensor present & box activation
#define MSP_RAW_IMU              102    //out message         9 DOF
#define MSP_SERVO                103    //out message         8 servos
#define MSP_MOTOR                104    //out message         8 motors
#define MSP_RC                   105    //out message         8 rc chan
#define MSP_RAW_GPS              106    //out message         fix, numsat, lat, lon, alt, speed
#define MSP_COMP_GPS             107    //out message         distance home, direction home
#define MSP_ATTITUDE             108    //out message         2 angles 1 heading
#define MSP_ALTITUDE             109    //out message         1 altitude
#define MSP_BAT                  110    //out message         vbat, powermetersum
#define MSP_RC_TUNING            111    //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112    //out message         up to 16 P I D (8 are used)
#define MSP_BOX                  113    //out message         up to 16 checkbox (11 are used)
#define MSP_MISC                 114    //out message         powermeter trig + 8 free for future use
#define MSP_MOTOR_PINS           115    //out message         which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES             116    //out message         the aux switch names
#define MSP_PIDNAMES             117    //out message         the PID names
#define MSP_WP                   118    //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold

#define MSP_SET_RAW_RC           200    //in message          8 rc chan
#define MSP_SET_RAW_GPS          201    //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202    //in message          up to 16 P I D (8 are used)
#define MSP_SET_BOX              203    //in message          up to 16 checkbox (11 are used)
#define MSP_SET_RC_TUNING        204    //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205    //in message          no param
#define MSP_MAG_CALIBRATION      206    //in message          no param
#define MSP_SET_MISC             207    //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208    //in message          no param
#define MSP_WP_SET               209    //in message          sets a given WP (WP#,lat, lon, alt, flags)

#define MSP_DIYVAR_READ					 251	//自定义变量读
#define MSP_DIYVAR_WRITE				 252	//写

#define MSP_SET_ACCZERO					 246	//设置
#define MSP_READ_ACCZERO				 245	//读取

#define MSP_EEPROM_WRITE         250    //in message          no param

#define MSP_DEBUG                254    //out message         debug1,debug2,debug3,debug4

#define INBUF_SIZE 64
/* variables ---------------------------------------------------------*/
static const char boxnames[] =
    "ACC;"
    "BARO;"
    "MAG;"
    "ONEROLL;"
    "LANDING;"
    "ARM;"
    "GPS HOME;"
    "GPS HOLD;"
    "PASSTHRU;"
    "HEADFREE;"
    "OPTFLOW;"
    "VISION;"
    "AVOID;"
    "HEADADJ;";

static const char pidnames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "ALT;"
    "Pos;"
    "PosR;"
    "NavR;"
    "LEVEL;"
    "MAG;"
    "VEL;";

static uint8_t checksum, indRX, inBuf[INBUF_SIZE];
static uint8_t cmdMSP;

/************************任务控制块*************************/
static  void  Serial_MainTask (void *p_arg);
static  OS_TCB   Serial_MainTaskTCB;
static  CPU_STK  Serial_MainTaskStk[APP_CFG_TASK_START_STK_SIZE];

/* function prototypes -----------------------------------------------*/
void Serial_Com(void);

/**
  * @brief  初始化
  *         
  * @param  USART的波特率
  * @retval none
  */
void Serial_Init(uint32_t baudrate)
{
	OS_ERR      err;
	USART1_Init(baudrate);
	OSTaskCreate((OS_TCB       *)&Serial_MainTaskTCB, 
						 (CPU_CHAR     *)"Serial_MainTask",
						 (OS_TASK_PTR   )Serial_MainTask, 
						 (void         *)0,
						 (OS_PRIO       )PRIO_SERIAL ,
						 (CPU_STK      *)&Serial_MainTaskStk[0],
						 (CPU_STK_SIZE  )Serial_MainTaskStk[APP_CFG_TASK_START_STK_SIZE / 10],
						 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
						 (OS_MSG_QTY    )0,
						 (OS_TICK       )0,
						 (void         *)0,
						 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
						 (OS_ERR       *)&err);
}

/**
  * @brief  主任务，负责与上位机调参软件通讯
  *         
  * @param  
  * @retval 
  */
static  void  Serial_MainTask (void *p_arg)
{
	OS_ERR      err;
  (void)p_arg;
  while (1)
	{
		OSTimeDly(1,OS_OPT_TIME_DLY,&err);//周期延时
		Serial_Com();
  }
}

static void serialize32(uint32_t a)
{
    static uint8_t t;
    t = a;
    USART1_Write(t);
    checksum ^= t;
    t = a >> 8;
    USART1_Write(t);
    checksum ^= t;
    t = a >> 16;
    USART1_Write(t);
    checksum ^= t;
    t = a >> 24;
    USART1_Write(t);
    checksum ^= t;
}

static void serialize16(int16_t a)
{
    static uint8_t t;
    t = a;
    USART1_Write(t);
    checksum ^= t;
    t = a >> 8 & 0xff;
    USART1_Write(t);
    checksum ^= t;
}

static void serialize8(uint8_t a)
{
    USART1_Write(a);
    checksum ^= a;
}

static uint8_t read8(void)
{
    return inBuf[indRX++] & 0xff;
}

static uint16_t read16(void)
{
    uint16_t t = read8();
    t += (uint16_t) read8() << 8;
    return t;
}

static uint32_t read32(void)
{
    uint32_t t = read16();
    t += (uint32_t) read16() << 16;
    return t;
}

static void headSerialResponse(uint8_t err, uint8_t s)
{
    serialize8('$');
    serialize8('M');
    serialize8(err ? '!' : '>');
    checksum = 0;               // start calculating a new checksum
    serialize8(s);
    serialize8(cmdMSP);
}

static void headSerialReply(uint8_t s)
{
    headSerialResponse(0, s);
}

static void headSerialError(uint8_t s)
{
    headSerialResponse(1, s);
}

static void tailSerialReply(void)
{
    serialize8(checksum);
}

static void serializeNames(const char *s)
{
    const char *c;
    for (c = s; *c; c++)
        serialize8(*c);
}

#define USERDEFINEVAR_MAX 4//用户自定义变量最大数量
struct DIY_Variables
{
	uint8_t Name[4];
	float * Var_Real;//真实变量的地址
	uint8_t Magnitude;//数量级	0，1,2,3等，其中，0为零位小数，1为一位小数，即0.1
}DIY_Var[USERDEFINEVAR_MAX];//={{.Name[0]='V'},{.Name[0]='V'},{.Name[0]='V'},{.Name[0]='V'}};
uint8_t DIY_VarCnt;//一共有多少个自定义变量
void UserDefineVariables_Creat(uint8_t name[4],float *var_real,uint8_t m)
{
	uint8_t i;
	if(DIY_VarCnt>=USERDEFINEVAR_MAX)return ;//检测是否已经达到最大创建数
	for(i=0;i<DIY_VarCnt;i++)
	{
		if(DIY_Var[i].Var_Real == var_real)return ;//检测是否已经存在了
	}
	//创建结构体数据
	for(i=0;i<4;i++)DIY_Var[DIY_VarCnt].Name[i] = name[i];
	DIY_Var[DIY_VarCnt].Var_Real = var_real;
	DIY_Var[DIY_VarCnt].Magnitude = m;
	DIY_VarCnt++;
}

static void evaluateCommand(void)
{
    uint32_t i;
    uint8_t wp_no;

    switch (cmdMSP) {
		case MSP_DIYVAR_READ://调参软件读自定义变量
				headSerialReply(USERDEFINEVAR_MAX * 6);
				for (i = 0; i < USERDEFINEVAR_MAX; i++)
				{
					serialize8(DIY_Var[i].Name[0]);
					serialize8(DIY_Var[i].Name[1]);
					serialize8(DIY_Var[i].Name[2]);
					serialize8(DIY_Var[i].Name[3]);
					serialize8((*DIY_Var[i].Var_Real)*pow(10,DIY_Var[i].Magnitude));
					serialize8(DIY_Var[i].Magnitude);
				}
				break;
		case MSP_DIYVAR_WRITE:
				for (i = 0; i < DIY_VarCnt; i++)
					*DIY_Var[i].Var_Real = (float)read8() / (float)pow(10,DIY_Var[i].Magnitude);
				for (i = DIY_VarCnt;i<USERDEFINEVAR_MAX;i++)read8();//将没用的读掉
				headSerialReply(0);
				break;
		
		case MSP_READ_ACCZERO:
				headSerialReply(20);
				serialize16(cfg.accZero[0]);
				serialize16(cfg.accZero[1]);
				serialize16(cfg.Vision_ErrorZero[0]);
				serialize16(cfg.Vision_ErrorZero[1]);
				serialize16(cfg.mixerConfiguration);
				serialize16(0);
				serialize16(0);
				serialize16(0);
				serialize16(0);
				serialize16(0);
				break;
		
		case MSP_SET_ACCZERO:
				cfg.accZero[0] = read16();
				cfg.accZero[1] = read16();
				cfg.Vision_ErrorZero[0] = read16();
				cfg.Vision_ErrorZero[1] = read16();
				cfg.mixerConfiguration = read16();//设置机型
				read16();
				read16();
				read16();
				read16();
				read16();
				headSerialReply(0);
				//WRITE_TO_FLASH;//将参数写入FLASH
				break;
		
    case MSP_SET_RAW_RC:
        for (i = 0; i < 8; i++);
            //RC_Data[i] = read16();
        headSerialReply(0);
        break;
    case MSP_SET_RAW_GPS:
        f.GPS_FIX = read8();
        GPS_numSat = read8();
        GPS_coord[LAT] = read32();
        GPS_coord[LON] = read32();
        GPS_altitude = read16();
        GPS_speed = read16();
        GPS_update |= 2;        // New data signalisation to GPS functions
        headSerialReply(0);
        break;
    case MSP_SET_PID:
        for (i = 0; i < PIDITEMS; i++) {
            cfg.P8[i] = read8();
            cfg.I8[i] = read8();
            cfg.D8[i] = read8();
        }
        headSerialReply(0);
        break;
    case MSP_SET_BOX:
        for (i = 0; i < CHECKBOXITEMS; i++)
            cfg.activate[i] = read16();
        headSerialReply(0);
        break;
    case MSP_SET_RC_TUNING:
        cfg.rcRate8 = read8();
        cfg.rcExpo8 = read8();
        cfg.rollPitchRate = read8();
        cfg.yawRate = read8();
        cfg.dynThrPID = read8();
        cfg.thrMid8 = read8();
        cfg.thrExpo8 = read8();
        headSerialReply(0);
        break;
    case MSP_SET_MISC:
        headSerialReply(0);
        break;
    case MSP_IDENT:
        headSerialReply(7);
        serialize8(VERSION);                // multiwii version
        serialize8(cfg.mixerConfiguration); // type of multicopter
        serialize8(MSP_VERSION);            // MultiWii Serial Protocol Version
        serialize32(PLATFORM_32BIT);        // "capability"
        break;
    case MSP_STATUS:
        headSerialReply(10);
        serialize16(CycleTime);
        serialize16(I2C_ErrorCount);//I2C错误数
        serialize16(Sensors(SENSOR_ACC) | Sensors(SENSOR_BARO) << 1 | Sensors(SENSOR_MAG) << 2 | Sensors(SENSOR_GPS) << 3 | Sensors(SENSOR_SONAR) << 4);
        serialize32(f.ACC_MODE << BOXACC | f.BARO_MODE << BOXBARO | f.MAG_MODE << BOXMAG | f.ARMED << BOXARM | RC_Options[BOXROLL] << BOXROLL | RC_Options[BOXLANDING] << BOXLANDING | 
                    f.GPS_HOME_MODE << BOXGPSHOME | f.GPS_HOLD_MODE << BOXGPSHOLD | f.HEADFREE_MODE << BOXHEADFREE | f.PASSTHRU_MODE << BOXPASSTHRU | 
                    RC_Options[BOXOPTFLOW] << BOXOPTFLOW | RC_Options[BOXVISION] << BOXVISION | RC_Options[BOXAVOID] << BOXAVOID | RC_Options[BOXHEADADJ] << BOXHEADADJ);
        break;
    case MSP_RAW_IMU:
        headSerialReply(18);
        for (i = 0; i < 3; i++)
            serialize16(accSmooth[i]);
        for (i = 0; i < 3; i++)
            serialize16(gyroData[i]);
        for (i = 0; i < 3; i++)
            serialize16(magADC[i]);
        break;
    case MSP_SERVO:
        headSerialReply(16);
        for (i = 0; i < 2; i++)
            serialize16(Servo_Val[i]);
						serialize16(1500);
						serialize16(1500);
						serialize16(1500);
						serialize16(1500);
						serialize16(1500);
						serialize16(1500);
        break;
    case MSP_MOTOR:
        headSerialReply(16);
					if(cfg.mixerConfiguration==MULTITYPE_QUADX)
					{
						serialize16(Motor_Val[3]);
						serialize16(Motor_Val[0]);
						serialize16(Motor_Val[2]);
						serialize16(Motor_Val[1]);
						serialize16(1000);
						serialize16(1000);
					}
					else//(cfg.mixerConfiguration==MULTITYPE_HEX6X)
					{
						serialize16(Motor_Val[4]);
						serialize16(Motor_Val[0]);
						serialize16(Motor_Val[3]);
						serialize16(Motor_Val[1]);
						serialize16(Motor_Val[5]);
						serialize16(Motor_Val[2]);
					}
					serialize16(1000);
					serialize16(1000);
//         for (i = 0; i < 8; i++)
//             serialize16(motor[i]);
        break;
    case MSP_RC:
        headSerialReply(16);
        for (i = 0; i < 8; i++)
            serialize16(RC_Data[i]);
        break;
    case MSP_RAW_GPS:
        headSerialReply(14);
        serialize8(f.GPS_FIX);
        serialize8(GPS_numSat);
        serialize32(GPS_coord[LAT]);
        serialize32(GPS_coord[LON]);
        serialize16(GPS_altitude);
        serialize16(GPS_speed);
        break;
    case MSP_COMP_GPS:
        headSerialReply(5);
        serialize16(GPS_distanceToHome);
        serialize16(GPS_directionToHome);
        serialize8(GPS_update & 1);
        break;
    case MSP_ATTITUDE:
        headSerialReply(8);
        for (i = 0; i < 2; i++)
            serialize16(angle[i]);
        serialize16(heading);
        serialize16(HeadFreeModeHold);
        break;
    case MSP_ALTITUDE:
        headSerialReply(4);
        serialize32(EstAlt_Baro);//EstAlt_BaroSonarPID); 		sonar.Distance		     	EstAlt_Baro
        break;
    case MSP_BAT:
        headSerialReply(3);
        serialize8(vbat);
        serialize16(EstAlt_Sonar); // power meter trash
        break;
    case MSP_RC_TUNING:
        headSerialReply(7);
        serialize8(cfg.rcRate8);
        serialize8(cfg.rcExpo8);
        serialize8(cfg.rollPitchRate);
        serialize8(cfg.yawRate);
        serialize8(cfg.dynThrPID);
        serialize8(cfg.thrMid8);
        serialize8(cfg.thrExpo8);
        break;
    case MSP_PID:
        headSerialReply(3 * PIDITEMS);
        for (i = 0; i < PIDITEMS; i++) {
            serialize8(cfg.P8[i]);
            serialize8(cfg.I8[i]);
            serialize8(cfg.D8[i]);
        }
        break;
    case MSP_BOX:
        headSerialReply(2 * CHECKBOXITEMS);
        for (i = 0; i < CHECKBOXITEMS; i++)
            serialize16(cfg.activate[i]);
        break;
    case MSP_BOXNAMES:
        headSerialReply(sizeof(boxnames) - 1);
        serializeNames(boxnames);
        break;
    case MSP_PIDNAMES:
        headSerialReply(sizeof(pidnames) - 1);
        serializeNames(pidnames);
        break;
    case MSP_MISC:
        headSerialReply(2);
        serialize16(0); // intPowerTrigger1
        break;
    case MSP_MOTOR_PINS:
        headSerialReply(8);
        for (i = 0; i < 8; i++)
            serialize8(i + 1);
        break;
    case MSP_WP:
        wp_no = read8();    // get the wp number
        headSerialReply(12);
        if (wp_no == 0) {
            serialize8(0);                   // wp0
            serialize32(GPS_home[LAT]);
            serialize32(GPS_home[LON]);
            serialize16(0);                  // altitude will come here
            serialize8(0);                   // nav flag will come here
        } else if (wp_no == 16) {
            serialize8(16);                  // wp16
            serialize32(GPS_hold[LAT]);
            serialize32(GPS_hold[LON]);
            serialize16(0);                  // altitude will come here
            serialize8(0);                   // nav flag will come here
        }
        break;
    case MSP_RESET_CONF:
        Flash_CheckFirstTime(true);
        headSerialReply(0);
        break;
    case MSP_ACC_CALIBRATION:
        CalibratingA = 400;
        headSerialReply(0);
        break;
    case MSP_MAG_CALIBRATION:
        f.CALIBRATE_MAG = 1;
        headSerialReply(0);
        break;
    case MSP_EEPROM_WRITE:
        WRITE_TO_FLASH;//将参数写入FLASH
        headSerialReply(0);
        break;
    case MSP_DEBUG:
        headSerialReply(8);
        for (i = 0; i < 4; i++)
            serialize16(debug[i]);      // 4 variables are here for general monitoring purpose
        break;
    default:                   // we do not know how to handle the (valid) message, indicate error MSP $M!
        headSerialError(0);
        break;
    }
    tailSerialReply();
		if (!(DMA2_Stream7->CR & 1))USART1_TxDMA();
}

// evaluate all other incoming serial data
static void evaluateOtherData(uint8_t sr)
{
    switch (sr) {
        case '#':
            //cliProcess();	//暂时不需要命令行功能
            break;
        case 'R':
            //systemReset(true);      // reboot to bootloader	 
            break;
    }
}

/*给外部调用，用于检测上位机是否发送数据给自己，然后进行交互*/
void Serial_Com(void)
{
	uint8_t c;
	static uint8_t offset;
	static uint8_t dataSize;
	static enum _serial_state
	{
		IDLE,
		HEADER_START,
		HEADER_M,
		HEADER_ARROW,
		HEADER_SIZE,
		HEADER_CMD,
	} c_state = IDLE;

	while (USART1_Available()) //判断是否接收到串口数据
	{
		c = USART1_Read();//读取来自上位机的数据

		if (c_state == IDLE) 
		{
			c_state = (c == '$') ? HEADER_START : IDLE;
			if (c_state == IDLE)
				evaluateOtherData(c); // evaluate all other incoming serial data
		}
		else if (c_state == HEADER_START) 
		{
			c_state = (c == 'M') ? HEADER_M : IDLE;
		} 
		else if (c_state == HEADER_M) 
		{
			c_state = (c == '<') ? HEADER_ARROW : IDLE;
		} 
		else if (c_state == HEADER_ARROW) 
		{
			if (c > INBUF_SIZE) 
			{       // now we are expecting the payload size
				c_state = IDLE;
				continue;
			}
			dataSize = c;
			offset = 0;
			checksum = 0;
			indRX = 0;
			checksum ^= c;
			c_state = HEADER_SIZE;      // the command is to follow
		}
		else if (c_state == HEADER_SIZE) 
		{
			cmdMSP = c;
			checksum ^= c;
			c_state = HEADER_CMD;
		}
		else if (c_state == HEADER_CMD && offset < dataSize) 
		{
			checksum ^= c;
			inBuf[offset++] = c;
		}
		else if (c_state == HEADER_CMD && offset >= dataSize) 
		{
			if (checksum == c)        // compare calculated and transferred checksum
				evaluateCommand();      // we got a valid packet, evaluate it
			c_state = IDLE;
		}
	}
}
