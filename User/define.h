#ifndef __DEFINE_H
#define __DEFINE_H

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
//#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define M_PI       3.14159265358979323846f

//得到时间间隔dt,这个宏定义必须放在函数开始，因为有变量的定义
#define dt_CREAT_S 			\
{\
static uint32_t pre_t=0;\
uint32_t now_t;\
float dt;\
now_t = TimUsGet();\
dt = (now_t - pre_t) * 0.000001f;\
pre_t = now_t;\
}

/*********** RC alias *****************/
#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3
#define AUX1       4
#define AUX2       5
#define AUX3       6
#define AUX4       7

#define PIDALT     3
#define PIDPOS     4
#define PIDPOSR    5
#define PIDNAVR    6
#define PIDLEVEL   7
#define PIDMAG     8
#define PIDVEL     9 // not used currently

//以下BOX都是用于rcOptions[]数组的，如rcOptions[BOXACC]，如果其为1，表示进入ACC HOLD模式，自稳
//rcOptions可通过辅助通道控制

#define BOXACC       0	//是否收到遥控器命令进入自稳模式
#define BOXBARO      1
#define BOXMAG       2
#define BOXROLL		   3
#define BOXLANDING   4
#define BOXARM       5
#define BOXGPSHOME   6	//原点保持模式
#define BOXGPSHOLD   7	//设定地点保持
#define BOXPASSTHRU  8	//这个用于空中加速度校准的一种方法
#define BOXHEADFREE  9	//无头模式
#define BOXOPTFLOW	 10	//光流模式
#define BOXVISION    11 //视觉自动降落
#define BOXAVOID	   12 //通用滴，大家都可以用来调试
#define BOXHEADADJ   13 // acquire heading for HEADFREE mode


#define PIDITEMS 10
#define CHECKBOXITEMS 14

typedef enum MultiType
{
    MULTITYPE_TRI = 1,
    MULTITYPE_QUADP = 2,
    MULTITYPE_QUADX = 3,
    MULTITYPE_BI = 4,
    MULTITYPE_GIMBAL = 5,
    MULTITYPE_Y6 = 6,
    MULTITYPE_HEX6 = 7,
    MULTITYPE_FLYING_WING = 8,      // UNSUPPORTED, do not select!
    MULTITYPE_Y4 = 9,
    MULTITYPE_HEX6X = 10,
    MULTITYPE_OCTOX8 = 11,          // Java GUI is same for the next 3 configs
    MULTITYPE_OCTOFLATP = 12,       // MultiWinGui shows this differently
    MULTITYPE_OCTOFLATX = 13,       // MultiWinGui shows this differently
    MULTITYPE_AIRPLANE = 14,        // airplane / singlecopter / dualcopter
    MULTITYPE_HELI_120_CCPM = 15,
    MULTITYPE_HELI_90_DEG = 16,
    MULTITYPE_VTAIL4 = 17,
    MULTITYPE_CUSTOM = 18,          // no current GUI displays this
    MULTITYPE_LAST = 19
} MultiType;


// Type of accelerometer used/detected
typedef enum AccelSensors {
    ACC_DEFAULT = 0,
    ACC_ADXL345 = 1,
    ACC_MPU6050 = 2,
    ACC_MMA8452 = 3,
} AccelSensors;

typedef enum {
    FEATURE_PPM = 1 << 0,
    FEATURE_VBAT = 1 << 1,
    FEATURE_INFLIGHT_ACC_CAL = 1 << 2,
    FEATURE_SPEKTRUM = 1 << 3,
    FEATURE_MOTOR_STOP = 1 << 4,
    FEATURE_SERVO_TILT = 1 << 5,
    FEATURE_GYRO_SMOOTHING = 1 << 6,
    FEATURE_LED_RING = 1 << 7,
    FEATURE_GPS = 1 << 8,
    FEATURE_FAILSAFE = 1 << 9,
    FEATURE_SONAR = 1 << 10,
    FEATURE_TELEMETRY = 1 << 11,
} AvailableFeatures;


typedef enum GimbalFlags {
    GIMBAL_NORMAL = 1 << 0,
    GIMBAL_TILTONLY = 1 << 1,
    GIMBAL_DISABLEAUX34 = 1 << 2,
    GIMBAL_FORWARDAUX = 1 << 3,
} GimbalFlags;


typedef enum {
    SENSOR_ACC = 1 << 0,
    SENSOR_BARO = 1 << 1,
    SENSOR_MAG = 1 << 2,
    SENSOR_SONAR = 1 << 3,
    SENSOR_GPS = 1 << 4,
} AvailableSensors;

typedef struct flags_t {
	uint8_t OK_TO_ARM;//为1时允许解锁
	uint8_t ARMED;		//为1时处于解锁状态
	uint8_t ACC_CALIBRATED;//加速度是否校准过
	uint8_t ACC_MODE;	//自稳
	uint8_t MAG_MODE;	//定向，方向定为开启时的方向，在关闭前无法改变
	uint8_t BARO_MODE;//定高
	uint8_t GPS_HOME_MODE;//GO HOME
	uint8_t GPS_HOLD_MODE;//HOLD
	uint8_t HEADFREE_MODE;//无头模式
	uint8_t PASSTHRU_MODE;
	uint8_t GPS_FIX;			//标记GPS得到的定位数据是否有效，1为有效
	uint8_t GPS_FIX_HOME; //是否存在HOME了
	uint8_t SMALL_ANGLES_25;//倾角超过25度
	uint8_t CALIBRATE_MAG;
	uint8_t SONAR_MODE;		//超声定高
	uint8_t SONAR_AVAILABLE;//超声存在且测距有效
	uint8_t RC_CTRL_MODE; //1为遥控器控制模式
	uint8_t RC_AVAILABLE;//1为遥控器存在，即有效的
	uint8_t HANDLE_CTRL_MODE;//1为手柄控制模式
	uint8_t HANDLE_AVAILABLE;//1为手柄存在，即有效的
	uint8_t AUTOLANDING;	//当为1时，正在进行自动降落过程
	uint8_t AUTOTAKEOFF;	//当为1时，正在进行自动起飞过程
	uint8_t OPTFLOW_MODE;//光流模式
	uint8_t OPTFLOW_AVAILABLE;//光流有效或者存在否
	uint8_t VISION_MODE;//视觉自动降落模式
	uint8_t VISION_AVAILABLE;//视觉模块是否存在或者有效
	uint8_t SONARAVOID_MODE;		//超声定高
	uint8_t SONARAVOID_AVAILABLE[4];//超声存在且测距有效
} flags_t;		//标志位


/*
*********************************************************************************************************
*                                         关于传感器的
*********************************************************************************************************
*/
typedef void (* sensorInitFuncPtr)(void);                   // sensor init prototype
typedef void (* sensorReadFuncPtr)(int16_t *data);          // sensor read and align prototype
typedef int32_t (* baroCalculateFuncPtr)(void);             // baro calculation (returns altitude in cm based on static data collected)
typedef void (* uartReceiveCallbackPtr)(uint16_t data);     // used by uart2 driver to return frames to app
typedef uint16_t (* rcReadRawDataPtr)(uint8_t chan);        // used by receiver driver to return channel data

typedef struct sensor_t
{
    sensorInitFuncPtr init;
    sensorReadFuncPtr read;
    sensorReadFuncPtr align;
} sensor_t;

typedef struct baro_t
{
    uint16_t ut_delay;
    uint16_t up_delay;
    uint16_t repeat_delay;
    sensorInitFuncPtr start_ut;
    sensorInitFuncPtr get_ut;
    sensorInitFuncPtr start_up;
    sensorInitFuncPtr get_up;
    baroCalculateFuncPtr calculate;
} baro_t;







#endif
