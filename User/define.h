#ifndef __DEFINE_H
#define __DEFINE_H

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
//#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define M_PI       3.14159265358979323846f

//�õ�ʱ����dt,����궨�������ں�����ʼ����Ϊ�б����Ķ���
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

//����BOX��������rcOptions[]����ģ���rcOptions[BOXACC]�������Ϊ1����ʾ����ACC HOLDģʽ������
//rcOptions��ͨ������ͨ������

#define BOXACC       0	//�Ƿ��յ�ң���������������ģʽ
#define BOXBARO      1
#define BOXMAG       2
#define BOXROLL		   3
#define BOXLANDING   4
#define BOXARM       5
#define BOXGPSHOME   6	//ԭ�㱣��ģʽ
#define BOXGPSHOLD   7	//�趨�ص㱣��
#define BOXPASSTHRU  8	//������ڿ��м��ٶ�У׼��һ�ַ���
#define BOXHEADFREE  9	//��ͷģʽ
#define BOXOPTFLOW	 10	//����ģʽ
#define BOXVISION    11 //�Ӿ��Զ�����
#define BOXAVOID	   12 //ͨ�õΣ���Ҷ�������������
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
	uint8_t OK_TO_ARM;//Ϊ1ʱ�������
	uint8_t ARMED;		//Ϊ1ʱ���ڽ���״̬
	uint8_t ACC_CALIBRATED;//���ٶ��Ƿ�У׼��
	uint8_t ACC_MODE;	//����
	uint8_t MAG_MODE;	//���򣬷���Ϊ����ʱ�ķ����ڹر�ǰ�޷��ı�
	uint8_t BARO_MODE;//����
	uint8_t GPS_HOME_MODE;//GO HOME
	uint8_t GPS_HOLD_MODE;//HOLD
	uint8_t HEADFREE_MODE;//��ͷģʽ
	uint8_t PASSTHRU_MODE;
	uint8_t GPS_FIX;			//���GPS�õ��Ķ�λ�����Ƿ���Ч��1Ϊ��Ч
	uint8_t GPS_FIX_HOME; //�Ƿ����HOME��
	uint8_t SMALL_ANGLES_25;//��ǳ���25��
	uint8_t CALIBRATE_MAG;
	uint8_t SONAR_MODE;		//��������
	uint8_t SONAR_AVAILABLE;//���������Ҳ����Ч
	uint8_t RC_CTRL_MODE; //1Ϊң��������ģʽ
	uint8_t RC_AVAILABLE;//1Ϊң�������ڣ�����Ч��
	uint8_t HANDLE_CTRL_MODE;//1Ϊ�ֱ�����ģʽ
	uint8_t HANDLE_AVAILABLE;//1Ϊ�ֱ����ڣ�����Ч��
	uint8_t AUTOLANDING;	//��Ϊ1ʱ�����ڽ����Զ��������
	uint8_t AUTOTAKEOFF;	//��Ϊ1ʱ�����ڽ����Զ���ɹ���
	uint8_t OPTFLOW_MODE;//����ģʽ
	uint8_t OPTFLOW_AVAILABLE;//������Ч���ߴ��ڷ�
	uint8_t VISION_MODE;//�Ӿ��Զ�����ģʽ
	uint8_t VISION_AVAILABLE;//�Ӿ�ģ���Ƿ���ڻ�����Ч
	uint8_t SONARAVOID_MODE;		//��������
	uint8_t SONARAVOID_AVAILABLE[4];//���������Ҳ����Ч
} flags_t;		//��־λ


/*
*********************************************************************************************************
*                                         ���ڴ�������
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
