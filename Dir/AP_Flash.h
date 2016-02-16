#ifndef __AP_FLASH_H
#define __AP_FLASH_H
#include "includes.h"

typedef struct config_t {
    uint8_t version;
    uint8_t mixerConfiguration;
    uint32_t enabledFeatures;

    uint16_t looptime;                      // imu loop time in us

    uint8_t P8[PIDITEMS];
    uint8_t I8[PIDITEMS];
    uint8_t D8[PIDITEMS];

    uint8_t rcRate8;
    uint8_t rcExpo8;
    uint8_t thrMid8;
    uint8_t thrExpo8;

    uint8_t rollPitchRate;
    uint8_t yawRate;

    uint8_t dynThrPID;
    int16_t accZero[3];		//加速度计0点值，校准时得到的
    int16_t magZero[3];
    int16_t mag_declination;                // Get your magnetic decliniation from here : http://magnetic-declination.com/
    int16_t angleTrim[2];                   // accelerometer trim

    // sensor-related stuff
    uint8_t acc_hardware;                   // Which acc hardware to use on boards with more than one device
    uint8_t acc_lpf_factor;                 // Set the Low Pass Filter factor for ACC. Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time. Zero = no filter
    uint16_t gyro_lpf;                      // mpuX050 LPF setting
    uint16_t gyro_cmpf_factor;              // Set the Gyro Weight for Gyro/Acc complementary filter. Increasing this value would reduce and delay Acc influence on the output of the filter.
    uint32_t gyro_smoothing_factor;         // How much to smoothen with per axis (32bit value with Roll, Pitch, Yaw in bits 24, 16, 8 respectively
    uint8_t mpu6050_scale;                  // seems es/non-es variance between MPU6050 sensors, half my boards are mpu6000ES, need this to be dynamic. fucking invenshit won't release chip IDs so I can't autodetect it.

    uint16_t activate[CHECKBOXITEMS];       // activate switches
    uint8_t vbatscale;                      // adjust this to match battery voltage to reported value
    uint8_t vbatmaxcellvoltage;             // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    uint8_t vbatmincellvoltage;             // minimum voltage per cell, this triggers battery out alarms, in 0.1V units, default is 33 (3.3V)

    // Radio/ESC-related configuration
    uint8_t rcmap[8];                       // mapping of radio channels to internal RPYTA+ order
    uint8_t deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t yawdeadband;                    // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
    uint8_t alt_hold_throttle_neutral;      // defines the neutral zone of throttle stick during altitude hold, default setting is +/-20
    uint8_t spektrum_hires;                 // spektrum high-resolution y/n (1024/2048bit)
    uint16_t midrc;                         // 将一些需要取中间值的遥控接收信号设置为1500
    uint16_t mincheck;                      // minimum rc end
    uint16_t maxcheck;                      // maximum rc end
    uint8_t retarded_arm;                   // allow disarsm/arm on throttle down + roll left/right
    
    // Failsafe related configuration
    uint8_t failsafe_delay;                 // 失去遥控信号后多少秒开启失效保护，单位基准为0.1s，假如是1秒后，则这个值为10
    uint8_t failsafe_off_delay;             // 失效保护开启后，多长时间后关闭电机，假设现在已经着陆了,单位时基为0.1s
    uint16_t failsafe_throttle;             // 用于失效保护时的特殊油门值Throttle level used for landing - specify value between 1000..2000 (pwm pulse width for slightly below hover). center throttle = 1500.

    // motor/esc/servo related stuff
    uint16_t minthrottle;                   // 电机最小PWM值，电机以低速转Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
    uint16_t maxthrottle;                   // 电机最大转速值This is the maximum value for the ESCs at full power this value can be increased up to 2000
    uint16_t mincommand;                    // 此值赋值给电机，可让其停转，且不会报没信号This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs

		int16_t Vision_ErrorZero[2];           // 视觉补偿,cfg.Vision_ErrorZero[axis]/100,放大100倍
		
		int16_t servotrim[8];                   // Adjust Servo MID Offset & Swash angles
    int8_t servoreverse[8];                 // Invert servos by setting -1

    // mixer-related configuration
    int8_t yaw_direction;
    uint16_t tri_yaw_middle;                // tail servo center pos. - use this for initial trim
    uint16_t tri_yaw_min;                   // tail servo min
    uint16_t tri_yaw_max;                   // tail servo max

    // gimbal-related configuration
    int8_t gimbal_pitch_gain;               // gimbal pitch servo gain (tied to angle) can be negative to invert movement
    int8_t gimbal_roll_gain;                // gimbal roll servo gain (tied to angle) can be negative to invert movement
    uint8_t gimbal_flags;                   // in servotilt mode, various things that affect stuff
    uint16_t gimbal_pitch_min;              // gimbal pitch servo min travel
    uint16_t gimbal_pitch_max;              // gimbal pitch servo max travel
    uint16_t gimbal_pitch_mid;              // gimbal pitch servo neutral value
    uint16_t gimbal_roll_min;               // gimbal roll servo min travel
    uint16_t gimbal_roll_max;               // gimbal roll servo max travel
    uint16_t gimbal_roll_mid;               // gimbal roll servo neutral value

    // gps-related stuff
    uint32_t gps_baudrate;
    uint16_t gps_wp_radius;                 // if we are within this distance to a waypoint then we consider it reached (distance is in cm)
    uint8_t gps_lpf;                        // Low pass filter cut frequency for derivative calculation (default 20Hz)
    uint8_t nav_slew_rate;                  // Adds a rate control to nav output, will smoothen out nav angle spikes
    uint8_t nav_controls_heading;           // copter faces toward the navigation point, maghold must be enabled for it
    uint16_t nav_speed_min;                 // cm/sec
    uint16_t nav_speed_max;                 // cm/sec

   // serial(uart1) baudrate
    uint32_t serial_baudrate;
} config_t;		 //配置用

extern config_t cfg;
extern int16_t LookupPitchRollRC[6];   // lookup table for expo & RC rate PITCH+ROLL
extern int16_t LookupThrottleRC[11];   // lookup table for expo & mid THROTTLE

void Flash_CheckFirstTime(bool reset);
void Flash_WriteParams(uint8_t b);
void Flash_Read(void);

bool Sensors(uint32_t mask);
void SensorsSet(uint32_t mask);
void SensorsClear(uint32_t mask);
uint32_t sensorsMask(void);
bool Feature(uint32_t mask);
void FeatureSet(uint32_t mask);
void FeatureClear(uint32_t mask);
void FeatureClearAll(void);
uint32_t FeatureMask(void);

#endif
