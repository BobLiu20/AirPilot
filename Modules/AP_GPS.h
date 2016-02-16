#ifndef __GPS_H
#define __GPS_H
#include "includes.h"

#define WaypointNum_Max	 20
extern bool GPS_RX_Succ;	   //��־���ݸ���һ�γɹ�����Ҫ�������ݴ���
extern int32_t Waypoint[WaypointNum_Max+1][2];	  //�ຽ�����걣��
extern u8 GS_Nav;					  //����վ���µ�������
extern u8 ReadyForNextWP;
extern u8 ReadyToMove;
extern uint8_t wp;
extern int16_t nav_takeoff_bearing;
extern int16_t NAV_Speed;
extern uint8_t AutoHeading;
extern uint8_t GPS_Speed_Mode;
extern int32_t GPS_target_speed[2];
extern int16_t actual_speed[2];
extern uint8_t GPS_numSat;		//��������������Ŀ
extern int32_t GPS_coord[2];			//���澭γ������,ԭʼ����
extern int32_t GPS_home[2];			//����GPS HOME�ľ�γ��
extern int32_t GPS_hold[2];			//����GPS��ͣ������
extern int8_t nav_mode;    //������ģʽѡ��NAV_MODE_WP=2��NAV_MODE_POSHOLD=1��
extern int16_t nav_rated[2];               // Adding a rate controller to the navigation to make it smoother
extern int16_t nav[2];
extern int16_t GPS_angle[2];    //GPS���ռ���õ�PID����
extern uint16_t GPS_altitude, GPS_speed;       // ���θ߶ȣ���λ���ף����ٶȣ���λCM/S��
extern uint8_t GPS_update;             // it's a binary toogle to distinct a GPS position update
extern uint16_t GPS_distanceToHome;                            // ��ԭ��ľ��루�ף�
extern int16_t GPS_directionToHome;                            // ��ԭ��ĽǶȣ��ȣ�

void GPS_Init(uint32_t Baudrate);
void GPS_reset_home_position(void);
void GPS_reset_nav(void);
void GPS_set_next_wp(int32_t* lat, int32_t* lon);
int32_t wrap_18000(int32_t error);


//extern int32_t GPS_coord[2];	//���澭γ�����꣬
//extern uint8_t GPS_numSat;		//��������������Ŀ
//extern uint16_t GPS_altitude, GPS_speed;       // ���θ߶ȣ���λ���ף����ٶȣ���λCM/S��

typedef struct {
    float _integrator;          ///< integrator value
    int32_t _last_input;        ///< last input for derivative
    float _last_derivative;     ///< last derivative for low-pass filter
    float _output;
    float _derivative;
} AC_PID;

// typedef struct {
//     float kP;
//     float kI;
//     float kD;
//     float Imax;
// } PID_PARAM;

// Serial GPS only variables
// navigation mode
typedef enum NavigationMode
{
    NAV_MODE_NONE = 0,
    NAV_MODE_POSHOLD,
    NAV_MODE_WP
} NavigationMode;

#define LAT  0		 //γ��
#define LON  1		 //����

#define GPS_FILTER_VECTOR_LENGTH 3
#define FRAME_GGA  1		  //���������ʽ
#define FRAME_RMC  2
#define NAV_TAIL_FIRST             0    // true - copter comes in with tail first
#define NAV_SET_TAKEOFF_HEADING    1    // true - when copter arrives to home position it rotates it's head to takeoff direction


#endif
