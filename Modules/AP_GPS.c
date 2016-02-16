/**
	******************************************************************************
	* @file    AP_GPS.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   ABOUT GPS
	******************************************************************************
**/
#include "AP_GPS.h"
#include <ctype.h>
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
u8 GS_Nav;					  		//����վ���µ�������
u8 ReadyForNextWP;			  //�Ƿ�ȥ��һ��Ŀ���
u8 ReadyToMove;
int32_t Waypoint[WaypointNum_Max+1][2];	  //�ຽ�����걣��,�������������WaypointNum_Max�������
int32_t GPS_coord[2];			//���澭γ������,ԭʼ����
int32_t GPS_home[2];			//����GPS HOME�ľ�γ��
int32_t GPS_hold[2];			//����GPS��ͣ������
int16_t GPS_angle[2] = { 0, 0 };    //GPS���ռ���õ�PID����
int32_t GPS_target_speed[2];

static int32_t GPS_WP[2];
uint8_t GPS_numSat;		//��������������Ŀ
uint16_t GPS_altitude, GPS_speed;       // ���θ߶ȣ���λ���ף����ٶȣ���λCM/S��
uint16_t GPS_distanceToHome;                            // ��ԭ��ľ��루�ף�
int16_t GPS_directionToHome;                            // ��ԭ��ĽǶȣ��ȣ�
uint8_t GPS_update = 0;             // it's a binary toogle to distinct a GPS position update
int16_t nav[2];
static int32_t target_bearing;	//���ڷɻ������Ҫȥ�ĵص�ķ�λ�Ƕȣ���x100��
static uint32_t wp_distance;	//�ɻ�����һ��Ҫȥ�ĵ�ľ���cm
int8_t nav_mode = NAV_MODE_NONE;    //������ģʽѡ��NAV_MODE_WP=2��NAV_MODE_POSHOLD=1��
static int32_t original_target_bearing;//��¼�ɵ�Ŀ�귽λ
static int32_t nav_bearing;	//�ӷɻ�����һ��Ŀ��ķ�λ�ǣ���x100��
int16_t nav_takeoff_bearing;//�������ʱ�ķ����Ա�ﵽĿ��Ļָ��ɻ�ͷ�ķ���
static int16_t waypoint_speed_gov;
int16_t nav_rated[2];               // Adding a rate controller to the navigation to make it smoother
uint8_t wp=0;
int16_t NAV_Speed = 200;
uint8_t AutoHeading = 1;		//Ѳ��ʱ�ĺ������ã��Զ�����Ϊ1���ֶ�����Ϊ0
uint8_t GPS_Speed_Mode = 0;

/********************PID��****************/
static AC_PID pi_poshold[2];		//
static AC_PID pid_poshold_rate[2];	//
static AC_PID pid_nav[2];			//
static AC_PID pid_nav_crosstrack; 
static PID_PARAM posholdPID;		//
static PID_PARAM poshold_ratePID;	//
static PID_PARAM navPID;
static PID_PARAM nav_crosstrackPID;
static int16_t rate_error[2];
static int32_t error[2];
#define RADX100                    0.000174532925f
#define CROSSTRACK_GAIN            2
#define NAV_SLOW_NAV               true	//����ģʽ��
#define NAV_BANK_MAX               3000 // 30deg max banking when navigating (just for security and testing)
static float dTnav;             //��¼���ε�������ط���ʱ�䣬��λs
int16_t actual_speed[2] = { 0, 0 };//��¼ʵ���ٶ�
static float GPS_scaleLonDown;  //���ھ����ڿ�����������ʱ������������С
static int16_t crosstrack_error;//���нǽ���

/***************�˲�ר��*****************/
static uint8_t GPS_filter_index = 0;
static int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
static int32_t GPS_filter_sum[2];
static int32_t GPS_read[2];
static int32_t GPS_filtered[2];
static int32_t GPS_degree[2];   //the lat lon degree without any decimals (lat/10 000 000)
static uint16_t fraction3[2];

/* OS -----------------------------------------------*/
static  void  GPS_MainTask (void *p_arg);
static  OS_TCB  GPS_MainTaskTCB;
static  CPU_STK  GPS_MainTaskStk[APP_CFG_TASK_START_STK_SIZE];

/* function prototypes -----------------------------------------------*/
static u32 Grab_Fields(char *src,u8 mult);
u32 GPS_Coord_to_Degrees(char *s);
static uint8_t hex_c(uint8_t n);
static void GPS_distance_cm_bearing(int32_t * lat1, int32_t * lon1, int32_t * lat2, int32_t * lon2, uint32_t * dist, int32_t * bearing);
static int32_t GPS_BearingForNextwp(int32_t * lat1, int32_t * lon1, int32_t * lat2, int32_t * lon2);
static void GPS_calc_longitude_scaling(int32_t lat);
static void GPS_calc_velocity(void);
static void GPS_calc_location_error(int32_t * target_lat, int32_t * target_lng, int32_t * gps_lat, int32_t * gps_lng);
static void GPS_calc_poshold(void);
static int16_t GPS_calc_desired_speed(int16_t max_speed, bool _slow);
static void GPS_update_crosstrack(void);
static void GPS_calc_nav_rate(int max_speed);
static bool check_missed_wp(void);
static int32_t AC_PID_get_p(AC_PID * ac_pid, int32_t error, PID_PARAM * pid);
static int32_t AC_PID_get_i(AC_PID * ac_pid, int32_t error, float *dt, PID_PARAM * pid);
static int32_t AC_PID_get_d(AC_PID * ac_pid, int32_t input, float *dt, PID_PARAM * pid);
static float AC_PID_get_integrator(AC_PID * ac_pid);
static void AC_PID_set_integrator(AC_PID * ac_pid, float i);
static bool GPS_NewFrame(char c);
static void Set_nextwp(void);				 //���غ�����ɱ�־λ�����Ϊ1
static void GPS_Set_PID(void);
static void GPS_NewData(void);

/************************
�������ܣ�GPS��ʼ��
������������ڲ�����
************************/
void GPS_Init(uint32_t baudrate)
{
	OS_ERR	err;
	GPS_Set_PID();
	USART2_Init(baudrate);	//��ʼ������2
	
	OSTaskCreate((OS_TCB   *)&GPS_MainTaskTCB,  
				 (CPU_CHAR     *)"GPS_MainTask",
				 (OS_TASK_PTR   )GPS_MainTask, 
				 (void         *)0,
				 (OS_PRIO       )PRIO_GPS ,
				 (CPU_STK      *)&GPS_MainTaskStk[0],
				 (CPU_STK_SIZE  )GPS_MainTaskStk[APP_CFG_TASK_START_STK_SIZE / 10],
				 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
				 (OS_MSG_QTY    )0,
				 (OS_TICK       )0,
				 (void         *)0,
				 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 (OS_ERR       *)&err);
}


/**
  * @brief  GPS������
  *         
  * @param  None
  * @retval None
  */
static  void  GPS_MainTask (void *p_arg)
{
	OS_ERR	err;
	CPU_TS ts;
  (void)p_arg;
  while (1)
	{
		OSTimeDly(10,OS_OPT_TIME_DLY,&err);//ms
		if(!USART2_RXAVAILABLE)continue; //û�н��յ����ݣ�������ȴ�
		OSTimeDly(1,OS_OPT_TIME_DLY,&err);//��΢��ʱһ�£��ȴ����ݽ�����ȫ
		while(USART2_RXAVAILABLE)//ֻҪ����Ч���ݾ�һֱ����
		{
			if(GPS_NewFrame(USART2_Read()))//�����������˵������һ֡������
			{
				GPS_NewData();//GPS����
				USART2_RXCLEAR;//�����������
			}
		}
  }
}

/***********************
�������ܣ�GPS�������ݴ���,�����ַ���
������������ڽ��յ�һ�ֽ�����
***********************/
static bool GPS_NewFrame(char c)
{
	u8 frameOK = 0;			//���ݸ����Ƿ����
	static u8 param = 0;	//��һ֡�е���һ�Σ�0��Ϊ������
	static u8 offset = 0;	//�ַ��������õ�
	static u8 parity = 0;	//У�����¼
	static char string[15];	//��ʱ����ÿһ����Ҫ�������ַ���
	static u8 checksum_param;//У��
	static u8 frame = 0;	//��һ�������ָ�ʽ�ģ������õ�GPGGA��GPRMC��ʽ
	if(c == '$')	//$����Ϊһ֡���ݵ���ʼ����
	{
		param = 0;	//һ֡��ʼ����ʼ����Щ����
		offset = 0;
		parity = 0;
	}
	else if(c == ',' || c == '*')//����Ϊһ���ַ���β���Ǻű�ʾ�������������У�����
	{
		string[offset] = 0;	//����ʱ�ַ������һλΪ0����,���0��ascii��0��
		if(param == 0)		//��0�δ������������ݸ�ʽ
		{
			frame = 0;
			if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A')
				frame = FRAME_GGA;	//GPGGA���ݸ�ʽ
			if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C')
				frame = FRAME_RMC; 	//GPRMC���ݸ�ʽ
		}
		else if(frame == FRAME_GGA)	//����GPGGA���ݸ�ʽ����
		{
			if(param == 2)	//γ��
			{
				GPS_coord[LAT] = GPS_Coord_to_Degrees(string);//��γ��ת��Ϊddmm.mmmmm���ȷָ�ʽ
			}
			else if (param == 3 && string[0] == 'S')		//�ж��Ƿ�����γ
         GPS_coord[LAT] = -GPS_coord[LAT];	//��γ����,��γ�Ǹ�
      else if (param == 4) 					//����
			{
       	GPS_coord[LON] = GPS_Coord_to_Degrees(string);//������ת��Ϊddmm.mmmmm���ȷָ�ʽ
      } 
			else if (param == 5 && string[0] == 'W')		//�ж��Ƿ�������
        GPS_coord[LON] = -GPS_coord[LON]; //�����ø���ʾ
      else if (param == 6)				//��λ������0Ϊ��Ч��λ��1Ϊ��Ч
			{
        f.GPS_FIX = string[0] > '0';	//string[0] > '0'������˵������Ч�����GPS_FIX
      }
			else if (param == 7) 			//ʹ�õ�������Ŀ
			{
	      GPS_numSat = Grab_Fields(string, 0);//���ַ�ת��Ϊ����,������Ŀ
	    }
			else if (param == 9) 			//���θ߶�
			{
	      GPS_altitude = Grab_Fields(string, 1);  //��λ�Ƿ���
			}
    }
		else if(frame == FRAME_RMC)	//�������ȡ�ٶ�
		{
			if (param == 7) //�ٶȣ�׼ȷ˵�����ʺǺǣ�
			{
        GPS_speed = ((uint32_t) Grab_Fields(string, 1) * 5144L) / 1000L;    // ���㵥λΪCM/S���ٶ�
      }
		}
		param++;		//�ַ��μ�1
		offset = 0;		//��ʱ�����ַ�����0
		if (c == '*')	//�ж��Ƿ�У������
			checksum_param = 1;//��ǵ���������
		else
			parity ^= c;	//��żУ�飬$��*֮���asciiλ���ֵ��Ҫ����У����
	}
	else if(c == '\r' || c == '\n')
	{
		if(checksum_param)	//����У��
		{
			u8 checksum = hex_c(string[0]);
			checksum <<= 4;
			checksum += hex_c(string[1]);
			if (checksum == parity)	//��ȣ���������ȷ����
				frameOK = 1;
    }
    checksum_param = 0;
	}
	else
	{
		if (offset < 15)
      string[offset++] = c;//�����ַ�
    if (!checksum_param)
      parity ^= c;
	}
	//if(frame)
	//	GPS_Present = 1;
	return frameOK && (frame == FRAME_GGA);	//������ȫ���³ɹ�һ�Σ�����1
}

/***********************
�������ܣ�GPS���ݸ���,���ɴ����жϵ��ã���GPS��ʼ��ʱ������
***********************/
static void GPS_NewData(void)
{
	int axis;
	static uint32_t nav_loopTimer;	//��¼��һ�ε�ѭ����ʱ���
	uint32_t dist;	//����
  int32_t dir;	//����
  int16_t speed;	//�ٶ�
	SensorsSet(SENSOR_GPS);	//��λGPS���ڱ�־
	if(GPS_update == 1)
		GPS_update = 0;
	else
		GPS_update = 1;
	if(f.GPS_FIX && GPS_numSat >= 5)//���������Ч������������5
	{
		if(!f.ARMED)	
		{
			f.GPS_FIX_HOME = 0;//��û�н���״̬�£����GPS HOME��
		}
		if(!f.GPS_FIX_HOME && f.ARMED)//��û��GPSԭ�����Ѿ������ˣ�������ԭ��
		{
			f.GPS_FIX_HOME = 1;//�����ԭ��
			GPS_home[LAT] = GPS_coord[LAT];//����ǰ��γ�ȼ�¼��
			GPS_home[LON] = GPS_coord[LON];
			GPS_calc_longitude_scaling(GPS_coord[LAT]); //�õ�һ����ʼֵ���ڼ������ͷ�λ
      nav_takeoff_bearing = heading; // save takeoff heading
			XbeePro_ImportanceInfo = XbeePro_ImportanceInfo | 0x10;//����HOME�����λ��
		}
		//�����˲���������Ի����ƽ���˲�
		GPS_filter_index = (GPS_filter_index + 1) % GPS_FILTER_VECTOR_LENGTH;
    for (axis = 0; axis < 2; axis++) //axis=0ʱΪγ�ȣ�=1ʱΪ����
		{
			GPS_read[axis] = GPS_coord[axis];       //�õ����µ�δ���˵ľ�γ��
      GPS_degree[axis] = GPS_read[axis] / 10000000;   //�õ���γ�ȵĶ�

			// How close we are to a degree line ? its the first three digits from the fractions of degree
			// later we use it to Check if we are close to a degree line, if yes, disable averaging,
			fraction3[axis] = (GPS_read[axis] - GPS_degree[axis] * 10000000) / 10000;//�õ�С�������λ�ķ�,��0.xxx�ȵ�xxx

			GPS_filter_sum[axis] -= GPS_filter[axis][GPS_filter_index];
			GPS_filter[axis][GPS_filter_index] = GPS_read[axis] - (GPS_degree[axis] * 10000000);
			GPS_filter_sum[axis] += GPS_filter[axis][GPS_filter_index];
			GPS_filtered[axis] = GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GPS_degree[axis] * 10000000);
			if (nav_mode == NAV_MODE_POSHOLD) //����ֻ��posholdģʽ��ʹ��GPSƽ��ֵ		nav_mode == NAV_MODE_POSHOLD
			{
				if (fraction3[axis] > 1 && fraction3[axis] < 999)
					GPS_coord[axis] = GPS_filtered[axis];	//���˲����ֵ����
      }
    }
		//������� x,y ���ٶȡ����� ��PID��
		dTnav = (float) (TimMsGet() - nav_loopTimer) / 1000.0f;//��������֮���ʱ���	  ���ڵ�λΪs
		nav_loopTimer = TimMsGet();		//��¼ʱ���  ��λΪms
		dTnav = min(dTnav, 1.0f);		//��ֹGPS������������ʱ���(������1S)
		//������롢��λ��GUI����ԭ�㵽�һ���
		GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_home[LAT], &GPS_home[LON], &dist, &dir);
    GPS_distanceToHome = dist / 100;	//����λ��Ϊ��
    GPS_directionToHome = dir / 100;	//���ڵ�λ�Ƕ���
		//�������GPS�����ʣ�����������������
		GPS_calc_velocity();	
		if(f.GPS_HOLD_MODE || f.GPS_HOME_MODE)	//��ͣ���߷���ԭ��ģʽ��������������
		{
			if(GS_Nav)		 //	����վ���µ�������  GS_Nav��־��һ	ReadyForNextWP��һ ReadyToMove = 0; nav_mode = NAV_MODE_POSHOLD;	wp=0;
			{
				if(ReadyForNextWP)			   //�Ѿ����ﺽ�㣬Ϊ��һ��������׼��
				{
					if(AutoHeading==0)			//�������Ҫ�Զ�ת��  ��ֱ�ӷ�����һ������
					{
						ReadyToMove = 1;	  //�����Ͳ���ִ�������ת��͵ȴ�ת��
					}
					if(ReadyToMove)		 //	  ���ת����ϣ�����Ҫת�����趨��һ�����㲢�����ɺ���ģʽ
					{
						Set_nextwp();	
						nav_mode = NAV_MODE_WP;	
											
						pid_nav_crosstrack._integrator = 0;
						pid_nav_crosstrack._last_input = 0;
						pid_nav_crosstrack._last_derivative = 0;
						
					}
					else				   //ִ���Զ�ת��
					{
						magHold = GPS_BearingForNextwp(&GPS_coord[LAT], &GPS_coord[LON], &Waypoint[wp][LAT], &Waypoint[wp][LON]);//�����¸���ķ�λ��
						magHold /=100;		//�м��м�
					//	if(abs(heading - magHold) < 10 || abs(heading - magHold) > 350)			 //�������������	   	nav_bearing/100
					//	{
							ReadyToMove = 1;
					//	}
					}
				}
			}
			//GPS��������
			GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance, &target_bearing);//����ֱ�߾���ͷ�λ��
			GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &GPS_coord[LAT], &GPS_coord[LON]);//�õ�X,Y����

			switch (nav_mode)
			{
				case NAV_MODE_POSHOLD:	//λ�ñ���ģʽ
						GPS_calc_poshold();
						break;
				case NAV_MODE_WP:
//						if(Waypoint[wp+1][0]==0)
							speed = GPS_calc_desired_speed(NAV_Speed, true);//ֻ�е����һ����ż���
// 						else
// 							speed = GPS_calc_desired_speed(NAV_Speed, false);	 //  cfg.nav_speed_max

				// use error as the desired rate towards the target
						// ������������ڵ���γ�Ⱥ;���,1��������100

						GPS_calc_nav_rate(speed);

						//������ƣ�������YAW
						if (AutoHeading) 
						{
							#if (NAV_TAIL_FIRST) 
							magHold = wrap_18000(target_bearing - 18000) / 100;
							#else 
							magHold = target_bearing / 100;			//		 nav_bearing
							#endif
						}
						//�ж��Ƿ񵽴�Ŀ�ĵأ���һ������İ뾶�ڣ�
						if ((wp_distance <= cfg.gps_wp_radius) || check_missed_wp())   //��ʱ��2�װ뾶
						{
//							nav_mode = NAV_MODE_POSHOLD;//��������ʧȥĿ���������ͣ
							ReadyForNextWP=1;
							ReadyToMove = 1 ;
							if(Waypoint[wp][LAT]==0)			//û����һ��������  
							{						
								nav_mode = NAV_MODE_POSHOLD;//��������ʧȥĿ���������ͣ
								GPS_hold[LAT] =     GPS_WP[LAT];			 //����hold������
								GPS_hold[LON] =     GPS_WP[LON];
								XbeePro_ImportanceInfo = XbeePro_ImportanceInfo | 0x01;//����HOLD�����λ��
								ReadyForNextWP=0;
								GS_Nav = 0;				//û���º���ʱ ����֤�����к���
								if (NAV_SET_TAKEOFF_HEADING)//�Ƿ񱣳������ʱ�ķ�����ͬ
								{
									magHold = nav_takeoff_bearing;
								}
							}
						}
						else
						{
							ReadyForNextWP = 0;		  //��Ҫ���º���
						}
						break;
			}
		}
	}
}

static void Set_nextwp(void)				 //���غ�����ɱ�־λ�����Ϊ1
{
	
	if(Waypoint[wp][0]!= 0)		   
	{
		GPS_set_next_wp(&Waypoint[wp][LAT], &Waypoint[wp][LON]);
		wp++;				//wp����ָ����һ������  �ǵ�ǰĿ�ĺ����
	}
}

/***********************
�������ܣ����ַ���ʽ�ľ�γ����ת������ddmm.mmmmm��ʵ�ʷ��ص���ddmmmmmmm
������������ڽ��յ����ַ���
***********************/
#define DIGIT_TO_VAL(_x)    (_x - '0')	//���ֵ�ascii��ȥ'0'���͵õ����������
u32 GPS_Coord_to_Degrees(char *s)
{
	char *p,*q;
	u8 deg = 0,min = 0;
	u32 frac_min = 0;
	int i;
	for(p=s;isdigit(*p);p++);	//��pָ��?ddmm.mmmmm�ַ����ĵ�
	q = s;
	while((p - q) > 2)			//���ȣ����Ǹ�ddඣ���ȡ������������deg
	{
		if(deg)
			deg*=10;
		deg += DIGIT_TO_VAL(*q++);
	}
	while(p > q)				//���Ǹ���ǰ��ķ���ȡ���������ֵ���������
	{
		if(min)
			min*=10;
		min += DIGIT_TO_VAL(*q++);
	}
	if(*p == '.')				//����С��������ȡ����
	{
		q = p + 1;
		for(i = 0;i < 4; i++)
		{
			frac_min *= 10;
			if(isdigit(*q))
				frac_min += *q++ - '0';
		}
	}
	//ddmmmmmmm
	return deg * 10000000UL + (min * 1000000UL + frac_min * 100UL) / 6;//10000000��ʾ1�ȣ������ֱ��ʸպ���1cm����Ȼ���ǲ���ȷ��
}

/***********************
�������ܣ����ַ�ת��Ϊ�������֣�С������Ҳ��Ҳת��Ϊ��������
���������1���ַ�����2��mult��������λС��
***********************/
static u32 Grab_Fields(char *src,u8 mult)
{
	u32 i;
	u32 tmp = 0;
	for(i = 0;src[i] != 0;i ++)
	{
		if(src[i] == '.')
		{
			i++;
			if(mult == 0)
				break;
			else
				src[i + mult]=0;	//���ý���λ
		}
		tmp *= 10;
		if(src[i] >= '0' && src[i] <='9')
			tmp += src[i] - '0';
	}
	return tmp;	//����ֵΪһ���������������а�������ǰ��С������
}

/***********************
�������ܣ���'0'..'9','A'..'F' ��Ϊ 0..15
���������ʮ����������һ��
***********************/
static uint8_t hex_c(uint8_t n)
{                 
    n -= '0';
    if (n > 9)
        n -= 7;
    n &= 0x0F;
    return n;
}

/***********************
�������ܣ��õ�������γ�����ľ��루CM�����õ�pos1��pos2�ķ�λ
���������γ��1������1��γ��2������2�����ؾ��룬���ط�λ
***********************/
static void GPS_distance_cm_bearing(int32_t * lat1, int32_t * lon1, int32_t * lat2, int32_t * lon2, uint32_t * dist, int32_t * bearing)
{
    float dLat = *lat2 - *lat1; // γ�Ȳ�ֵ��1/10 000 000��)
    float dLon = (float) (*lon2 - *lon1) * GPS_scaleLonDown;//����
    *dist = sqrtf(dLat*dLat + dLon*dLon) * 1.113195f;	//�õ�����

    *bearing = 9000.0f + atan2f(-dLat, dLon) * 5729.57795f;      //ת������Ƕȣ���ֵΪ100���Զȣ�Ҳ���Ƕȵ�100��
    if (*bearing < 0)
        *bearing += 36000;
}

/***********************
�������ܣ��õ�������γ�����ľ��루CM�����õ�pos1��pos2�ķ�λ
���������γ��1������1��γ��2������2�����ؾ��룬���ط�λ
***********************/
static int32_t GPS_BearingForNextwp(int32_t * lat1, int32_t * lon1, int32_t * lat2, int32_t * lon2)
{
	int32_t bearing;
    float dLat = *lat2 - *lat1; // γ�Ȳ�ֵ��1/10 000 000��)
    float dLon = (float) (*lon2 - *lon1) * GPS_scaleLonDown;//����

    bearing = 9000.0f + atan2f(-dLat, dLon) * 5729.57795f;      //ת������Ƕȣ���ֵΪ100���Զȣ�Ҳ���Ƕȵ�100��
    if (bearing < 0)
        bearing += 36000;

	return bearing;
}

/***********************
�������ܣ����㵱��������ʱ���������ľ������ţ�����һ�ȣ��ڳٵ�������
�������������
***********************/
static void GPS_calc_longitude_scaling(int32_t lat)
{
    float rads = (abs((float)lat)) * (0.0174532925f / 10000000.0f);
    GPS_scaleLonDown = cosf(rads);
}

#define _X 1
#define _Y 0
/***********************
�������ܣ��������ǵ�ǰ���ٶȸ���GPS��λ����
�������������Ҳû��
***********************/
static void GPS_calc_velocity(void)
{
	static s16 speed_old[2] = {0,0};//��һ�ε��ٶ�����
	static s32 last[2] = {0,0};		//��һ�εľ�γ������
	static u8 init = 0;				//��������Ƿ��һ�ν����������
	if(init)
	{
		float tmp = 1.0f / dTnav;
        actual_speed[_X] = (float) (GPS_coord[LON] - last[LON]) * GPS_scaleLonDown * tmp;//�����ٶ�
        actual_speed[_Y] = (float) (GPS_coord[LAT] - last[LAT]) * tmp;

        actual_speed[_X] = (actual_speed[_X] + speed_old[_X]) / 2;//ƽ��
        actual_speed[_Y] = (actual_speed[_Y] + speed_old[_Y]) / 2;

        speed_old[_X] = actual_speed[_X];//����
        speed_old[_Y] = actual_speed[_Y];
	}
	init = 1;
	last[LON] = GPS_coord[LON];//����
	last[LAT] = GPS_coord[LAT];
}

/***********************
�������ܣ���������GPS�����֮���λ�ò�
		   ��Ϊ������ʹ�þ�γ�����������ǵľ��룬��һ���ٲ��
			//      100     = 1m
			//      1000    = 11m    = 36 feet
			//      1800    = 19.80m = 60 feet
			//      3000    = 33m
			//      10000   = 111m
�����������������
***********************/
static void GPS_calc_location_error(int32_t * target_lat, int32_t * target_lng, int32_t * gps_lat, int32_t * gps_lng)
{
    error[LON] = (float) (*target_lng - *gps_lng) * GPS_scaleLonDown;   // X Error
    error[LAT] = *target_lat - *gps_lat;        // Y Error
}

/***********************
�������ܣ�����X,Y�������ٶȼ��㵼����γ��
�������������û��
***********************/
static void GPS_calc_poshold(void)
{
    int32_t p, i, d;
    int32_t output;
    int32_t target_speed;
	static int32_t d2[2],d1[2],dsum;
    int axis;

    for (axis = 0; axis < 2; axis++) {	//0��γ�ȣ�1�Ǿ���
		if(GPS_Speed_Mode)			     //GPS��̬ģʽ��  ʹ��ҡ�˿��Ʒ����ٶ�
		{
			target_speed = GPS_target_speed[axis] ;
		}
		else
	        target_speed = constrain(AC_PID_get_p(&pi_poshold[axis], error[axis], &posholdPID),-300,300);       //��������������ת��  ���ֵ 150cm/s

        rate_error[axis] = constrain(target_speed - actual_speed[axis],-300,300);   // calc the speed error

				//��ʱ
//	debug_A7108[1]=target_speed;

//        p = AC_PID_get_p(&pid_poshold_rate[axis], error[axis], &poshold_ratePID);		//	 rate_error[axis]
//        i = AC_PID_get_i(&pid_poshold_rate[axis], error[axis], &dTnav, &poshold_ratePID);	   //imax=2000;20*100
    //    d = 100 * AC_PID_get_d(&pid_poshold_rate[axis], error[axis], &dTnav, &poshold_ratePID);
	//	
 //       d = constrain(d, -2.0*abs(error[axis]), 2.0*abs(error[axis]));
 //		if(abs(error[axis])<200)   d = constrain(d, -200, 200);
//		else		d = constrain(d, -800, 800);
        p = AC_PID_get_p(&pid_poshold_rate[axis], rate_error[axis], &poshold_ratePID);
        i = AC_PID_get_i(&pid_poshold_rate[axis], rate_error[axis], &dTnav, &poshold_ratePID);	   //  + error[axis] imax=2000;20*100
 
		d = -100 * poshold_ratePID.kD * actual_speed[axis];

		d = constrain(d, -500, 500);

		dsum = d + d1[axis] + d2[axis];
		d2[axis] = d1[axis];
		d1[axis] = d;

        // get rid of noise							 
//#if defined(GPS_LOW_SPEED_D_FILTER)
//        if (abs(error[axis]) < 50)
//            d = 0;
//#endif
        output = p + i + dsum/3;	 // dsum/3		  // output/10�������Ǳ任����ANGLE��

        nav[axis] = constrain(output, -NAV_BANK_MAX, NAV_BANK_MAX);
        AC_PID_set_integrator(&pid_nav[axis], AC_PID_get_integrator(&pid_poshold_rate[axis]));
    }
}

/***********************
�������ܣ�ȷ����һ��·�꺽��ʱ�������ٶȣ�ͬʱ�ڿ�ʼһ������ʵ����������
//      |< WP Radius
//      0  1   2   3   4   5   6   7   8m
//      ...|...|...|...|...|...|...|...|
//                100  |  200     300     400cm/s
�������������ٶȣ��Ƿ�������
***********************/
static int16_t GPS_calc_desired_speed(int16_t max_speed, bool _slow)
{
    // max_speed Ĭ��Ϊ 400 or 4m/s
    if (_slow) 
	{
        max_speed = min(max_speed, wp_distance / 4);
        max_speed = max(max_speed, 0);
    }
	else 
	{
        max_speed = min(max_speed, wp_distance);
        max_speed = max(max_speed, cfg.nav_speed_min);      //�����ٶ�С��100cm/s
    }

    // �������ӵ��ٶ�
    // waypoint_speed_gov��0��ÿһ���º��㿪ʼ
    if (max_speed > waypoint_speed_gov) 
	{
        waypoint_speed_gov += (int) (100.0f * dTnav);    // increase at .5/ms
        max_speed = waypoint_speed_gov;
    }
    return max_speed;
}

/*Reset the PID integrator*/
static void AC_PID_reset(AC_PID * ac_pid)
{
    ac_pid->_integrator = 0;
    ac_pid->_last_input = 0;
    ac_pid->_last_derivative = 0;
}

/***********************
�������ܣ���������õ�С����	 ��[180,360]ֵ��ת�Ƶ�  [-180,0]	 ��ֵ֤����[-180,180]
���������
***********************/
int32_t wrap_18000(int32_t error)
{
    if (error > 18000)
        error -= 36000;
    if (error < -18000)
        error += 36000;
    return error;
}

static int32_t wrap_36000(int32_t angle)	//תһ��360  ��ֵ����[0,360]
{
    if (angle > 36000)
        angle -= 36000;
    if (angle < 0)
        angle += 36000;
    return angle;
}

/***********************
�������ܣ�ƫ�뺽�߼�������ǣ������ǻ������Ŀ�꺽����
���������
***********************/
static void GPS_update_crosstrack(void)	  //ƫ�뺽�߼�������ǣ������ǻ������Ŀ�꺽����
{
	float crosstrack_output;
    if (abs(wrap_18000(target_bearing - original_target_bearing)) < 4500) // ƫ��Ƕ�̫��Ͳ���Ҫ��������  ������45�Ⱦ�������־��������90��
	{     
        float temp = (target_bearing - original_target_bearing) * RADX100;
        crosstrack_error = sinf(temp) * wp_distance; //���ƫ�뺽�ߵľ��룬�㵽�ߵľ��� 
		crosstrack_output = AC_PID_get_p(&pid_nav_crosstrack, crosstrack_error, &nav_crosstrackPID)
            + AC_PID_get_i(&pid_nav_crosstrack, crosstrack_error, &dTnav, &nav_crosstrackPID)
            + AC_PID_get_d(&pid_nav_crosstrack, crosstrack_error, &dTnav, &nav_crosstrackPID); 
        nav_bearing = target_bearing + constrain(crosstrack_output, -4500, 4500);	  //���ƾ�����Ϊ30��
        nav_bearing = wrap_36000(nav_bearing);
    } 
	else 
	{
        nav_bearing = target_bearing;
    }
}

/***********************
�������ܣ�Calculate the desired nav_lat and nav_lon for distance flying such as RTH
���������
***********************/
static void GPS_calc_nav_rate(int max_speed)
{
    float trig[2];
    float temp;
    int axis;

    // ����������ԭ���Ĺ��
    GPS_update_crosstrack();

    // nav_bearing includes cross track
    temp = (9000l - nav_bearing) * RADX100;	   //Ҫȥ�ķ���Ļ���
    trig[_X] = cosf(temp);
    trig[_Y] = sinf(temp);

    for (axis = 0; axis < 2; axis++) {	//0��γ�ȣ�1�Ǿ���
        rate_error[axis] = (trig[axis] * max_speed) - actual_speed[axis];
        rate_error[axis] = constrain(rate_error[axis], -1000, 1000);
        // P + I + D
        nav[axis] = AC_PID_get_p(&pid_nav[axis], rate_error[axis], &navPID)
            + AC_PID_get_i(&pid_nav[axis], rate_error[axis], &dTnav, &navPID)
            + AC_PID_get_d(&pid_nav[axis], rate_error[axis], &dTnav, &navPID);
        nav[axis] = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
        AC_PID_set_integrator(&pid_poshold_rate[axis], AC_PID_get_integrator(&pid_nav[axis]));
    }
}

/***********************
�������ܣ����Ŀ���Ƿ������ԭ����ʧ
���������ľ��
***********************/
static bool check_missed_wp(void)
{
    int32_t temp;
    temp = target_bearing - original_target_bearing;
    temp = wrap_18000(temp);
    return (abs(temp) > 10000); //�������100�Ⱦ���Ϊû��Ŀ����
}

/*�����趨HOME������*/
void GPS_reset_home_position(void)
{
    if (f.GPS_FIX && GPS_numSat >= 5) 
	{
        GPS_home[LAT] = GPS_coord[LAT];
        GPS_home[LON] = GPS_coord[LON];
        nav_takeoff_bearing = heading;             //save takeoff heading
        //Set ground altitude
        f.GPS_FIX_HOME = 1;
    }
}

/*�����趨����ֹͣԭ�ȵĺ�������������*/
void GPS_reset_nav(void)
{
    int i;

    for (i = 0; i < 2; i++) 
	{
        GPS_angle[i] = 0;
        nav[i] = 0;
        AC_PID_reset(&pi_poshold[i]);
        AC_PID_reset(&pid_poshold_rate[i]);
        AC_PID_reset(&pid_nav[i]);
    }
}

/*������һ��·����*/
void GPS_set_next_wp(int32_t * lat, int32_t * lon)
{
    GPS_WP[LAT] = *lat;
    GPS_WP[LON] = *lon;

    GPS_calc_longitude_scaling(*lat);
    GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance, &target_bearing);

    nav_bearing = target_bearing;
    GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &GPS_coord[LAT], &GPS_coord[LON]);
    original_target_bearing = target_bearing;
    waypoint_speed_gov = cfg.nav_speed_min;
}

/***********************
/// Iterate the PID, return the new control value
///
/// Positive error produces positive output.
///
/// @param error        The measured error value
/// @param dt           The time delta in milliseconds (note
///                                     that update interval cannot be more
///                                     than 65.535 seconds due to limited range
///                                     of the data type).
/// @param scaler       An arbitrary scale factor
///
/// @returns            The updated control output.
***********************/
static int32_t AC_PID_get_p(AC_PID * ac_pid, int32_t error, PID_PARAM * pid)
{
    return (float) error * pid->kP;
}

static int32_t AC_PID_get_i(AC_PID * ac_pid, int32_t error, float *dt, PID_PARAM * pid)
{
    ac_pid->_integrator += ((float) error * pid->kI) * *dt;
    if (ac_pid->_integrator < -pid->Imax) {
        ac_pid->_integrator = -pid->Imax;
    } else if (ac_pid->_integrator > pid->Imax) {
        ac_pid->_integrator = pid->Imax;
    }
    return ac_pid->_integrator;
}

#define AC_PID_FILTER       (1.0f / (2.0f * M_PI * (float)cfg.gps_lpf))
static int32_t AC_PID_get_d(AC_PID * ac_pid, int32_t input, float *dt, PID_PARAM * pid)
{
    ac_pid->_derivative = (input - ac_pid->_last_input) / *dt;
    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
//    ac_pid->_derivative = ac_pid->_last_derivative + (*dt / (AC_PID_FILTER + *dt)) * (ac_pid->_derivative - ac_pid->_last_derivative);
    // update state
    ac_pid->_last_input = input;
    ac_pid->_last_derivative = ac_pid->_derivative;
    // add in derivative component
    return pid->kD * ac_pid->_derivative;
}

static float AC_PID_get_integrator(AC_PID * ac_pid)
{
    return ac_pid->_integrator;
}

static void AC_PID_set_integrator(AC_PID * ac_pid, float i)
{
    ac_pid->_integrator = i;
}

#define POSHOLD_RATE_IMAX      20
static void GPS_Set_PID(void)
{
    posholdPID.kP = (float) cfg.P8[PIDPOS] / 100.0f;
    posholdPID.kI = (float) cfg.I8[PIDPOS] / 100.0f;
    posholdPID.Imax = POSHOLD_RATE_IMAX * 100;

    poshold_ratePID.kP = (float) cfg.P8[PIDPOSR] / 10.0f;
    poshold_ratePID.kI = (float) cfg.I8[PIDPOSR] / 100.0f;
    poshold_ratePID.kD = (float) cfg.D8[PIDPOSR] / 1000.0f;
    poshold_ratePID.Imax = POSHOLD_RATE_IMAX * 100;

    navPID.kP = (float) cfg.P8[PIDNAVR] / 10.0f;
    navPID.kI = (float) cfg.I8[PIDNAVR] / 100.0f;
    navPID.kD = (float) cfg.D8[PIDNAVR] / 1000.0f;
    navPID.Imax = POSHOLD_RATE_IMAX * 100;

    nav_crosstrackPID.kP = 5.0f;
    nav_crosstrackPID.kI = 0.0f;
    nav_crosstrackPID.kD = 20.0f;
    nav_crosstrackPID.Imax = 0;

}

