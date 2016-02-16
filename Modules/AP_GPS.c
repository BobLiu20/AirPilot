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
u8 GS_Nav;					  		//地面站有新导航任务
u8 ReadyForNextWP;			  //是否去下一个目标点
u8 ReadyToMove;
int32_t Waypoint[WaypointNum_Max+1][2];	  //多航点坐标保存,航点的最大个数由WaypointNum_Max定义决定
int32_t GPS_coord[2];			//保存经纬度坐标,原始数据
int32_t GPS_home[2];			//保存GPS HOME的经纬度
int32_t GPS_hold[2];			//保存GPS悬停点坐标
int16_t GPS_angle[2] = { 0, 0 };    //GPS最终计算得到PID参量
int32_t GPS_target_speed[2];

static int32_t GPS_WP[2];
uint8_t GPS_numSat;		//搜索到的卫星数目
uint16_t GPS_altitude, GPS_speed;       // 海拔高度（单位分米）和速度（单位CM/S）
uint16_t GPS_distanceToHome;                            // 离原点的距离（米）
int16_t GPS_directionToHome;                            // 和原点的角度（度）
uint8_t GPS_update = 0;             // it's a binary toogle to distinct a GPS position update
int16_t nav[2];
static int32_t target_bearing;	//现在飞机方向和要去的地点的方位角度（度x100）
static uint32_t wp_distance;	//飞机和下一个要去的点的距离cm
int8_t nav_mode = NAV_MODE_NONE;    //导航的模式选择（NAV_MODE_WP=2，NAV_MODE_POSHOLD=1）
static int32_t original_target_bearing;//记录旧的目标方位
static int32_t nav_bearing;	//从飞机到下一个目标的方位角（度x100）
int16_t nav_takeoff_bearing;//保存起飞时的方向，以便达到目标的恢复飞机头的方向
static int16_t waypoint_speed_gov;
int16_t nav_rated[2];               // Adding a rate controller to the navigation to make it smoother
uint8_t wp=0;
int16_t NAV_Speed = 200;
uint8_t AutoHeading = 1;		//巡航时的航向设置，自动航向为1，手动航向为0
uint8_t GPS_Speed_Mode = 0;

/********************PID用****************/
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
#define NAV_SLOW_NAV               true	//慢速模式？
#define NAV_BANK_MAX               3000 // 30deg max banking when navigating (just for security and testing)
static float dTnav;             //记录两次到达这个地方的时间，单位s
int16_t actual_speed[2] = { 0, 0 };//记录实际速度
static float GPS_scaleLonDown;  //用于经度在靠近地球两地时，地理距离的缩小
static int16_t crosstrack_error;//航行角矫正

/***************滤波专用*****************/
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
static void Set_nextwp(void);				 //返回航点完成标志位，完成为1
static void GPS_Set_PID(void);
static void GPS_NewData(void);

/************************
函数功能：GPS初始化
输入参数：串口波特率
************************/
void GPS_Init(uint32_t baudrate)
{
	OS_ERR	err;
	GPS_Set_PID();
	USART2_Init(baudrate);	//初始化串口2
	
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
  * @brief  GPS主任务
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
		if(!USART2_RXAVAILABLE)continue; //没有接收到数据，则继续等待
		OSTimeDly(1,OS_OPT_TIME_DLY,&err);//稍微延时一下，等待数据接收齐全
		while(USART2_RXAVAILABLE)//只要有有效数据就一直解析
		{
			if(GPS_NewFrame(USART2_Read()))//如果成立，则说明读完一帧数据了
			{
				GPS_NewData();//GPS数据
				USART2_RXCLEAR;//清除无用数据
			}
		}
  }
}

/***********************
函数功能：GPS接收数据处理,解码字符串
输入参数：串口接收到一字节数据
***********************/
static bool GPS_NewFrame(char c)
{
	u8 frameOK = 0;			//数据更新是否完成
	static u8 param = 0;	//在一帧中的哪一段，0段为引导码
	static u8 offset = 0;	//字符串数组用到
	static u8 parity = 0;	//校验码记录
	static char string[15];	//临时保存每一段需要解析的字符串
	static u8 checksum_param;//校验
	static u8 frame = 0;	//这一桢是哪种格式的，这里用到GPGGA和GPRMC格式
	if(c == '$')	//$符号为一帧数据的起始符号
	{
		param = 0;	//一帧开始，初始化这些变量
		offset = 0;
		parity = 0;
	}
	else if(c == ',' || c == '*')//逗号为一段字符结尾，星号表示结束，后面的是校验码等
	{
		string[offset] = 0;	//让临时字符串最后一位为0结束,这个0是ascii的0呢
		if(param == 0)		//第0段储存着哪种数据格式
		{
			frame = 0;
			if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A')
				frame = FRAME_GGA;	//GPGGA数据格式
			if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C')
				frame = FRAME_RMC; 	//GPRMC数据格式
		}
		else if(frame == FRAME_GGA)	//进入GPGGA数据格式处理
		{
			if(param == 2)	//纬度
			{
				GPS_coord[LAT] = GPS_Coord_to_Degrees(string);//将纬度转换为ddmm.mmmmm，度分格式
			}
			else if (param == 3 && string[0] == 'S')		//判断是否是南纬
         GPS_coord[LAT] = -GPS_coord[LAT];	//南纬是正,北纬是负
      else if (param == 4) 					//经度
			{
       	GPS_coord[LON] = GPS_Coord_to_Degrees(string);//将经度转换为ddmm.mmmmm，度分格式
      } 
			else if (param == 5 && string[0] == 'W')		//判断是否是西经
        GPS_coord[LON] = -GPS_coord[LON]; //西经用负表示
      else if (param == 6)				//定位质量，0为无效定位，1为有效
			{
        f.GPS_FIX = string[0] > '0';	//string[0] > '0'成立则说明是有效，标记GPS_FIX
      }
			else if (param == 7) 			//使用的卫星数目
			{
	      GPS_numSat = Grab_Fields(string, 0);//将字符转换为数字,卫星数目
	    }
			else if (param == 9) 			//海拔高度
			{
	      GPS_altitude = Grab_Fields(string, 1);  //单位是分米
			}
    }
		else if(frame == FRAME_RMC)	//在这桢获取速度
		{
			if (param == 7) //速度，准确说是速率呵呵，
			{
        GPS_speed = ((uint32_t) Grab_Fields(string, 1) * 5144L) / 1000L;    // 换算单位为CM/S的速度
      }
		}
		param++;		//字符段加1
		offset = 0;		//临时保存字符串清0
		if (c == '*')	//判断是否到校验码了
			checksum_param = 1;//标记到检验码了
		else
			parity ^= c;	//奇偶校验，$和*之间的ascii位或的值，要等于校验码
	}
	else if(c == '\r' || c == '\n')
	{
		if(checksum_param)	//进入校验
		{
			u8 checksum = hex_c(string[0]);
			checksum <<= 4;
			checksum += hex_c(string[1]);
			if (checksum == parity)	//相等，则数据正确无误
				frameOK = 1;
    }
    checksum_param = 0;
	}
	else
	{
		if (offset < 15)
      string[offset++] = c;//保存字符
    if (!checksum_param)
      parity ^= c;
	}
	//if(frame)
	//	GPS_Present = 1;
	return frameOK && (frame == FRAME_GGA);	//数据完全更新成功一次，返回1
}

/***********************
函数功能：GPS数据更新,将由串口中断调用，在GPS初始化时传入了
***********************/
static void GPS_NewData(void)
{
	int axis;
	static uint32_t nav_loopTimer;	//记录上一次的循环的时间戳
	uint32_t dist;	//距离
  int32_t dir;	//方向
  int16_t speed;	//速度
	SensorsSet(SENSOR_GPS);	//置位GPS存在标志
	if(GPS_update == 1)
		GPS_update = 0;
	else
		GPS_update = 1;
	if(f.GPS_FIX && GPS_numSat >= 5)//如果数据有效和卫星数大于5
	{
		if(!f.ARMED)	
		{
			f.GPS_FIX_HOME = 0;//在没有解锁状态下，清楚GPS HOME点
		}
		if(!f.GPS_FIX_HOME && f.ARMED)//在没有GPS原点又已经解锁了，则设置原点
		{
			f.GPS_FIX_HOME = 1;//标记有原点
			GPS_home[LAT] = GPS_coord[LAT];//将当前经纬度记录下
			GPS_home[LON] = GPS_coord[LON];
			GPS_calc_longitude_scaling(GPS_coord[LAT]); //得到一个初始值用于计算距离和方位
      nav_takeoff_bearing = heading; // save takeoff heading
			XbeePro_ImportanceInfo = XbeePro_ImportanceInfo | 0x10;//发送HOME点给上位机
		}
		//进行滤波，美其名曰滑动平均滤波
		GPS_filter_index = (GPS_filter_index + 1) % GPS_FILTER_VECTOR_LENGTH;
    for (axis = 0; axis < 2; axis++) //axis=0时为纬度，=1时为经度
		{
			GPS_read[axis] = GPS_coord[axis];       //得到最新的未过滤的经纬度
      GPS_degree[axis] = GPS_read[axis] / 10000000;   //得到经纬度的度

			// How close we are to a degree line ? its the first three digits from the fractions of degree
			// later we use it to Check if we are close to a degree line, if yes, disable averaging,
			fraction3[axis] = (GPS_read[axis] - GPS_degree[axis] * 10000000) / 10000;//得到小数点后三位的分,即0.xxx度的xxx

			GPS_filter_sum[axis] -= GPS_filter[axis][GPS_filter_index];
			GPS_filter[axis][GPS_filter_index] = GPS_read[axis] - (GPS_degree[axis] * 10000000);
			GPS_filter_sum[axis] += GPS_filter[axis][GPS_filter_index];
			GPS_filtered[axis] = GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GPS_degree[axis] * 10000000);
			if (nav_mode == NAV_MODE_POSHOLD) //我们只在poshold模式下使用GPS平均值		nav_mode == NAV_MODE_POSHOLD
			{
				if (fraction3[axis] > 1 && fraction3[axis] < 999)
					GPS_coord[axis] = GPS_filtered[axis];	//将滤波后的值保存
      }
    }
		//进入计算 x,y 、速度、航向 的PID等
		dTnav = (float) (TimMsGet() - nav_loopTimer) / 1000.0f;//计算两次之间的时间差	  现在单位为s
		nav_loopTimer = TimMsGet();		//记录时间戳  单位为ms
		dTnav = min(dTnav, 1.0f);		//防止GPS出错，产生超长时间差(不大于1S)
		//计算距离、方位给GUI，从原点到灰机的
		GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_home[LAT], &GPS_home[LON], &dist, &dir);
    GPS_distanceToHome = dist / 100;	//将单位变为米
    GPS_directionToHome = dir / 100;	//现在单位是度了
		//计算基于GPS的速率，当我们有启动导航
		GPS_calc_velocity();	
		if(f.GPS_HOLD_MODE || f.GPS_HOME_MODE)	//悬停或者返回原点模式，则启动导航哈
		{
			if(GS_Nav)		 //	地面站有新导航任务  GS_Nav标志置一	ReadyForNextWP置一 ReadyToMove = 0; nav_mode = NAV_MODE_POSHOLD;	wp=0;
			{
				if(ReadyForNextWP)			   //已经到达航点，为下一个航点做准备
				{
					if(AutoHeading==0)			//如果不需要自动转向  就直接飞向下一个航点
					{
						ReadyToMove = 1;	  //这样就不会执行下面的转向和等待转向
					}
					if(ReadyToMove)		 //	  如果转向完毕，或不需要转向，则设定下一个航点并开启飞航线模式
					{
						Set_nextwp();	
						nav_mode = NAV_MODE_WP;	
											
						pid_nav_crosstrack._integrator = 0;
						pid_nav_crosstrack._last_input = 0;
						pid_nav_crosstrack._last_derivative = 0;
						
					}
					else				   //执行自动转向
					{
						magHold = GPS_BearingForNextwp(&GPS_coord[LAT], &GPS_coord[LON], &Waypoint[wp][LAT], &Waypoint[wp][LON]);//计算下个点的方位角
						magHold /=100;		//切记切记
					//	if(abs(heading - magHold) < 10 || abs(heading - magHold) > 350)			 //航向调整完再走	   	nav_bearing/100
					//	{
							ReadyToMove = 1;
					//	}
					}
				}
			}
			//GPS导航计算
			GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance, &target_bearing);//计算直线距离和方位角
			GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &GPS_coord[LAT], &GPS_coord[LON]);//得到X,Y距离

			switch (nav_mode)
			{
				case NAV_MODE_POSHOLD:	//位置保持模式
						GPS_calc_poshold();
						break;
				case NAV_MODE_WP:
//						if(Waypoint[wp+1][0]==0)
							speed = GPS_calc_desired_speed(NAV_Speed, true);//只有到最后一个点才减速
// 						else
// 							speed = GPS_calc_desired_speed(NAV_Speed, false);	 //  cfg.nav_speed_max

				// use error as the desired rate towards the target
						// 期望的输出是在导航纬度和经度,1度倾向是100

						GPS_calc_nav_rate(speed);

						//方向控制，即调整YAW
						if (AutoHeading) 
						{
							#if (NAV_TAIL_FIRST) 
							magHold = wrap_18000(target_bearing - 18000) / 100;
							#else 
							magHold = target_bearing / 100;			//		 nav_bearing
							#endif
						}
						//判断是否到达目的地（在一个允许的半径内）
						if ((wp_distance <= cfg.gps_wp_radius) || check_missed_wp())   //暂时是2米半径
						{
//							nav_mode = NAV_MODE_POSHOLD;//如果到达或失去目标则进入悬停
							ReadyForNextWP=1;
							ReadyToMove = 1 ;
							if(Waypoint[wp][LAT]==0)			//没有下一个航点了  
							{						
								nav_mode = NAV_MODE_POSHOLD;//如果到达或失去目标则进入悬停
								GPS_hold[LAT] =     GPS_WP[LAT];			 //更新hold点坐标
								GPS_hold[LON] =     GPS_WP[LON];
								XbeePro_ImportanceInfo = XbeePro_ImportanceInfo | 0x01;//发送HOLD点给上位机
								ReadyForNextWP=0;
								GS_Nav = 0;				//没有新航点时 ，保证不更行航点
								if (NAV_SET_TAKEOFF_HEADING)//是否保持与起飞时的方向相同
								{
									magHold = nav_takeoff_bearing;
								}
							}
						}
						else
						{
							ReadyForNextWP = 0;		  //不要更新航点
						}
						break;
			}
		}
	}
}

static void Set_nextwp(void)				 //返回航点完成标志位，完成为1
{
	
	if(Waypoint[wp][0]!= 0)		   
	{
		GPS_set_next_wp(&Waypoint[wp][LAT], &Waypoint[wp][LON]);
		wp++;				//wp总是指向下一个航点  非当前目的航点号
	}
}

/***********************
函数功能：将字符形式的经纬坐标转换数字ddmm.mmmmm，实际返回的是ddmmmmmmm
输入参数：串口接收到的字符串
***********************/
#define DIGIT_TO_VAL(_x)    (_x - '0')	//数字的ascii减去'0'，就得到这个数字了
u32 GPS_Coord_to_Degrees(char *s)
{
	char *p,*q;
	u8 deg = 0,min = 0;
	u32 frac_min = 0;
	int i;
	for(p=s;isdigit(*p);p++);	//让p指向?ddmm.mmmmm字符串的点
	q = s;
	while((p - q) > 2)			//将度（即那个dd喽）提取出来，保存在deg
	{
		if(deg)
			deg*=10;
		deg += DIGIT_TO_VAL(*q++);
	}
	while(p > q)				//将那个点前面的分提取出来，即分的整数部分
	{
		if(min)
			min*=10;
		min += DIGIT_TO_VAL(*q++);
	}
	if(*p == '.')				//将分小数部分提取出来
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
	return deg * 10000000UL + (min * 1000000UL + frac_min * 100UL) / 6;//10000000表示1度，这样分辨率刚好是1cm，虽然这是不精确的
}

/***********************
函数功能：将字符转换为整数数字，小数部分也的也转化为整数部分
输入参数：1、字符串；2、mult保留多少位小数
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
				src[i + mult]=0;	//设置结束位
		}
		tmp *= 10;
		if(src[i] >= '0' && src[i] <='9')
			tmp += src[i] - '0';
	}
	return tmp;	//返回值为一个整数，可能其中包含了以前的小数部分
}

/***********************
函数功能：将'0'..'9','A'..'F' 变为 0..15
输入参数：十六进制数字一个
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
函数功能：得到两个经纬坐标间的距离（CM），得到pos1到pos2的方位
输入参数：纬度1，经度1，纬度2，经度2，返回距离，返回方位
***********************/
static void GPS_distance_cm_bearing(int32_t * lat1, int32_t * lon1, int32_t * lat2, int32_t * lon2, uint32_t * dist, int32_t * bearing)
{
    float dLat = *lat2 - *lat1; // 纬度差值（1/10 000 000度)
    float dLon = (float) (*lon2 - *lon1) * GPS_scaleLonDown;//经度
    *dist = sqrtf(dLat*dLat + dLon*dLon) * 1.113195f;	//得到距离

    *bearing = 9000.0f + atan2f(-dLat, dLon) * 5729.57795f;      //转换输出角度，其值为100乘以度，也就是度的100倍
    if (*bearing < 0)
        *bearing += 36000;
}

/***********************
函数功能：得到两个经纬坐标间的距离（CM），得到pos1到pos2的方位
输入参数：纬度1，经度1，纬度2，经度2，返回距离，返回方位
***********************/
static int32_t GPS_BearingForNextwp(int32_t * lat1, int32_t * lon1, int32_t * lat2, int32_t * lon2)
{
	int32_t bearing;
    float dLat = *lat2 - *lat1; // 纬度差值（1/10 000 000度)
    float dLon = (float) (*lon2 - *lon1) * GPS_scaleLonDown;//经度

    bearing = 9000.0f + atan2f(-dLat, dLon) * 5729.57795f;      //转换输出角度，其值为100乘以度，也就是度的100倍
    if (bearing < 0)
        bearing += 36000;

	return bearing;
}

/***********************
函数功能：计算当靠近两极时，地球表面的距离缩放，经度一度，在迟到距离会大
输入参数：经度
***********************/
static void GPS_calc_longitude_scaling(int32_t lat)
{
    float rads = (abs((float)lat)) * (0.0174532925f / 10000000.0f);
    GPS_scaleLonDown = cosf(rads);
}

#define _X 1
#define _Y 0
/***********************
函数功能：计算我们当前的速度根据GPS定位数据
输入参数：神马也没有
***********************/
static void GPS_calc_velocity(void)
{
	static s16 speed_old[2] = {0,0};//上一次的速度数据
	static s32 last[2] = {0,0};		//上一次的经纬度坐标
	static u8 init = 0;				//用来标记是否第一次进入这个函数
	if(init)
	{
		float tmp = 1.0f / dTnav;
        actual_speed[_X] = (float) (GPS_coord[LON] - last[LON]) * GPS_scaleLonDown * tmp;//计算速度
        actual_speed[_Y] = (float) (GPS_coord[LAT] - last[LAT]) * tmp;

        actual_speed[_X] = (actual_speed[_X] + speed_old[_X]) / 2;//平均
        actual_speed[_Y] = (actual_speed[_Y] + speed_old[_Y]) / 2;

        speed_old[_X] = actual_speed[_X];//保存
        speed_old[_Y] = actual_speed[_Y];
	}
	init = 1;
	last[LON] = GPS_coord[LON];//保存
	last[LAT] = GPS_coord[LAT];
}

/***********************
函数功能：计算两个GPS坐标点之间的位置差
		   因为我们是使用经纬度来计算我们的距离，有一个速查表
			//      100     = 1m
			//      1000    = 11m    = 36 feet
			//      1800    = 19.80m = 60 feet
			//      3000    = 33m
			//      10000   = 111m
输入参数：两个坐标
***********************/
static void GPS_calc_location_error(int32_t * target_lat, int32_t * target_lng, int32_t * gps_lat, int32_t * gps_lng)
{
    error[LON] = (float) (*target_lng - *gps_lng) * GPS_scaleLonDown;   // X Error
    error[LAT] = *target_lat - *gps_lat;        // Y Error
}

/***********************
函数功能：根据X,Y的误差和速度计算导航经纬度
输入参数：神马都没有
***********************/
static void GPS_calc_poshold(void)
{
    int32_t p, i, d;
    int32_t output;
    int32_t target_speed;
	static int32_t d2[2],d1[2],dsum;
    int axis;

    for (axis = 0; axis < 2; axis++) {	//0是纬度，1是经度
		if(GPS_Speed_Mode)			     //GPS姿态模式下  使用摇杆控制飞行速度
		{
			target_speed = GPS_target_speed[axis] ;
		}
		else
	        target_speed = constrain(AC_PID_get_p(&pi_poshold[axis], error[axis], &posholdPID),-300,300);       //根据误差计算期望转速  最大值 150cm/s

        rate_error[axis] = constrain(target_speed - actual_speed[axis],-300,300);   // calc the speed error

				//临时
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
        output = p + i + dsum/3;	 // dsum/3		  // output/10就再三角变换就是ANGLE了

        nav[axis] = constrain(output, -NAV_BANK_MAX, NAV_BANK_MAX);
        AC_PID_set_integrator(&pid_nav[axis], AC_PID_get_integrator(&pid_poshold_rate[axis]));
    }
}

/***********************
函数功能：确定想一个路标航行时的理想速度，同时在开始一个导航实现慢速增加
//      |< WP Radius
//      0  1   2   3   4   5   6   7   8m
//      ...|...|...|...|...|...|...|...|
//                100  |  200     300     400cm/s
输入参数：最大速度，是否开启慢速
***********************/
static int16_t GPS_calc_desired_speed(int16_t max_speed, bool _slow)
{
    // max_speed 默认为 400 or 4m/s
    if (_slow) 
	{
        max_speed = min(max_speed, wp_distance / 4);
        max_speed = max(max_speed, 0);
    }
	else 
	{
        max_speed = min(max_speed, wp_distance);
        max_speed = max(max_speed, cfg.nav_speed_min);      //飞行速度小于100cm/s
    }

    // 限制增加的速度
    // waypoint_speed_gov清0在每一个新航点开始
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
函数功能：供下面调用的小程序	 把[180,360]值域转移到  [-180,0]	 保证值域在[-180,180]
输入参数：
***********************/
int32_t wrap_18000(int32_t error)
{
    if (error > 18000)
        error -= 36000;
    if (error < -18000)
        error += 36000;
    return error;
}

static int32_t wrap_36000(int32_t angle)	//转一个360  让值域在[0,360]
{
    if (angle > 36000)
        angle -= 36000;
    if (angle < 0)
        angle += 36000;
    return angle;
}

/***********************
函数功能：偏离航线计算纠正角，纠正角会叠加在目标航向上
输入参数：
***********************/
static void GPS_update_crosstrack(void)	  //偏离航线计算纠正角，纠正角会叠加在目标航向上
{
	float crosstrack_output;
    if (abs(wrap_18000(target_bearing - original_target_bearing)) < 4500) // 偏差角度太大就不需要这种做法  ，大于45度纠正会出现纠航向大于90度
	{     
        float temp = (target_bearing - original_target_bearing) * RADX100;
        crosstrack_error = sinf(temp) * wp_distance; //算出偏离航线的距离，点到线的距离 
		crosstrack_output = AC_PID_get_p(&pid_nav_crosstrack, crosstrack_error, &nav_crosstrackPID)
            + AC_PID_get_i(&pid_nav_crosstrack, crosstrack_error, &dTnav, &nav_crosstrackPID)
            + AC_PID_get_d(&pid_nav_crosstrack, crosstrack_error, &dTnav, &nav_crosstrackPID); 
        nav_bearing = target_bearing + constrain(crosstrack_output, -4500, 4500);	  //限制纠正角为30度
        nav_bearing = wrap_36000(nav_bearing);
    } 
	else 
	{
        nav_bearing = target_bearing;
    }
}

/***********************
函数功能：Calculate the desired nav_lat and nav_lon for distance flying such as RTH
输入参数：
***********************/
static void GPS_calc_nav_rate(int max_speed)
{
    float trig[2];
    float temp;
    int axis;

    // 将我们推向原来的轨道
    GPS_update_crosstrack();

    // nav_bearing includes cross track
    temp = (9000l - nav_bearing) * RADX100;	   //要去的方向的弧度
    trig[_X] = cosf(temp);
    trig[_Y] = sinf(temp);

    for (axis = 0; axis < 2; axis++) {	//0是纬度，1是经度
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
函数功能：检查目标是否因各种原因消失
输入参数：木有
***********************/
static bool check_missed_wp(void)
{
    int32_t temp;
    temp = target_bearing - original_target_bearing;
    temp = wrap_18000(temp);
    return (abs(temp) > 10000); //如果大于100度就认为没有目标了
}

/*重新设定HOME的坐标*/
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

/*重新设定航向（停止原先的航向处理和清除它）*/
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

/*设置下一个路径点*/
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

