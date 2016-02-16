/**
	******************************************************************************
	* @file    AP_XbeePro.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   无线数传模块XBEE PRO 900HP
	*					 引脚：PC10-UART4_TX
	*								 PC11-UART4_RX
	*					 本驱动可兼容各种通过UART透传的无线模块，如蓝牙串口
	******************************************************************************
**/
#include "AP_XbeePro.h"
#ifdef XBEEPRO
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
//定义需要发送给上位机的重要消息，位0-7
//位0：表示巡航结束时发送给上位机相应的定点位置	，GPS.C  353行
//位1:飞机进入遥控器失效保护，IMA.C  309行
//位2：飞机在飞行过程中出现倾角超过100度，上锁，飞机事故，发送求解信号给上位机   IMA.C  907行
//位3：飞机上电，完成初始化			   main.c
//位4：发送HOME点给上位机
unsigned char XbeePro_ImportanceInfo=0;

int16_t Xbee_Debug[3];//测试显示用

/**********************串口通讯协议区***************************/
#define UART_RxMax 50	//串口数据字节数
#define UART_FramsMax 30//一帧数据最大字节数
static unsigned char UART_Rx_Buf[UART_RxMax];//串口接收缓存
static unsigned char Rx_CircleRead=0;		 //串口数据读取循环标志
static unsigned char Rx_CircleRx=0;			 //串口数据接收循环标志

static unsigned char RX_Data_Buf[UART_FramsMax];	//为了进行校验数据，用来临时缓存数据的
unsigned char UART_Tx_Buf[UART_FramsMax];			//发送缓冲

extern uint8_t Vision_ErrorZero_Cal;

/* function prototypes -----------------------------------------------*/
void XbeePro_SetAndTx(u8 c);

/*串口接收*/
void UART4_IRQHandler(void)
{
//	OSIntEnter();
	UART_Rx_Buf[Rx_CircleRx]=UART4->DR;
	UART4->SR=UART4->SR&(~0X20);
	Rx_CircleRx++;
	if(Rx_CircleRx >= UART_RxMax)
		Rx_CircleRx = 0;
//	OSIntExit();
}

/*初始化*/
void XbeePro_Init(void)
{
	UART4_Init();	//初始化用到的串口
}

/*数据处理*/
#define CircleReadAdd Rx_CircleRead++;if(Rx_CircleRead>=UART_RxMax)Rx_CircleRead = 0
void XbeePro_Process(void)
{
	OS_ERR err;
	u8 i=0,cal=0;
	if(Rx_CircleRead!=Rx_CircleRx)
	{
		if(UART_Rx_Buf[Rx_CircleRead]=='$')
		{
			OSTimeDly(30,OS_OPT_TIME_DLY,&err); //延时一段时间再进行数据处理，单位ms
			CircleReadAdd;//让Rx_CircleRead自加1
			if(UART_Rx_Buf[Rx_CircleRead]=='M')
			{
				CircleReadAdd;
				if(UART_Rx_Buf[Rx_CircleRead]=='<')
				{
					CircleReadAdd;
					RX_Data_Buf[0]=UART_Rx_Buf[Rx_CircleRead];	//记录数据个数，表示后面跟着多少个数据，不包括校验码
					CircleReadAdd;
					for(i=0;i<RX_Data_Buf[0];i++)//读取数据
					{
						RX_Data_Buf[1+i]=UART_Rx_Buf[Rx_CircleRead];
						if(i==0)cal=RX_Data_Buf[1+i];//校验码
						else cal^=RX_Data_Buf[1+i];
						CircleReadAdd;
					}
					if(cal!=UART_Rx_Buf[Rx_CircleRead])//验证最后一位校验码是否与计算出来的相等
					{	//校验码不对
						XbeePro_SetAndTx(0);//校验码错误传递
					}	
					else//校验码正确
					{
						XbeePro_SetAndTx(1);
					}
					Rx_CircleRead=Rx_CircleRx;//结束	
				}
			}
		}
		else
		{
			CircleReadAdd;
		}
	}
}

//头信息和数据个数num和识别码msp
static void XbeePro_HeadResponse(u8 num,u8 msp)
{
	UART_Tx_Buf[0]='$';
	UART_Tx_Buf[1]='M';
	UART_Tx_Buf[2]='<';
	UART_Tx_Buf[3]=num;		 //数据个数
	UART_Tx_Buf[4]=msp;		 //识别码	
}

static void XbeePro_SendMainInfo(void)
{
	u32 j;
	u8 cal=0;
	if(XbeePro_ImportanceInfo&0x02)//飞机进入遥控器失效保护				//紧急
	{
		RX_Data_Buf[1] = 0x20;			 //强制进入紧急事故通道
		XbeePro_HeadResponse(2,0x20);	  //头段信息
		UART_Tx_Buf[5] = 0x01;
		UART_Tx_Buf[6] = 0x20^0x01;	  //校验码
		XbeePro_ImportanceInfo&=0xfd;	
	}
	else if(XbeePro_ImportanceInfo&0x04)//飞机在飞行过程中出现倾角超过100度	    //紧急
	{
		RX_Data_Buf[1] = 0x20;			 //强制进入紧急事故通道
		XbeePro_HeadResponse(2,0x20);	  //头段信息
		UART_Tx_Buf[5] = 0x02;
		UART_Tx_Buf[6] = 0x20^0x02;	  //校验码
		XbeePro_ImportanceInfo&=0xfb;	
	}
	else if(XbeePro_ImportanceInfo&0x08)//飞机上电，完成初始化
	{
		RX_Data_Buf[1] = 0x20;			 //强制进入紧急事故通道
		XbeePro_HeadResponse(2,0x20);	  //头段信息
		UART_Tx_Buf[5] = 0x03;
		UART_Tx_Buf[6] = 0x20^0x03;	  //校验码
		XbeePro_ImportanceInfo&=0xf7;
	}
	else if(XbeePro_ImportanceInfo&0x01)//巡航完成时发送定点位置
	{
		if(RX_Data_Buf[1] != 0x10)return ;
		RX_Data_Buf[1] = 0x13;//强制修改为发送定点位置
		XbeePro_ImportanceInfo&=0xfe;	
	}
	else if(XbeePro_ImportanceInfo&0x10)//发送HOME点给上位机
	{
		RX_Data_Buf[1] = 0x20;			 //强制进入紧急事故通道
		XbeePro_HeadResponse(9,0x23);	  //头段信息
		cal=UART_Tx_Buf[4];
		j=GPS_home[LAT];
		UART_Tx_Buf[5]=j&0xff;
		cal^=UART_Tx_Buf[5];
		UART_Tx_Buf[6]=(j>>8)&0xff;
		cal^=UART_Tx_Buf[6];
		UART_Tx_Buf[7]=(j>>16)&0xff;
		cal^=UART_Tx_Buf[7];
		UART_Tx_Buf[8]=(j>>24)&0xff;
		cal^=UART_Tx_Buf[8];
		j=GPS_home[LON];
		UART_Tx_Buf[9]=j&0xff;
		cal^=UART_Tx_Buf[9];
		UART_Tx_Buf[10]=(j>>8)&0xff;
		cal^=UART_Tx_Buf[10];
		UART_Tx_Buf[11]=(j>>16)&0xff;
		cal^=UART_Tx_Buf[11];
		UART_Tx_Buf[12]=(j>>24)&0xff;
		cal^=UART_Tx_Buf[12];
		UART_Tx_Buf[13]=cal;//校验码
		XbeePro_ImportanceInfo&=0xef;	
	}
}

/*设置参数和回送参数给上位机*/
void XbeePro_SetAndTx(u8 c)
{
	u8 cal=0,i=0;
	u16 ff;
	s32 alt;
	u32 j;
	static u8 wp_count = 0;	 //多航点个数计数用
	if(c==0)//校验码错误
	{	/*可根据需要决定是否回复校验错误信息*/
//		XbeePro_HeadResponse(1,0x02);	  //校验错误，回复校验错误信号
//		UART_Tx_Buf[5] = 0x02;	  //校验码
//		/*将数据发送出去*/
//		DMA2_Channel5->CCR &= (uint16_t)(~DMA_CCR1_EN);
//		DMA2_Channel5->CNDTR = UART_Tx_Buf[3]+5;//帧长度
//		DMA2_Channel5->CCR |= DMA_CCR1_EN;
		return ;	
	}
	if(XbeePro_ImportanceInfo)XbeePro_SendMainInfo();//重要消息发送
	if(RX_Data_Buf[1] == 0x10)		 //发送GPS数据给上位机
	{
		XbeePro_HeadResponse(25,0x10);	  //头段信息
		cal=UART_Tx_Buf[4];
		UART_Tx_Buf[5]=(f.GPS_FIX<<7)|(AutoHeading<<6)|(((heading<0)?(360+heading):(heading))/10);	//最高位是GPS是否有效位，第六位为航向手动自动，低6位是飞机头
		cal^=UART_Tx_Buf[5];
		UART_Tx_Buf[6]=(0<<7)|GPS_numSat;	 //最高位是电量充足指示，其余为卫星数					Battery_LowPower
		cal^=UART_Tx_Buf[6];
		j=GPS_coord[LAT];
		UART_Tx_Buf[7]=j&0xff;
		cal^=UART_Tx_Buf[7];
		UART_Tx_Buf[8]=(j>>8)&0xff;
		cal^=UART_Tx_Buf[8];
		UART_Tx_Buf[9]=(j>>16)&0xff;
		cal^=UART_Tx_Buf[9];
		UART_Tx_Buf[10]=(j>>24)&0xff;
		cal^=UART_Tx_Buf[10];
		j=GPS_coord[LON];
		UART_Tx_Buf[11]=j&0xff;
		cal^=UART_Tx_Buf[11];
		UART_Tx_Buf[12]=(j>>8)&0xff;
		cal^=UART_Tx_Buf[12];
		UART_Tx_Buf[13]=(j>>16)&0xff;
		cal^=UART_Tx_Buf[13];
		UART_Tx_Buf[14]=(j>>24)&0xff;
		cal^=UART_Tx_Buf[14];
		alt = EstAlt_Baro +10000;	   //发送高度，因为气压高度有负值，所以这里加上10000修正为正值		   EstAlt_Baro  //临时
		ff = alt;
		UART_Tx_Buf[15]=ff;	 
		cal^=UART_Tx_Buf[15];
		UART_Tx_Buf[16]=ff>>8;
		cal^=UART_Tx_Buf[16];
		alt = AltHold_Baro +10000;	   //发送定高的高度				   //临时  AltHold_Baro	   GPS_altitude
		ff = alt;
		UART_Tx_Buf[17]=ff;
		cal^=UART_Tx_Buf[17];
		UART_Tx_Buf[18]=ff>>8;
		cal^=UART_Tx_Buf[18];
		UART_Tx_Buf[19]=Battery_Val-170;//电池电压,减170，然后上位机记得修正回去。。。Battery_Val
		cal^=UART_Tx_Buf[19];
		UART_Tx_Buf[20]=NAV_Speed/10;			//定速
		cal^=UART_Tx_Buf[20];
		UART_Tx_Buf[21]=constrain(angle[PITCH]/10+127,0,255);//PITCH角度	范围为-127至+128度
		cal^=UART_Tx_Buf[21];		  
		UART_Tx_Buf[22]=constrain(angle[ROLL]/10+127,0,255);			//ROLL	 范围为-127至+128
		cal^=UART_Tx_Buf[22];
		UART_Tx_Buf[23]=Xbee_Debug[0];//测试专用
		cal^=UART_Tx_Buf[23];
		UART_Tx_Buf[24]=Xbee_Debug[0]>>8;
		cal^=UART_Tx_Buf[24];
		UART_Tx_Buf[25]=Xbee_Debug[1];
		cal^=UART_Tx_Buf[25];
		UART_Tx_Buf[26]=Xbee_Debug[1]>>8;
		cal^=UART_Tx_Buf[26];
		UART_Tx_Buf[27]=Xbee_Debug[2];
		cal^=UART_Tx_Buf[27];
		UART_Tx_Buf[28]=Xbee_Debug[2]>>8;
		cal^=UART_Tx_Buf[28];
		
		UART_Tx_Buf[29]=cal;		 //校验码	
	}
	else if(RX_Data_Buf[1] == 0x11)	   //单点巡航目标点设置
	{
		Waypoint[0][LON] = (RX_Data_Buf[2] << 24) + (RX_Data_Buf[3] << 16) + (RX_Data_Buf[4] << 8) + RX_Data_Buf[5];
		Waypoint[0][LAT] = (RX_Data_Buf[6] << 24) + (RX_Data_Buf[7] << 16) + (RX_Data_Buf[8] << 8) + RX_Data_Buf[9];
		for(i=1;i<WaypointNum_Max;i++)  //将没用到的都清0
		{
			Waypoint[i][LON]=0;
			Waypoint[i][LAT]=0;	
		}
		GS_Nav = 1;		//置位有新任务标志位
		ReadyForNextWP=1; 
		ReadyToMove = 0;
		nav_mode = NAV_MODE_POSHOLD;	
		nav_takeoff_bearing = heading;   //保存当前航向
		wp=0;
		AutoHeading = 0;	//单点的强制为手动转向

		XbeePro_HeadResponse(1,0x04);	  //头段信息
		UART_Tx_Buf[5] = 0x04;	  //校验码
	}
	else if(RX_Data_Buf[1] == 0x12)	//设置定高高度
	{
		AltHold_Baro = RX_Data_Buf[2] + (RX_Data_Buf[3]<<8) - 10000;

		XbeePro_HeadResponse(1,0x04);	  //头段信息
		UART_Tx_Buf[5] = 0x04;	  //校验码
	}
	else if(RX_Data_Buf[1] == 0x13)	 //发送当前GPS定点的点坐标
	{
		XbeePro_HeadResponse(9,0x13);	  //头段信息
		cal=UART_Tx_Buf[4];
		j=GPS_hold[LAT];
		UART_Tx_Buf[5]=j&0xff;
		cal^=UART_Tx_Buf[5];
		UART_Tx_Buf[6]=(j>>8)&0xff;
		cal^=UART_Tx_Buf[6];
		UART_Tx_Buf[7]=(j>>16)&0xff;
		cal^=UART_Tx_Buf[7];
		UART_Tx_Buf[8]=(j>>24)&0xff;
		cal^=UART_Tx_Buf[8];
		j=GPS_hold[LON];
		UART_Tx_Buf[9]=j&0xff;
		cal^=UART_Tx_Buf[9];
		UART_Tx_Buf[10]=(j>>8)&0xff;
		cal^=UART_Tx_Buf[10];
		UART_Tx_Buf[11]=(j>>16)&0xff;
		cal^=UART_Tx_Buf[11];
		UART_Tx_Buf[12]=(j>>24)&0xff;
		cal^=UART_Tx_Buf[12];
		UART_Tx_Buf[13]=cal;//校验码	
	}
	else if(RX_Data_Buf[1] == 0x14)  //多点巡航航点设置
	{
		Waypoint[wp_count][LON]	= (RX_Data_Buf[2] << 24) + (RX_Data_Buf[3] << 16) + (RX_Data_Buf[4] << 8) + RX_Data_Buf[5];	//经度
		Waypoint[wp_count][LAT]	= (RX_Data_Buf[6] << 24) + (RX_Data_Buf[7] << 16) + (RX_Data_Buf[8] << 8) + RX_Data_Buf[9];//纬度
		wp_count ++;
		Waypoint[wp_count][LON]	= (RX_Data_Buf[10] << 24) + (RX_Data_Buf[11] << 16) + (RX_Data_Buf[12] << 8) + RX_Data_Buf[13];	//经度
		Waypoint[wp_count][LAT]	= (RX_Data_Buf[14] << 24) + (RX_Data_Buf[15] << 16) + (RX_Data_Buf[16] << 8) + RX_Data_Buf[17];//纬度
		wp_count ++;

		XbeePro_HeadResponse(1,0x14);	  //头段信息
		UART_Tx_Buf[5] = 0x14;	  //校验码
	}
	else if(RX_Data_Buf[1] == 0x15)  //开始多点巡航航点设置
	{
		wp_count = 0;
		XbeePro_HeadResponse(1,0x15);	  //头段信息
		UART_Tx_Buf[5] = 0x15;	  //校验码
	}
	else if(RX_Data_Buf[1] == 0x16)  //结束多点巡航航点设置,进行航点数校验
	{
		if(Waypoint[wp_count-1][LAT] == 0)
		{
			wp_count--;				   //wp_count在这里表示一个有多少个有效航点，从1开始不是从0开始，但是数组里保存的是从0开始
		}
		if(wp_count == RX_Data_Buf[2])	//航点个数符合
		{
			XbeePro_HeadResponse(2,0x16);	  //头段信息
			UART_Tx_Buf[5] = 0x01;							//航线设置成功
			UART_Tx_Buf[6] = 0x16 ^ 0x01;	  //校验码

			for(i=wp_count;i<WaypointNum_Max;i++)  //将没用到的都清0
			{
				Waypoint[i][LON]=0;
				Waypoint[i][LAT]=0;	
			}
			/*到这里说明航点已经接收完成，可以开始启动航线导航，在这里置位标志位*/
			GS_Nav = 1;		//置位有新任务标志位
			ReadyForNextWP=1; 
			ReadyToMove = 0;
			nav_mode = NAV_MODE_POSHOLD;	
			nav_takeoff_bearing = heading;   //保存当前航向
			wp=0;
		}
		else
		{
			XbeePro_HeadResponse(2,0x16);	  //头段信息
			UART_Tx_Buf[5] = 0x00;							 //航线设置失败
			UART_Tx_Buf[6] = 0x16 ^ 0x00;	  //校验码

			for(i=0;i<WaypointNum_Max;i++)  //航线设置失败，清空所有点
			{
				Waypoint[i][LON]=0;
				Waypoint[i][LAT]=0;	
			}
		}

	}
	else if(RX_Data_Buf[1] == 0x17)	 //地面站的键盘控制移动
	{
		GPS_hold[LAT] = GPS_hold[LAT] + (int32_t)(RX_Data_Buf[3]-50)*100;		//接收地面站键盘控制，那个乘数是粗略计算出来的
        GPS_hold[LON] = GPS_hold[LON] + (int32_t)(RX_Data_Buf[2]-50)*100;
		GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);

		magHold += (int16_t)(RX_Data_Buf[4] - 50)*10;
		
		XbeePro_HeadResponse(9,0x13);	  //头段信息
		cal=UART_Tx_Buf[4];
		j=GPS_hold[LAT];
		UART_Tx_Buf[5]=j&0xff;
		cal^=UART_Tx_Buf[5];
		UART_Tx_Buf[6]=(j>>8)&0xff;
		cal^=UART_Tx_Buf[6];
		UART_Tx_Buf[7]=(j>>16)&0xff;
		cal^=UART_Tx_Buf[7];
		UART_Tx_Buf[8]=(j>>24)&0xff;
		cal^=UART_Tx_Buf[8];
		j=GPS_hold[LON];
		UART_Tx_Buf[9]=j&0xff;
		cal^=UART_Tx_Buf[9];
		UART_Tx_Buf[10]=(j>>8)&0xff;
		cal^=UART_Tx_Buf[10];
		UART_Tx_Buf[11]=(j>>16)&0xff;
		cal^=UART_Tx_Buf[11];
		UART_Tx_Buf[12]=(j>>24)&0xff;
		cal^=UART_Tx_Buf[12];
		UART_Tx_Buf[13]=cal;//校验码
	}
	else if(RX_Data_Buf[1] == 0x18)		//一键降落一键起飞等
	{
		switch (RX_Data_Buf[2])
		{
			case 0xff:	//一键降落
				f.AUTOLANDING=1;
				break;
			case 0xfe:	//一键返航
				GPS_hold[LAT]=GPS_home[LAT];
				GPS_hold[LON]=GPS_home[LON];
				GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
				break;
			case 0xfd:	//一键悬停
				nav_mode = NAV_MODE_POSHOLD;
				GPS_hold[LAT] = GPS_coord[LAT];	 //保存当前所在坐标
				GPS_hold[LON] = GPS_coord[LON];
				ReadyForNextWP=0;
				GS_Nav = 0;		//没有新航点时 ，保证不更行
				break;
			case 0xfc:	//手动转向
				AutoHeading = 0;
				break;
			case 0xfb:	//自动转向
				AutoHeading = 1;
				break;
			case 0xfa:
				Vision_ErrorZero_Cal = 1;
				break;
			default:	//一键起飞
				//EasyLaunching_MODE = 1;
				AltHold_TakeOff = 100*RX_Data_Buf[2];
				//AltHold_Baro = 1000;
				if(f.OK_TO_ARM )
				{	
					f.ARMED = 1;
					f.AUTOTAKEOFF=1;
				}
		}
		XbeePro_HeadResponse(1,0x04);	  //头段信息
		UART_Tx_Buf[5] = 0x04;	  //校验码
	}
	else if(RX_Data_Buf[1] == 0x19)	//航速设置
	{
		NAV_Speed += (int16_t)(RX_Data_Buf[2] - 50)*10;
		XbeePro_HeadResponse(1,0x04);	  //头段信息
		UART_Tx_Buf[5] = 0x04;	  //校验码
	}
	else if(RX_Data_Buf[1] == 0x20)
	{
		//紧急消息通道，请查看static void XbeePro_SendMainInfo(void)函数，拥有很高的发送优先级
	}
	else if(RX_Data_Buf[1] == 0x21)	  //云台
	{
		XbeePro_HeadResponse(5,0x21);	  //头段信息
		UART_Tx_Buf[5]=0;//Camstab_Gain[0];
		UART_Tx_Buf[6]=0;//Camstab_Gain[1];
		UART_Tx_Buf[7]=0;//(Camstab_Midval[0]-1000)/4;
		UART_Tx_Buf[8]=0;//(Camstab_Midval[1]-1000)/4;
		UART_Tx_Buf[9] = UART_Tx_Buf[4]^UART_Tx_Buf[5]^UART_Tx_Buf[6]^UART_Tx_Buf[7]^UART_Tx_Buf[8];	  //校验码
	}
	else if(RX_Data_Buf[1] == 0x22)	//云台
	{
// 		Camstab_Gain[0] = RX_Data_Buf[2];
// 		Camstab_Gain[1] = RX_Data_Buf[3];
// 		Camstab_Midval[0] = RX_Data_Buf[4] * 4 + 1000;
// 		Camstab_Midval[1] = RX_Data_Buf[5] * 4 + 1000;

		XbeePro_HeadResponse(1,0x04);	  //头段信息
		UART_Tx_Buf[5] = 0x04;	  //校验码
	}
	else if(RX_Data_Buf[1]==0x30||RX_Data_Buf[1]==0x31||RX_Data_Buf[1]==0x32)//手柄姿态控制
	{
		HandAttitude_Ctrl(RX_Data_Buf);
		return ;
	}

	/*将数据发送出去*/
	DMA1->HIFCR = (uint32_t)((DMA_IT_TCIF4|DMA_IT_HTIF4|DMA_IT_TEIF4|DMA_IT_DMEIF4|DMA_IT_FEIF4) & 0x0F7D0F7D);//使能发送之前，必须清除各种中断标记
	DMA1_Stream4->CR &= ~(uint32_t)DMA_SxCR_EN;
	DMA1_Stream4->NDTR = UART_Tx_Buf[3]+5;			//此帧长度
	DMA1_Stream4->CR |= (uint32_t)DMA_SxCR_EN;
}

#endif







