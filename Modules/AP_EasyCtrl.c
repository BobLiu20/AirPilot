/**
	******************************************************************************
	* @file    AP_EasyCtrl.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   �û�����ģ�飬�������ֱ���̬���׿��Ƶȣ�����ң����
	*					 �����ֱ���ң�����������
	*					 ��ң�������ֱ�ͬʱ����ʱ����AUX2Ϊ��־������ʹ��˭
	*					 ��ֻ��������һ��ʱ�����ɵ�������
	*					 �����ʹ�ù����У��˿���ͻȻʧЧ������Ϊ����һ�ִ��ڣ�ǿ���л�����һ��
	*					 ��һ�ֲ����ڣ��Զ�����
	******************************************************************************
**/
#include "AP_EasyCtrl.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
int16_t HandAttitude_CtrlAngle[2];	//���ƵĽǶȣ�-35----+35
static int16_t HandAttitude_RcDataTh=1000;//ģ�����ŵ���ֵ1000-2000

int16_t RC_Data[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 }; //ʵ��ʹ�õ�ͨ������,��ΧΪ1000-2000

static uint8_t GPSFollow_Mode;//1ΪGPS����ģʽ

static uint16_t OneKey_Roll_Trig;//�����ֱ�����һ������

/* OS -----------------------------------------------*/
static  void  EasyCtrl_MainTask (void *p_arg);
static  OS_TCB  EasyCtrl_MainTaskTCB;
static  CPU_STK  EasyCtrl_MainTaskStk[APP_CFG_TASK_START_STK_SIZE];

/* function prototypes -----------------------------------------------*/
void OneKey_Roll(void);

/**
  * @brief  ��ʼ��
  *         ��������
  * @param  None
  * @retval None
  */
void EasyCtrl_Init(void)
{
	OS_ERR	err;
	OSTaskCreate((OS_TCB   *)&EasyCtrl_MainTaskTCB,  
				 (CPU_CHAR     *)"EasyCtrl_MainTask",
				 (OS_TASK_PTR   )EasyCtrl_MainTask, 
				 (void         *)0,
				 (OS_PRIO       )PRIO_EASYCTRL ,
				 (CPU_STK      *)&EasyCtrl_MainTaskStk[0],
				 (CPU_STK_SIZE  )EasyCtrl_MainTaskStk[APP_CFG_TASK_START_STK_SIZE / 10],
				 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
				 (OS_MSG_QTY    )0,
				 (OS_TICK       )0,
				 (void         *)0,
				 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 (OS_ERR       *)&err);
}

/**
  * @brief  EasyCtrl������
  *         
  * @param  None
  * @retval None
  */
static  void  EasyCtrl_MainTask (void *p_arg)
{
	OS_ERR	err;
	uint8_t i;
	static uint8_t first_in=1;
  (void)p_arg;
  while (1)
	{
		OSTimeDly(2,OS_OPT_TIME_DLY,&err);//ms
		if((f.HANDLE_AVAILABLE==0)||(f.RC_AVAILABLE && (RC_RawData[AUX2] > 1600)))//ʹ��ң��������
		{
			f.RC_CTRL_MODE=1;
			f.HANDLE_CTRL_MODE=0;
		}
		else if(f.HANDLE_AVAILABLE)//�ֱ�ģʽ
		{
			f.RC_CTRL_MODE=0;
			f.HANDLE_CTRL_MODE=1;
		}
		else//δ֪����BUG
		{
			f.RC_CTRL_MODE=0;
			f.HANDLE_CTRL_MODE=0;
		}
		
		if(f.RC_CTRL_MODE)//ʹ��ң����
		{
			for(i=0;i<8;i++)RC_Data[i]=RC_RawData[i];
			HandAttitude_CtrlAngle[0]=0;
			HandAttitude_CtrlAngle[1]=0;
		}
		else if(f.HANDLE_CTRL_MODE)//ʹ���ֱ�
		{
			RC_Data[ROLL]=1500;
			RC_Data[PITCH]=1500;
			RC_Data[YAW]=1500;
			RC_Data[THROTTLE]=HandAttitude_RcDataTh;
			if(GPSFollow_Mode)
				RC_Data[AUX1]=1100;//��������ģʽʱ�ͽ�AUX1����ͣ����ڵ��������ѡ�������GPS
			else
				RC_Data[AUX1]=1500;//�м䣬��Ҫ������ѹ�ƺʹ���
			RC_Data[AUX2]=1500;
		}
		else//�Ƚ�
		{
			HandAttitude_CtrlAngle[0]=0;
			HandAttitude_CtrlAngle[1]=0;
		}
		
		
		if(RC_Options[BOXROLL]||OneKey_Roll_Trig)
		{
			if(first_in&&RC_Data[THROTTLE]>1300)
			{
				first_in=0;
				OneKey_RollStep = 1;
			}
			if(OneKey_Roll_Trig++>500)//1s
				OneKey_Roll_Trig=0;
		}
		else
		{
			first_in=1;
			OneKey_RollStep = 0;
		}
		
		OneKey_Roll();//һ������
  }
}

/**
  * @brief  �ֱ���̬����
  *         
  * @param  ���յ�������ָ�룬��XBEE��
  * @retval None
  */
void HandAttitude_Ctrl(uint8_t *hand_data)
{
	static uint8_t first_in=0,first_in2=0;
	static int32_t GPS_HoldOffset[2];
	static double Point_Distance=0;
	int32_t gps_coord[2];
	static int32_t gps_coord2[2];
	int16_t mag_bear;
	float val;
	Handle_LostCnt=0;//ʵʱ��0�������

	
	if(f.HANDLE_CTRL_MODE==0)return;//û������ģʽ
	
	//��ͨ��GPS�µ���̬����    ��    ����
	if(hand_data[1]==0x30||hand_data[1]==0x31)
	{
		if(!(hand_data[7]&0x01))//û�п�����ͣ
		{
			HandAttitude_CtrlAngle[ROLL]=constrain((hand_data[2]-35),-35,35);//��λ��
			HandAttitude_CtrlAngle[PITCH]=constrain((hand_data[3]-35),-35,35);
			magHold=(hand_data[4]<<8)|hand_data[5];
		}
		else//��ͣ,����̬����
		{
			HandAttitude_CtrlAngle[ROLL]=0;
			HandAttitude_CtrlAngle[PITCH]=0;
		}
		
		if(hand_data[6]==1)//��������
		{
			if(HandAttitude_RcDataTh<1450)
			{
				if(f.ARMED)HandAttitude_RcDataTh+=5;
				f.BARO_MODE = 0;
				f.SONAR_MODE = 0;
				AltHold_Baro=50.0f;
				AltHold_Sonar = 50.0f;
			}
			else
			{
				AltHold_Baro+=3.0f;//Ӧ�ÿ�����1�����Ӱ���
				AltHold_Sonar += 3.0f;
			}
		}
		else if(hand_data[6]==2)//���½���
		{
			if(f.SONAR_MODE)//������
			{
				if(AltHold_Sonar>-50.0f)//ֻ�г����Ż���йرյ���ж�
				{
					AltHold_Sonar-=3.0f;
				}
				else
				{
					if(EstAlt_Sonar<20.0f)
						if(HandAttitude_RcDataTh>1000)HandAttitude_RcDataTh-=5;
				}
			}
			AltHold_Baro-=3.0f;//��ѹ
		}
		else if(hand_data[6]==4)//�ж���˫����������������߽���
		{
			if(f.ARMED)
				f.ARMED = 0;		//����
			else if(f.OK_TO_ARM&&!f.ARMED)
			{
				HandAttitude_RcDataTh=1000;
				AltHold_Baro=0;
				AltHold_Sonar=0.0f;
				CalibratingG = 1000;		//У׼������	1000
				CalibratingB = 10;			//У׼��ѹ�ƣ�Ҳ�����趨��ѹ��ԭ��
				f.ARMED = 1;		//����
			}
		}
		else if(hand_data[6]==3)//������������1�룬У׼
		{
			if(!f.ARMED)
			{
				CalibratingA = 400;		//�������ٶȼ�У׼
			}
		}
		//�ɻ������ֱ��˶�
		if(hand_data[1]==0x31)//����
		{
			gps_coord[LON]=(hand_data[11] << 24) + (hand_data[10] << 16) + (hand_data[9] << 8) + hand_data[8];
			gps_coord[LAT]=(hand_data[15] << 24) + (hand_data[14] << 16) + (hand_data[13] << 8) + hand_data[12];
			if(first_in)
			{
				first_in = 0;
				GPS_HoldOffset[LON]=GPS_coord[LON]-gps_coord[LON];
				GPS_HoldOffset[LAT]=GPS_coord[LAT]-gps_coord[LAT];
			}
			else
			{
				if(abs(GPS_hold[LON]-(gps_coord[LON]+GPS_HoldOffset[LON]))>50||abs(GPS_hold[LAT]-(gps_coord[LAT]+GPS_HoldOffset[LAT]))>50)
				{
					GPS_hold[LON]=gps_coord[LON]+GPS_HoldOffset[LON];//����
					GPS_hold[LAT]=gps_coord[LAT]+GPS_HoldOffset[LAT];//γ��
					GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
				}
			}
			GPSFollow_Mode = 1;
		}
		else
		{
			first_in = 1;
			if(hand_data[7]&0x01)//��ͣ
				GPSFollow_Mode = 1;
			else
				GPSFollow_Mode = 0;
		}
		
		if((hand_data[7]&0x04)&&f.SONAR_AVAILABLE  &&f.ARMED )//һ����ɻ���һ������
		{
			if(EstAlt_Sonar<15.0f)//take off
			{
				//magHold = heading-7;
				if (magHold > 180)
					magHold = magHold - 360;
				else if (magHold < -180)
					magHold = magHold + 360;
				HandAttitude_RcDataTh = 1450;
				AltHold_TakeOff = 150.0f;
				f.AUTOTAKEOFF=1;
				f.AUTOLANDING=0;
			}
			else//landing
			{
				f.AUTOLANDING=1;
				f.AUTOTAKEOFF=0;
			}
		}
		
		
		if(hand_data[7]&0x02)OneKey_Roll_Trig=1;//����һ������
		
		first_in2 = 1;
	}
	else if(hand_data[1]==0x32)//ָ�����
	{
		HandAttitude_CtrlAngle[ROLL]=0;
		HandAttitude_CtrlAngle[PITCH]=0;

		if(first_in2)
		{
			first_in2 = 0;//�����һ�ξ���
			
			gps_coord2[LON]=(hand_data[11] << 24) + (hand_data[10] << 16) + (hand_data[9] << 8) + hand_data[8];//�ֱ���GPS����
			gps_coord2[LAT]=(hand_data[15] << 24) + (hand_data[14] << 16) + (hand_data[13] << 8) + hand_data[12];
			Point_Distance = sqrt((gps_coord2[LON] - GPS_coord[LON])*(gps_coord2[LON] - GPS_coord[LON])+(gps_coord2[LAT] - GPS_coord[LAT])*(gps_coord2[LAT] - GPS_coord[LAT]));
		}
		else
		{
			mag_bear = (hand_data[4]<<8)|hand_data[5];
			gps_coord[LON] = gps_coord2[LON] + Point_Distance*sinf((float)mag_bear*2.0f*PI/360.0f);//����
			gps_coord[LAT] = gps_coord2[LAT] + Point_Distance*cosf((float)mag_bear*2.0f*PI/360.0f);//γ��
			if(abs(GPS_hold[LON]-gps_coord[LON])>50||abs(GPS_hold[LAT]-gps_coord[LAT])>50)
			{
				GPS_hold[LON]=gps_coord[LON];//����
				GPS_hold[LAT]=gps_coord[LAT];//γ��
				GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
			}
		}
		GPSFollow_Mode = 1;
	}
	else
	{
		GPSFollow_Mode = 0;
	}


//  	debug[0]=GPS_hold[LAT];
//  	debug[1]=mag_bear;
// 	debug[2]=Point_Distance;
// 	debug[3]=Point_Distance*cosf((float)mag_bear*2.0f*PI/360.0f);//γ��
}

uint8_t OneKey_RollStep;//ָʾ�����Ĳ��������뷭��ģʽʱ�����ر����ȣ�����3Dģʽ
void OneKey_Roll(void)
{
	static uint8_t step1_tim;
	static float alt_last=100;
	switch (OneKey_RollStep)
	{
		case 1://��һ��,˲����������һ��ʱ�䣬�ûһ�������
		{
			if(step1_tim++>100)//100x2ms=500ms
			{
				step1_tim=0;
				OneKey_RollStep=2;
				if(step1_tim ==1)
				{
					if(f.SONAR_MODE)
						alt_last = AltHold_Sonar;
					else
						alt_last = AltHold_Baro;
				}
			}
			RC_Data[THROTTLE]=2000;
		}
		break;
		case 2://�ڶ���,��
		{
			RC_Data[ROLL]=2000;//���󷭹�
			if(angle[ROLL]<-200&&angle[ROLL]>-900)//����������Ƕ�ʱ�����½�������ģʽ
			{
				OneKey_RollStep=0;
				if(f.SONAR_MODE)
					AltHold_Sonar = alt_last;
				else
					AltHold_Baro = alt_last;
			}
		}
		break;
		default:OneKey_RollStep=0;
	}
}



