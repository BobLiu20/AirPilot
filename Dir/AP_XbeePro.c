/**
	******************************************************************************
	* @file    AP_XbeePro.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   ��������ģ��XBEE PRO 900HP
	*					 ���ţ�PC10-UART4_TX
	*								 PC11-UART4_RX
	*					 �������ɼ��ݸ���ͨ��UART͸��������ģ�飬����������
	******************************************************************************
**/
#include "AP_XbeePro.h"
#ifdef XBEEPRO
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
//������Ҫ���͸���λ������Ҫ��Ϣ��λ0-7
//λ0����ʾѲ������ʱ���͸���λ����Ӧ�Ķ���λ��	��GPS.C  353��
//λ1:�ɻ�����ң����ʧЧ������IMA.C  309��
//λ2���ɻ��ڷ��й����г�����ǳ���100�ȣ��������ɻ��¹ʣ���������źŸ���λ��   IMA.C  907��
//λ3���ɻ��ϵ磬��ɳ�ʼ��			   main.c
//λ4������HOME�����λ��
unsigned char XbeePro_ImportanceInfo=0;

int16_t Xbee_Debug[3];//������ʾ��

/**********************����ͨѶЭ����***************************/
#define UART_RxMax 50	//���������ֽ���
#define UART_FramsMax 30//һ֡��������ֽ���
static unsigned char UART_Rx_Buf[UART_RxMax];//���ڽ��ջ���
static unsigned char Rx_CircleRead=0;		 //�������ݶ�ȡѭ����־
static unsigned char Rx_CircleRx=0;			 //�������ݽ���ѭ����־

static unsigned char RX_Data_Buf[UART_FramsMax];	//Ϊ�˽���У�����ݣ�������ʱ�������ݵ�
unsigned char UART_Tx_Buf[UART_FramsMax];			//���ͻ���

extern uint8_t Vision_ErrorZero_Cal;

/* function prototypes -----------------------------------------------*/
void XbeePro_SetAndTx(u8 c);

/*���ڽ���*/
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

/*��ʼ��*/
void XbeePro_Init(void)
{
	UART4_Init();	//��ʼ���õ��Ĵ���
}

/*���ݴ���*/
#define CircleReadAdd Rx_CircleRead++;if(Rx_CircleRead>=UART_RxMax)Rx_CircleRead = 0
void XbeePro_Process(void)
{
	OS_ERR err;
	u8 i=0,cal=0;
	if(Rx_CircleRead!=Rx_CircleRx)
	{
		if(UART_Rx_Buf[Rx_CircleRead]=='$')
		{
			OSTimeDly(30,OS_OPT_TIME_DLY,&err); //��ʱһ��ʱ���ٽ������ݴ�����λms
			CircleReadAdd;//��Rx_CircleRead�Լ�1
			if(UART_Rx_Buf[Rx_CircleRead]=='M')
			{
				CircleReadAdd;
				if(UART_Rx_Buf[Rx_CircleRead]=='<')
				{
					CircleReadAdd;
					RX_Data_Buf[0]=UART_Rx_Buf[Rx_CircleRead];	//��¼���ݸ�������ʾ������Ŷ��ٸ����ݣ�������У����
					CircleReadAdd;
					for(i=0;i<RX_Data_Buf[0];i++)//��ȡ����
					{
						RX_Data_Buf[1+i]=UART_Rx_Buf[Rx_CircleRead];
						if(i==0)cal=RX_Data_Buf[1+i];//У����
						else cal^=RX_Data_Buf[1+i];
						CircleReadAdd;
					}
					if(cal!=UART_Rx_Buf[Rx_CircleRead])//��֤���һλУ�����Ƿ��������������
					{	//У���벻��
						XbeePro_SetAndTx(0);//У������󴫵�
					}	
					else//У������ȷ
					{
						XbeePro_SetAndTx(1);
					}
					Rx_CircleRead=Rx_CircleRx;//����	
				}
			}
		}
		else
		{
			CircleReadAdd;
		}
	}
}

//ͷ��Ϣ�����ݸ���num��ʶ����msp
static void XbeePro_HeadResponse(u8 num,u8 msp)
{
	UART_Tx_Buf[0]='$';
	UART_Tx_Buf[1]='M';
	UART_Tx_Buf[2]='<';
	UART_Tx_Buf[3]=num;		 //���ݸ���
	UART_Tx_Buf[4]=msp;		 //ʶ����	
}

static void XbeePro_SendMainInfo(void)
{
	u32 j;
	u8 cal=0;
	if(XbeePro_ImportanceInfo&0x02)//�ɻ�����ң����ʧЧ����				//����
	{
		RX_Data_Buf[1] = 0x20;			 //ǿ�ƽ�������¹�ͨ��
		XbeePro_HeadResponse(2,0x20);	  //ͷ����Ϣ
		UART_Tx_Buf[5] = 0x01;
		UART_Tx_Buf[6] = 0x20^0x01;	  //У����
		XbeePro_ImportanceInfo&=0xfd;	
	}
	else if(XbeePro_ImportanceInfo&0x04)//�ɻ��ڷ��й����г�����ǳ���100��	    //����
	{
		RX_Data_Buf[1] = 0x20;			 //ǿ�ƽ�������¹�ͨ��
		XbeePro_HeadResponse(2,0x20);	  //ͷ����Ϣ
		UART_Tx_Buf[5] = 0x02;
		UART_Tx_Buf[6] = 0x20^0x02;	  //У����
		XbeePro_ImportanceInfo&=0xfb;	
	}
	else if(XbeePro_ImportanceInfo&0x08)//�ɻ��ϵ磬��ɳ�ʼ��
	{
		RX_Data_Buf[1] = 0x20;			 //ǿ�ƽ�������¹�ͨ��
		XbeePro_HeadResponse(2,0x20);	  //ͷ����Ϣ
		UART_Tx_Buf[5] = 0x03;
		UART_Tx_Buf[6] = 0x20^0x03;	  //У����
		XbeePro_ImportanceInfo&=0xf7;
	}
	else if(XbeePro_ImportanceInfo&0x01)//Ѳ�����ʱ���Ͷ���λ��
	{
		if(RX_Data_Buf[1] != 0x10)return ;
		RX_Data_Buf[1] = 0x13;//ǿ���޸�Ϊ���Ͷ���λ��
		XbeePro_ImportanceInfo&=0xfe;	
	}
	else if(XbeePro_ImportanceInfo&0x10)//����HOME�����λ��
	{
		RX_Data_Buf[1] = 0x20;			 //ǿ�ƽ�������¹�ͨ��
		XbeePro_HeadResponse(9,0x23);	  //ͷ����Ϣ
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
		UART_Tx_Buf[13]=cal;//У����
		XbeePro_ImportanceInfo&=0xef;	
	}
}

/*���ò����ͻ��Ͳ�������λ��*/
void XbeePro_SetAndTx(u8 c)
{
	u8 cal=0,i=0;
	u16 ff;
	s32 alt;
	u32 j;
	static u8 wp_count = 0;	 //�ຽ�����������
	if(c==0)//У�������
	{	/*�ɸ�����Ҫ�����Ƿ�ظ�У�������Ϣ*/
//		XbeePro_HeadResponse(1,0x02);	  //У����󣬻ظ�У������ź�
//		UART_Tx_Buf[5] = 0x02;	  //У����
//		/*�����ݷ��ͳ�ȥ*/
//		DMA2_Channel5->CCR &= (uint16_t)(~DMA_CCR1_EN);
//		DMA2_Channel5->CNDTR = UART_Tx_Buf[3]+5;//֡����
//		DMA2_Channel5->CCR |= DMA_CCR1_EN;
		return ;	
	}
	if(XbeePro_ImportanceInfo)XbeePro_SendMainInfo();//��Ҫ��Ϣ����
	if(RX_Data_Buf[1] == 0x10)		 //����GPS���ݸ���λ��
	{
		XbeePro_HeadResponse(25,0x10);	  //ͷ����Ϣ
		cal=UART_Tx_Buf[4];
		UART_Tx_Buf[5]=(f.GPS_FIX<<7)|(AutoHeading<<6)|(((heading<0)?(360+heading):(heading))/10);	//���λ��GPS�Ƿ���Чλ������λΪ�����ֶ��Զ�����6λ�Ƿɻ�ͷ
		cal^=UART_Tx_Buf[5];
		UART_Tx_Buf[6]=(0<<7)|GPS_numSat;	 //���λ�ǵ�������ָʾ������Ϊ������					Battery_LowPower
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
		alt = EstAlt_Baro +10000;	   //���͸߶ȣ���Ϊ��ѹ�߶��и�ֵ�������������10000����Ϊ��ֵ		   EstAlt_Baro  //��ʱ
		ff = alt;
		UART_Tx_Buf[15]=ff;	 
		cal^=UART_Tx_Buf[15];
		UART_Tx_Buf[16]=ff>>8;
		cal^=UART_Tx_Buf[16];
		alt = AltHold_Baro +10000;	   //���Ͷ��ߵĸ߶�				   //��ʱ  AltHold_Baro	   GPS_altitude
		ff = alt;
		UART_Tx_Buf[17]=ff;
		cal^=UART_Tx_Buf[17];
		UART_Tx_Buf[18]=ff>>8;
		cal^=UART_Tx_Buf[18];
		UART_Tx_Buf[19]=Battery_Val-170;//��ص�ѹ,��170��Ȼ����λ���ǵ�������ȥ������Battery_Val
		cal^=UART_Tx_Buf[19];
		UART_Tx_Buf[20]=NAV_Speed/10;			//����
		cal^=UART_Tx_Buf[20];
		UART_Tx_Buf[21]=constrain(angle[PITCH]/10+127,0,255);//PITCH�Ƕ�	��ΧΪ-127��+128��
		cal^=UART_Tx_Buf[21];		  
		UART_Tx_Buf[22]=constrain(angle[ROLL]/10+127,0,255);			//ROLL	 ��ΧΪ-127��+128
		cal^=UART_Tx_Buf[22];
		UART_Tx_Buf[23]=Xbee_Debug[0];//����ר��
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
		
		UART_Tx_Buf[29]=cal;		 //У����	
	}
	else if(RX_Data_Buf[1] == 0x11)	   //����Ѳ��Ŀ�������
	{
		Waypoint[0][LON] = (RX_Data_Buf[2] << 24) + (RX_Data_Buf[3] << 16) + (RX_Data_Buf[4] << 8) + RX_Data_Buf[5];
		Waypoint[0][LAT] = (RX_Data_Buf[6] << 24) + (RX_Data_Buf[7] << 16) + (RX_Data_Buf[8] << 8) + RX_Data_Buf[9];
		for(i=1;i<WaypointNum_Max;i++)  //��û�õ��Ķ���0
		{
			Waypoint[i][LON]=0;
			Waypoint[i][LAT]=0;	
		}
		GS_Nav = 1;		//��λ���������־λ
		ReadyForNextWP=1; 
		ReadyToMove = 0;
		nav_mode = NAV_MODE_POSHOLD;	
		nav_takeoff_bearing = heading;   //���浱ǰ����
		wp=0;
		AutoHeading = 0;	//�����ǿ��Ϊ�ֶ�ת��

		XbeePro_HeadResponse(1,0x04);	  //ͷ����Ϣ
		UART_Tx_Buf[5] = 0x04;	  //У����
	}
	else if(RX_Data_Buf[1] == 0x12)	//���ö��߸߶�
	{
		AltHold_Baro = RX_Data_Buf[2] + (RX_Data_Buf[3]<<8) - 10000;

		XbeePro_HeadResponse(1,0x04);	  //ͷ����Ϣ
		UART_Tx_Buf[5] = 0x04;	  //У����
	}
	else if(RX_Data_Buf[1] == 0x13)	 //���͵�ǰGPS����ĵ�����
	{
		XbeePro_HeadResponse(9,0x13);	  //ͷ����Ϣ
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
		UART_Tx_Buf[13]=cal;//У����	
	}
	else if(RX_Data_Buf[1] == 0x14)  //���Ѳ����������
	{
		Waypoint[wp_count][LON]	= (RX_Data_Buf[2] << 24) + (RX_Data_Buf[3] << 16) + (RX_Data_Buf[4] << 8) + RX_Data_Buf[5];	//����
		Waypoint[wp_count][LAT]	= (RX_Data_Buf[6] << 24) + (RX_Data_Buf[7] << 16) + (RX_Data_Buf[8] << 8) + RX_Data_Buf[9];//γ��
		wp_count ++;
		Waypoint[wp_count][LON]	= (RX_Data_Buf[10] << 24) + (RX_Data_Buf[11] << 16) + (RX_Data_Buf[12] << 8) + RX_Data_Buf[13];	//����
		Waypoint[wp_count][LAT]	= (RX_Data_Buf[14] << 24) + (RX_Data_Buf[15] << 16) + (RX_Data_Buf[16] << 8) + RX_Data_Buf[17];//γ��
		wp_count ++;

		XbeePro_HeadResponse(1,0x14);	  //ͷ����Ϣ
		UART_Tx_Buf[5] = 0x14;	  //У����
	}
	else if(RX_Data_Buf[1] == 0x15)  //��ʼ���Ѳ����������
	{
		wp_count = 0;
		XbeePro_HeadResponse(1,0x15);	  //ͷ����Ϣ
		UART_Tx_Buf[5] = 0x15;	  //У����
	}
	else if(RX_Data_Buf[1] == 0x16)  //�������Ѳ����������,���к�����У��
	{
		if(Waypoint[wp_count-1][LAT] == 0)
		{
			wp_count--;				   //wp_count�������ʾһ���ж��ٸ���Ч���㣬��1��ʼ���Ǵ�0��ʼ�����������ﱣ����Ǵ�0��ʼ
		}
		if(wp_count == RX_Data_Buf[2])	//�����������
		{
			XbeePro_HeadResponse(2,0x16);	  //ͷ����Ϣ
			UART_Tx_Buf[5] = 0x01;							//�������óɹ�
			UART_Tx_Buf[6] = 0x16 ^ 0x01;	  //У����

			for(i=wp_count;i<WaypointNum_Max;i++)  //��û�õ��Ķ���0
			{
				Waypoint[i][LON]=0;
				Waypoint[i][LAT]=0;	
			}
			/*������˵�������Ѿ�������ɣ����Կ�ʼ�������ߵ�������������λ��־λ*/
			GS_Nav = 1;		//��λ���������־λ
			ReadyForNextWP=1; 
			ReadyToMove = 0;
			nav_mode = NAV_MODE_POSHOLD;	
			nav_takeoff_bearing = heading;   //���浱ǰ����
			wp=0;
		}
		else
		{
			XbeePro_HeadResponse(2,0x16);	  //ͷ����Ϣ
			UART_Tx_Buf[5] = 0x00;							 //��������ʧ��
			UART_Tx_Buf[6] = 0x16 ^ 0x00;	  //У����

			for(i=0;i<WaypointNum_Max;i++)  //��������ʧ�ܣ�������е�
			{
				Waypoint[i][LON]=0;
				Waypoint[i][LAT]=0;	
			}
		}

	}
	else if(RX_Data_Buf[1] == 0x17)	 //����վ�ļ��̿����ƶ�
	{
		GPS_hold[LAT] = GPS_hold[LAT] + (int32_t)(RX_Data_Buf[3]-50)*100;		//���յ���վ���̿��ƣ��Ǹ������Ǵ��Լ��������
        GPS_hold[LON] = GPS_hold[LON] + (int32_t)(RX_Data_Buf[2]-50)*100;
		GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);

		magHold += (int16_t)(RX_Data_Buf[4] - 50)*10;
		
		XbeePro_HeadResponse(9,0x13);	  //ͷ����Ϣ
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
		UART_Tx_Buf[13]=cal;//У����
	}
	else if(RX_Data_Buf[1] == 0x18)		//һ������һ����ɵ�
	{
		switch (RX_Data_Buf[2])
		{
			case 0xff:	//һ������
				f.AUTOLANDING=1;
				break;
			case 0xfe:	//һ������
				GPS_hold[LAT]=GPS_home[LAT];
				GPS_hold[LON]=GPS_home[LON];
				GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
				break;
			case 0xfd:	//һ����ͣ
				nav_mode = NAV_MODE_POSHOLD;
				GPS_hold[LAT] = GPS_coord[LAT];	 //���浱ǰ��������
				GPS_hold[LON] = GPS_coord[LON];
				ReadyForNextWP=0;
				GS_Nav = 0;		//û���º���ʱ ����֤������
				break;
			case 0xfc:	//�ֶ�ת��
				AutoHeading = 0;
				break;
			case 0xfb:	//�Զ�ת��
				AutoHeading = 1;
				break;
			case 0xfa:
				Vision_ErrorZero_Cal = 1;
				break;
			default:	//һ�����
				//EasyLaunching_MODE = 1;
				AltHold_TakeOff = 100*RX_Data_Buf[2];
				//AltHold_Baro = 1000;
				if(f.OK_TO_ARM )
				{	
					f.ARMED = 1;
					f.AUTOTAKEOFF=1;
				}
		}
		XbeePro_HeadResponse(1,0x04);	  //ͷ����Ϣ
		UART_Tx_Buf[5] = 0x04;	  //У����
	}
	else if(RX_Data_Buf[1] == 0x19)	//��������
	{
		NAV_Speed += (int16_t)(RX_Data_Buf[2] - 50)*10;
		XbeePro_HeadResponse(1,0x04);	  //ͷ����Ϣ
		UART_Tx_Buf[5] = 0x04;	  //У����
	}
	else if(RX_Data_Buf[1] == 0x20)
	{
		//������Ϣͨ������鿴static void XbeePro_SendMainInfo(void)������ӵ�кܸߵķ������ȼ�
	}
	else if(RX_Data_Buf[1] == 0x21)	  //��̨
	{
		XbeePro_HeadResponse(5,0x21);	  //ͷ����Ϣ
		UART_Tx_Buf[5]=0;//Camstab_Gain[0];
		UART_Tx_Buf[6]=0;//Camstab_Gain[1];
		UART_Tx_Buf[7]=0;//(Camstab_Midval[0]-1000)/4;
		UART_Tx_Buf[8]=0;//(Camstab_Midval[1]-1000)/4;
		UART_Tx_Buf[9] = UART_Tx_Buf[4]^UART_Tx_Buf[5]^UART_Tx_Buf[6]^UART_Tx_Buf[7]^UART_Tx_Buf[8];	  //У����
	}
	else if(RX_Data_Buf[1] == 0x22)	//��̨
	{
// 		Camstab_Gain[0] = RX_Data_Buf[2];
// 		Camstab_Gain[1] = RX_Data_Buf[3];
// 		Camstab_Midval[0] = RX_Data_Buf[4] * 4 + 1000;
// 		Camstab_Midval[1] = RX_Data_Buf[5] * 4 + 1000;

		XbeePro_HeadResponse(1,0x04);	  //ͷ����Ϣ
		UART_Tx_Buf[5] = 0x04;	  //У����
	}
	else if(RX_Data_Buf[1]==0x30||RX_Data_Buf[1]==0x31||RX_Data_Buf[1]==0x32)//�ֱ���̬����
	{
		HandAttitude_Ctrl(RX_Data_Buf);
		return ;
	}

	/*�����ݷ��ͳ�ȥ*/
	DMA1->HIFCR = (uint32_t)((DMA_IT_TCIF4|DMA_IT_HTIF4|DMA_IT_TEIF4|DMA_IT_DMEIF4|DMA_IT_FEIF4) & 0x0F7D0F7D);//ʹ�ܷ���֮ǰ��������������жϱ��
	DMA1_Stream4->CR &= ~(uint32_t)DMA_SxCR_EN;
	DMA1_Stream4->NDTR = UART_Tx_Buf[3]+5;			//��֡����
	DMA1_Stream4->CR |= (uint32_t)DMA_SxCR_EN;
}

#endif







