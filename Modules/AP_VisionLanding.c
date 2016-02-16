/**
	******************************************************************************
	* @file    AP_VisionLanding.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.8
	* @brief   �Ӿ������Զ����䣬�Զ�������
	*					 ע�⣺�����ģ�鹲�ô���ͨ��
	******************************************************************************
**/
#include "AP_VisionLanding.h"
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
static uint16_t VisionLanding_LostCnt=0;//����ʵʱ����Ƿ�ʧ�Ӿ�����ϵͳ,�ڽ������ݴ�����ʵʱ��0

int16_t VisionLanding_Altitude=0;//cm����ģ���л�ȡ�ĳ����߶�
int16_t VisionLanding_Angle[2];

static int16_t Vision_PixelError[2];		//����ƫ��
static float Vision_Error[2] =	{	0.0f, 0.0f };//ʵ��ƫ�ƾ��룬M
static PID_DATA VL_piddata[2];
static PID_PARAM VL_pidparam;

/* OS -----------------------------------------------*/
static  void  VisionLanding_MainTask (void *p_arg);
static  OS_TCB  VisionLanding_MainTaskTCB;
static  CPU_STK  VisionLanding_MainTaskStk[APP_CFG_TASK_START_STK_SIZE];

OS_FLAG_GRP VisionLanding_TaskRunFlagGrp;	//���������¼���־��

/* function prototypes -----------------------------------------------*/
static void VisionLanding_ErrorEstimation(int16_t *ang ,int16_t *pixelError ,int16_t alt,float *error);
static void VisionLanding_PID_Init(void);
static void VisionLanding_AngleUpdate(void);

/**
  * @brief  ��ʼ��
  *         ��������
  * @param  None
  * @retval None
  */
void VisionLanding_Init(void)
{
	OS_ERR	err;
	
	OSFlagCreate(&VisionLanding_TaskRunFlagGrp,	//�����¼���־��
						"VisionLanding_TaskRunFlagGrp",
						(OS_FLAGS)0,//������б��
						&err);
	
	OSTaskCreate((OS_TCB   *)&VisionLanding_MainTaskTCB,  
				 (CPU_CHAR     *)"VisionLanding_MainTask",
				 (OS_TASK_PTR   )VisionLanding_MainTask, 
				 (void         *)0,
				 (OS_PRIO       )PRIO_VISIONLANDING,
				 (CPU_STK      *)&VisionLanding_MainTaskStk[0],
				 (CPU_STK_SIZE  )VisionLanding_MainTaskStk[APP_CFG_TASK_START_STK_SIZE / 10],
				 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
				 (OS_MSG_QTY    )0,
				 (OS_TICK       )0,
				 (void         *)0,
				 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 (OS_ERR       *)&err);
	
	VisionLanding_PID_Init();
}

/**
  * @brief  VisionLanding������
  *         
  * @param  None
  * @retval None
  */
static  void  VisionLanding_MainTask (void *p_arg)
{
	OS_ERR	err;
	CPU_TS ts;
  (void)p_arg;
  while (1)
	{
		OSTimeDly(5,OS_OPT_TIME_DLY,&err);//ms
		
		if(VisionLanding_LostCnt++>200)//��VisionLanding_AltitudeΪ0ʱ˵���������ʧ��    || VisionLanding_Altitude==0
		{
			f.VISION_AVAILABLE = 0;//�������Ӿ�����ģ�����������Ч
		}
		else
		{
			f.VISION_AVAILABLE = 1;//����ģ�飬��������
		}
		
		if(RC_Options[BOXVISION] && f.VISION_AVAILABLE )
		{
			f.VISION_MODE = 1;//�����Ӿ�����
		}
		else
		{
			f.VISION_MODE = 0;
		}
		
		OSFlagPend(&VisionLanding_TaskRunFlagGrp,
							(OS_FLAGS)0x01,			//ʹ�õ�0λ��Ϊ����¼�
							0,
							OS_OPT_PEND_NON_BLOCKING+OS_OPT_PEND_FLAG_SET_ANY,
							&ts,
							&err);
		if(err==OS_ERR_NONE)//�������ݸ���ʱ���Ž��봦��
		{
			static uint8_t counter = 0;
			static int16_t PreVision_PixelError[2] = {0,0};
			OSFlagPost(&VisionLanding_TaskRunFlagGrp,//������
								(OS_FLAGS)0x01,
								OS_OPT_POST_FLAG_CLR,
								&err);
// 			if(abs(Vision_PixelError[0] - PreVision_PixelError[0]) > 20 || abs(Vision_PixelError[1] - PreVision_PixelError[1]) > 20 )
// 			{
// 				

// 				if(++counter<3)
// 				{
// 					Vision_PixelError[0] = PreVision_PixelError[0];
// 					Vision_PixelError[1] = PreVision_PixelError[1];
// 					
// 				}
// 				else
// 					counter = 0;
// 			}
// 			PreVision_PixelError[0] = Vision_PixelError[0];
// 			PreVision_PixelError[1] = Vision_PixelError[1];		
			
	
			if(f.SONAR_AVAILABLE)			
				VisionLanding_ErrorEstimation(angle ,Vision_PixelError ,EstAlt_Sonar,Vision_Error);//����ʵ�ʾ���			
			else		
				VisionLanding_ErrorEstimation(angle ,Vision_PixelError ,EstAlt_Baro,Vision_Error);//����ʵ�ʾ���						

//				debug[1] = Vision_Error[0];

			if(f.VISION_MODE)
			{
				VisionLanding_AngleUpdate();//
			}
			else
			{
				VisionLanding_Angle[0] = 0;
				VisionLanding_Angle[1] = 0;
				PID_reset(&VL_piddata[0]);
				PID_reset(&VL_piddata[1]);
			}
		}
  }
}

static void VisionLanding_PID_Init(void)
{
	
	VL_pidparam.kP = 2.2f;  //2.0
	VL_pidparam.kI = 0.3f;//0.1
	VL_pidparam.kD = 1.4f;    //1.0
	VL_pidparam.Imax = 30.0f;
	VL_pidparam.apply_lpf = 1;
	
	PID_reset(&VL_piddata[0]);
	PID_reset(&VL_piddata[1]);
	
}

static void VisionLanding_AngleUpdate(void)
{
	int8_t axis;
	float dt;
	float DecayGain;
	static uint32_t dt_pre;
	static float PreVision_Error[2] = {0 ,0};
	static int8_t invalid_counter = 0;
	dt = (TimUsGet() - dt_pre) * 0.000001f;
	dt_pre = TimUsGet();

	if(Vision_PixelError[0]==120||Vision_PixelError[0]==-120||Vision_PixelError[1]==120||Vision_PixelError[1]==-120)//��Ч	
	{
		if(++invalid_counter>2)
		{
			VisionLanding_Angle[0] = 0;
			VisionLanding_Angle[1] = 0;
			PID_reset(&VL_piddata[0]);
			PID_reset(&VL_piddata[1]);
//			PID_reset(&VL_piddata);
			invalid_counter = 3;
		}

	}
	else
	{
		DecayGain = constrain (1.0f - 0.0015f * ( EstAlt_Baro ) ,0.1f ,1.0f );
		for(axis=0;	axis<2;	axis++)	
		{
			Vision_Error[axis] = PreVision_Error[axis] * 0.6 + Vision_Error[axis] * 0.4;
			PreVision_Error[axis] = Vision_Error[axis];
			VisionLanding_Angle[axis] = apply_pid(&VL_piddata[axis], Vision_Error[axis]*100, dt, &VL_pidparam) ;		
			
			VisionLanding_Angle[axis] *= DecayGain;
			VisionLanding_Angle[axis] = constrain( VisionLanding_Angle[axis] , -80 , 80 );
		}
		invalid_counter = 0;

	}
	
	if(f.AUTOTAKEOFF && EstAlt_Sonar < 10.0f)
	{
			VisionLanding_Angle[0] = 0;
			VisionLanding_Angle[1] = 0;
			PID_reset(&VL_piddata[0]);
			PID_reset(&VL_piddata[1]);
	}

// 	debug[1] = Vision_Error[0] * 100;
// 	debug[2] = VisionLanding_Angle[0];


}

uint8_t Vision_ErrorZero_Cal=0;
#define focal_length_px (2.1 / (2.0f * 6.0f) * 1000.0f) //original focal lenght: 12mm pixelsize: 6um, binning 4 enabled
#define M_PI       3.14159265358979323846f
static void VisionLanding_ErrorEstimation(int16_t *ang ,int16_t *pixelError ,int16_t alt,float *error)
{
	uint8_t axis;
	static float Pre_alt = 0;
	alt = Pre_alt * 0.6 + (float)alt * 0.4;
	Pre_alt = alt;
	for(axis=0;	axis<2;	axis++)
	{
		error[axis]	=	( pixelError[axis] / focal_length_px  +  tanf((float)ang[axis]/10 * ( M_PI / 180.0f )) ) * (float)(alt+10)/100.0f ;
		
// 		if(Vision_ErrorZero_Cal)
// 		{
// 			cfg.Vision_ErrorZero[axis] = error[axis] * 100.0f;
// 			if(axis==1)Vision_ErrorZero_Cal = 0;
// 		}
		error[axis] -= ((float)cfg.Vision_ErrorZero[axis]/100.0f);//У׼
	}
	
	debug[0] = pixelError[0] / focal_length_px *100;
	debug[1] = tanf((float)ang[0]/10 * ( M_PI / 180.0f )) * 100  ;
	

}

/**
  * @brief  ��ģ���н���ԭʼ����
  * 
  * @param  None
  * @retval None
  */
void VisionLanding_ReceiveRawData(uint8_t *buf)
{
	OS_ERR err;
	Vision_PixelError[0]=(int8_t)buf[1];
	Vision_PixelError[1]=(int8_t)buf[2];
	VisionLanding_Altitude=buf[3]|(buf[4]<<8);

	VisionLanding_LostCnt	= 0;//ʵʱ��0��˵������ģ��
	
	OSFlagPost(&VisionLanding_TaskRunFlagGrp,//��������
					(OS_FLAGS)0x01,
					OS_OPT_POST_FLAG_SET,
					&err);

}
