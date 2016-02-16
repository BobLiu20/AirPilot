/**
	******************************************************************************
	* @file    AP_OpticalFlow.c
	* @author  AirPilot Team
	* @version V1.0.0
	* @date    2013.7
	* @brief   光流
	*					 使用USART通讯，详细见AP_USART.C
	******************************************************************************
**/
#include "AP_OpticalFlow.h"
#ifdef OPTICALFLOW
/* typedef -----------------------------------------------------------*/
/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
static uint8_t OpticalFlow_Data_Buf[20];//缓存解码出来的有用数据,其中第一位为识别码

static uint16_t OpticalFlow_LostCnt=0;//用于实时检测是否丢失光流,在接收数据处理中实时清0

static int16_t OpticalFlow_EstHVel[2]	=	{	0, 0 };	 //	horisontal velocity, cm/sec	(constrained -100, 100)
static uint8_t OpticalFlow_Qual;
int16_t OpticalFlow_Altitude=0;
int16_t OpticalFlow_Angle[2]={0,0};



/* OS -----------------------------------------------*/
static  void  OpticalFlow_MainTask (void *p_arg);
static  OS_TCB  OpticalFlow_MainTaskTCB;
static  CPU_STK  OpticalFlow_MainTaskStk[APP_CFG_TASK_START_STK_SIZE];

OS_FLAG_GRP OpticalFlow_TaskRunFlagGrp;	//任务运行事件标志组

/* function prototypes -----------------------------------------------*/
static void OpticalFlow_Com(void);
static void OpticalFlow_AngleUpdate(void);

/**
  * @brief  初始化
  *         创建任务
  * @param  None
  * @retval None
  */
void OpticalFlow_Init(void)
{
	OS_ERR	err;
	USART3_Init();
	
	OSFlagCreate(&OpticalFlow_TaskRunFlagGrp,	//创建事件标志组
					"OpticalFlow_TaskRunFlagGrp",
					(OS_FLAGS)0,//清除所有标记
					&err);
	
	OSTaskCreate((OS_TCB   *)&OpticalFlow_MainTaskTCB,  
				 (CPU_CHAR     *)"OpticalFlow_MainTask",
				 (OS_TASK_PTR   )OpticalFlow_MainTask, 
				 (void         *)0,
				 (OS_PRIO       )PRIO_OPTICALFLOW ,
				 (CPU_STK      *)&OpticalFlow_MainTaskStk[0],
				 (CPU_STK_SIZE  )OpticalFlow_MainTaskStk[APP_CFG_TASK_START_STK_SIZE / 10],
				 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
				 (OS_MSG_QTY    )0,
				 (OS_TICK       )0,
				 (void         *)0,
				 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 (OS_ERR       *)&err);
}

/**
  * @brief  OpticalFlow主任务
  *         
  * @param  None
  * @retval None
  */
static  void  OpticalFlow_MainTask (void *p_arg)
{
	OS_ERR	err;
	CPU_TS ts;
	static float qual_pre=0;
  (void)p_arg;
  while (1)
	{
		OSTimeDly(5,OS_OPT_TIME_DLY,&err);//ms
		
		OpticalFlow_Com();
		
		OpticalFlow_Qual = qual_pre * 0.9f + (float)OpticalFlow_Qual * 0.1;//质量滤波
		qual_pre = OpticalFlow_Qual;
		if((OpticalFlow_LostCnt++>200) || (OpticalFlow_Altitude>300))//要满足所有条件
		{
			f.OPTFLOW_AVAILABLE = 0;//光流无效
		}
		else
		{
			f.OPTFLOW_AVAILABLE = 1;//光流有效
		}
		
		if(RC_Options[BOXOPTFLOW] && f.OPTFLOW_AVAILABLE && OpticalFlow_Qual > 30)
		{
			f.OPTFLOW_MODE=1;//开启光流模式
		}
		else
		{
			OpticalFlow_Angle[0]=0;
			OpticalFlow_Angle[1]=0;
			f.OPTFLOW_MODE=0;//关闭光流模式
		}
		
 		OSFlagPend(&OpticalFlow_TaskRunFlagGrp,
							(OS_FLAGS)0x01,			//使用第0位作为标记事件
							0,
							OS_OPT_PEND_NON_BLOCKING+OS_OPT_PEND_FLAG_SET_ANY,
							&ts,
							&err);
		if(err==OS_ERR_NONE)//当有数据更新时，才进入处理
		{
			OSFlagPost(&OpticalFlow_TaskRunFlagGrp,//清除标记
								(OS_FLAGS)0x01,
								OS_OPT_POST_FLAG_CLR,
								&err);

			OpticalFlow_AngleUpdate();//计算PID
 		}
  }
}

/* Lense focal distance, mm (set it for your own lense) 
 (How to check: debug4 in GUI should not react on ROLL tilt, but react on ROLL slide) */
#define OF_FOCAL_DIST 8//9//镜头的焦距，mm
/* Deadband for ROLL,PITCH sticks where position hold is enabled. Max value 100 */
#define OF_DEADBAND 15
/* Rotate I-term with heading rotation. It will well compensate wind */
#define OF_ROTATE_I
/* Low-pass filter factor to prevent shaking. Possible values 1..8.  Default is 5. */
#define OF_LPF_FACTOR 5

/* Exponential moving average filter (optimized for integers) with factor = 2^n */
typedef struct avg_var16 {
  int32_t buf; // internal bufer to store non-rounded average value
  int16_t res; // result (rounded to int)
} t_avg_var16;

typedef struct avg_var8 {
  int16_t buf; // internal bufer to store non-rounded average value
  int8_t res; // result (rounded to int)
} t_avg_var8;

/* Rotate vector V(x,y) to angle delta (in 0.1 degree) using small angle approximation and integers. */
/* (Not precise but fast) */
void rotate16(int16_t *V, int16_t delta) {
  int16_t tmp = V[0];
  V[0]-= (int16_t)( ((int32_t)delta) * V[1] / 573);
  V[1]+= (int16_t)( ((int32_t)delta) * tmp / 573); 
}

void average16(struct avg_var16 *avg, int16_t cur, int8_t n) {
	avg->buf+= cur - avg->res;
	avg->res = avg->buf >> n;
}
/* n=(1..8) */
void average8(struct avg_var8 *avg, int8_t cur, int8_t n) {
	avg->buf+= cur - avg->res;
	avg->res = avg->buf >> n;
}


static void OpticalFlow_AngleUpdate(void)
{
	static int16_t optflowErrorI[2]	=	{	0, 0 };
	static int16_t prevHeading = 0;
	//static int8_t	optflowUse = 0;
	int8_t axis;
	int16_t	dif;
//	static uint8_t qual_count=0;

    cfg.P8[PIDVEL] = 2.0f;
    cfg.I8[PIDVEL] = 0;
    cfg.D8[PIDVEL] = 0;

// 	if(OpticalFlow_Altitude > 300)
// 	{
// 		OpticalFlow_Angle[0]=0;
// 		OpticalFlow_Angle[1]=0;
// 		f.OPTFLOW_MODE = 0;
// 		return ;
// 	}

// 	if(OpticalFlow_Qual<30)
// 	{
// 		qual_count++;
// 		if(qual_count>40)
// 		{
// 			OpticalFlow_Angle[0]=0;
// 			OpticalFlow_Angle[1]=0;
// 			f.OPTFLOW_MODE = 0;
// 			return ;
// 		}
// 	}
// 	else
// 		qual_count=0;

	if(f.OPTFLOW_MODE==0)
	{
		optflowErrorI[0] = 0;
		optflowErrorI[1] = 0;
		prevHeading	=	heading;
		//f.OPTFLOW_MODE=1;
		//optflowUse = 1;
		return;
	}
	
	// Rotate	I	to follow	global axis
	#ifdef OF_ROTATE_I
		dif	= heading	-	prevHeading;
		if (dif	<= - 180)	dif	+= 360;
		else if	(dif >=	+	180) dif -=	360;

		if(abs(dif)	>	5) { //	rotate by	5-degree steps
			rotate16(optflowErrorI,	dif*10);
			prevHeading	=	heading;
		}
	#endif

		// Use sensor	only inside	DEADBAND
		//if(abs(rcCommand[ROLL])	<	OF_DEADBAND	&& abs(rcCommand[PITCH]) < OF_DEADBAND)	{	去掉了这角度限制
	
	// calculate velocity
	//optflow_get_vel();

	for(axis=0;	axis<2;	axis++)	
	{
		// correction	should be	less near	deadband limits
		//下面那句我注释的，看起来好像没用不知道
		//OpticalFlow_EstHVel[axis]	=	OpticalFlow_EstHVel[axis]	*	(OF_DEADBAND - abs(rcCommand[axis])) / OF_DEADBAND;	// 16	bit	ok:	100*100	=	10000
		optflowErrorI[axis]+=	OpticalFlow_EstHVel[axis]; 
		optflowErrorI[axis]	=	constrain(optflowErrorI[axis], -20000, 20000);

		OpticalFlow_Angle[axis]	=	OpticalFlow_EstHVel[axis]	*	cfg.P8[PIDVEL];	// 16	bit	ok:	100	*	200	=	20000	
	}
	// Apply I-term	unconditionally
	for(axis=0;	axis<2;	axis++)	
	{
		OpticalFlow_Angle[axis]	=	constrain(OpticalFlow_Angle[axis]	+	(int16_t)((int32_t)optflowErrorI[axis] * cfg.I8[PIDVEL]	/	5000), -80,	80);
	}
}

/**
  * @brief  处理接收到的数据
  *         
  * @param  None
  * @retval None
  */
static void OpticalFlow_Process(void)
{
	OS_ERR err;
	switch (OpticalFlow_Data_Buf[0])
	{
		case 0x50:
			OpticalFlow_Qual = OpticalFlow_Data_Buf[1];
			OpticalFlow_EstHVel[0] = OpticalFlow_Data_Buf[2] | (OpticalFlow_Data_Buf[3] << 8);//  cm/s  方向向右
			OpticalFlow_EstHVel[1] = - (OpticalFlow_Data_Buf[4] | (OpticalFlow_Data_Buf[5] << 8));// cm/s   方向转换成向上
			OpticalFlow_Altitude = OpticalFlow_Data_Buf[6] | (OpticalFlow_Data_Buf[7] << 8);   // cm

			OpticalFlow_LostCnt=0;
		 	OSFlagPost(&OpticalFlow_TaskRunFlagGrp,//接收到新数据
								(OS_FLAGS)0x01,
								OS_OPT_POST_FLAG_SET,
								&err);
		

			break;
		
		case 0x51://这个是视觉自动降落的数据更新
			VisionLanding_ReceiveRawData(OpticalFlow_Data_Buf);
			break;
		default:;
	}
}

/**
  * @brief  解码传输协议数据并校验
  *         在间隔时间调用
  * @param  None
  * @retval None
  */
static void OpticalFlow_Com(void)
{
	uint8_t c;
	static uint8_t offset;
	static uint8_t dataSize;
	static uint8_t checksum=0;
	static enum _serial_state
	{
		IDLE,
		HEADER_START,
		HEADER_M,
		HEADER_ARROW,
		HEADER_SIZE,
		HEADER_CMD,
	} c_state = IDLE;

	while (USART3_Available()) //判断是否接收到串口数据
	{
		c = USART3_Read();//读取来自上位机的数据

		if (c_state == IDLE) 
		{
			c_state = (c == '$') ? HEADER_START : IDLE;
		}
		else if (c_state == HEADER_START) 
		{
			c_state = (c == 'M') ? HEADER_M : IDLE;
		}
		else if (c_state == HEADER_M) 
		{
			c_state = (c == '<') ? HEADER_ARROW : IDLE;
		} 
		else if (c_state == HEADER_ARROW)
		{
			dataSize = c;
			c_state = HEADER_SIZE;      // the command is to follow
		}
		else if (c_state == HEADER_SIZE)
		{
			checksum = 0;
			offset = 0;
			checksum ^= c;
			OpticalFlow_Data_Buf[offset++] = c;
			c_state = HEADER_CMD;
		}
		else if (c_state == HEADER_CMD && offset < dataSize) 
		{
			checksum ^= c;
			OpticalFlow_Data_Buf[offset++] = c;
		}
		else if (c_state == HEADER_CMD && offset >= dataSize) 
		{
			if (checksum == c)//检查校验码
				OpticalFlow_Process();//进入数据处理
			c_state = IDLE;
		}
	}
}

#endif
