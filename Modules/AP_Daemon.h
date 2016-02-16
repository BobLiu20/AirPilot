#ifndef __DAEMON_H
#define __DAEMON_H
#include "includes.h"

/*
*********************************************************************************************************
*                                         LED1ָʾ
*********************************************************************************************************
*/
extern uint16_t LED_FlashBit;//��¼����Ҫ��˸��λ
#define LEDFLASH_ARMD   		(1<<0)		//����״̬
#define LEDFLASH_SENSORFAIL (1<<1)		//������������������˸
#define LEDFLASH_MAGCAL     (1<<2)		//����У׼ʱ��˸30��
#define LEDFLASH_SENSORCAL	(1<<3)		//�����ǻ��߼��ٶ�У׼
#define LEDFLASH_AIRSLOPE   (1<<4)		//����ǰ�����������б

#define LEDFLASH_SET(x) LED_FlashBit|=x	//����
#define LEDFLASH_CLR(x) LED_FlashBit&=~x //���

/*
*********************************************************************************************************
*                                         �����������
*********************************************************************************************************
*/
extern uint8_t NotOkToArmBit;//��ĳһλΪ1ʱ���򲻿ɽ���,�����f.OK_TO_ARM
#define NOTARMBIT_LOSTRC		(1<<0)		//����ǰ����ʧң���źţ����������
#define NOTARMBIT_ANGLE100	(1<<1)		//��б����100�ȣ�ͨ��������Ϊ��׹��
#define NOTARMBIT_ANGLE25		(1<<2)		//����ǰ��б����25��
#define NOTARMBIT_CALING		(1<<3)		//���ڽ��������ǻ���ٶȻ�����У׼�����������
#define NOTARMBIT_I2CERR		(1<<4)		//I2CͨѶ���������󣬲��������

#define NOTOKTOARM_SET(x) NotOkToArmBit|=x	//����
#define NOTOKTOARM_CLR(x) NotOkToArmBit&=~x //�����Ӧ��ֹ����λ


extern uint8_t NeedToWriteFlash;//����ֵΪ1ʱ����Ҫд��FLASH����
#define WRITE_TO_FLASH NeedToWriteFlash=1	//��ǽ�����д��FLASH

extern uint16_t Rc_LostCnt;//ң���źŶ�ʧ����
extern uint16_t Handle_LostCnt;//�ֱ��źŶ�ʧ����



extern void Daemon_Init(void);

#endif
