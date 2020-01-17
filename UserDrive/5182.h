#ifndef __5182_H
#define __5182_H
#include "stm32f10x.h"
#include "globalvariable.h"
#define SPI5182_RXBUFSIZE     50
#define SPI5182_TXBUFSIZE     50

#define CMD_HEADER               0XEF
#define POWER                    0X70  //����
#define CALIBRATION              0X71  //У��
#define DRIVEZERORPOSITON	     0x72  //��λУ��

#define SETMOTORVOLTAGE          0x73  //���õ�ѹ

#define MOTORFAULTSTOP           0x74  //�������ͣ��
#define SET_INSTALL_LOCATION     0X75  //���ð�װλ��

#define OPEN_POWER               0X76  //�����̿���
#define ClOSE_POWER              0X77  //�����̹ػ�




#define CAL_BACK                 0X50  //˿��У��
#define MOTOR_STATE              0X51  //���״̬

#define OPENCOMMUNICATE		     0X52  //��ͨ��

#define DRIVEZEROACKNOWLEGE      0x53  //��������λӦ��
#define MOTORVOLTAGEACKNOWLEGE   0x54  //�����ѹӦ��
#define MotorStopAcknowed        0x55  //���ֹͣӦ��
#define Install_LocationAcknowed 0x56  //��װλ��Ӧ��


#define OPEN_POWER_ready         0x57  //�ص��յ�
#define ClOSE_POWER_Acknowed     0x58  //�ص�Ӧ��
#define CLOSE_POWER_ready        0x59  //�ص��յ�


typedef struct {
    u8 RxBuf[SPI5182_RXBUFSIZE];
    u8 TxBuf[SPI5182_TXBUFSIZE];
    u8 RxHeadPoint;
    u8 RxTailPoint;
    u8 TxHeadPoint;
    u8 TxTailPoint;
    u8 RxCount;
    volatile u8 TxCount;
}SPI5182_Buffer_t;

extern u8 open_power_flag;            //�����̿���־λ
extern u8 open_power_flag_finish;
extern u8 power_finish_flag;

extern int Position_All;              //HAllλ��


u8 SPI5182_ReadWriteByte(u8 TxData);
u8 SPI5182_PutChar(u8 ch);
u8 SPI5182_WriteRxBuf (u8 ch);
int CheckMotorError(void);
u16 SPI5182_HandleTxd(void);
void TxCommand(unsigned char cmdbyte);
void CheckCmdData(void);
#endif


