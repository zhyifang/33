#ifndef __5182_H
#define __5182_H
#include "stm32f10x.h"
#include "globalvariable.h"
#define SPI5182_RXBUFSIZE     50
#define SPI5182_TXBUFSIZE     50

#define CMD_HEADER               0XEF
#define POWER                    0X70  //油门
#define CALIBRATION              0X71  //校正
#define DRIVEZERORPOSITON	     0x72  //零位校正

#define SETMOTORVOLTAGE          0x73  //设置电压

#define MOTORFAULTSTOP           0x74  //电机错误停机
#define SET_INSTALL_LOCATION     0X75  //设置安装位置

#define OPEN_POWER               0X76  //方向盘开机
#define ClOSE_POWER              0X77  //方向盘关机




#define CAL_BACK                 0X50  //丝杆校正
#define MOTOR_STATE              0X51  //电机状态

#define OPENCOMMUNICATE		     0X52  //打开通信

#define DRIVEZEROACKNOWLEGE      0x53  //方向盘零位应答
#define MOTORVOLTAGEACKNOWLEGE   0x54  //电机电压应答
#define MotorStopAcknowed        0x55  //电机停止应答
#define Install_LocationAcknowed 0x56  //安装位置应答


#define OPEN_POWER_ready         0x57  //关电收到
#define ClOSE_POWER_Acknowed     0x58  //关电应答
#define CLOSE_POWER_ready        0x59  //关电收到


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

extern u8 open_power_flag;            //方向盘开标志位
extern u8 open_power_flag_finish;
extern u8 power_finish_flag;

extern int Position_All;              //HAll位置


u8 SPI5182_ReadWriteByte(u8 TxData);
u8 SPI5182_PutChar(u8 ch);
u8 SPI5182_WriteRxBuf (u8 ch);
int CheckMotorError(void);
u16 SPI5182_HandleTxd(void);
void TxCommand(unsigned char cmdbyte);
void CheckCmdData(void);
#endif


