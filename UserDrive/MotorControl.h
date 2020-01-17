#ifndef _MOTOR_CONTROL_
#define _MOTOR_CONTROL_
#include "stm32f10x.h"
#include "globalvariable.h"
#define LENGTH  220         //240  230
#define SPEEDUSER 1500

//10-58V
#define MOT_MAX_VOLT1		  		  586   					 
#define MOT_MAX_VOLT_RESET1			560
#define MOT_WARN_VOLT_RESET1		110
#define MOT_WARN_VOLT1          100
#define MOT_MIN_VOLT1				    90	 
#define MOT_MIN_VOLT_RESET1			100 

//温度
#define MOS_MAX_TEMP		 	 	 100     //100  90
#define MOS_MAX_TEMP_RESET   85
#define MOS_WARN_TEMP				 90      //90   80
#define MOS_WARN_TEMP_RESET  80

typedef struct _PID_
{
    int Kp;
    int Ki;
    int Kd;
    int Ref;
    int Fbk;
    int Err;
    
    int pre_Err;         //上一次误差
    int pre_SatErr;      //上上次误差
    int Kc;
    int Up;
    int Ui;
    int Ud;
    int OutPreSat;
    int OutMax;
    int OutMin;
    int Out;
    
    int Upl;
}PID_Para;
typedef struct
{
    u16 NewData;
    u16 Index;
    u16 DataOut;
    u16 AimSpeed;
    u32 DataSum;
}DATA_PARA;

extern DATA_PARA SpeedData;
extern u16 uiCalibrationSpeed ; //校准速度设置
extern u16 uiSpeedSet;               //给定速度

extern u16 uiHallSpeedTimeCnt;
extern u8 keytemp;
extern u8 uc1msFlag;

extern u16 uiVqrefOut;
extern u16 ucAngle;
extern int uiTargetLocation;
extern u16 uiLeftLength;
extern u16 uiRigthLength;
extern u16 uiLengthDifferent;
extern int uiCurrentLocation;
extern u8 ucMotorDrection;
extern u8 ucSteeringDrection;
extern u8 ucCalibrationFlag; //丝杆校正


extern u8 ucDriveCalibrationFlag;//方向盘0偏置点校准
extern u8 uc50msCnt;
extern u16 uc2msCnt;
extern u16 uiHallSpeedCnt;
extern u16 uiHallSpeedBackCnt;
extern u16 uiHallSpeedBackTempCnt;
extern int uiCurrentLocationHallCnt;
extern const u8 ucForwardHall[8];
extern const u8 ucReverseHall[8];
extern u8 HallTemp;
extern PID_Para SpeedPara;
extern PID_Para CurrentPara;

extern PID_Para PositionPara;

extern PID_Para PowerPara;
extern u8 ucReadZeroFlag;
extern u16 uiCurrentZero;
extern u16 uiCurrentAD;
extern u16 uiVoltageAD; //电压AD值
extern u16 uiMosTemAD;  //温度AD值
extern u16 uiVoltageADFbk;
extern u16 uiMosTemADFbk;
extern u16 uiCurrentFbk;

extern u16 uiLimitCurrent;
extern u8 ucForewardStallFlag;
extern u8 ucReversedStallFlag;
extern u8 ucOverTemFlag;
extern u8 ucLocation;
extern int UserMosTem;
extern u16 UserCurrent10Times;
extern u16 UserVolatge10Times;
extern u16 UserPower;
/*********Calibration************/
extern u8 ucCalibrationSuccessFlag;
extern u8 ucCalibrationFailedFlag;
extern u8 ucDriveCalibrationSucFlag;
extern u16 ucDistanceAngle;
extern int ucDriveDistanceError;
//extern u8 SetBatteryVoltageNeeded5182;
extern u16 uiLimitPower;
extern u16 uiPowerFbk;

extern u8 SetBatteryVoltage;
extern u8 BatteryVolgate;     //电池电压信息
extern u8 SetBatteryVoltageOK;
extern u8 SetOK;

extern u8 SetBatteryVoltageFailed;


/**********Warning******************/
extern u8 MototStatus;
extern u8 ucLeftShortFlag;
extern u8 ucRightShortFlag;

extern u8 MotorStopFlag;
extern uint8_t ucClearFault;

typedef enum
{
	MOTOR_Status_UVWarn			= (1 << 0),   //Motor Undervoltage warning 
	MOTOR_Status_MotOV		 	= (1 << 1),		//Motor Overvoltage
	MOTOR_Status_MotUV		  = (1 << 2),		//Motor Undervoltage
	MOTOR_Status_MotOC			= (1 << 3),		//OCTW is low and fault is low
	MOTOR_Status_Stall	    = (1 << 4),		//Motor Stall
	MOTOR_Status_MotOT			= (1 << 5),		//MOS Over Heating 
	MOTOR_Status_OTWarn			= (1 << 6),		//MOS Over Heating Warning
	MOTOR_Status_SENSORFault= (1 << 7)		//HALL Sensor Error
}MOTOR_Status_e;


extern u16 Left_RightLengh[4];    
extern u16 datatemp[4];                 //校准数据
/*********Current Position***********/
extern u16 Current_Position[4];   
extern u16 Current_Position1[4];  

extern u16 Posion_temp[4];              //位置校准数据
extern u8 Record_Current_Position_Flag; //当前位置成功标志位
extern u8 Record_Current_Position_Start; //当前位置记录开始


void SpeedParaInit(void);
void CurrentParaInit(void);
void PowerParaInit(void);
void PositionParaInit(void);

void PID_Position(PID_Para *PositionPara);
void PID_Speed(PID_Para *SpeedPara);
void PID_Power(PID_Para *PowerPara);
void PID_Current(PID_Para *CurrentPara);



void Motor_speed_calulate(void);

void SpeedSum(DATA_PARA *Data);
void SpeedSumInit(void);
void CalculateDistance(void);
void CalibrationPosition(void);
void CheckFault(void);
void Error_Acknowledge(void);
void DriveCalibration(void);

void QueryUser_Calibration(void);
void QueryLeftRight(void);
void QueryCurrent_position(void);       //查询Flash当前位置
void Record_Current_Position(void);

void swap(u16 *p1, u16 *p2);

extern u16 My_PWM;
extern u16 BatterySet[3];
extern u16 Battery;
extern u8 SetBatteryVoltageNeeded;

u16 FindMin3(u16 a,u16 b,u16 c);
u16 FindMin4(u16 a,u16 b,u16 c,u16 d);
#endif

