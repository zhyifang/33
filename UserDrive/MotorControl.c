#include "MotorControl.h"
#include "flash.h"
#define RIGTH 1540        //导程为3mm，1540
#define LEFT  1540        //导程为3mm，1540

//#define RIGTH 2310      //导程为2mm，2310
//#define LEFT  2310      //导程为2mm，2310
//对应的行程的240mm，左右1680.行程为210mm，左右1470 ，行程180mm，左右1260；1540对应220;1610对应230。
u8 keytemp=0;             //1开始，2停止
u16 uiVqrefOut=0;         //环路电压输出
u16 ucAngle=0;            //方向盘获取角度信息
u8 BatteryVolgate = 0;    //电池电压信息
int uiTargetLocation=0;   //目标位置

//对应的行程的240mm，左右1680.行程为210mm，左右1470    。行程180mm，左右1260。
u16 uiLengthSum = 0;      //丝杠总长度
u16 uiLeftLength=RIGTH;   //左侧长度。改动左右长度可以实现，摆动的角度值。。。。。。。。。。。。
u16 uiRigthLength=LEFT;   //右侧长度。

u16 uiLengthDifferent=0;  //长度差值
int uiCurrentLocation=0;  //当前HAll位置
u8 ucMotorDrection=0;     //定义电机转向

u8 ucSteeringDrection=3;  //方向盘方向

/*********丝杆校正Calibration************/
u8 ucCalibrationFlag=0;             //丝杆0点校准标志

u8 ucCalibrationStep1Flag  = 0;     //向左校准0点成功标志
u8 ucCalibrationSuccessFlag= 0;     //丝杆校准成功标志
u8 ucCalibrationFailedFlag = 0;


/*********方向盘0位校正************/
u8 ucDriveCalibrationFlag = 0;      //方向盘0偏置点校准
u8 ucDriveCalibrationSucFlag = 0;   //方向盘校准成功标志

u16 ucDistanceAngle = 128;
u8 ucDistanceDirction =0;
int ucDriveDistanceError = 0;       //方向盘偏置距离差

u16 Left_RightLengh[4] = {0,0,0,0};
u16 datatemp[4]= {0,0,0,0};         //中位点校准数据
/**********Warning***********/
u8 MototStatus      = 0;
u8 ucLeftShortFlag  = 0;
u8 ucRightShortFlag = 0;
/*********Battery************/
u16 Battery = 0;
u8 SetBatteryVoltage = 0;
u8 SetBatteryVoltageOK = 0;
u8 SetOK = 0;

u8 SetBatteryVoltageFailed = 0;
u8 SetBatteryVoltageNeeded = 0;

/*************环路控制速度位置计数****************/
u8 uc50msCnt=0;                      //50ms计数
u16 uc2msCnt=0;                      //2ms计数
u16 uiSpeedFbk=0;
u16 uiHallSpeedCnt=0;                //我给丝杆位置校准用
u16 uiHallSpeedBackCnt=0;            //霍尔变更计数，用来计算反馈速度
u16 uiHallSpeedBackTempCnt=0;        //当反馈速度计算temp
u16 uiHallSpeedTimeCnt=0;            //用来计算动一下对应的时间
const u8 ucReverseHall[8]={0,3,6,2,5,1,4,0};//正向数组
const u8 ucForwardHall[8]={0,5,3,1,6,4,2,0};//反向数组462315.AB AC BC BA CA CB
int uiCurrentLocationHallCnt=0;      //用来计算当前霍尔值
u8 HallTemp;                         //方向数组index

//PID结构体的定义
PID_Para SpeedPara;
PID_Para CurrentPara;
PID_Para PowerPara;
PID_Para PositionPara;


DATA_PARA SpeedData;

u32 ulSpeedArry[42]={0};            //用来计速度的数组

u8 ucChangeDrectionFlag = 0;        //换向，其实是开过了
u16 uiCalibrationSpeed  = 1000;     //校准速度设置
u16 uiSpeedSet=0;                   //给定速度

u8 ucReadZeroFlag=0;                //读0电流标志
u16 uiCurrentZero=0;                //0电流值
u16 uiCurrentAD=0;                  //电流AD值
u16 uiVoltageAD = 0;                //电压AD值
u16 uiVoltageADFbk = 0;             //做电压滤波
u16 uiMosTemAD = 0;                 //温度AD值
u16 uiMosTemADFbk = 0;              //温度滤波

u8 ucForewardStallFlag=0;           //右堵转
u8 ucReversedStallFlag=0;           //左堵转
int UserMosTem = 0;
u16 UserCurrent10Times = 0;
u16 UserVolatge10Times = 0;
u16 UserPower = 0;

u8 MotorStopFlag = 0;
uint8_t ucClearFault = 0;

u16 uiLimitPower  =240;             //15A 和 200功率极限
u16 uiLimitCurrent=150;             //30A的电流AD采样2047到0对应电流大小为  0到41.25A。。。电流环620为30A极限

u16 uiCurrentFbk=0;
void SpeedSumInit(void)
{
    int i=0;
    for(i=0;i<42;i++)
    {
        ulSpeedArry[i]=1488;
    }
    SpeedData.DataSum=62500;
    SpeedData.Index=0;
    SpeedData.NewData=750;
    SpeedData.AimSpeed=9000;

}

void SpeedSum(DATA_PARA *Data)
{
    if(Data->Index>=42)
    {
        Data->Index=0;
    }
    Data->DataSum = Data->DataSum + Data->NewData - ulSpeedArry[Data->Index];
    ulSpeedArry[Data->Index]=Data->NewData;
    Data->Index++;

}

//#define MIN_DISTANCE    6    //21
//#define MAX_DISTANCE    100    //101 300
//#define MIN_SPEED       0    //220rpm  60rpm
//#define MAX_SPEED       1500   //rpm
//#define Speed_k         (MAX_SPEED-MIN_SPEED)/(MAX_DISTANCE-MIN_DISTANCE) //16
//#define Speed_b         (MIN_SPEED-Speed_k*MIN_DISTANCE)                  //-116

//#define MIN_DISTANCE    6    //21
//#define MAX_DISTANCE    200    //101 300
//#define MIN_SPEED       0    //220rpm  60rpm
//#define MAX_SPEED       1500   //rpm
//#define Speed_k         (MAX_SPEED-MIN_SPEED)/(MAX_DISTANCE-MIN_DISTANCE) //16
//#define Speed_b         (MIN_SPEED-Speed_k*MIN_DISTANCE)                  //-116

#define MIN_DISTANCE    6      //21
#define MAX_DISTANCE    300    //101 300
#define MIN_SPEED       150    //220rpm  60rpm
#define MAX_SPEED       1500   //rpm
#define Speed_k         (MAX_SPEED-MIN_SPEED)/(MAX_DISTANCE-MIN_DISTANCE) //16
#define Speed_b         (MIN_SPEED-Speed_k*MIN_DISTANCE)                  //-116

//#define MIN_DISTANCE    20    //21
//#define MAX_DISTANCE    100    //101 300
//#define MIN_SPEED       50   //220rpm  60rpm
//#define MAX_SPEED       1500   //rpm
//#define Speed_k         (MAX_SPEED-MIN_SPEED)/(MAX_DISTANCE-MIN_DISTANCE) //16
//#define Speed_b         (MIN_SPEED-Speed_k*MIN_DISTANCE)                  //-116

//#define MIN_DISTANCE    20    //21
//#define MAX_DISTANCE    100    //101 300
//#define MIN_SPEED       200   //220rpm  60rpm
//#define MAX_SPEED       1500   //rpm
//#define Speed_k         (MAX_SPEED-MIN_SPEED)/(MAX_DISTANCE-MIN_DISTANCE) //16
//#define Speed_b         (MIN_SPEED-Speed_k*MIN_DISTANCE)                  //-116

u8 ucMotorDrectionTemp=0;
u16 temp11=0;
/***************************************************/
/*********方向盘转角跟踪函数，电机丝杠运动�********/
/***************************************************/
void CalculateDistance(void)
{ 
  
    if((SetOK==0)&&(ucSteeringDrection!=3)&&(power_finish_flag==1)&&(Battery==1))//用户自校正后产生标志位才能进雀俸
    {
 
        if(ucSteeringDrection==1)//方向盘左转
        {
            uiTargetLocation=uiLeftLength+(ucAngle*uiRigthLength)/127;
        }
        else if(ucSteeringDrection==0)//方向盘右转
        {
            uiTargetLocation=uiLeftLength-(ucAngle*uiLeftLength)/127;
			if(uiTargetLocation<0)
			 uiTargetLocation = 0;
        }

		/**********目标值与当前hall值比较判断************/
        if(uiTargetLocation>uiCurrentLocationHallCnt)                     //电机需要推动到目标值
        {
            uiLengthDifferent=uiTargetLocation-uiCurrentLocationHallCnt;  //hall的当前位置，长度差分值
            if((ucMotorDrection==1)&&(uiVqrefOut>0))                      //假如电机左，需要就换向，转在运行中,才能判断反向（此时电机左转）
            {
                ucChangeDrectionFlag=1;                                   //换向flag立起来
            }
            ucMotorDrectionTemp=0;                                        //电机右转temp
            ucReversedStallFlag=0;
        }
        else                                                              //目标值小于当前值，电机需要拉回，电机右转1
        {
            uiLengthDifferent=uiCurrentLocationHallCnt-uiTargetLocation;
            if((ucMotorDrection==0)&&(uiVqrefOut>0))                      //如果电机转向为0，需要换向，在运行中,才能判断反向
            {
                ucChangeDrectionFlag=1;                                   //换向flag立起来
            }
            ucMotorDrectionTemp=1;                                        //电机左转temp
            ucForewardStallFlag=0;
        }

    }

    if(SetOK==0)
    {
        ucMotorDrection=ucMotorDrectionTemp;
    }
     temp11++;
	/**********位置环路长度差值，目标值与实际hall的圈数大于9，进行转动************/
    if(temp11==1600)               //
    {
       temp11=0;
       if(uiLengthDifferent>6)
      {
        keytemp=1;                                     //启动运行	
        PositionPara.Err=uiLengthDifferent;
        PID_Position(&PositionPara); 
        uiSpeedSet=PositionPara.Out;
            
		uiLimitPower=240;
		uiLimitCurrent = 150;	
        
//	     if(uiLengthDifferent>MAX_DISTANCE)             //101
//        {
//            uiSpeedSet=1500;                           //设置转速1500rpm
//        }
//        else if(uiLengthDifferent>MIN_DISTANCE)
//        {
//            uiSpeedSet=Speed_k*uiLengthDifferent+Speed_b; //speed_set=16*X-116
//        }
//        else
//        {
//            uiSpeedSet=MIN_SPEED;
//        }

       }
      else
      {
        uiSpeedSet=0;
		uiLimitPower=0;
		uiLimitCurrent = 0;
        uiLengthDifferent = 0;
      }
    }
    
/**********当换向flag立起，进行电机的停止************/	
	if(ucChangeDrectionFlag==1)//换向减速
    {
        uiSpeedSet   = 0;
		uiLimitPower = 0;
		uiLengthDifferent = 0;
	    uiLimitCurrent    = 0;
//        if((uiVqrefOut<=300)&&(UserVolatge10Times<=190))
//        {
//            keytemp=2;
//        }
//	    else if((uiVqrefOut<=160)&&(UserVolatge10Times<=290))
//        {
//            keytemp=2;
//        }
//		else if((uiVqrefOut<=140)&&(UserVolatge10Times<=390))
//        {
//            keytemp=2;
//        }
//	   	else if((uiVqrefOut<=80)&&(UserVolatge10Times<=580))
		keytemp=2;
    }
	/**********方向盘校准************/
	if((ucDriveCalibrationSucFlag == 1)&&(ucDriveCalibrationFlag == 1))
	{
		if((ucDistanceAngle == ucAngle)&&(ucDistanceDirction==ucSteeringDrection))//没动
		{
			uiSpeedSet=0;
			uiLimitPower=0;
			uiLengthDifferent = 0;
			uiLimitCurrent = 0;
			keytemp=2;
		}
		else
		{
			ucDriveCalibrationSucFlag= 0;
			ucDriveCalibrationFlag   = 0;
			uiLimitPower   = 240;
			uiLimitCurrent = 150;
			uiSpeedSet=Speed_k*uiLengthDifferent+Speed_b;
		}
	}
	
	/**********(ucCalibrationSuccessFlag==1)&&(*丝杆校准成功，方向盘没有转动，转向电机标志位停止***********/
	if(ucSteeringDrection==3)  keytemp=2;
	
	if(MototStatus==0)  MotorStopFlag = 0;

	/******短距离未动，停机，获取零电流，左右堵转，未校准||(ucCalibrationFlag==0)||(ucCalibrationFailedFlag==1)均不动************/
    if((uiLengthDifferent<1)||(keytemp==2)||(ucForewardStallFlag==1)||(ucReversedStallFlag==1)||(MotorStopFlag==1))
    {
        uiVqrefOut=0;
        ucChangeDrectionFlag=0;
        SpeedParaInit();
        PositionParaInit();
        SpeedSumInit();
        
        CurrentParaInit();
		PowerParaInit();
        keytemp=2;
    }
    else
    {
	  uiVqrefOut      = FindMin3(CurrentPara.Out,SpeedPara.Out,PowerPara.Out);     
//    uiVqrefOut      = SpeedPara.Out;     
//    uiVqrefOut      = CurrentPara.Out;     
//    uiVqrefOut      = PowerPara.Out; 
        
//	  SpeedPara.Ui    = uiVqrefOut;
//    SpeedPara.Out   = uiVqrefOut;
        
//    CurrentPara.Out = uiVqrefOut;
//    CurrentPara.Ui  = uiVqrefOut; 

//	  PowerPara.Out   = uiVqrefOut;
//	  PowerPara.Ui    = uiVqrefOut;
    }

    My_PWM=uiVqrefOut;
}


/************************************************************************************/
/*****丝杆校准推拉函数，测试丝杆长度，电机顺时针，向右旋转即拉动，推动反之*******/
/************************************************************************************/
void CalibrationPosition(void)
{
	if(SetOK==1)  //通过方向盘设置校正，进行校准中(没成功也没失败) 
    {
	   	uiLengthDifferent = 80;
		if(ucCalibrationStep1Flag == 0)
	  	{
		   	//100max，相当于给一个匀速
		  	ucMotorDrection = 1;//左，顺时针拉回
		  	if((GPIOB->IDR & GPIO_Pin_9)==0)//接触到行程开关了
			 {	
                 //左端此处限幅减速   
                 uiCurrentLocationHallCnt = 0;//标记0点
			  	uiHallSpeedCnt = 0;          //标记0点
               if(My_PWM<351){
                
                keytemp = 2;                 
                PowerPara.Out = 0;
			  	PowerPara.Ui  = 0;
			  	SpeedPara.Out = 0;
			  	SpeedPara.Ui  = 0;
                CurrentPara.Out = 0;
			  	CurrentPara.Ui  = 0;

			   	ucCalibrationStep1Flag = 1;  //左校准完成标志位 
                    
                }
                 else SpeedPara.Out=SpeedPara.Out>>1;
               //就行减速限幅        
               
	  		 }
		  	  else if(ucReversedStallFlag ==1) //堵转了，但是没检测到行程开关信号
             {
                ucReversedStallFlag = 0;
                keytemp = 2;
			  	PowerPara.Out = 0;
			  	PowerPara.Ui  = 0;
			   	SpeedPara.Out = 0;
			  	SpeedPara.Ui  = 0;
                CurrentPara.Out = 0;
			  	CurrentPara.Ui  = 0;
                
				ucLeftShortFlag = 1;		
				uiCurrentLocationHallCnt = 0;   //左端计数清零
				uiHallSpeedCnt = 0;
				ucCalibrationStep1Flag = 1;    //左校准完成
			  }
			
		}
	    if(ucCalibrationStep1Flag == 1)       //左校正完成进行右校正
	    {
			  keytemp = 1;			                          //启动，因为堵转导致停止了
		  	ucMotorDrection = 0;	                      //往右，电机逆时针，推动丝杆往前
 
		    if(uiHallSpeedCnt<((uiLeftLength+uiRigthLength)>>1)){
				uiCalibrationSpeed=1000;//-uiHallSpeedCnt/3
				
				}
		    else if(uiHallSpeedCnt<(((uiLeftLength+uiRigthLength)*8)/10))
		    {
		     uiCalibrationSpeed=1000;

             }
			 else{uiCalibrationSpeed=1000;}
		 
		  	if(uiHallSpeedCnt>=(uiLeftLength+uiRigthLength))  //当计数值达到总计数值
		  	{  	
			  	uiLengthSum = uiHallSpeedCnt;             //记录丝杠长度	
			  	ucMotorDrection=1;                        //左，电机顺时针，拉动丝杆往后
               if(My_PWM<351){
                   
                   keytemp = 2;                 
                   PowerPara.Out = 0;
			  	   PowerPara.Ui  = 0;
			  	   SpeedPara.Out = 0;
			  	   SpeedPara.Ui  = 0;
                   CurrentPara.Out = 0;
			  	   CurrentPara.Ui  = 0;
			   	   ucCalibrationSuccessFlag = 1;//左校准完成标志位 
                    
                }
                else SpeedPara.Out=SpeedPara.Out>>1;
		     }
			 else if((ucCalibrationSuccessFlag==0)&&(uiHallSpeedCnt<(uiLeftLength+uiRigthLength))&&ucForewardStallFlag == 1)//堵转了还没达到
			 {
			  	  if(uiHallSpeedCnt<((uiLeftLength+uiRigthLength)*9/10))
			  	 {
				      ucCalibrationFailedFlag = 1;
                     
					  if(ucLeftShortFlag==0)
						  ucRightShortFlag = 1;
				 }
				 else
				 {
                       ucForewardStallFlag = 0;
				  	   uiLengthSum = uiHallSpeedCnt;//记录丝杠长度           
					   ucCalibrationSuccessFlag = 1; 
					   ucMotorDrection=1;//左
				 }
			}
		}
        if((ucCalibrationSuccessFlag==1)||(ucCalibrationFailedFlag==1))
		{
			  ucCalibrationStep1Flag = 0;
              SetOK=0;                    //校正完成清零操作
              uiCalibrationSpeed = 0;
		   	  uiHallSpeedCnt = 0;
		  	  uiLengthDifferent = 0;
			  keytemp = 2;
		}
	}
}



#define MAX_OUT_PWM  1800
#define MIN_OUT_PWM  0

#define MAX_ERROR    800

#define KP_TEMP      20
#define KI_TEMP      15
//速度环初始化
void SpeedParaInit(void)
{
    SpeedPara.Kp=1;
    SpeedPara.Ki=1;
    SpeedPara.Ui=0;
    SpeedPara.Out=0;
    SpeedPara.pre_Err=0;
    SpeedPara.OutMin=MIN_OUT_PWM;  //150
    SpeedPara.OutMax=MAX_OUT_PWM;  //1800
}
//功率环初始化
void PowerParaInit(void)
{
    PowerPara.Kp=2;
    PowerPara.Ki=1;
    PowerPara.Ui=0;
    PowerPara.Out=0;
    PowerPara.pre_Err=0;
    PowerPara.OutMin=MIN_OUT_PWM;
    PowerPara.OutMax=MAX_OUT_PWM;
}
//电流环初始化
void CurrentParaInit(void)
{
    CurrentPara.Kp=2;
    CurrentPara.Ki=1;
    CurrentPara.Ui=0;
    CurrentPara.Out=0;
    CurrentPara.pre_Err=0;
    CurrentPara.OutMin=MIN_OUT_PWM;
    CurrentPara.OutMax=MAX_OUT_PWM;
}
//位置环初始化
void PositionParaInit(void)
{
    PositionPara.Kp=1;
    PositionPara.Ki=3;
    PositionPara.Kd=4;
    
    PositionPara.Ui=0;
    PositionPara.Ud=0;
    PositionPara.Up=0;
    
    PositionPara.Out=0;
    PositionPara.pre_Err=0;             //上误差
    PositionPara.pre_SatErr=0;          //上上次误差
    PositionPara.OutMin=0;
    PositionPara.OutMax=1500;
}
//位置环PID计算
void PID_Position(PID_Para *PositionPara)
{
//  PositionPara->Err = PositionPara->Ref - PositionPara->Fbk;

    if(PositionPara->Err > 3000)//3000
    {
        PositionPara->Err=3000;
    } 
    PositionPara->Up=PositionPara->Kp*(PositionPara->Err-PositionPara->pre_Err)>>2;
    PositionPara->Ui=PositionPara->Ki*(PositionPara->Err)>>1;
    PositionPara->Ud=PositionPara->Kd*(PositionPara->Err-2*PositionPara->pre_SatErr+PositionPara->pre_Err);
    PositionPara->Out=PositionPara->Out+PositionPara->Up+PositionPara->Ui+PositionPara->Ud;
    
    PositionPara->pre_Err=PositionPara->Err;
    PositionPara->pre_SatErr=PositionPara->pre_Err;
    
    if(PositionPara->Out > PositionPara->OutMax)
    {
        PositionPara->Out = PositionPara->OutMax; //1500
    }
    else if(PositionPara->Out < PositionPara->OutMin)
    {
        PositionPara->Out = PositionPara->OutMin; //0
    }
}


//速度环PI计算
void PID_Speed(PID_Para *SpeedPara)
{
    SpeedPara->Err = SpeedPara->Ref - SpeedPara->Fbk;
    if(SpeedPara->Err>MAX_ERROR)//800
    {
        SpeedPara->Err=MAX_ERROR;
     } 
//  SpeedPara->Up = (SpeedPara->Kp*SpeedPara->Err)/KP_TEMP; //20
//  SpeedPara->Ui = SpeedPara->Ui + ((SpeedPara->Ki*SpeedPara->Err)/KI_TEMP);//15
//  SpeedPara->Out = SpeedPara->Up + SpeedPara->Ui;    
    
    SpeedPara->Up = ((SpeedPara->Kp+SpeedPara->Ki)*SpeedPara->Err)/100;
    SpeedPara->Ui =  (SpeedPara->Kp*SpeedPara->pre_Err)/20;
    SpeedPara->Out= SpeedPara->Out + SpeedPara->Up + SpeedPara->Ui;
    SpeedPara->pre_Err = SpeedPara->Err; 

    if(SpeedPara->Out > SpeedPara->OutMax)
    {
        SpeedPara->Out = SpeedPara->OutMax; 
    }
    else if(SpeedPara->Out < SpeedPara->OutMin)
    {
        SpeedPara->Out = SpeedPara->OutMin; 
    }
	
}
//电流环PI计算
void PID_Current(PID_Para *CurrentPara)
{

    CurrentPara->Err = CurrentPara->Ref - CurrentPara->Fbk;

    CurrentPara->Up = ((CurrentPara->Kp+CurrentPara->Ki)*CurrentPara->Err);

    CurrentPara->Ui =  (CurrentPara->Kp*CurrentPara->pre_Err);

    CurrentPara->Out = CurrentPara->Out + CurrentPara->Up + CurrentPara->Ui;

    CurrentPara->pre_Err = CurrentPara->Err;    
    if(CurrentPara->Out > CurrentPara->OutMax)
    {
        CurrentPara->Out = CurrentPara->OutMax;
    }
    else if(CurrentPara->Out < CurrentPara->OutMin)
    {
    
         CurrentPara->Out = CurrentPara->OutMin;
    
    }

}
//功率环PI计算
void PID_Power(PID_Para *PowerPara)
{

    PowerPara->Err = PowerPara->Ref - PowerPara->Fbk;
    PowerPara->Up = ((PowerPara->Kp+PowerPara->Ki)*PowerPara->Err)/100;   //
    PowerPara->Ui = (PowerPara->Kp*PowerPara->pre_Err)/2;                 //

    PowerPara->Out = PowerPara->Out + PowerPara->Up + PowerPara->Ui;
    PowerPara->pre_Err = PowerPara->Err ;

    if(PowerPara->Out > PowerPara->OutMax)
    {
        PowerPara->Out = PowerPara->OutMax;
    }
    else if(PowerPara->Out < PowerPara->OutMin)
    {
       PowerPara->Out = PowerPara->OutMin;

    }
}





/***********************************************************************/
/*********Flash读写函数，检测电池电压故障，mos温度和电流过流*************/
/***********************************************************************/
u16 dataVolgate[3] = {0,0,0};
u16 BatterySet[3]  = {0,0,0};
void CheckFault(void)
{
	static char UnderVoltageCnt = 0;     //欠压
	static char OverVoltageCnt  = 0;     //过压
	static char UnderVoltageWarnCnt = 0;
	if(SetBatteryVoltage == 1)
	{
		if((BatteryVolgate==1)||(BatteryVolgate==2))
		{

			dataVolgate[0] = BatteryVolgate;//电池电压
			dataVolgate[1] = BatteryVolgate^0xaaaa;
			dataVolgate[2] = 0xaaaa;
			STMFLASH_Write(FlASH_BATTERY_ADDR,(u16*)dataVolgate,3);
			STMFLASH_Read(FlASH_BATTERY_ADDR,(u16*)BatterySet,3);
			Battery = BatterySet[0];
			
			if(((Battery^0xaaaa)==BatterySet[1])&&(BatterySet[2]==0xaaaa))
			{            
                SetOK=1;     //设置校正ok
                SetBatteryVoltageOK = 1;
                SetBatteryVoltage = 0;         //清除设置电池标志位
                BatteryVolgate = 0;            
				SetBatteryVoltageNeeded = 0;
				SetBatteryVoltageFailed = 0;
			}else SetBatteryVoltageFailed = 1; //没校验完成

		}
		else SetBatteryVoltageFailed = 1;    //没校验完成
	}
	
	//电池的过电压和欠电压设置
	if(SetBatteryVoltageNeeded == 0)
	{

		if((Battery==1)||(Battery==2))
		{
			if(UserVolatge10Times>MOT_MAX_VOLT1) //过压586
			{
				if(OverVoltageCnt++>50)//50ms
				{
					OverVoltageCnt = 50;
					MototStatus |= MOTOR_Status_MotOV;
                    //过压保护
                    
				}
			}
			else if(UserVolatge10Times<MOT_MAX_VOLT_RESET1)//560
			{
				if(OverVoltageCnt>0)
				{
					OverVoltageCnt --;
				}
				else
				{
					MototStatus &= ~MOTOR_Status_MotOV;//MOTOR_Status_MotOV	= (1 << 1),//Motor Overvoltage过压清零
				}
			}
			
			
			if(UserVolatge10Times<MOT_WARN_VOLT1)//欠压警告电压100
			{
				if(UnderVoltageWarnCnt++>50)//50ms
				{
					UnderVoltageWarnCnt = 50;
					MototStatus |= MOTOR_Status_UVWarn;
				}
			}
			else if(UserVolatge10Times>MOT_WARN_VOLT_RESET1)//110
			{
				if(UnderVoltageWarnCnt>0)
				{
					UnderVoltageWarnCnt--;
				}
				else
				{
					MototStatus &= ~MOTOR_Status_UVWarn;
				}
			}
			
			if(UserVolatge10Times<MOT_MIN_VOLT1)//90
			{
				if(UnderVoltageCnt++>50)//50ms
				{
					UnderVoltageCnt = 50;
					MototStatus |= MOTOR_Status_MotUV;    //欠压
					MototStatus &= ~MOTOR_Status_UVWarn;  //欠压警告
				}
			}
			else if(UserVolatge10Times>MOT_MIN_VOLT_RESET1)//100
			{
				if(UnderVoltageCnt>0)
				{
					UnderVoltageCnt--;
				}
				else
				{
					MototStatus &= ~MOTOR_Status_MotUV;  //欠压
				}
			}
		}


	}
	
//#define MOS_MAX_TEMP		 	 	 100     //100  90
//#define MOS_MAX_TEMP_RESET   85
//#define MOS_WARN_TEMP				 90      //90   80
//#define MOS_WARN_TEMP_RESET  80
//mos温度
	if(UserMosTem>MOS_MAX_TEMP)	
	{
		MototStatus |= MOTOR_Status_MotOT;
		MototStatus &= ~MOTOR_Status_OTWarn;
	}
	else if(UserMosTem<MOS_MAX_TEMP_RESET)
		MototStatus &= ~MOTOR_Status_MotOT;
	if(UserMosTem>MOS_WARN_TEMP)	
		MototStatus |= MOTOR_Status_OTWarn;
	else if(UserMosTem<MOS_WARN_TEMP_RESET)
		MototStatus &= ~MOTOR_Status_OTWarn;
	
	if(UserMosTem>80)	
		uiLimitPower= 192;
	else if(UserMosTem>85)
		uiLimitPower= 120;
	//电流过流故障
	if(UserCurrent10Times>300)             //母线电流保护30A
	{
		MototStatus |= MOTOR_Status_MotOC;
	}
}
/******************************************/
/*********方向盘0点校准位置***************/
/******************************************/
void DriveCalibration(void)
{
	if((ucDriveCalibrationSucFlag == 0)&&(ucDriveCalibrationFlag==1))
	{
		if(ucSteeringDrection==1)       //1方向盘左。电机拉动丝杆往后走了小于1680，或者大于1680
		{
			//转动角度和转动方向
			ucDistanceAngle    = ucAngle;
			ucDistanceDirction = ucSteeringDrection;
			//对半分
			uiLeftLength  = uiLengthSum / 2;
			uiRigthLength = uiLengthSum / 2;
			
//			ucDriveDistanceError = uiCurrentLocationHallCnt - uiLeftLength; //两种情况          
           //发现零点位置设置时候，掉电时候，方向盘反了，改动以下，将
//			uiLeftLength  = uiLeftLength + ucDriveDistanceError;//原来程�
//			uiRigthLength = uiRigthLength- ucDriveDistanceError;
//			
			uiLeftLength  = uiCurrentLocationHallCnt;
			uiRigthLength = (RIGTH + LEFT)-uiCurrentLocationHallCnt;
    
			datatemp[0] = uiLeftLength;    //左边 原来程序
			datatemp[1] = uiRigthLength;   //右边

			datatemp[2] = uiRigthLength^uiLeftLength;
			datatemp[3] = 0xaaaa;
			
			STMFLASH_Write(FLASH_DRIVER_ADDR,(u16*)datatemp,4);      //flash写
			STMFLASH_Read(FLASH_DRIVER_ADDR,(u16*)Left_RightLengh,4);//flash读
			
			if((Left_RightLengh[3]==datatemp[3])&&(Left_RightLengh[2]==datatemp[2]))
				ucDriveCalibrationSucFlag = 1;       //方向盘0点校准成功标志位
		}
		else if(ucSteeringDrection==0)               //0方向盘右。电机推动丝杆往前走了大于1680 小于1680
		{
			ucDistanceAngle    = ucAngle;
			ucDistanceDirction = ucSteeringDrection;
			//对半分丝杠长度
			uiLeftLength  = uiLengthSum /2;
			uiRigthLength = uiLengthSum /2;
			
//			ucDriveDistanceError = uiRigthLength - uiCurrentLocationHallCnt;			
//			uiLeftLength  = uiLeftLength - ucDriveDistanceError;
//			uiRigthLength = uiRigthLength + ucDriveDistanceError;
		    uiLeftLength  = uiCurrentLocationHallCnt; 
            uiRigthLength = (RIGTH + LEFT)-uiCurrentLocationHallCnt;
			datatemp[0] = uiLeftLength;
			datatemp[1] = uiRigthLength;
			datatemp[2] = uiRigthLength^uiLeftLength;
			datatemp[3] = 0xaaaa;
			STMFLASH_Write(FLASH_DRIVER_ADDR,(u16*)datatemp,4);      //flash写
			STMFLASH_Read(FLASH_DRIVER_ADDR,(u16*)Left_RightLengh,4);//flash读
			if((Left_RightLengh[3]==datatemp[3])&&(Left_RightLengh[2]==datatemp[2]))
				ucDriveCalibrationSucFlag = 1;
		}
	}	
}
/*******************************************/
/*********使用在5182.c中，清零操作**********/
/********************************************/
void Error_Acknowledge(void)
{
	MotorStopFlag = 0;
	MototStatus   = 0;
	ucLeftShortFlag = 0;
	ucRightShortFlag= 0;
	ucForewardStallFlag = 0;
	ucReversedStallFlag = 0;
}



/**********************************/
/*********查询校正设置*****************/
/**********************************/
void QueryUser_Calibration(void)
{
	  	
    STMFLASH_Read(FlASH_BATTERY_ADDR,(u16*)BatterySet,3);  //读取FlASH_BATTERY_ADDR给BatterySet的值
	Battery = BatterySet[0];
	if(((Battery^0xaaaa)==BatterySet[1])&&(BatterySet[2]==0xaaaa))  //????
		Battery = BatterySet[0];
	else
	{
		Battery = 2;
	}
	
}

/**************************************/
/*********查询HALL位置*****************/
/**************************************/
/*********Current Position***********/
u16 Current_Position[4]  = {0,0,0,0};
u16 Current_Position1[4] = {0,0,0,0};
u16 Posion_temp[4]       = {0,0,0,0};        //校准数据

void QueryCurrent_position(void){
	
    STMFLASH_Read(FlASH_INSTALL_ADDR,(u16*)Current_Position,4);
    if((Current_Position[3]==0xaaaa)&&(Current_Position[2]==(Current_Position[0]^Current_Position[1])))
    {  
        uiCurrentLocationHallCnt=Current_Position[0];  //flash中存储的上一时刻的位置值。
    }
    else
    {
        uiCurrentLocationHallCnt=0;
    }
    
}

//flash记录当前位置
u8 Record_Current_Position_Flag  = 0; //当前位置成功标志位
u8 Record_Current_Position_Start = 0; //当前位置记录开始
void Record_Current_Position(void){
    
    if(Record_Current_Position_Start==1){
        
      Posion_temp[0] = uiCurrentLocationHallCnt;    //左边 原来程序
	    Posion_temp[1] = RIGTH+LEFT-uiCurrentLocationHallCnt;   //右边
            
	    Posion_temp[2] = uiCurrentLocationHallCnt^(RIGTH+LEFT-uiCurrentLocationHallCnt);
	    Posion_temp[3] = 0xaaaa;
	
	    STMFLASH_Write(FlASH_INSTALL_ADDR,(u16*)Posion_temp,4);    //flash写
	    STMFLASH_Read(FlASH_INSTALL_ADDR,(u16*)Current_Position,4);//flash读
			
	    if((Current_Position[3]==Posion_temp[3])&&(Current_Position[2]==Posion_temp[2]))
       {
	       Record_Current_Position_Flag = 1; 
     
         Record_Current_Position_Start = 0;
       }
       else Record_Current_Position_Flag = 0; 
    }   
   
//    if(open_power_flag==1)
//    {
//        
//        Posion_temp[0] = Position_All;    //左边 原来程序
//	      Posion_temp[1] = uiCurrentLocationHallCnt;   //右边
//            
//	      Posion_temp[2] = Position_All^(uiCurrentLocationHallCnt);
//	      Posion_temp[3] = 0xaaaa;
//	
//	      STMFLASH_Write(FlASH_BATTERY_ADDR,(u16*)Posion_temp,4);    //flash写
//	      STMFLASH_Read(FlASH_BATTERY_ADDR,(u16*)Current_Position1,4);//flash读
//			
//	      if((Current_Position1[3]==Posion_temp[3])&&(Current_Position1[2]==Posion_temp[2]))
//        {
//	        open_power_flag=0;
//        }
//        else  open_power_flag = 1;     
//    }
    
}

/**************************************/
/*********查询丝杠长度*****************/
/**************************************/
void QueryLeftRight(void)
{
	STMFLASH_Read(FLASH_DRIVER_ADDR,(u16*)Left_RightLengh,4);
    
	if((Left_RightLengh[3]==0xaaaa)&&(Left_RightLengh[2]==(Left_RightLengh[0]^Left_RightLengh[1])))
	{
		 uiLeftLength = Left_RightLengh[0];
		 uiRigthLength= Left_RightLengh[1];
	}
	else
	{
        uiLeftLength  = LEFT;
        uiRigthLength = RIGTH;
	}

}



u16 FindMin3(u16 a,u16 b,u16 c)
{
	if(a<b)
	{
		if(a<c)
			return a;
		else
			return c;
	}
	else
	{
		if(b<c)
			return b;
		else
			return c;
	}
}

u16 FindMin4(u16 a,u16 b,u16 c,u16 d)
{
	if(a<b)
	{
		if(a<c)
		{
			if(a<d)
				return a;
			else
				return d;
		}
		else
		{
			if(c<d)
				return c;
			else
				return d;
		}
	}
	else
	{
		if(b<c)
		{
			if(b<d)
				return b;
			else
				return d;
		}
		else
		{
			if(c<d)
				return c;
			else
				return d;
		}
	}
}
//交换数值
void swap(u16 *p1, u16 *p2)
{
    u16 tmp = *p1;
    *p1 = *p2;
    *p2 =tmp;
}
