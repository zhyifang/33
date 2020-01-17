#include "MotorControl.h"
#include "flash.h"
#define RIGTH 1540        //����Ϊ3mm��1540
#define LEFT  1540        //����Ϊ3mm��1540

//#define RIGTH 2310      //����Ϊ2mm��2310
//#define LEFT  2310      //����Ϊ2mm��2310
//��Ӧ���г̵�240mm������1680.�г�Ϊ210mm������1470 ���г�180mm������1260��1540��Ӧ220;1610��Ӧ230��
u8 keytemp=0;             //1��ʼ��2ֹͣ
u16 uiVqrefOut=0;         //��·��ѹ���
u16 ucAngle=0;            //�����̻�ȡ�Ƕ���Ϣ
u8 BatteryVolgate = 0;    //��ص�ѹ��Ϣ
int uiTargetLocation=0;   //Ŀ��λ��

//��Ӧ���г̵�240mm������1680.�г�Ϊ210mm������1470    ���г�180mm������1260��
u16 uiLengthSum = 0;      //˿���ܳ���
u16 uiLeftLength=RIGTH;   //��೤�ȡ��Ķ����ҳ��ȿ���ʵ�֣��ڶ��ĽǶ�ֵ������������������������
u16 uiRigthLength=LEFT;   //�Ҳ೤�ȡ�

u16 uiLengthDifferent=0;  //���Ȳ�ֵ
int uiCurrentLocation=0;  //��ǰHAllλ��
u8 ucMotorDrection=0;     //������ת��

u8 ucSteeringDrection=3;  //�����̷���

/*********˿��У��Calibration************/
u8 ucCalibrationFlag=0;             //˿��0��У׼��־

u8 ucCalibrationStep1Flag  = 0;     //����У׼0��ɹ���־
u8 ucCalibrationSuccessFlag= 0;     //˿��У׼�ɹ���־
u8 ucCalibrationFailedFlag = 0;


/*********������0λУ��************/
u8 ucDriveCalibrationFlag = 0;      //������0ƫ�õ�У׼
u8 ucDriveCalibrationSucFlag = 0;   //������У׼�ɹ���־

u16 ucDistanceAngle = 128;
u8 ucDistanceDirction =0;
int ucDriveDistanceError = 0;       //������ƫ�þ����

u16 Left_RightLengh[4] = {0,0,0,0};
u16 datatemp[4]= {0,0,0,0};         //��λ��У׼����
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

/*************��·�����ٶ�λ�ü���****************/
u8 uc50msCnt=0;                      //50ms����
u16 uc2msCnt=0;                      //2ms����
u16 uiSpeedFbk=0;
u16 uiHallSpeedCnt=0;                //�Ҹ�˿��λ��У׼��
u16 uiHallSpeedBackCnt=0;            //��������������������㷴���ٶ�
u16 uiHallSpeedBackTempCnt=0;        //�������ٶȼ���temp
u16 uiHallSpeedTimeCnt=0;            //�������㶯һ�¶�Ӧ��ʱ��
const u8 ucReverseHall[8]={0,3,6,2,5,1,4,0};//��������
const u8 ucForwardHall[8]={0,5,3,1,6,4,2,0};//��������462315.AB AC BC BA CA CB
int uiCurrentLocationHallCnt=0;      //�������㵱ǰ����ֵ
u8 HallTemp;                         //��������index

//PID�ṹ��Ķ���
PID_Para SpeedPara;
PID_Para CurrentPara;
PID_Para PowerPara;
PID_Para PositionPara;


DATA_PARA SpeedData;

u32 ulSpeedArry[42]={0};            //�������ٶȵ�����

u8 ucChangeDrectionFlag = 0;        //������ʵ�ǿ�����
u16 uiCalibrationSpeed  = 1000;     //У׼�ٶ�����
u16 uiSpeedSet=0;                   //�����ٶ�

u8 ucReadZeroFlag=0;                //��0������־
u16 uiCurrentZero=0;                //0����ֵ
u16 uiCurrentAD=0;                  //����ADֵ
u16 uiVoltageAD = 0;                //��ѹADֵ
u16 uiVoltageADFbk = 0;             //����ѹ�˲�
u16 uiMosTemAD = 0;                 //�¶�ADֵ
u16 uiMosTemADFbk = 0;              //�¶��˲�

u8 ucForewardStallFlag=0;           //�Ҷ�ת
u8 ucReversedStallFlag=0;           //���ת
int UserMosTem = 0;
u16 UserCurrent10Times = 0;
u16 UserVolatge10Times = 0;
u16 UserPower = 0;

u8 MotorStopFlag = 0;
uint8_t ucClearFault = 0;

u16 uiLimitPower  =240;             //15A �� 200���ʼ���
u16 uiLimitCurrent=150;             //30A�ĵ���AD����2047��0��Ӧ������СΪ  0��41.25A������������620Ϊ30A����

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
/*********������ת�Ǹ��ٺ��������˿���˶��********/
/***************************************************/
void CalculateDistance(void)
{ 
  
    if((SetOK==0)&&(ucSteeringDrection!=3)&&(power_finish_flag==1)&&(Battery==1))//�û���У���������־λ���ܽ�ȸ��ٺ����
    {
 
        if(ucSteeringDrection==1)//��������ת
        {
            uiTargetLocation=uiLeftLength+(ucAngle*uiRigthLength)/127;
        }
        else if(ucSteeringDrection==0)//��������ת
        {
            uiTargetLocation=uiLeftLength-(ucAngle*uiLeftLength)/127;
			if(uiTargetLocation<0)
			 uiTargetLocation = 0;
        }

		/**********Ŀ��ֵ�뵱ǰhallֵ�Ƚ��ж�************/
        if(uiTargetLocation>uiCurrentLocationHallCnt)                     //�����Ҫ�ƶ���Ŀ��ֵ
        {
            uiLengthDifferent=uiTargetLocation-uiCurrentLocationHallCnt;  //hall�ĵ�ǰλ�ã����Ȳ��ֵ
            if((ucMotorDrection==1)&&(uiVqrefOut>0))                      //����������Ҫ�ͻ���ת��������,�����жϷ��򣨴�ʱ�����ת��
            {
                ucChangeDrectionFlag=1;                                   //����flag������
            }
            ucMotorDrectionTemp=0;                                        //�����תtemp
            ucReversedStallFlag=0;
        }
        else                                                              //Ŀ��ֵС�ڵ�ǰֵ�������Ҫ���أ������ת1
        {
            uiLengthDifferent=uiCurrentLocationHallCnt-uiTargetLocation;
            if((ucMotorDrection==0)&&(uiVqrefOut>0))                      //������ת��Ϊ0����Ҫ������������,�����жϷ���
            {
                ucChangeDrectionFlag=1;                                   //����flag������
            }
            ucMotorDrectionTemp=1;                                        //�����תtemp
            ucForewardStallFlag=0;
        }

    }

    if(SetOK==0)
    {
        ucMotorDrection=ucMotorDrectionTemp;
    }
     temp11++;
	/**********λ�û�·���Ȳ�ֵ��Ŀ��ֵ��ʵ��hall��Ȧ������9������ת��************/
    if(temp11==1600)               //
    {
       temp11=0;
       if(uiLengthDifferent>6)
      {
        keytemp=1;                                     //��������	
        PositionPara.Err=uiLengthDifferent;
        PID_Position(&PositionPara); 
        uiSpeedSet=PositionPara.Out;
            
		uiLimitPower=240;
		uiLimitCurrent = 150;	
        
//	     if(uiLengthDifferent>MAX_DISTANCE)             //101
//        {
//            uiSpeedSet=1500;                           //����ת��1500rpm
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
    
/**********������flag���𣬽��е����ֹͣ************/	
	if(ucChangeDrectionFlag==1)//�������
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
	/**********������У׼************/
	if((ucDriveCalibrationSucFlag == 1)&&(ucDriveCalibrationFlag == 1))
	{
		if((ucDistanceAngle == ucAngle)&&(ucDistanceDirction==ucSteeringDrection))//û��
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
	
	/**********(ucCalibrationSuccessFlag==1)&&(*˿��У׼�ɹ���������û��ת����ת������־λֹͣ***********/
	if(ucSteeringDrection==3)  keytemp=2;
	
	if(MototStatus==0)  MotorStopFlag = 0;

	/******�̾���δ����ͣ������ȡ����������Ҷ�ת��δУ׼||(ucCalibrationFlag==0)||(ucCalibrationFailedFlag==1)������************/
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
/*****˿��У׼��������������˿�˳��ȣ����˳ʱ�룬������ת���������ƶ���֮*******/
/************************************************************************************/
void CalibrationPosition(void)
{
	if(SetOK==1)  //ͨ������������У��������У׼��(û�ɹ�Ҳûʧ��) 
    {
	   	uiLengthDifferent = 80;
		if(ucCalibrationStep1Flag == 0)
	  	{
		   	//100max���൱�ڸ�һ������
		  	ucMotorDrection = 1;//��˳ʱ������
		  	if((GPIOB->IDR & GPIO_Pin_9)==0)//�Ӵ����г̿�����
			 {	
                 //��˴˴��޷�����   
                 uiCurrentLocationHallCnt = 0;//���0��
			  	uiHallSpeedCnt = 0;          //���0��
               if(My_PWM<351){
                
                keytemp = 2;                 
                PowerPara.Out = 0;
			  	PowerPara.Ui  = 0;
			  	SpeedPara.Out = 0;
			  	SpeedPara.Ui  = 0;
                CurrentPara.Out = 0;
			  	CurrentPara.Ui  = 0;

			   	ucCalibrationStep1Flag = 1;  //��У׼��ɱ�־λ 
                    
                }
                 else SpeedPara.Out=SpeedPara.Out>>1;
               //���м����޷�        
               
	  		 }
		  	  else if(ucReversedStallFlag ==1) //��ת�ˣ�����û��⵽�г̿����ź�
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
				uiCurrentLocationHallCnt = 0;   //��˼�������
				uiHallSpeedCnt = 0;
				ucCalibrationStep1Flag = 1;    //��У׼���
			  }
			
		}
	    if(ucCalibrationStep1Flag == 1)       //��У����ɽ�����У��
	    {
			  keytemp = 1;			                          //��������Ϊ��ת����ֹͣ��
		  	ucMotorDrection = 0;	                      //���ң������ʱ�룬�ƶ�˿����ǰ
 
		    if(uiHallSpeedCnt<((uiLeftLength+uiRigthLength)>>1)){
				uiCalibrationSpeed=1000;//-uiHallSpeedCnt/3
				
				}
		    else if(uiHallSpeedCnt<(((uiLeftLength+uiRigthLength)*8)/10))
		    {
		     uiCalibrationSpeed=1000;

             }
			 else{uiCalibrationSpeed=1000;}
		 
		  	if(uiHallSpeedCnt>=(uiLeftLength+uiRigthLength))  //������ֵ�ﵽ�ܼ���ֵ
		  	{  	
			  	uiLengthSum = uiHallSpeedCnt;             //��¼˿�ܳ���	
			  	ucMotorDrection=1;                        //�󣬵��˳ʱ�룬����˿������
               if(My_PWM<351){
                   
                   keytemp = 2;                 
                   PowerPara.Out = 0;
			  	   PowerPara.Ui  = 0;
			  	   SpeedPara.Out = 0;
			  	   SpeedPara.Ui  = 0;
                   CurrentPara.Out = 0;
			  	   CurrentPara.Ui  = 0;
			   	   ucCalibrationSuccessFlag = 1;//��У׼��ɱ�־λ 
                    
                }
                else SpeedPara.Out=SpeedPara.Out>>1;
		     }
			 else if((ucCalibrationSuccessFlag==0)&&(uiHallSpeedCnt<(uiLeftLength+uiRigthLength))&&ucForewardStallFlag == 1)//��ת�˻�û�ﵽ
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
				  	   uiLengthSum = uiHallSpeedCnt;//��¼˿�ܳ���           
					   ucCalibrationSuccessFlag = 1; 
					   ucMotorDrection=1;//��
				 }
			}
		}
        if((ucCalibrationSuccessFlag==1)||(ucCalibrationFailedFlag==1))
		{
			  ucCalibrationStep1Flag = 0;
              SetOK=0;                    //У������������
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
//�ٶȻ���ʼ��
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
//���ʻ���ʼ��
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
//��������ʼ��
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
//λ�û���ʼ��
void PositionParaInit(void)
{
    PositionPara.Kp=1;
    PositionPara.Ki=3;
    PositionPara.Kd=4;
    
    PositionPara.Ui=0;
    PositionPara.Ud=0;
    PositionPara.Up=0;
    
    PositionPara.Out=0;
    PositionPara.pre_Err=0;             //�����
    PositionPara.pre_SatErr=0;          //���ϴ����
    PositionPara.OutMin=0;
    PositionPara.OutMax=1500;
}
//λ�û�PID����
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


//�ٶȻ�PI����
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
//������PI����
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
//���ʻ�PI����
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
/*********Flash��д����������ص�ѹ���ϣ�mos�¶Ⱥ͵�������*************/
/***********************************************************************/
u16 dataVolgate[3] = {0,0,0};
u16 BatterySet[3]  = {0,0,0};
void CheckFault(void)
{
	static char UnderVoltageCnt = 0;     //Ƿѹ
	static char OverVoltageCnt  = 0;     //��ѹ
	static char UnderVoltageWarnCnt = 0;
	if(SetBatteryVoltage == 1)
	{
		if((BatteryVolgate==1)||(BatteryVolgate==2))
		{

			dataVolgate[0] = BatteryVolgate;//��ص�ѹ
			dataVolgate[1] = BatteryVolgate^0xaaaa;
			dataVolgate[2] = 0xaaaa;
			STMFLASH_Write(FlASH_BATTERY_ADDR,(u16*)dataVolgate,3);
			STMFLASH_Read(FlASH_BATTERY_ADDR,(u16*)BatterySet,3);
			Battery = BatterySet[0];
			
			if(((Battery^0xaaaa)==BatterySet[1])&&(BatterySet[2]==0xaaaa))
			{            
                SetOK=1;     //����У��ok
                SetBatteryVoltageOK = 1;
                SetBatteryVoltage = 0;         //������õ�ر�־λ
                BatteryVolgate = 0;            
				SetBatteryVoltageNeeded = 0;
				SetBatteryVoltageFailed = 0;
			}else SetBatteryVoltageFailed = 1; //ûУ�����

		}
		else SetBatteryVoltageFailed = 1;    //ûУ�����
	}
	
	//��صĹ���ѹ��Ƿ��ѹ����
	if(SetBatteryVoltageNeeded == 0)
	{

		if((Battery==1)||(Battery==2))
		{
			if(UserVolatge10Times>MOT_MAX_VOLT1) //��ѹ586
			{
				if(OverVoltageCnt++>50)//50ms
				{
					OverVoltageCnt = 50;
					MototStatus |= MOTOR_Status_MotOV;
                    //��ѹ����
                    
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
					MototStatus &= ~MOTOR_Status_MotOV;//MOTOR_Status_MotOV	= (1 << 1),//Motor Overvoltage��ѹ����
				}
			}
			
			
			if(UserVolatge10Times<MOT_WARN_VOLT1)//Ƿѹ�����ѹ100
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
					MototStatus |= MOTOR_Status_MotUV;    //Ƿѹ
					MototStatus &= ~MOTOR_Status_UVWarn;  //Ƿѹ����
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
					MototStatus &= ~MOTOR_Status_MotUV;  //Ƿѹ
				}
			}
		}


	}
	
//#define MOS_MAX_TEMP		 	 	 100     //100  90
//#define MOS_MAX_TEMP_RESET   85
//#define MOS_WARN_TEMP				 90      //90   80
//#define MOS_WARN_TEMP_RESET  80
//mos�¶�
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
	//������������
	if(UserCurrent10Times>300)             //ĸ�ߵ�������30A
	{
		MototStatus |= MOTOR_Status_MotOC;
	}
}
/******************************************/
/*********������0��У׼λ��***************/
/******************************************/
void DriveCalibration(void)
{
	if((ucDriveCalibrationSucFlag == 0)&&(ucDriveCalibrationFlag==1))
	{
		if(ucSteeringDrection==1)       //1�������󡣵������˿����������С��1680�����ߴ���1680
		{
			//ת���ǶȺ�ת������
			ucDistanceAngle    = ucAngle;
			ucDistanceDirction = ucSteeringDrection;
			//�԰��
			uiLeftLength  = uiLengthSum / 2;
			uiRigthLength = uiLengthSum / 2;
			
//			ucDriveDistanceError = uiCurrentLocationHallCnt - uiLeftLength; //�������          
           //�������λ������ʱ�򣬵���ʱ�򣬷����̷��ˣ��Ķ����£���
//			uiLeftLength  = uiLeftLength + ucDriveDistanceError;//ԭ�����
//			uiRigthLength = uiRigthLength- ucDriveDistanceError;
//			
			uiLeftLength  = uiCurrentLocationHallCnt;
			uiRigthLength = (RIGTH + LEFT)-uiCurrentLocationHallCnt;
    
			datatemp[0] = uiLeftLength;    //��� ԭ������
			datatemp[1] = uiRigthLength;   //�ұ�

			datatemp[2] = uiRigthLength^uiLeftLength;
			datatemp[3] = 0xaaaa;
			
			STMFLASH_Write(FLASH_DRIVER_ADDR,(u16*)datatemp,4);      //flashд
			STMFLASH_Read(FLASH_DRIVER_ADDR,(u16*)Left_RightLengh,4);//flash��
			
			if((Left_RightLengh[3]==datatemp[3])&&(Left_RightLengh[2]==datatemp[2]))
				ucDriveCalibrationSucFlag = 1;       //������0��У׼�ɹ���־λ
		}
		else if(ucSteeringDrection==0)               //0�������ҡ�����ƶ�˿����ǰ���˴���1680 С��1680
		{
			ucDistanceAngle    = ucAngle;
			ucDistanceDirction = ucSteeringDrection;
			//�԰��˿�ܳ���
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
			STMFLASH_Write(FLASH_DRIVER_ADDR,(u16*)datatemp,4);      //flashд
			STMFLASH_Read(FLASH_DRIVER_ADDR,(u16*)Left_RightLengh,4);//flash��
			if((Left_RightLengh[3]==datatemp[3])&&(Left_RightLengh[2]==datatemp[2]))
				ucDriveCalibrationSucFlag = 1;
		}
	}	
}
/*******************************************/
/*********ʹ����5182.c�У��������**********/
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
/*********��ѯУ������*****************/
/**********************************/
void QueryUser_Calibration(void)
{
	  	
    STMFLASH_Read(FlASH_BATTERY_ADDR,(u16*)BatterySet,3);  //��ȡFlASH_BATTERY_ADDR��BatterySet��ֵ
	Battery = BatterySet[0];
	if(((Battery^0xaaaa)==BatterySet[1])&&(BatterySet[2]==0xaaaa))  //????
		Battery = BatterySet[0];
	else
	{
		Battery = 2;
	}
	
}

/**************************************/
/*********��ѯHALLλ��*****************/
/**************************************/
/*********Current Position***********/
u16 Current_Position[4]  = {0,0,0,0};
u16 Current_Position1[4] = {0,0,0,0};
u16 Posion_temp[4]       = {0,0,0,0};        //У׼����

void QueryCurrent_position(void){
	
    STMFLASH_Read(FlASH_INSTALL_ADDR,(u16*)Current_Position,4);
    if((Current_Position[3]==0xaaaa)&&(Current_Position[2]==(Current_Position[0]^Current_Position[1])))
    {  
        uiCurrentLocationHallCnt=Current_Position[0];  //flash�д洢����һʱ�̵�λ��ֵ��
    }
    else
    {
        uiCurrentLocationHallCnt=0;
    }
    
}

//flash��¼��ǰλ��
u8 Record_Current_Position_Flag  = 0; //��ǰλ�óɹ���־λ
u8 Record_Current_Position_Start = 0; //��ǰλ�ü�¼��ʼ
void Record_Current_Position(void){
    
    if(Record_Current_Position_Start==1){
        
      Posion_temp[0] = uiCurrentLocationHallCnt;    //��� ԭ������
	    Posion_temp[1] = RIGTH+LEFT-uiCurrentLocationHallCnt;   //�ұ�
            
	    Posion_temp[2] = uiCurrentLocationHallCnt^(RIGTH+LEFT-uiCurrentLocationHallCnt);
	    Posion_temp[3] = 0xaaaa;
	
	    STMFLASH_Write(FlASH_INSTALL_ADDR,(u16*)Posion_temp,4);    //flashд
	    STMFLASH_Read(FlASH_INSTALL_ADDR,(u16*)Current_Position,4);//flash��
			
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
//        Posion_temp[0] = Position_All;    //��� ԭ������
//	      Posion_temp[1] = uiCurrentLocationHallCnt;   //�ұ�
//            
//	      Posion_temp[2] = Position_All^(uiCurrentLocationHallCnt);
//	      Posion_temp[3] = 0xaaaa;
//	
//	      STMFLASH_Write(FlASH_BATTERY_ADDR,(u16*)Posion_temp,4);    //flashд
//	      STMFLASH_Read(FlASH_BATTERY_ADDR,(u16*)Current_Position1,4);//flash��
//			
//	      if((Current_Position1[3]==Posion_temp[3])&&(Current_Position1[2]==Posion_temp[2]))
//        {
//	        open_power_flag=0;
//        }
//        else  open_power_flag = 1;     
//    }
    
}

/**************************************/
/*********��ѯ˿�ܳ���*****************/
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
//������ֵ
void swap(u16 *p1, u16 *p2)
{
    u16 tmp = *p1;
    *p1 = *p2;
    *p2 =tmp;
}
