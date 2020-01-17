#include "MotorControl.h"
#include "flash.h"
#define RIGTH 1540        //µ¼³ÌÎª3mm£¬1540
#define LEFT  1540        //µ¼³ÌÎª3mm£¬1540

//#define RIGTH 2310      //µ¼³ÌÎª2mm£¬2310
//#define LEFT  2310      //µ¼³ÌÎª2mm£¬2310
//¶ÔÓ¦µÄÐÐ³ÌµÄ240mm£¬×óÓÒ1680.ÐÐ³ÌÎª210mm£¬×óÓÒ1470 £¬ÐÐ³Ì180mm£¬×óÓÒ1260£»1540¶ÔÓ¦220;1610¶ÔÓ¦230¡£
u8 keytemp=0;             //1¿ªÊ¼£¬2Í£Ö¹
u16 uiVqrefOut=0;         //»·Â·µçÑ¹Êä³ö
u16 ucAngle=0;            //·½ÏòÅÌ»ñÈ¡½Ç¶ÈÐÅÏ¢
u8 BatteryVolgate = 0;    //µç³ØµçÑ¹ÐÅÏ¢
int uiTargetLocation=0;   //Ä¿±êÎ»ÖÃ

//¶ÔÓ¦µÄÐÐ³ÌµÄ240mm£¬×óÓÒ1680.ÐÐ³ÌÎª210mm£¬×óÓÒ1470    ¡£ÐÐ³Ì180mm£¬×óÓÒ1260¡£
u16 uiLengthSum = 0;      //Ë¿¸Ü×Ü³¤¶È
u16 uiLeftLength=RIGTH;   //×ó²à³¤¶È¡£¸Ä¶¯×óÓÒ³¤¶È¿ÉÒÔÊµÏÖ£¬°Ú¶¯µÄ½Ç¶ÈÖµ¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£
u16 uiRigthLength=LEFT;   //ÓÒ²à³¤¶È¡£

u16 uiLengthDifferent=0;  //³¤¶È²îÖµ
int uiCurrentLocation=0;  //µ±Ç°HAllÎ»ÖÃ
u8 ucMotorDrection=0;     //¶¨Òåµç»ú×ªÏò

u8 ucSteeringDrection=3;  //·½ÏòÅÌ·½Ïò

/*********Ë¿¸ËÐ£ÕýCalibration************/
u8 ucCalibrationFlag=0;             //Ë¿¸Ë0µãÐ£×¼±êÖ¾

u8 ucCalibrationStep1Flag  = 0;     //Ïò×óÐ£×¼0µã³É¹¦±êÖ¾
u8 ucCalibrationSuccessFlag= 0;     //Ë¿¸ËÐ£×¼³É¹¦±êÖ¾
u8 ucCalibrationFailedFlag = 0;


/*********·½ÏòÅÌ0Î»Ð£Õý************/
u8 ucDriveCalibrationFlag = 0;      //·½ÏòÅÌ0Æ«ÖÃµãÐ£×¼
u8 ucDriveCalibrationSucFlag = 0;   //·½ÏòÅÌÐ£×¼³É¹¦±êÖ¾

u16 ucDistanceAngle = 128;
u8 ucDistanceDirction =0;
int ucDriveDistanceError = 0;       //·½ÏòÅÌÆ«ÖÃ¾àÀë²î

u16 Left_RightLengh[4] = {0,0,0,0};
u16 datatemp[4]= {0,0,0,0};         //ÖÐÎ»µãÐ£×¼Êý¾Ý
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

/*************»·Â·¿ØÖÆËÙ¶ÈÎ»ÖÃ¼ÆÊý****************/
u8 uc50msCnt=0;                      //50ms¼ÆÊý
u16 uc2msCnt=0;                      //2ms¼ÆÊý
u16 uiSpeedFbk=0;
u16 uiHallSpeedCnt=0;                //ÎÒ¸øË¿¸ËÎ»ÖÃÐ£×¼ÓÃ
u16 uiHallSpeedBackCnt=0;            //»ô¶û±ä¸ü¼ÆÊý£¬ÓÃÀ´¼ÆËã·´À¡ËÙ¶È
u16 uiHallSpeedBackTempCnt=0;        //µ±·´À¡ËÙ¶È¼ÆËãtemp
u16 uiHallSpeedTimeCnt=0;            //ÓÃÀ´¼ÆËã¶¯Ò»ÏÂ¶ÔÓ¦µÄÊ±¼ä
const u8 ucReverseHall[8]={0,3,6,2,5,1,4,0};//ÕýÏòÊý×é
const u8 ucForwardHall[8]={0,5,3,1,6,4,2,0};//·´ÏòÊý×é462315.AB AC BC BA CA CB
int uiCurrentLocationHallCnt=0;      //ÓÃÀ´¼ÆËãµ±Ç°»ô¶ûÖµ
u8 HallTemp;                         //·½ÏòÊý×éindex

//PID½á¹¹ÌåµÄ¶¨Òå
PID_Para SpeedPara;
PID_Para CurrentPara;
PID_Para PowerPara;
PID_Para PositionPara;


DATA_PARA SpeedData;

u32 ulSpeedArry[42]={0};            //ÓÃÀ´¼ÆËÙ¶ÈµÄÊý×é

u8 ucChangeDrectionFlag = 0;        //»»Ïò£¬ÆäÊµÊÇ¿ª¹ýÁË
u16 uiCalibrationSpeed  = 1000;     //Ð£×¼ËÙ¶ÈÉèÖÃ
u16 uiSpeedSet=0;                   //¸ø¶¨ËÙ¶È

u8 ucReadZeroFlag=0;                //¶Á0µçÁ÷±êÖ¾
u16 uiCurrentZero=0;                //0µçÁ÷Öµ
u16 uiCurrentAD=0;                  //µçÁ÷ADÖµ
u16 uiVoltageAD = 0;                //µçÑ¹ADÖµ
u16 uiVoltageADFbk = 0;             //×öµçÑ¹ÂË²¨
u16 uiMosTemAD = 0;                 //ÎÂ¶ÈADÖµ
u16 uiMosTemADFbk = 0;              //ÎÂ¶ÈÂË²¨

u8 ucForewardStallFlag=0;           //ÓÒ¶Â×ª
u8 ucReversedStallFlag=0;           //×ó¶Â×ª
int UserMosTem = 0;
u16 UserCurrent10Times = 0;
u16 UserVolatge10Times = 0;
u16 UserPower = 0;

u8 MotorStopFlag = 0;
uint8_t ucClearFault = 0;

u16 uiLimitPower  =240;             //15A ºÍ 200¹¦ÂÊ¼«ÏÞ
u16 uiLimitCurrent=150;             //30AµÄµçÁ÷AD²ÉÑù2047µ½0¶ÔÓ¦µçÁ÷´óÐ¡Îª  0µ½41.25A¡£¡£¡£µçÁ÷»·620Îª30A¼«ÏÞ

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
/*********·½ÏòÅÌ×ª½Ç¸ú×Ùº¯Êý£¬µç»úË¿¸ÜÔË¶¯¡********/
/***************************************************/
void CalculateDistance(void)
{ 
  
    if((SetOK==0)&&(ucSteeringDrection!=3)&&(power_finish_flag==1)&&(Battery==1))//ÓÃ»§×ÔÐ£Õýºó²úÉú±êÖ¾Î»²ÅÄÜ½øÈ¸ú×Ùº¯Êýë
    {
 
        if(ucSteeringDrection==1)//·½ÏòÅÌ×ó×ª
        {
            uiTargetLocation=uiLeftLength+(ucAngle*uiRigthLength)/127;
        }
        else if(ucSteeringDrection==0)//·½ÏòÅÌÓÒ×ª
        {
            uiTargetLocation=uiLeftLength-(ucAngle*uiLeftLength)/127;
			if(uiTargetLocation<0)
			 uiTargetLocation = 0;
        }

		/**********Ä¿±êÖµÓëµ±Ç°hallÖµ±È½ÏÅÐ¶Ï************/
        if(uiTargetLocation>uiCurrentLocationHallCnt)                     //µç»úÐèÒªÍÆ¶¯µ½Ä¿±êÖµ
        {
            uiLengthDifferent=uiTargetLocation-uiCurrentLocationHallCnt;  //hallµÄµ±Ç°Î»ÖÃ£¬³¤¶È²î·ÖÖµ
            if((ucMotorDrection==1)&&(uiVqrefOut>0))                      //¼ÙÈçµç»ú×ó£¬ÐèÒª¾Í»»Ïò£¬×ªÔÚÔËÐÐÖÐ,²ÅÄÜÅÐ¶Ï·´Ïò£¨´ËÊ±µç»ú×ó×ª£©
            {
                ucChangeDrectionFlag=1;                                   //»»ÏòflagÁ¢ÆðÀ´
            }
            ucMotorDrectionTemp=0;                                        //µç»úÓÒ×ªtemp
            ucReversedStallFlag=0;
        }
        else                                                              //Ä¿±êÖµÐ¡ÓÚµ±Ç°Öµ£¬µç»úÐèÒªÀ­»Ø£¬µç»úÓÒ×ª1
        {
            uiLengthDifferent=uiCurrentLocationHallCnt-uiTargetLocation;
            if((ucMotorDrection==0)&&(uiVqrefOut>0))                      //Èç¹ûµç»ú×ªÏòÎª0£¬ÐèÒª»»Ïò£¬ÔÚÔËÐÐÖÐ,²ÅÄÜÅÐ¶Ï·´Ïò
            {
                ucChangeDrectionFlag=1;                                   //»»ÏòflagÁ¢ÆðÀ´
            }
            ucMotorDrectionTemp=1;                                        //µç»ú×ó×ªtemp
            ucForewardStallFlag=0;
        }

    }

    if(SetOK==0)
    {
        ucMotorDrection=ucMotorDrectionTemp;
    }
     temp11++;
	/**********Î»ÖÃ»·Â·³¤¶È²îÖµ£¬Ä¿±êÖµÓëÊµ¼ÊhallµÄÈ¦Êý´óÓÚ9£¬½øÐÐ×ª¶¯************/
    if(temp11==1600)               //
    {
       temp11=0;
       if(uiLengthDifferent>6)
      {
        keytemp=1;                                     //Æô¶¯ÔËÐÐ	
        PositionPara.Err=uiLengthDifferent;
        PID_Position(&PositionPara); 
        uiSpeedSet=PositionPara.Out;
            
		uiLimitPower=240;
		uiLimitCurrent = 150;	
        
//	     if(uiLengthDifferent>MAX_DISTANCE)             //101
//        {
//            uiSpeedSet=1500;                           //ÉèÖÃ×ªËÙ1500rpm
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
    
/**********µ±»»ÏòflagÁ¢Æð£¬½øÐÐµç»úµÄÍ£Ö¹************/	
	if(ucChangeDrectionFlag==1)//»»Ïò¼õËÙ
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
	/**********·½ÏòÅÌÐ£×¼************/
	if((ucDriveCalibrationSucFlag == 1)&&(ucDriveCalibrationFlag == 1))
	{
		if((ucDistanceAngle == ucAngle)&&(ucDistanceDirction==ucSteeringDrection))//Ã»¶¯
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
	
	/**********(ucCalibrationSuccessFlag==1)&&(*Ë¿¸ËÐ£×¼³É¹¦£¬·½ÏòÅÌÃ»ÓÐ×ª¶¯£¬×ªÏòµç»ú±êÖ¾Î»Í£Ö¹***********/
	if(ucSteeringDrection==3)  keytemp=2;
	
	if(MototStatus==0)  MotorStopFlag = 0;

	/******¶Ì¾àÀëÎ´¶¯£¬Í£»ú£¬»ñÈ¡ÁãµçÁ÷£¬×óÓÒ¶Â×ª£¬Î´Ð£×¼||(ucCalibrationFlag==0)||(ucCalibrationFailedFlag==1)¾ù²»¶¯************/
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
/*****Ë¿¸ËÐ£×¼ÍÆÀ­º¯Êý£¬²âÊÔË¿¸Ë³¤¶È£¬µç»úË³Ê±Õë£¬ÏòÓÒÐý×ª¼´À­¶¯£¬ÍÆ¶¯·´Ö®*******/
/************************************************************************************/
void CalibrationPosition(void)
{
	if(SetOK==1)  //Í¨¹ý·½ÏòÅÌÉèÖÃÐ£Õý£¬½øÐÐÐ£×¼ÖÐ(Ã»³É¹¦Ò²Ã»Ê§°Ü) 
    {
	   	uiLengthDifferent = 80;
		if(ucCalibrationStep1Flag == 0)
	  	{
		   	//100max£¬Ïàµ±ÓÚ¸øÒ»¸öÔÈËÙ
		  	ucMotorDrection = 1;//×ó£¬Ë³Ê±ÕëÀ­»Ø
		  	if((GPIOB->IDR & GPIO_Pin_9)==0)//½Ó´¥µ½ÐÐ³Ì¿ª¹ØÁË
			 {	
                 //×ó¶Ë´Ë´¦ÏÞ·ù¼õËÙ   
                 uiCurrentLocationHallCnt = 0;//±ê¼Ç0µã
			  	uiHallSpeedCnt = 0;          //±ê¼Ç0µã
               if(My_PWM<351){
                
                keytemp = 2;                 
                PowerPara.Out = 0;
			  	PowerPara.Ui  = 0;
			  	SpeedPara.Out = 0;
			  	SpeedPara.Ui  = 0;
                CurrentPara.Out = 0;
			  	CurrentPara.Ui  = 0;

			   	ucCalibrationStep1Flag = 1;  //×óÐ£×¼Íê³É±êÖ¾Î» 
                    
                }
                 else SpeedPara.Out=SpeedPara.Out>>1;
               //¾ÍÐÐ¼õËÙÏÞ·ù        
               
	  		 }
		  	  else if(ucReversedStallFlag ==1) //¶Â×ªÁË£¬µ«ÊÇÃ»¼ì²âµ½ÐÐ³Ì¿ª¹ØÐÅºÅ
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
				uiCurrentLocationHallCnt = 0;   //×ó¶Ë¼ÆÊýÇåÁã
				uiHallSpeedCnt = 0;
				ucCalibrationStep1Flag = 1;    //×óÐ£×¼Íê³É
			  }
			
		}
	    if(ucCalibrationStep1Flag == 1)       //×óÐ£ÕýÍê³É½øÐÐÓÒÐ£Õý
	    {
			  keytemp = 1;			                          //Æô¶¯£¬ÒòÎª¶Â×ªµ¼ÖÂÍ£Ö¹ÁË
		  	ucMotorDrection = 0;	                      //ÍùÓÒ£¬µç»úÄæÊ±Õë£¬ÍÆ¶¯Ë¿¸ËÍùÇ°
 
		    if(uiHallSpeedCnt<((uiLeftLength+uiRigthLength)>>1)){
				uiCalibrationSpeed=1000;//-uiHallSpeedCnt/3
				
				}
		    else if(uiHallSpeedCnt<(((uiLeftLength+uiRigthLength)*8)/10))
		    {
		     uiCalibrationSpeed=1000;

             }
			 else{uiCalibrationSpeed=1000;}
		 
		  	if(uiHallSpeedCnt>=(uiLeftLength+uiRigthLength))  //µ±¼ÆÊýÖµ´ïµ½×Ü¼ÆÊýÖµ
		  	{  	
			  	uiLengthSum = uiHallSpeedCnt;             //¼ÇÂ¼Ë¿¸Ü³¤¶È	
			  	ucMotorDrection=1;                        //×ó£¬µç»úË³Ê±Õë£¬À­¶¯Ë¿¸ËÍùºó
               if(My_PWM<351){
                   
                   keytemp = 2;                 
                   PowerPara.Out = 0;
			  	   PowerPara.Ui  = 0;
			  	   SpeedPara.Out = 0;
			  	   SpeedPara.Ui  = 0;
                   CurrentPara.Out = 0;
			  	   CurrentPara.Ui  = 0;
			   	   ucCalibrationSuccessFlag = 1;//×óÐ£×¼Íê³É±êÖ¾Î» 
                    
                }
                else SpeedPara.Out=SpeedPara.Out>>1;
		     }
			 else if((ucCalibrationSuccessFlag==0)&&(uiHallSpeedCnt<(uiLeftLength+uiRigthLength))&&ucForewardStallFlag == 1)//¶Â×ªÁË»¹Ã»´ïµ½
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
				  	   uiLengthSum = uiHallSpeedCnt;//¼ÇÂ¼Ë¿¸Ü³¤¶È           
					   ucCalibrationSuccessFlag = 1; 
					   ucMotorDrection=1;//×ó
				 }
			}
		}
        if((ucCalibrationSuccessFlag==1)||(ucCalibrationFailedFlag==1))
		{
			  ucCalibrationStep1Flag = 0;
              SetOK=0;                    //Ð£ÕýÍê³ÉÇåÁã²Ù×÷
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
//ËÙ¶È»·³õÊ¼»¯
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
//¹¦ÂÊ»·³õÊ¼»¯
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
//µçÁ÷»·³õÊ¼»¯
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
//Î»ÖÃ»·³õÊ¼»¯
void PositionParaInit(void)
{
    PositionPara.Kp=1;
    PositionPara.Ki=3;
    PositionPara.Kd=4;
    
    PositionPara.Ui=0;
    PositionPara.Ud=0;
    PositionPara.Up=0;
    
    PositionPara.Out=0;
    PositionPara.pre_Err=0;             //ÉÏÎó²î
    PositionPara.pre_SatErr=0;          //ÉÏÉÏ´ÎÎó²î
    PositionPara.OutMin=0;
    PositionPara.OutMax=1500;
}
//Î»ÖÃ»·PID¼ÆËã
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


//ËÙ¶È»·PI¼ÆËã
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
//µçÁ÷»·PI¼ÆËã
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
//¹¦ÂÊ»·PI¼ÆËã
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
/*********Flash¶ÁÐ´º¯Êý£¬¼ì²âµç³ØµçÑ¹¹ÊÕÏ£¬mosÎÂ¶ÈºÍµçÁ÷¹ýÁ÷*************/
/***********************************************************************/
u16 dataVolgate[3] = {0,0,0};
u16 BatterySet[3]  = {0,0,0};
void CheckFault(void)
{
	static char UnderVoltageCnt = 0;     //Ç·Ñ¹
	static char OverVoltageCnt  = 0;     //¹ýÑ¹
	static char UnderVoltageWarnCnt = 0;
	if(SetBatteryVoltage == 1)
	{
		if((BatteryVolgate==1)||(BatteryVolgate==2))
		{

			dataVolgate[0] = BatteryVolgate;//µç³ØµçÑ¹
			dataVolgate[1] = BatteryVolgate^0xaaaa;
			dataVolgate[2] = 0xaaaa;
			STMFLASH_Write(FlASH_BATTERY_ADDR,(u16*)dataVolgate,3);
			STMFLASH_Read(FlASH_BATTERY_ADDR,(u16*)BatterySet,3);
			Battery = BatterySet[0];
			
			if(((Battery^0xaaaa)==BatterySet[1])&&(BatterySet[2]==0xaaaa))
			{            
                SetOK=1;     //ÉèÖÃÐ£Õýok
                SetBatteryVoltageOK = 1;
                SetBatteryVoltage = 0;         //Çå³ýÉèÖÃµç³Ø±êÖ¾Î»
                BatteryVolgate = 0;            
				SetBatteryVoltageNeeded = 0;
				SetBatteryVoltageFailed = 0;
			}else SetBatteryVoltageFailed = 1; //Ã»Ð£ÑéÍê³É

		}
		else SetBatteryVoltageFailed = 1;    //Ã»Ð£ÑéÍê³É
	}
	
	//µç³ØµÄ¹ýµçÑ¹ºÍÇ·µçÑ¹ÉèÖÃ
	if(SetBatteryVoltageNeeded == 0)
	{

		if((Battery==1)||(Battery==2))
		{
			if(UserVolatge10Times>MOT_MAX_VOLT1) //¹ýÑ¹586
			{
				if(OverVoltageCnt++>50)//50ms
				{
					OverVoltageCnt = 50;
					MototStatus |= MOTOR_Status_MotOV;
                    //¹ýÑ¹±£»¤
                    
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
					MototStatus &= ~MOTOR_Status_MotOV;//MOTOR_Status_MotOV	= (1 << 1),//Motor Overvoltage¹ýÑ¹ÇåÁã
				}
			}
			
			
			if(UserVolatge10Times<MOT_WARN_VOLT1)//Ç·Ñ¹¾¯¸æµçÑ¹100
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
					MototStatus |= MOTOR_Status_MotUV;    //Ç·Ñ¹
					MototStatus &= ~MOTOR_Status_UVWarn;  //Ç·Ñ¹¾¯¸æ
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
					MototStatus &= ~MOTOR_Status_MotUV;  //Ç·Ñ¹
				}
			}
		}


	}
	
//#define MOS_MAX_TEMP		 	 	 100     //100  90
//#define MOS_MAX_TEMP_RESET   85
//#define MOS_WARN_TEMP				 90      //90   80
//#define MOS_WARN_TEMP_RESET  80
//mosÎÂ¶È
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
	//µçÁ÷¹ýÁ÷¹ÊÕÏ
	if(UserCurrent10Times>300)             //Ä¸ÏßµçÁ÷±£»¤30A
	{
		MototStatus |= MOTOR_Status_MotOC;
	}
}
/******************************************/
/*********·½ÏòÅÌ0µãÐ£×¼Î»ÖÃ***************/
/******************************************/
void DriveCalibration(void)
{
	if((ucDriveCalibrationSucFlag == 0)&&(ucDriveCalibrationFlag==1))
	{
		if(ucSteeringDrection==1)       //1·½ÏòÅÌ×ó¡£µç»úÀ­¶¯Ë¿¸ËÍùºó×ßÁËÐ¡ÓÚ1680£¬»òÕß´óÓÚ1680
		{
			//×ª¶¯½Ç¶ÈºÍ×ª¶¯·½Ïò
			ucDistanceAngle    = ucAngle;
			ucDistanceDirction = ucSteeringDrection;
			//¶Ô°ë·Ö
			uiLeftLength  = uiLengthSum / 2;
			uiRigthLength = uiLengthSum / 2;
			
//			ucDriveDistanceError = uiCurrentLocationHallCnt - uiLeftLength; //Á½ÖÖÇé¿ö          
           //·¢ÏÖÁãµãÎ»ÖÃÉèÖÃÊ±ºò£¬µôµçÊ±ºò£¬·½ÏòÅÌ·´ÁË£¬¸Ä¶¯ÒÔÏÂ£¬½«
//			uiLeftLength  = uiLeftLength + ucDriveDistanceError;//Ô­À´³ÌÐ
//			uiRigthLength = uiRigthLength- ucDriveDistanceError;
//			
			uiLeftLength  = uiCurrentLocationHallCnt;
			uiRigthLength = (RIGTH + LEFT)-uiCurrentLocationHallCnt;
    
			datatemp[0] = uiLeftLength;    //×ó±ß Ô­À´³ÌÐò
			datatemp[1] = uiRigthLength;   //ÓÒ±ß

			datatemp[2] = uiRigthLength^uiLeftLength;
			datatemp[3] = 0xaaaa;
			
			STMFLASH_Write(FLASH_DRIVER_ADDR,(u16*)datatemp,4);      //flashÐ´
			STMFLASH_Read(FLASH_DRIVER_ADDR,(u16*)Left_RightLengh,4);//flash¶Á
			
			if((Left_RightLengh[3]==datatemp[3])&&(Left_RightLengh[2]==datatemp[2]))
				ucDriveCalibrationSucFlag = 1;       //·½ÏòÅÌ0µãÐ£×¼³É¹¦±êÖ¾Î»
		}
		else if(ucSteeringDrection==0)               //0·½ÏòÅÌÓÒ¡£µç»úÍÆ¶¯Ë¿¸ËÍùÇ°×ßÁË´óÓÚ1680 Ð¡ÓÚ1680
		{
			ucDistanceAngle    = ucAngle;
			ucDistanceDirction = ucSteeringDrection;
			//¶Ô°ë·ÖË¿¸Ü³¤¶È
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
			STMFLASH_Write(FLASH_DRIVER_ADDR,(u16*)datatemp,4);      //flashÐ´
			STMFLASH_Read(FLASH_DRIVER_ADDR,(u16*)Left_RightLengh,4);//flash¶Á
			if((Left_RightLengh[3]==datatemp[3])&&(Left_RightLengh[2]==datatemp[2]))
				ucDriveCalibrationSucFlag = 1;
		}
	}	
}
/*******************************************/
/*********Ê¹ÓÃÔÚ5182.cÖÐ£¬ÇåÁã²Ù×÷**********/
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
/*********²éÑ¯Ð£ÕýÉèÖÃ*****************/
/**********************************/
void QueryUser_Calibration(void)
{
	  	
    STMFLASH_Read(FlASH_BATTERY_ADDR,(u16*)BatterySet,3);  //¶ÁÈ¡FlASH_BATTERY_ADDR¸øBatterySetµÄÖµ
	Battery = BatterySet[0];
	if(((Battery^0xaaaa)==BatterySet[1])&&(BatterySet[2]==0xaaaa))  //????
		Battery = BatterySet[0];
	else
	{
		Battery = 2;
	}
	
}

/**************************************/
/*********²éÑ¯HALLÎ»ÖÃ*****************/
/**************************************/
/*********Current Position***********/
u16 Current_Position[4]  = {0,0,0,0};
u16 Current_Position1[4] = {0,0,0,0};
u16 Posion_temp[4]       = {0,0,0,0};        //Ð£×¼Êý¾Ý

void QueryCurrent_position(void){
	
    STMFLASH_Read(FlASH_INSTALL_ADDR,(u16*)Current_Position,4);
    if((Current_Position[3]==0xaaaa)&&(Current_Position[2]==(Current_Position[0]^Current_Position[1])))
    {  
        uiCurrentLocationHallCnt=Current_Position[0];  //flashÖÐ´æ´¢µÄÉÏÒ»Ê±¿ÌµÄÎ»ÖÃÖµ¡£
    }
    else
    {
        uiCurrentLocationHallCnt=0;
    }
    
}

//flash¼ÇÂ¼µ±Ç°Î»ÖÃ
u8 Record_Current_Position_Flag  = 0; //µ±Ç°Î»ÖÃ³É¹¦±êÖ¾Î»
u8 Record_Current_Position_Start = 0; //µ±Ç°Î»ÖÃ¼ÇÂ¼¿ªÊ¼
void Record_Current_Position(void){
    
    if(Record_Current_Position_Start==1){
        
      Posion_temp[0] = uiCurrentLocationHallCnt;    //×ó±ß Ô­À´³ÌÐò
	    Posion_temp[1] = RIGTH+LEFT-uiCurrentLocationHallCnt;   //ÓÒ±ß
            
	    Posion_temp[2] = uiCurrentLocationHallCnt^(RIGTH+LEFT-uiCurrentLocationHallCnt);
	    Posion_temp[3] = 0xaaaa;
	
	    STMFLASH_Write(FlASH_INSTALL_ADDR,(u16*)Posion_temp,4);    //flashÐ´
	    STMFLASH_Read(FlASH_INSTALL_ADDR,(u16*)Current_Position,4);//flash¶Á
			
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
//        Posion_temp[0] = Position_All;    //×ó±ß Ô­À´³ÌÐò
//	      Posion_temp[1] = uiCurrentLocationHallCnt;   //ÓÒ±ß
//            
//	      Posion_temp[2] = Position_All^(uiCurrentLocationHallCnt);
//	      Posion_temp[3] = 0xaaaa;
//	
//	      STMFLASH_Write(FlASH_BATTERY_ADDR,(u16*)Posion_temp,4);    //flashÐ´
//	      STMFLASH_Read(FlASH_BATTERY_ADDR,(u16*)Current_Position1,4);//flash¶Á
//			
//	      if((Current_Position1[3]==Posion_temp[3])&&(Current_Position1[2]==Posion_temp[2]))
//        {
//	        open_power_flag=0;
//        }
//        else  open_power_flag = 1;     
//    }
    
}

/**************************************/
/*********²éÑ¯Ë¿¸Ü³¤¶È*****************/
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
//½»»»ÊýÖµ
void swap(u16 *p1, u16 *p2)
{
    u16 tmp = *p1;
    *p1 = *p2;
    *p2 =tmp;
}
