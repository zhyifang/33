#include "5182.h"
#include "GPIO.h"
#include "MotorControl.h"
u8 uctest[5];
SPI5182_Buffer_t SPI5182_Buf;



u8 LastCmdByte = 0;
u8 change=0;
u16 test=0;
u16 test1=0;
u8 open_power_flag=0;
u8 open_power_flag_finish=0;

u8 power_finish_flag=0;

unsigned char cmdTxBuf[5];

//取接收循环队列数据
u8 SPI5182_QueryChar(u8 *ch)
{
    if (SPI5182_Buf.RxCount == 0)
    return 0;
    *ch = SPI5182_Buf.RxBuf[SPI5182_Buf.RxHeadPoint];
    SPI5182_Buf.RxHeadPoint++;
    if (SPI5182_Buf.RxHeadPoint >= SPI5182_RXBUFSIZE)
        SPI5182_Buf.RxHeadPoint = 0;
    __disable_irq();//__set_PRIMASK(1);
    SPI5182_Buf.RxCount--;
    __enable_irq();//__set_PRIMASK(0);
    return 1;
}

//获取发送队列数据
u8 SPI5182_ReadTxBuf (void)
{
    u8 ch;
    ch = SPI5182_Buf.TxBuf[SPI5182_Buf.TxHeadPoint];
    SPI5182_Buf.TxHeadPoint++;
    if (SPI5182_Buf.TxHeadPoint >= SPI5182_TXBUFSIZE)
    SPI5182_Buf.TxHeadPoint = 0;
    SPI5182_Buf.TxCount--;
    return ch;
}

//填充接收队列数据
u8 SPI5182_WriteRxBuf (u8 ch)
{
    SPI5182_Buf.RxBuf[SPI5182_Buf.RxTailPoint] = ch;
    SPI5182_Buf.RxTailPoint++;
    if (SPI5182_Buf.RxTailPoint >= SPI5182_RXBUFSIZE)
    {
        SPI5182_Buf.RxTailPoint = 0;
    }
    // overflow
    if (++SPI5182_Buf.RxCount >= SPI5182_RXBUFSIZE)
    {
        return 1;
    }
    return 0;
}
//填充发送数据队列
u8 SPI5182_PutChar(u8 ch)
{
    if (SPI5182_Buf.TxCount >= SPI5182_TXBUFSIZE) 
	{
		return 0;
	}
    SPI5182_Buf.TxBuf[SPI5182_Buf.TxTailPoint] = ch;
    SPI5182_Buf.TxTailPoint++;
    if (SPI5182_Buf.TxTailPoint >= SPI5182_TXBUFSIZE)
		SPI5182_Buf.TxTailPoint = 0;
    SPI5182_Buf.TxCount++;
    return 1;
}



u8 cmdStep=0;
u8 cmdPtr=0;
u8 ucCheckSum=0;
u8 ucCmdData[4];
u8 ucCheckSumTest=0;
u8 Time=0;
#if 1
u8 RxCommand(void)
{
    u8 ch;

    if (SPI5182_QueryChar(&ch))
    {
        switch(cmdStep)
        {
            
        case 0:
            if (ch == CMD_HEADER)//命令头
            {
                cmdStep++;
                ucCheckSum=CMD_HEADER;
            }
			 else
		    {
			    cmdStep = 0;
		    }
            cmdPtr = 0;
            break;
        case 1:                 //命令码
            if((ch==ClOSE_POWER)||(ch==OPEN_POWER)||(ch==POWER)||(ch==CALIBRATION)||
                (ch==DRIVEZERORPOSITON)||(ch==SETMOTORVOLTAGE)||(ch ==MOTORFAULTSTOP)||
                 (ch ==OPENCOMMUNICATE)||(ch==SET_INSTALL_LOCATION))
            {
                cmdStep++;
                ucCheckSum^=ch;
                ucCmdData[cmdPtr++]=ch;
            }
            else
            {
                cmdStep=0;
                cmdPtr=0;
            }

            break;
        case 2:
            ucCmdData[cmdPtr++]=ch;
            ucCheckSum^=ch;
            if(cmdPtr>=3)
            {
                cmdStep++;
            }
            break;
        case 3:
            cmdStep=0;
           // cmdPtr=0;
//            ucCmdData[cmdPtr++]=ch;
			cmdPtr = 0;
			ucCheckSumTest = ch;
				
				
            if((ucCheckSum&0xff)==ch)//0x7f
            {
                test--;             //减一 将GPIO拉低，测量两个通信的时间  
//               if(Time==0)              
//              { 
//                 GPIO_ResetBits(GPIOA,  GPIO_Pin_0);              
//                 Time=1; 
//              }
//              else
//              {
//                
//                 GPIO_SetBits(GPIOA,  GPIO_Pin_0);              
//                 Time=0; 
//              }

				return ucCmdData[0];
				
            }

            break;
        default:
            break;
        }
    }
    return 0;
}
#endif

static uint8_t Motor_CheckSum(uint8_t* data, uint8_t size)
{
	uint8_t i;
	uint8_t result = 0;
	
	for(i = 0; i < size; i++)
	{
		result = result ^ data[i];
	}
	return result;
}


void MotorPrepareCmd(u8 cmd,u8 dataH,u8 dataL)
{
	cmdTxBuf[0] = CMD_HEADER;
	cmdTxBuf[1] = cmd;
	cmdTxBuf[2] = dataH;
	cmdTxBuf[3] = dataL;
	cmdTxBuf[4] = Motor_CheckSum(cmdTxBuf, 4);
}

void TxCommand(unsigned char cmdbyte)
{
    unsigned char cmdtxCS=0;
    switch(cmdbyte)
    {
		
        case ClOSE_POWER:
			    MotorPrepareCmd(CLOSE_POWER_ready,0,0);
                Record_Current_Position_Start=1;
				MototStatus=0;

		    	break;
        case OPEN_POWER:    //

			    MotorPrepareCmd(OPEN_POWER_ready,0,0);
				open_power_flag_finish=1;		
                GPIO_SetBits(GPIOA,  GPIO_Pin_0);

			    break;
        
        case POWER:   //方向盘油门角度
		    	SetBatteryVoltageOK = 0;
		    	SetBatteryVoltageFailed = 0;
               if(change==0)//发送的实时位置给方向盘
			   {
                MotorPrepareCmd(ClOSE_POWER_Acknowed,uiCurrentLocationHallCnt>>8,uiCurrentLocationHallCnt&0x00FF);//区分高八位，低八位。
                change=1;
//              GPIO_SetBits(GPIOA,  GPIO_Pin_0);
               }
               else
               {
                   MotorPrepareCmd(MOTOR_STATE,0,MototStatus);    
                   change=0;
//                 GPIO_ResetBits(GPIOA,  GPIO_Pin_0);
               }   
               
        
			   break;
		case CALIBRATION:

			    if(SetOK==0)
			    {
			  	   MotorPrepareCmd(CAL_BACK,0,0X10);
		    	}
			    else if(ucCalibrationFailedFlag==1)
			    {
			     	if((ucLeftShortFlag==1)&&(ucRightShortFlag==1))
				     	MotorPrepareCmd(CAL_BACK,0,0X23);//cmdTxBuf[ipos++] = 0X23;
				    else if((ucLeftShortFlag==1)&&(ucRightShortFlag==0))
				   	    MotorPrepareCmd(CAL_BACK,0,0X21);//cmdTxBuf[ipos++] = 0X21;
			   	    else if((ucLeftShortFlag==0)&&(ucRightShortFlag==1))
				   	    MotorPrepareCmd(CAL_BACK,0,0X22);//cmdTxBuf[ipos++] = 0X22;
			    }
			    else
				     MotorPrepareCmd(CAL_BACK,0,0X40);    //cmdTxBuf[ipos++] = 0X40;     
			  break;
			
		   case OPENCOMMUNICATE:
		    	MotorPrepareCmd(OPENCOMMUNICATE,0,0);
			  break;
		   case DRIVEZERORPOSITON:
		     	if(ucDriveCalibrationSucFlag==0)
		   		MotorPrepareCmd(DRIVEZEROACKNOWLEGE,0,0X02);//cmdTxBuf[ipos++] = 0x02;
		    	else if(ucDriveCalibrationSucFlag==1)
			    {
			    	ucDriveCalibrationSucFlag = 0;
			     	MotorPrepareCmd(DRIVEZEROACKNOWLEGE,0,0x01);//cmdTxBuf[ipos++] = 0x01;
			    }
			    break;
		   case  SETMOTORVOLTAGE:
			    if(SetBatteryVoltageOK==1)
		  	   {
			        if(MototStatus!=0)
			  		MotorPrepareCmd(MOTORVOLTAGEACKNOWLEGE,0,0x03);
		   	     	else
			   		MotorPrepareCmd(MOTORVOLTAGEACKNOWLEGE,0,0x01);//cmdTxBuf[ipos++] = 0X01;
			   	    SetBatteryVoltageOK = 0;
			   }
			   else if(SetBatteryVoltageFailed==1)
				    MotorPrepareCmd(MOTORVOLTAGEACKNOWLEGE,0,0X02);//cmdTxBuf[ipos++] = 0X02;
		       else
				    MotorPrepareCmd(MOTORVOLTAGEACKNOWLEGE,0,0X02);//cmdTxBuf[ipos++] = 0X02;
			    break;
			
		case MOTORFAULTSTOP:
			if(ucClearFault==100)
			{
				ucClearFault = 0;
				Error_Acknowledge();
				MotorPrepareCmd(MOTOR_STATE,0,MototStatus);//s
			}
			else	
			{
				MotorPrepareCmd(MotorStopAcknowed,0,0X00);
			}
			break;
		default:
			break;
    }
    test++;         //加一将GPIO至高
    

    for (cmdtxCS = 0; cmdtxCS < 5; cmdtxCS++)
    {
		uctest[cmdtxCS]= cmdTxBuf[cmdtxCS];  //测试
        		
		SPI5182_PutChar(cmdTxBuf[cmdtxCS]);
		SPI5182_Buf.TxBuf[cmdtxCS] = cmdTxBuf[cmdtxCS];
    }
}


u16 sendTest[6] = {10,11,12,13,14,15};
u16 Position_H=0;
u8  Position_L=0;
int Position_All=0;
//接受到命令进行处理，并将tx返回必要数据
unsigned char HandleCommand(void)
{
    unsigned char cmdbyte;
	
    if (RxCommand())
    {
        cmdbyte = ucCmdData[0];
		LastCmdByte = cmdbyte;
        switch(cmdbyte)
        {

        case ClOSE_POWER:     //当收到关机指令
            

            break;
        case OPEN_POWER:     // 0X76 
            //当收到开机指令
            //test++;
            open_power_flag=1;
			Position_H=ucCmdData[1]<<8;
            Position_L=ucCmdData[2];
			Position_All=Position_H+Position_L;
			uiCurrentLocationHallCnt=Position_All;
        

            break;
        case POWER:

            ucSteeringDrection=ucCmdData[1];
            ucAngle=(u16)ucCmdData[2];
			power_finish_flag=1;
		   	if(ucAngle>127)
			   ucAngle = 127; 
//           if(Time==0)              
//           { 
//               GPIO_ResetBits(GPIOA,  GPIO_Pin_0);              
//               Time=1; 
//            }
//            else
//            {
//                
//               GPIO_SetBits(GPIOA,  GPIO_Pin_0);              
//               Time=0; 
//           }
            break;

        case CALIBRATION:
            ucCalibrationFlag=1;
		//	ucCalibrationFlag=0;
        //	ucCalibrationSuccessFlag=0;
        //  ucCalibrationFailedFlag=0;
            break;
		
	    case DRIVEZERORPOSITON:
		      	ucDriveCalibrationFlag= 1;
		        ucSteeringDrection = ucCmdData[1];
                ucAngle     = (u16)ucCmdData[2];
		  	    if(ucAngle>127)
			  	  ucAngle = 127;
		       	break;
	    case SETMOTORVOLTAGE:
		     	SetBatteryVoltage = 1;                //设置电压标志位
		      	BatteryVolgate = ucCmdData[2];
			      break;
	    case MOTORFAULTSTOP:                       //停机处理
			      MotorStopFlag = 1;
			      if(ucCmdData[2]==100)
				    ucClearFault = 100;
			      break;
        default:
            break;

        }
    }
	TxCommand(cmdbyte);
	return 0;
}

//SPI片选发送buf里面SPI5182_ReadTxBuf()的五个字节分别是[头地址、命令指令、高字节、低字节、校正码]
//将接受的五字节存入BUf中，SPI5182_WriteRxBuf(SPI_I2S_ReceiveData(SPI1));
u16 SPI5182_HandleTxd(void)
{
	u16 retry=0;	
	while (SPI5182_Buf.TxCount)
	{
		GPIO_ResetBits(MDBT40_CS_PORT, MDBT40_CS_PIN);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
		{
			retry++;
			if(retry>200)return 0;
		}
		retry=0;
		SPI_I2S_SendData(SPI1,SPI5182_ReadTxBuf());
        
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
		{
			retry++;
			if(retry>200)return 0;
		}
		retry = 0;
		SPI5182_WriteRxBuf(SPI_I2S_ReceiveData(SPI1));
	}
    
    
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
	{
		retry++;
		if(retry>1000)
		{
			return 0;
		}
	}
    
    //加入片选计数
	GPIO_SetBits(MDBT40_CS_PORT, MDBT40_CS_PIN);
	return 1;
}



//检测通讯命令
void CheckCmdData(void)
{
	 if(ucCalibrationFlag==0)   //SetBatteryVoltageNeeded==0，if((ucCalibrationFlag==0)&&(SetBatteryVoltageNeeded==0))  
	 {
       if(SetOK==1)
	       TxCommand(CALIBRATION);
       else  
		   TxCommand(POWER);
	 }

   HandleCommand();         //接受到命令存入buf中
   SPI5182_HandleTxd();     //发送命令，接受到命令存入buf中
}

