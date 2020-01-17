#include "globalvariable.h"
#include "UART.h"
#include <stdio.h>
int fputc(int ch, FILE *f)
{
 
 USART_SendData(USART2, (uint8_t) ch);
 
 while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
 
 return ch;

}
/************************
名称：mian(void)
功能：系统mian函数
参数：None
作者：zyf
************************/
extern u16 uiHall;                        //记录霍尔传感器值
u16 SPI1_RxData[DRV8301RS_MSG_NUM] = {1}; 
//short ADC_ConvertedValue[5]={0,0,0,0,0};
u8 FAULT_state,OC_state;

int main(void)
{	
	//系统初始化
    initsysblock();
	//通信速率
    //UART2_Iint(9600);   
	//读取当前hall位置
    uiHall=GPIO_ReadInputData(GPIOB);           //读取引脚值
    uiHall=uiHall&0x01c0;
    uiHall=uiHall>>6;
    HallTemp=(u8)(uiHall&0x07);
    //控制参数初始化
    SpeedParaInit();
    SpeedSumInit();
	PowerParaInit();
    CurrentParaInit();
    PositionParaInit();
//
//  GPIO_InitTypeDef GPIO_InitStructure;
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );
//  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOA,&GPIO_InitStructure);
	
    //采样零点校正
    uiCurrentZero=Caculate_CurrentZero(20);     //理论值2048，2047校准
    //查询用户校正设置
	QueryUser_Calibration();
//  QueryCurrent_position();
    //查询用户设置，丝杆左右长度
	QueryLeftRight();
    //启动前自举电容充电，如果三路充电，则可能错误检测检测过流，可以忽略
    //充电电流大时间就短，上桥臂关断，下桥开通
    //三个上管是PWM强制关断，三个下管是GPIO开通
    /*****PWM的设置******/
	 GPIO_SetBits(GPIOA, GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
	 delay_us(40);
	 GPIO_ResetBits(GPIOA, GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
	 delay_us(40);
	 while (1)
	 {
//		USART_SendData(USART2, ucAngle);
//		USART_SendData(USART2, uiHallSpeedBackTempCnt);
//		USART_SendData(USART2, SpeedPara.Fbk);
//		printf("%d\n",HallTemp);
//		Uart2_TxCommand(0x88);
		
//	    USART2_PutChar(uiHall);
//		USART2_PutChar(SpeedPara.Fbk/256);
//		USART2_PutChar(SpeedPara.Fbk%256);
		
//      USART2_HandleTxd();		
//	    Uart2_CheckCmdData();
      
//      FAULT_state = GPIO_ReadInputDataBit(DRV8301_FAULT_GPIO_PORT, DRV8301_FAULT_PIN);
//      OC_state   = GPIO_ReadInputDataBit(DRV8301_OC_GPIO_PORT, DRV8301_OC_PIN);
//      GPIO_SetBits( GPIOB,  GPIO_Pin_5);
//      GPIO_ResetBits(GPIOB,  GPIO_Pin_5);
		if(keytemp==1)   //启动
		{
		   TIM_CtrlPWMOutputs(TIM1, ENABLE); 
           TIM_Cmd(TIM1, ENABLE);
           Hall_SW();
		}	
		if(keytemp==2)   //停止,这边可以改变停止的方式
		{
			TIM_CtrlPWMOutputs(TIM1, DISABLE);
			GPIO_ResetBits(GPIOA, GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
            
            PositionParaInit();
			SpeedParaInit();
			SpeedSumInit();
            PowerParaInit();
            CurrentParaInit();
		}
		if(uc1msFlag)
		{
			uc1msFlag=0;
			uc50msCnt++;
			uc2msCnt++;

            if(keytemp==1)
			{
				//进行了电流环的计算
				CurrentPara.Ref = uiLimitCurrent;     //电流A
				CurrentPara.Fbk = UserCurrent10Times;
				PID_Current(&CurrentPara);            
                //进行了功率环的计算               
				PowerPara.Ref = uiLimitPower;         //功率W
				PowerPara.Fbk = UserPower;
				PID_Power(&PowerPara);               
			}	                  
			if(uc2msCnt>=2)
			{
				uc2msCnt = 0;
				CheckCmdData();                      //5182通讯
			}
			if(uc50msCnt>=10)                        //10ms
			{
				uc50msCnt=0;
				uiHallSpeedBackTempCnt=uiHallSpeedBackCnt;             //一圈是42个
                if(keytemp==1)
                {
				
	              if(SetOK==1)
		           SpeedPara.Ref = uiCalibrationSpeed;                 //校正速度设置uiCalibrationSpeed = 1000;
	              if(SetOK==0)
                   {SpeedPara.Ref = uiSpeedSet;}
	              
                  SpeedPara.Fbk = uiHallSpeedBackTempCnt*1000/7;       //speed = (cnt/(6*7))*60/50ms = 200 cnt/7，rpm / min
	              PID_Speed(&SpeedPara);                               //速度环路
                                                                       //speed = (cnt/(6*7))*60/10ms = 1000 cnt/7，rpm / min
                }      
				uiHallSpeedBackCnt=0;                                  //因为50ms变化的脉冲已经拿到了
				
                UserMosTem = Caculate_Tem((4096-uiMosTemADFbk)) - 20;
                overtemperature_protect();                             //过温度1s保护               
			}
	
		}
        CheckFault();                    //故障查询
        //Record_Current_Position();     //记录当前丝杆位置
		CalibrationPosition();           //丝杠校准
		DriveCalibration();              //船前进正方向对应方向盘0位校准
		IWDG_ReloadCounter();            //reload
	}
}
