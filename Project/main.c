#include "globalvariable.h"
/************************
名称：mian(void)
功能：系统mian函数
参数：None
作者：zyf
************************/
extern u16 uiHall;                        //记录霍尔传感器值
u16 SPI1_RxData[DRV8301RS_MSG_NUM] = {1}; 
short ADC_ConvertedValue[5]={0,0,0,0,0};
//int testOc  = 0;
//int testFaullt = 0;
int main(void)
{	
    //系统初始化
    initsysblock();
	//读取当前hall位置
    uiHall=GPIO_ReadInputData(GPIOB);        //读取引脚值
    uiHall=uiHall&0x01c0;
    uiHall=uiHall>>6;
    HallTemp=(u8)(uiHall&0x07);
    //控制参数初始化
    SpeedParaInit();
    SpeedSumInit();
    
    //采样零点校正
	uiCurrentZero=Caculate_CurrentZero(10);  //理论值2048，2047校准
    
    
    //查询电压设置，丝杆左右长度
	QueryVoltage();
    
	QueryLeftRight();
    //启动前自举电容充电，如果三路充电，则可能错误检测检测过流，可以忽略
    //充电电流大时间就短，上桥臂关断，下桥开通
    //三个上管是PWM强制关断，三个下管是GPIO开通
    /*****PWM的设置******/

	GPIO_SetBits(GPIOA, GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
	delay_us(20);
	GPIO_ResetBits(GPIOA, GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
	delay_us(20);
	while (1)
	{
//		if(DRV8301_OC_STATUS==0) testOc = testOc+1;
//		if(DRV8301_FAULT_STATUS==0) testFaullt = testFaullt+1;

		CheckFault();
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
            
			SpeedParaInit();
			SpeedSumInit();
		}
		if(uc1msFlag)
		{
//			GateDriverRead();
			uc1msFlag=0;
			uc50msCnt++;
			uc2msCnt++;
			if(uc2msCnt>=2)
			{
				uc2msCnt = 0;
				CheckCmdData();//（先不测试通讯，给定制测试）
			}
			if(uc50msCnt>=50)  //进行50换成14
			{
				uc50msCnt=0;
				uiHallSpeedBackTempCnt=uiHallSpeedBackCnt;//一圈是42个
				Motor_speed_calulate();                   //进行了速度环位置环的计算
                           
				uiHallSpeedBackCnt=0;                     //因为50ms变化的脉冲已经拿到了
				UserMosTem = Caculate_Tem((4096-uiMosTemADFbk)) - 20;
                
                overtemperature_protect();               //过温度5s保护               
			}
	
		}
		CalibrationPosition(); //丝杠校准
		DriveCalibration();    //船前进正方向对应方向盘0位校准
		IWDG_ReloadCounter();  //reload
	}
}
