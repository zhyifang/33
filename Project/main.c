#include "globalvariable.h"
/************************
���ƣ�mian(void)
���ܣ�ϵͳmian����
������None
���ߣ�zyf
************************/
extern u16 uiHall;                        //��¼����������ֵ
u16 SPI1_RxData[DRV8301RS_MSG_NUM] = {1}; 
short ADC_ConvertedValue[5]={0,0,0,0,0};
//int testOc  = 0;
//int testFaullt = 0;
int main(void)
{	
    //ϵͳ��ʼ��
    initsysblock();
	//��ȡ��ǰhallλ��
    uiHall=GPIO_ReadInputData(GPIOB);        //��ȡ����ֵ
    uiHall=uiHall&0x01c0;
    uiHall=uiHall>>6;
    HallTemp=(u8)(uiHall&0x07);
    //���Ʋ�����ʼ��
    SpeedParaInit();
    SpeedSumInit();
    
    //�������У��
	uiCurrentZero=Caculate_CurrentZero(10);  //����ֵ2048��2047У׼
    
    
    //��ѯ��ѹ���ã�˿�����ҳ���
	QueryVoltage();
    
	QueryLeftRight();
    //����ǰ�Ծٵ��ݳ�磬�����·��磬����ܴ���������������Ժ���
    //��������ʱ��Ͷ̣����ű۹ضϣ����ſ�ͨ
    //�����Ϲ���PWMǿ�ƹضϣ������¹���GPIO��ͨ
    /*****PWM������******/

	GPIO_SetBits(GPIOA, GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
	delay_us(20);
	GPIO_ResetBits(GPIOA, GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
	delay_us(20);
	while (1)
	{
//		if(DRV8301_OC_STATUS==0) testOc = testOc+1;
//		if(DRV8301_FAULT_STATUS==0) testFaullt = testFaullt+1;

		CheckFault();
		if(keytemp==1)   //����
		{

			TIM_CtrlPWMOutputs(TIM1, ENABLE); 
            TIM_Cmd(TIM1, ENABLE);
            Hall_SW();
		}	
		if(keytemp==2)   //ֹͣ,��߿��Ըı�ֹͣ�ķ�ʽ
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
				CheckCmdData();//���Ȳ�����ͨѶ�������Ʋ��ԣ�
			}
			if(uc50msCnt>=50)  //����50����14
			{
				uc50msCnt=0;
				uiHallSpeedBackTempCnt=uiHallSpeedBackCnt;//һȦ��42��
				Motor_speed_calulate();                   //�������ٶȻ�λ�û��ļ���
                           
				uiHallSpeedBackCnt=0;                     //��Ϊ50ms�仯�������Ѿ��õ���
				UserMosTem = Caculate_Tem((4096-uiMosTemADFbk)) - 20;
                
                overtemperature_protect();               //���¶�5s����               
			}
	
		}
		CalibrationPosition(); //˿��У׼
		DriveCalibration();    //��ǰ���������Ӧ������0λУ׼
		IWDG_ReloadCounter();  //reload
	}
}
