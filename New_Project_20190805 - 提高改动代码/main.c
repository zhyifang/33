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
���ƣ�mian(void)
���ܣ�ϵͳmian����
������None
���ߣ�zyf
************************/
extern u16 uiHall;                        //��¼����������ֵ
u16 SPI1_RxData[DRV8301RS_MSG_NUM] = {1}; 
//short ADC_ConvertedValue[5]={0,0,0,0,0};
u8 FAULT_state,OC_state;

int main(void)
{	
	//ϵͳ��ʼ��
    initsysblock();
	//ͨ������
    //UART2_Iint(9600);   
	//��ȡ��ǰhallλ��
    uiHall=GPIO_ReadInputData(GPIOB);           //��ȡ����ֵ
    uiHall=uiHall&0x01c0;
    uiHall=uiHall>>6;
    HallTemp=(u8)(uiHall&0x07);
    //���Ʋ�����ʼ��
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
	
    //�������У��
    uiCurrentZero=Caculate_CurrentZero(20);     //����ֵ2048��2047У׼
    //��ѯ�û�У������
	QueryUser_Calibration();
//  QueryCurrent_position();
    //��ѯ�û����ã�˿�����ҳ���
	QueryLeftRight();
    //����ǰ�Ծٵ��ݳ�磬�����·��磬����ܴ���������������Ժ���
    //��������ʱ��Ͷ̣����ű۹ضϣ����ſ�ͨ
    //�����Ϲ���PWMǿ�ƹضϣ������¹���GPIO��ͨ
    /*****PWM������******/
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
				//�����˵������ļ���
				CurrentPara.Ref = uiLimitCurrent;     //����A
				CurrentPara.Fbk = UserCurrent10Times;
				PID_Current(&CurrentPara);            
                //�����˹��ʻ��ļ���               
				PowerPara.Ref = uiLimitPower;         //����W
				PowerPara.Fbk = UserPower;
				PID_Power(&PowerPara);               
			}	                  
			if(uc2msCnt>=2)
			{
				uc2msCnt = 0;
				CheckCmdData();                      //5182ͨѶ
			}
			if(uc50msCnt>=10)                        //10ms
			{
				uc50msCnt=0;
				uiHallSpeedBackTempCnt=uiHallSpeedBackCnt;             //һȦ��42��
                if(keytemp==1)
                {
				
	              if(SetOK==1)
		           SpeedPara.Ref = uiCalibrationSpeed;                 //У���ٶ�����uiCalibrationSpeed = 1000;
	              if(SetOK==0)
                   {SpeedPara.Ref = uiSpeedSet;}
	              
                  SpeedPara.Fbk = uiHallSpeedBackTempCnt*1000/7;       //speed = (cnt/(6*7))*60/50ms = 200 cnt/7��rpm / min
	              PID_Speed(&SpeedPara);                               //�ٶȻ�·
                                                                       //speed = (cnt/(6*7))*60/10ms = 1000 cnt/7��rpm / min
                }      
				uiHallSpeedBackCnt=0;                                  //��Ϊ50ms�仯�������Ѿ��õ���
				
                UserMosTem = Caculate_Tem((4096-uiMosTemADFbk)) - 20;
                overtemperature_protect();                             //���¶�1s����               
			}
	
		}
        CheckFault();                    //���ϲ�ѯ
        //Record_Current_Position();     //��¼��ǰ˿��λ��
		CalibrationPosition();           //˿��У׼
		DriveCalibration();              //��ǰ���������Ӧ������0λУ׼
		IWDG_ReloadCounter();            //reload
	}
}
