//#include "stm32f10x.h"
//#include "MotorControl.h"
//#include "USART.h"
//void USART3_Init(u32 bound)
//{
//  //GPIO端口设置
//    USART_InitTypeDef USART_InitStructure;

//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

//    //USART1_TX   GPIOA.2


//  //Usart NVIC 配置


//   //USART 初始化设置

//    USART_InitStructure.USART_BaudRate = bound;//串口波特率
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
//    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//    USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;   //收发模式

//    USART_Init(USART3, &USART_InitStructure); //初始化串口
//    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
//    USART_Cmd(USART3, ENABLE);                  //使能串口
//}


//#define USART_RXBUFSIZE     32
//#define USART_TXBUFSIZE     32

//#define CMD_HEADER           0X55
//#define POWER                0X70
//#define CALIBRATION          0X71
//#define MOTOR_STATE          0X51
//#define CAL_BACK             0X50



//typedef struct {
//    u8 RxBuf[USART_RXBUFSIZE];
//    u8 TxBuf[USART_TXBUFSIZE];
//    u8 RxHeadPoint;
//    u8 RxTailPoint;
//    u8 TxHeadPoint;
//    u8 TxTailPoint;
//    u8 RxCount;
//    volatile u8 TxCount;
//}Usart_Buffer_t;

//Usart_Buffer_t UsartBuf;
//extern u8 keytemp;

//u8 USART_QueryChar(u8 *ch)
//{
//    if (UsartBuf.RxCount == 0)
//    return 0;
//    *ch = UsartBuf.RxBuf[UsartBuf.RxHeadPoint];
//    UsartBuf.RxHeadPoint++;
//    if (UsartBuf.RxHeadPoint >= USART_RXBUFSIZE)
//        UsartBuf.RxHeadPoint = 0;
//    __disable_irq();//__set_PRIMASK(1);
//    UsartBuf.RxCount--;
//    __enable_irq();//__set_PRIMASK(0);
//    return 1;
//}

//u8 USART_ReadTxBuf (void)
//{
//    u8 ch;
//    ch = UsartBuf.TxBuf[UsartBuf.TxHeadPoint];
//    UsartBuf.TxHeadPoint++;
//    if (UsartBuf.TxHeadPoint >= USART_TXBUFSIZE)
//    UsartBuf.TxHeadPoint = 0;
//    UsartBuf.TxCount--;
//    return ch;
//}

//static u8 USART_WriteRxBuf (u8 ch)
//{
//    UsartBuf.RxBuf[UsartBuf.RxTailPoint] = ch;
//    UsartBuf.RxTailPoint++;
//    if (UsartBuf.RxTailPoint >= USART_RXBUFSIZE)
//    {
//        UsartBuf.RxTailPoint = 0;
//    }
//    // overflow
//    if (++UsartBuf.RxCount >= USART_RXBUFSIZE)
//    {
//        return 1;
//    }
//    return 0;
//}

//void USART_HandleTxd(void)
//{

//    if ((USART3->SR & 0x0080) != (u16)RESET)
//    {

//        if (UsartBuf.TxCount)
//        {
//           USART3->DR = USART_ReadTxBuf();
//        }
//    }

//    if (UsartBuf.TxCount == 0)
//    {
//        if(USART3->SR & 0x40)
//        {
//            RECIVE_DATA();
//        }
//    }
//}
//u8 USART_PutChar(u8 ch)
//{
//    if (UsartBuf.TxCount >= USART_TXBUFSIZE) return 0;
//    UsartBuf.TxBuf[UsartBuf.TxTailPoint] = ch;
//    UsartBuf.TxTailPoint++;
//    if (UsartBuf.TxTailPoint >= USART_TXBUFSIZE)
//    UsartBuf.TxTailPoint = 0;
//    UsartBuf.TxCount++;
//    return 1;
//}
//u16 uiErrorInterrupt;
//void USART3_IRQHandler(void)
//{
//    if ((USART3->SR & (u32)0x00000020) != (u16)RESET)
//    {
//        USART_ClearITPendingBit(USART3,USART_IT_RXNE);
//        USART_WriteRxBuf (USART3->DR);
//    }
//}
//u8 cmdStep=0;
//u8 cmdPtr=0;
//u8 ucCheckSum=0;
//u8 ucCmdData[3];
//u8 RxCommand(void)
//{
//    u8 ch;
//    if (USART_QueryChar(&ch))
//    {
//        switch(cmdStep)
//        {
//        case 0:
//            if (ch == CMD_HEADER)//命令头
//            {
//                cmdStep++;
//                ucCheckSum=CMD_HEADER;
//            }
//            cmdPtr = 0;
//            break;
//        case 1:                 //命令码
//            if((ch==POWER)||(ch==CALIBRATION))
//            {
//                cmdStep++;
//                ucCheckSum^=ch;
//                ucCmdData[cmdPtr++]=ch;
//            }
//            else
//            {
//                cmdStep=0;
//                cmdPtr=0;
//            }

//            break;
//        case 2:
//            ucCmdData[cmdPtr++]=ch;
//            ucCheckSum^=ch;
//            if(cmdPtr>=3)
//            {
//                cmdStep++;
//            }
//            break;
//        case 3:
//            cmdStep=0;
//            cmdPtr=0;
//            if((ucCheckSum&0x7f)==ch)
//            {
//                return ucCmdData[0];
//            }

//            break;
//        default:
//            break;
//        }
//    }
//    return 0;
//}
//unsigned char cmdTxBuf[5];
//void TxCommand(unsigned char cmdbyte)
//{
//    unsigned char ipos=0;
//    unsigned char cmdtxCS=0;
//    unsigned char icnt=0;

//    ipos = 0;
//    cmdTxBuf[ipos++] = CMD_HEADER;
//    switch(cmdbyte)
//    {

//    case POWER:
//        cmdTxBuf[ipos++] = MOTOR_STATE;
//        cmdTxBuf[ipos++] = 0X00;
//        cmdTxBuf[ipos++] = 0X00;
//        break;
//    case CALIBRATION:
//        cmdTxBuf[ipos++] = CAL_BACK;
//        cmdTxBuf[ipos++] = 0X00;
//        if(ucCalibrationSuccessFlag==1)
//        {
//            cmdTxBuf[ipos++] = 0X01;
//        }
//        else
//        {
//            cmdTxBuf[ipos++] = 0X00;
//        }
//        break;
//    default:
//        break;
//    }
//    for(icnt=0,cmdtxCS=0;icnt<ipos;icnt++)
//    {
//        cmdtxCS^=cmdTxBuf[icnt];
//    }
//    cmdTxBuf[ipos++] = cmdtxCS&0X7F;
//    for (cmdtxCS = 0; cmdtxCS < ipos; cmdtxCS++)
//    {
//        USART_PutChar(cmdTxBuf[cmdtxCS]);
//    }
//}


//unsigned char HandleCommand(void)
//{
//    unsigned char cmdbyte;
//    if (RxCommand())
//    {
//        cmdbyte = ucCmdData[0];
//        switch(cmdbyte)
//        {

//        case POWER:
//            ucSteeringDrection=ucCmdData[1];
//            ucAngle=ucCmdData[2];
//            break;

//        case CALIBRATION:
//            ucCalibrationFlag=1;
//            break;
//        default:
//            break;

//        }

//        SEND_DATA();
//        TxCommand(cmdbyte);

//    }
//		return 0;

//}

//void CheckCmdData(void)
//{
//    HandleCommand();
//    USART_HandleTxd();
//}

//void CheckCmdData(void)
//{
//	ucCalibrationFlag=1;
//}

