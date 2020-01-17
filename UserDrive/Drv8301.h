#ifndef DRV8301RS_H
#define DRV8301RS_H
#include "stm32f10x.h"
#define DRV8301RS_MSG_NUM    9

#define DRV8301_SPI_DATA_FORMATTER(RW, ADDR, DATA)    ((((RW) & 0x01) << 15) | (((ADDR) & 0x3) << 11) | (((DATA) & 0x7FF)))
#define DRV8301_SPI_CONTROL_REGISTER_1_DATA(OC_ADJ_SET,OCP_MODE,PWM_MODE,GATE_RESET,GATE_CURRENT)  ((((OC_ADJ_SET) & 0x1F) << 6)| \
										                                                           (((OCP_MODE) & 0x03) << 4)| \
                                                                                                   (((PWM_MODE) & 0X01) << 3)| \
                                                                                                   (((GATE_RESET) & 0X01) << 2)|\
                                                                                                   (((GATE_CURRENT) & 0X03)))
										
#define DRV8301_SPI_CONTROL_REGISTER_2_DATA(OC_TOFF,DC_CAL_CH2,DC_CAL_CH1,GAIN,OCTW_MODE)	((((OC_TOFF) & 0x01) << 6)| \
										(((DC_CAL_CH2) & 0x01) << 5)| (((DC_CAL_CH1) & 0X01) << 4)| (((GAIN) & 0X03) << 2)|(((OCTW_MODE) & 0X03)))
enum Drv8301_RW_Cmd_t
{
	DRV8301_CMD_WRITE = 0,
	DRV8301_CMD_READ  = 1
};

enum Drv8301_Reg_Addr_t
{
	DRV8301_ADDR_STATUS_REGISTER_1  = 0x00,
	DRV8301_ADDR_STATUS_REGISTER_2  = 0x01,
	DRV8301_ADDR_CONTROL_REGISTER_1 = 0x02,
	DRV8301_ADDR_CONTROL_REGISTER_2 = 0x03,
};
//GATE_CURRENT
enum Drv8301_GATE_CURRENT_Addr_t
{
  DRV8301_PeakCurrent_1p70_A  = 0x00,   //!< drv8301 driver peak current 1.70A
  DRV8301_PeakCurrent_0p70_A  = 0x01,   //!< drv8301 driver peak current 0.70A
  DRV8301_PeakCurrent_0p25_A  = 0x02    //!< drv8301 driver peak current 0.25A
};
//GATE_RESET
enum Drv8301_GATE_RESET_Addr_t
{
   DRV8301_Reset_Normal = 0x00,   //!< normal
   DRV8301_Reset_All = 0x01       //!< reset all
};
//PWM_MODE
enum Drv8301_PWM_MODE_Addr_t
{
  DRV8301_PwmMode_Six_Inputs   = 0x00,   //!< six independent inputs
  DRV8301_PwmMode_Three_Inputs = 0x01    //!< three independent nputs
};
//OCP_MODE
enum Drv8301_OCP_MODE_Addr_t
{
  DRV8301_OcMode_CurrentLimit  = 0x00,   //!< current limit when OC detected
  DRV8301_OcMode_LatchShutDown = 0x01,   //!< latch shut down when OC detected
  DRV8301_OcMode_ReportOnly    = 0x02,   //!< report only when OC detected
  DRV8301_OcMode_Disabled      = 0x03    //!< OC protection disabled
};
//OC_ADJ_SET
enum Drv8301_OC_ADJ_SET_Addr_t
{
  DRV8301_VdsLevel_0p060_V =  0x00,      //!< Vds = 0.060 V
  DRV8301_VdsLevel_0p068_V =  0x01,      //!< Vds = 0.068 V
  DRV8301_VdsLevel_0p076_V =  0x02,      //!< Vds = 0.076 V
  DRV8301_VdsLevel_0p086_V =  0x03,      //!< Vds = 0.086 V
  DRV8301_VdsLevel_0p097_V =  0x04,      //!< Vds = 0.097 V
  DRV8301_VdsLevel_0p109_V =  0x05,      //!< Vds = 0.109 V
  DRV8301_VdsLevel_0p123_V =  0x06,      //!< Vds = 0.123 V
  DRV8301_VdsLevel_0p138_V =  0x07,      //!< Vds = 0.138 V
  DRV8301_VdsLevel_0p155_V =  0x08,      //!< Vds = 0.155 V
  DRV8301_VdsLevel_0p175_V =  0x09,      //!< Vds = 0.175 V
  DRV8301_VdsLevel_0p197_V = 0x0A,      //!< Vds = 0.197 V
  DRV8301_VdsLevel_0p222_V = 0x0B,      //!< Vds = 0.222 V
  DRV8301_VdsLevel_0p250_V = 0x0C,      //!< Vds = 0.250 V
  DRV8301_VdsLevel_0p282_V = 0x0D,      //!< Vds = 0.282 V
  DRV8301_VdsLevel_0p317_V = 0x0E,      //!< Vds = 0.317 V
  DRV8301_VdsLevel_0p358_V = 0x0F,      //!< Vds = 0.358 V
  DRV8301_VdsLevel_0p403_V = 0x10,      //!< Vds = 0.403 V
  DRV8301_VdsLevel_0p454_V = 0x11,      //!< Vds = 0.454 V
  DRV8301_VdsLevel_0p511_V = 0x12,      //!< Vds = 0.511 V
  DRV8301_VdsLevel_0p576_V = 0x13,      //!< Vds = 0.576 V
  DRV8301_VdsLevel_0p648_V = 0x14,      //!< Vds = 0.648 V
  DRV8301_VdsLevel_0p730_V = 0x15,      //!< Vds = 0.730 V
  DRV8301_VdsLevel_0p822_V = 0x16,      //!< Vds = 0.822 V
  DRV8301_VdsLevel_0p926_V = 0x17,      //!< Vds = 0.926 V
  DRV8301_VdsLevel_1p043_V = 0x18,      //!< Vds = 1.403 V
  DRV8301_VdsLevel_1p175_V = 0x19,      //!< Vds = 1.175 V
  DRV8301_VdsLevel_1p324_V = 0x1A,      //!< Vds = 1.324 V
  DRV8301_VdsLevel_1p491_V = 0x1B,      //!< Vds = 1.491 V
  DRV8301_VdsLevel_1p679_V = 0x1C,      //!< Vds = 1.679 V
  DRV8301_VdsLevel_1p892_V = 0x1D,      //!< Vds = 1.892 V
  DRV8301_VdsLevel_2p131_V = 0x1E,      //!< Vds = 2.131 V
  DRV8301_VdsLevel_2p400_V = 0x1F       //!< Vds = 2.400 V
};
//OCTW_MODE
enum Drv8301_OCTW_MODE_Addr_t
{
  DRV8301_OcTwMode_Both    = 0x00,   //!< report both OT and OC at /OCTW pin
  DRV8301_OcTwMode_OT_Only = 0x01,   //!< report only OT at /OCTW pin
  DRV8301_OcTwMode_OC_Only = 0x02    //!< report only OC at /OCTW pin
};
//GAIN
enum Drv8301_GAIN_Addr_t
{
  DRV8301_ShuntAmpGain_10VpV = 0x00,   //!< 10 V per V
  DRV8301_ShuntAmpGain_20VpV = 0x01,   //!< 20 V per V
  DRV8301_ShuntAmpGain_40VpV = 0x02,   //!< 40 V per V
  DRV8301_ShuntAmpGain_80VpV = 0x03    //!< 80 V per V
};
//DC_CAL_CH1
enum Drv8301_DC_CAL_CH1_Addr_t
{
  DRV8301_DcCalMode_Ch1_Load   = 0x00,   //!< Shunt amplifier 1 connected to load via input pins
  DRV8301_DcCalMode_Ch1_NoLoad = 0x01,   //!< Shunt amplifier 1 disconnected from load and input pins are shorted
};
//DC_CAL_CH2
enum Drv8301_DC_CAL_CH2_Addr_t
{
  DRV8301_DcCalMode_Ch2_Load   = 0x00,   //!< Shunt amplifier 2 connected to load via input pins
  DRV8301_DcCalMode_Ch2_NoLoad = 0x01    //!< Shunt amplifier 2 disconnected from load and input pins are shorted
};
//OC_TOFF
enum Drv8301_OC_TOFF_Addr_t
{
  DRV8301_OcOffTimeMode_Normal  = 0x00,   //!< normal CBC operation
  DRV8301_OcOffTimeMode_Ctrl    = 0x01    //!< off time control during OC
};


u16 GateDriverConf(void);
u16 GateDriverRead(void);
#endif /* DRV8301*/
