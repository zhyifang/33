#include "IWDG.h"
void IWDG_Init(u8 prer,u16 rlr)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //ʹ�ܶԼĴ���IWDG_PR��IWDG_RLR��д����

    IWDG_SetPrescaler(prer);  //����IWDGԤ��Ƶֵ

    IWDG_SetReload(rlr);  //����IWDG��װ��ֵ

    IWDG_ReloadCounter();  //����IWDG��װ�ؼĴ�����ֵ��װ��IWDG������

    IWDG_Enable();  //ʹ��IWDG
}
