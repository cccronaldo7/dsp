/*********************************************
-- ��Ȩ(copyright)�����ҵ��ܱ任����ƹ��̼����о�����(NECC)
-- ��Ŀ��:
-- ģ����:
-- �ļ���:
-- ����:  �ſ�
-- ���ܺ��ص����: FPGA�������߳��� DMA XINTF��ʼ��
-- ��ʼ�汾�ͷ���ʱ��: 1.00��2022-03-20
---------------------------------------------------
-- ������ʷ:
---------------------------------------------------
-- ���İ汾�͸���ʱ�䣺
-- ������Ա��
-- ��������:
-- ���İ汾�͸���ʱ�䣺
-- ������Ա����
-- ��������: ��
*********************************************/

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "User.h"

/************* �����ڴ�ռ� *************/
#pragma DATA_SECTION(DMABuf1,"DMARAML4");
#pragma DATA_SECTION(DMABuf2,"ZONE7DATA");

volatile Uint16 DMABuf1[32];
volatile Uint16 DMABuf2[32];
volatile Uint16 *DMADest;
volatile Uint16 *DMASource;

void DMA_Chn_Init(void)
{
    DMASource = &DMABuf1[0];
    DMADest = &DMABuf2[0];

    DMACH1AddrConfig(DMADest,DMASource);  // ���õ�ַ
    DMACH1BurstConfig(31,1,1);  // ����ÿ֡�����֣�word��1+31��֡��Դ��ַƫ��+1��֡��Ŀ�ĵ�ַƫ��+1
    DMACH1TransferConfig(0,1,1);  // ����ÿ�δ���DMAת�ƶ���֡1+0��֡����Դ��ַƫ��+1��֡����Ŀ�ĵ�ַƫ��+1
    DMACH1WrapConfig(0xFFFF,0,0xFFFF,0);  // ����Wrap(����ѭ��)

    // DMA CH1ģʽ����
    // �����ж�ʹ�ܣ����δ���(һ�δ����ź��������Burst����)����������������ͬ����ֹ��Դͬ��������жϽ�ֹ��16λģʽ��DMA���������жϣ��жϽ�ֹ
    DMACH1ModeConfig(0,PERINT_ENABLE,ONESHOT_ENABLE,CONT_ENABLE,SYNC_DISABLE,SYNC_SRC,OVRFLOW_DISABLE,SIXTEEN_BIT,CHINT_END,CHINT_DISABLE);

    // ����CH1 DMA�� ��������ģʽ��(CONT_ENABLE)��DMAִ�н��������³�ʼ����CONT_DISENABLE��DMAִ�н�����DMAֹͣ��RUNSTSλ����
    StartDMACH1();
}

void DMA_Ch1_Trigger(void)
{
    //�����������PERINTFLG��λ��DMAִ�к��Զ���PERINTFLG����
    EALLOW;
    DmaRegs.CH1.CONTROL.bit.PERINTFRC = 1;
    EDIS;
}

void Zone7_Init(void)
{
    EALLOW;
    // Make sure the XINTF clock is enabled
    SysCtrlRegs.PCLKCR3.bit.XINTFENCLK = 1;
    EDIS;

    // Configure the GPIO for XINTF with a 16-bit data bus
    // This function is in DSP2833x_Xintf.c
    InitXintf16Gpio();

    // All Zones---------------------------------
    EALLOW;
    // Timing for all zones based on XTIMCLK = SYSCLKOUT(XTIMCLK = 0),XTIMCLK = SYSCLKOUT/2(XTIMCLK = 1)
    XintfRegs.XINTCNF2.bit.XTIMCLK = 1;
    // Buffer up to 0 writes
    XintfRegs.XINTCNF2.bit.WRBUFF = 0;
    // XCLKOUT is disabled
    XintfRegs.XINTCNF2.bit.CLKOFF = 1;
    // XCLKOUT = XTIMCLK
    XintfRegs.XINTCNF2.bit.CLKMODE = 0;

    // Zone 7------------------------------------
    // Zone write timing
    XintfRegs.XTIMING7.bit.XWRLEAD = 3;   //��λ������ʱ������3
    XintfRegs.XTIMING7.bit.XWRACTIVE = 7; //��λ����Чʱ��������7
    XintfRegs.XTIMING7.bit.XWRTRAIL = 3;  //��λ������ʱ��������3

    // Zone read timing
    XintfRegs.XTIMING7.bit.XRDLEAD = 3;   //��λ������ʱ������3
    XintfRegs.XTIMING7.bit.XRDACTIVE = 7; //��λ����Чʱ��������7
    XintfRegs.XTIMING7.bit.XRDTRAIL = 3;  //��λ������ʱ��������3

    // don't double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING7.bit.X2TIMING = 0;

    // Zone will not sample XREADY signal
    XintfRegs.XTIMING7.bit.USEREADY = 0;
    XintfRegs.XTIMING7.bit.READYMODE = 1;

    // 1,1 = x16 data bus
    // 0,1 = x32 data bus
    // other values are reserved
    XintfRegs.XTIMING7.bit.XSIZE = 3;

    EDIS;

    asm(" RPT #7 || NOP");
}
