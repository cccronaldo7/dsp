/*********************************************
-- 版权(copyright)：国家电能变换与控制工程技术研究中心(NECC)
-- 项目名:
-- 模块名:
-- 文件名:
-- 作者:  张凯
-- 功能和特点概述: FPGA并行总线程序 DMA XINTF初始化
-- 初始版本和发布时间: 1.00，2022-03-20
---------------------------------------------------
-- 更改历史:
---------------------------------------------------
-- 更改版本和更改时间：
-- 更改人员：
-- 更改描述:
-- 更改版本和更改时间：
-- 更改人员：无
-- 更改描述: 无
*********************************************/

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "User.h"

/************* 分配内存空间 *************/
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

    DMACH1AddrConfig(DMADest,DMASource);  // 配置地址
    DMACH1BurstConfig(31,1,1);  // 配置每帧多少字（word）1+31、帧内源地址偏移+1和帧内目的地址偏移+1
    DMACH1TransferConfig(0,1,1);  // 配置每次触发DMA转移多少帧1+0、帧间内源地址偏移+1和帧间内目的地址偏移+1
    DMACH1WrapConfig(0xFFFF,0,0xFFFF,0);  // 禁用Wrap(控制循环)

    // DMA CH1模式配置
    // 外设中断使能，单次触发(一次触发信号完成所有Burst传送)，连续触发，外设同步禁止，源同步，溢出中断禁止，16位模式，DMA结束产生中断，中断禁止
    DMACH1ModeConfig(0,PERINT_ENABLE,ONESHOT_ENABLE,CONT_ENABLE,SYNC_DISABLE,SYNC_SRC,OVRFLOW_DISABLE,SIXTEEN_BIT,CHINT_END,CHINT_DISABLE);

    // 启动CH1 DMA， 连续触发模式下(CONT_ENABLE)，DMA执行结束，重新初始化；CONT_DISENABLE，DMA执行结束，DMA停止，RUNSTS位清零
    StartDMACH1();
}

void DMA_Ch1_Trigger(void)
{
    //软件触发，将PERINTFLG置位，DMA执行后，自动将PERINTFLG清零
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
    XintfRegs.XTIMING7.bit.XWRLEAD = 3;   //两位，建立时周期数3
    XintfRegs.XTIMING7.bit.XWRACTIVE = 7; //三位，有效时间周期数7
    XintfRegs.XTIMING7.bit.XWRTRAIL = 3;  //两位，跟踪时间周期数3

    // Zone read timing
    XintfRegs.XTIMING7.bit.XRDLEAD = 3;   //两位，建立时周期数3
    XintfRegs.XTIMING7.bit.XRDACTIVE = 7; //三位，有效时间周期数7
    XintfRegs.XTIMING7.bit.XRDTRAIL = 3;  //两位，跟踪时间周期数3

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
