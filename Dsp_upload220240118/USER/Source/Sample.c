/*********************************************
-- 版权(copyright)：国家电能变换与控制工程技术研究中心(NECC)
-- 项目名:
-- 模块名:
-- 文件名:
-- 作者:  张凯
-- 功能和特点概述: AD采样初始化 读取
-- 初始版本和发布时间: 1.00，2022-03-24
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

void Adc_Init(void)
{
    InitAdc();

    // AD时钟 SYSCLKOUT/(2*HISPCP*(2*ADCLKPS)*(CPS+1)) = 12.5M
    EALLOW;
    SysCtrlRegs.HISPCP.all = ADC_MODCLK;
    EDIS;

    AdcRegs.ADCTRL3.bit.ADCCLKPS = ADC_CKPS;
    AdcRegs.ADCTRL1.bit.CPS = 0;

    // 顺序模式：采样速度 1/[(2+ACQ_PS)*ADC clock in ns]
    AdcRegs.ADCTRL1.bit.ACQ_PS = ADC_SHCLK; // 采样脉冲宽度 ACQ_PS+1个AD时钟周期，转换宽度1个时钟周期

    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;  // 序列发生器工作在级联模式
    AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 15;  // 设置最大转换次数MAX_CONV1 + 1。对SEQ1，MAX_CONV1[2:0]起作用；对SEQ2，MAX_CONV2[2:0]起作用；对SEQ，MAX_CONV1[3:0]起作用.14

    AdcRegs.ADCTRL3.bit.SMODE_SEL = 0;  // 0:顺序采样；1：连续采样
    AdcRegs.ADCTRL1.bit.CONT_RUN = 0;  // 0：启/停模式；1：连续运行模式

    // 设置采样顺序
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x7;  //第一组电流(APF电流)
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x6;
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x1;

    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x9;  //第二组电流(注入支路电流) d c 8
    AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0xf;
    AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0xe;

    AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0xd;  //第三组电流(负载电流) 9 f e
    AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0xc;
    AdcRegs.ADCCHSELSEQ3.bit.CONV08 = 0x8;

    AdcRegs.ADCCHSELSEQ3.bit.CONV09 = 0x5;  //第一组电压(逆变器PCC电压) 5 4 2
    AdcRegs.ADCCHSELSEQ3.bit.CONV10 = 0x4;
    AdcRegs.ADCCHSELSEQ3.bit.CONV11 = 0x2;

    AdcRegs.ADCCHSELSEQ4.bit.CONV12 = 0x3;  //第二组电压(注入支路PCC电压) 3 a b
    AdcRegs.ADCCHSELSEQ4.bit.CONV13 = 0xa;
    AdcRegs.ADCCHSELSEQ4.bit.CONV14 = 0xb;

    AdcRegs.ADCCHSELSEQ4.bit.CONV15 = 0x0;  //DC电压

    // 设置AD启动与中断
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 0x1; // 允许EPWM_SOCA启动SEQ1/SEQ
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 0x1;  // 允许INT_SEQ1中断 (every EOS)
    // AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0x0;  // 0：每次SEQ1结束时置位；1：间隔一次SEQ1结束时置位

}

void adc_read(void)
{
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1; // 序列发生器复位到CONV00
}
