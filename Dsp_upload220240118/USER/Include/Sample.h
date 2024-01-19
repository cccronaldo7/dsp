/*********************************************
-- 版权(copyright)：国家电能变换与控制工程技术研究中心(NECC)
-- 项目名:
-- 模块名:
-- 文件名:
-- 作者:  张凯
-- 功能和特点概述: AD采样程序头文件
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

#ifndef SAMPLE_H
#define SAMPLE_H

#if (CPU_FRQ_150MHZ)     // 默认系统时钟 SYSCLKOUT = 150 MHz
  #define ADC_MODCLK 0x3 // 高速外设时钟 HSPCLK = SYSCLKOUT/2*ADC_MODCLK=150/(2*3)=25.0 MHz
#endif
#if (CPU_FRQ_100MHZ)
  #define ADC_MODCLK 0x2 // 高速外设时钟 HSPCLK = SYSCLKOUT/2*ADC_MODCLK2=100/(2*2)=25.0 MHz
#endif
#define ADC_CKPS   0x1   // AD时钟    ADCmodule clock = HSPCLK/(2*ADC_CKPS)=25MHz/(2)=12.5 MHz

#define ADC_SHCLK  0x0   // 采样保持器宽度S/H width in ADC module periods = (ADC_SHCLK+1) ADC cycle

void Adc_Init(void);

#endif /* SAMPLE_H */
