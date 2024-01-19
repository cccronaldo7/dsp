/*********************************************
-- 版权(copyright)：国家电能变换与控制工程技术研究中心(NECC)
-- 项目名:
-- 模块名:
-- 文件名:
-- 作者:  张凯
-- 功能和特点概述: FPGA并行总线程序头文件
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

#ifndef FPGA_H
#define FPGA_H

extern volatile Uint16 DMABuf1[32];
extern volatile Uint16 DMABuf2[32];
extern volatile Uint16 *DMADest;
extern volatile Uint16 *DMASource;

void DMA_Chn_Init(void);
void DMA_Ch1_Trigger(void);
void Zone7_Init(void);

#endif /* FPGA_H */
