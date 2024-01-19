/*********************************************
-- 版权(copyright)：国家电能变换与控制工程技术研究中心(NECC)
-- 项目名:
-- 模块名:
-- 文件名:
-- 作者:  张凯
-- 功能和特点概述: PWM程序 EPWM4 5 6配置
-- 初始版本和发布时间: 1.00，2022-03-21
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

void Pwm_Init(void)
{
    InitEPwm4Gpio();
    InitEPwm5Gpio();
    InitEPwm6Gpio();

    // Disable TBCLK within the ePWM
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    // 仿真模式位，默认为0
    //0：TBCTR立刻停止，1：TBCTR完成整个计数周期后停止，2或3：free run
    EPwm4Regs.TBCTL.bit.FREE_SOFT = 0x3;
    EPwm5Regs.TBCTL.bit.FREE_SOFT = 0x3;
    EPwm6Regs.TBCTL.bit.FREE_SOFT = 0x3;

    /*************** EPWM4 setup ***************/
    // Setup TBCLK
    EPwm4Regs.TBPRD = PWM_perid;  // setup pwm period TBCLK/(3750*2)
    EPwm4Regs.TBPHS.half.TBPHS = 0x0000;  // 相位寄存器清零
    EPwm4Regs.TBCTR = 0x0000;  // 清零计数器

    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // 增减计数
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;  // TBCLK = SYSCLKOUT/(HSPCLKDIV*CLKDIV)
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;  // 禁止相位装载
    EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;  // 0:在CTR=0时，将映射寄存器的数据装载到周期寄存器中；1：禁用映射寄存器
    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;  // 0：接收同步信号；1：在CTR等于0时刻发出同步信号；2：在CTR等于CMPB时刻发出同步信号；3：禁止EPWMxSYNCO

    // 在CTR等于0时刻装载比较寄存器的值
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;  // 0：映射模式；1：立即模式
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // 0:CTR=0时刻装载；1：CTR=PRD(周期寄存器)时刻装载；:2：既在CTR=0时刻又在CTR=PRD(周期寄存器)时刻装载；3：禁止装载
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //Setup Action
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;  // 增计数清零
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;  // 减计数置位
    EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm4Regs.AQCTLB.bit.CBD = AQ_SET;

    //Setup  Deadband
    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // 0：禁止全部延时；1：EPWMxA直接通过；2：EPWMxB直接通过；3：使能全部延时
    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;  // 0：EPWMxA，EPWMxB都不反转极性；1：EPWMxA反转极性；2：EPWMxB反转极性；3：EPWMxA，EPWMxB都反转极性
    EPwm4Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;  // 0：EPWMxA上升沿下降沿延时信号源；1：EPWMxB上升沿延时，EPWMxA下降沿延时；2：EPWMxA上升沿延时，EPWMxB下降沿延时；3：EPWMxB上升沿下降沿延时
    EPwm4Regs.DBRED = PWM_DB;  // 150 死区时间1us
    EPwm4Regs.DBFED = PWM_DB;

    //Setup Event and INT
    //EPwm4Regs.ETSEL.bit.INTEN = 0;
    EPwm4Regs.ETSEL.bit.SOCAEN = 1;
    EPwm4Regs.ETSEL.bit.SOCASEL = 1;  // EPWMxSOCA产生条件 1
    EPwm4Regs.ETPS.bit.SOCAPRD = 1;

    //Setup Compare
    EPwm4Regs.CMPA.half.CMPA = 0;  //占空比为0%
    EPwm4Regs.CMPB = PWM_perid;  //占空比为0%


    /************** EPWM5 setup **************/
    // Setup TBCLK
    EPwm5Regs.TBPRD = PWM_perid;  // setup pwm period TBCLK/(3750*2)
    EPwm5Regs.TBPHS.half.TBPHS = 0x0000;  // 相位寄存器清零
    EPwm5Regs.TBCTR = 0x0000;  // 清零计数器

    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // 增减计数
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;  // TBCLK = SYSCLKOUT/(HSPCLKDIV*CLKDIV)
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;  // 使能相位装载
    EPwm5Regs.TBCTL.bit.PRDLD = TB_SHADOW;  // 0:在CTR=0时，将映射寄存器的数据装载到周期寄存器中；1：禁用映射寄存器
    EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;  // 0：接收同步信号；1：在CTR等于0时刻发出同步信号；2：在CTR等于CMPB时刻发出同步信号；3：禁止EPWMxSYNCO

    // 在CTR等于0时刻装载比较寄存器的值
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;  // 0：映射模式；1：立即模式
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // 0:CTR=0时刻装载；1：CTR=PRD(周期寄存器)时刻装载；:2：既在CTR=0时刻又在CTR=PRD(周期寄存器)时刻装载；3：禁止装载
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //Setup Action
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;  // 增计数清零
    EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;  //减计数置位
    EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm5Regs.AQCTLB.bit.CBD = AQ_SET;

    //Setup  Deadband
    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // 0：禁止全部延时；1：EPWMxA直接通过；2：EPWMxB直接通过；3：使能全部延时
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;  // 0：EPWMxA，EPWMxB都不反转极性；1：EPWMxA反转极性；2：EPWMxB反转极性；3：EPWMxA，EPWMxB都反转极性
    EPwm5Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;  // 0：EPWMxA上升沿下降沿延时信号源；1：EPWMxB上升沿延时，EPWMxA下降沿延时；2：EPWMxA上升沿延时，EPWMxB下降沿延时；3：EPWMxB上升沿下降沿延时
    EPwm5Regs.DBRED = PWM_DB;  // 死区时间1us
    EPwm5Regs.DBFED = PWM_DB;

    //Setup Compare
    EPwm5Regs.CMPA.half.CMPA = 0;  //占空比为0%
    EPwm5Regs.CMPB = PWM_perid;  //占空比为0%

    /************** EPWM6 setup **************/
    // Setup TBCLK
    EPwm6Regs.TBPRD = PWM_perid;  // setup pwm period TBCLK/(3750*2)
    EPwm6Regs.TBPHS.half.TBPHS = 0x0000;  // 相位寄存器清零
    EPwm6Regs.TBCTR = 0x0000;  // 清零计数器

    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // 增减计数
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;  // TBCLK = SYSCLKOUT/(HSPCLKDIV*CLKDIV)
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;  // 使能相位装载
    EPwm6Regs.TBCTL.bit.PRDLD = TB_SHADOW;  // 0:在CTR=0时，将映射寄存器的数据装载到周期寄存器中；1：禁用映射寄存器
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;  // 0：接收同步信号；1：在CTR等于0时刻发出同步信号；2：在CTR等于CMPB时刻发出同步信号；3：禁止EPWMxSYNCO

    // 在CTR等于0时刻装载比较寄存器的值
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;  // 0：映射模式；1：立即模式
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // 0:CTR=0时刻装载；1：CTR=PRD(周期寄存器)时刻装载；:2：既在CTR=0时刻又在CTR=PRD(周期寄存器)时刻装载；3：禁止装载
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //Setup Action
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;  // 增计数清零
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;  // 减计数置位
    EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm6Regs.AQCTLB.bit.CBD = AQ_SET;

    //Setup  Deadband
    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // 0：禁止全部延时；1：EPWMxA直接通过；2：EPWMxB直接通过；3：使能全部延时
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;  // 0：EPWMxA，EPWMxB都不反转极性；1：EPWMxA反转极性；2：EPWMxB反转极性；3：EPWMxA，EPWMxB都反转极性
    EPwm6Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;  // 0：EPWMxA上升沿下降沿延时信号源；1：EPWMxB上升沿延时，EPWMxA下降沿延时；2：EPWMxA上升沿延时，EPWMxB下降沿延时；3：EPWMxB上升沿下降沿延时
    EPwm6Regs.DBRED = PWM_DB;  // 死区时间1us
    EPwm6Regs.DBFED = PWM_DB;

    //Setup Compare
    EPwm6Regs.CMPA.half.CMPA = 0;  //占空比为0%
    EPwm6Regs.CMPB = PWM_perid;  //占空比为0%

    // Start all the timers synced
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

}
