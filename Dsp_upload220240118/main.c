/*********************************************
-- 版权(copyright)：国家电能变换与控制工程技术研究中心(NECC)
-- 项目名:
-- 模块名:
-- 文件名:
-- 作者:  张凯
-- 功能和特点概述:
-- 初始版本和发布时间: 1.00，2022-03-14
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
#include "math.h"

Uint16 j = 0;

void main(void)
{

    /************* CPU&外设初始化 *************/
    InitSysCtrl();
    InitGpio();

    // Tp_Init();

    /************* 控制算法初始化 *************/
    // 瞬时功率初始化
    Ins_Power_Param_Init();
    Ins_Power_Init(&Ins_Power_Load);

    // 陷波器初始化
    NF_Param_Init();
    NF_Init(&NFA_Load);
    NF_Init(&NFB_Load);
    NF_Init(&NFC_Load);
    NF_Init(&NFA_Inj);
    NF_Init(&NFB_Inj);
    NF_Init(&NFC_Inj);

    // 重复控制初始化
    RC_Param_Init();
    RC_Init(&RCA);
    RC_Init(&RCB);
    RC_Init(&RCC);

    // 二阶 Butterworth LPF初始化
    BW_LPF_Init(&BW_LPFA);
    BW_LPF_Init(&BW_LPFB);
    BW_LPF_Init(&BW_LPFC);

    // MAF初始化
    MAF_Init(&MAFD);
    MAF_Init(&MAFQ);

    // damping control初始化
    Damp_Param_Init();
    Damp_Init(&DPD);
    Damp_Init(&DPQ);

    // QPR初始化
    QPR_Param_Init();

    QPR_Init(&QPRA_50hz);
    QPR_Init(&QPRB_50hz);
    QPR_Init(&QPRC_50hz);

    /************* 外设初始化 *************/
    Pwm_Init();
    Zone7_Init();
    Adc_Init();

    /************* HAPF初始化 *************/
    HAPF_Init();

    /************* 中断初始化 *************/
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCINT = &ISRAdc;
    EDIS;

    // 使能CPU中断INTx
    IER |= M_INT1;

    // 使能中断向量表
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;  // Enable ADCINT in the PIE: Group 1 interrupt 6
    EINT;
    ERTM;

    // 清AD中断
    /*AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1; // 序列发生器复位到CONV00
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;*/

    /************* 变量初始化 *************/
    pwm_flag = 0;
    for (j = 0; j<=399; j++)
    {
        ad_disp1[j] = 0.0;
        ad_disp2[j] = 0.0;
    }

    while (1)
    {
        if(ctrl_clr_flag == 1)  //清零控制器
        {
            RC_Param_Init();
            RC_Init(&RCA);
            RC_Init(&RCB);
            RC_Init(&RCC);

            BW_LPF_Init(&BW_LPFA);
            BW_LPF_Init(&BW_LPFB);
            BW_LPF_Init(&BW_LPFC);

            // MAF_Init(&MAFD);
            // MAF_Init(&MAFQ);

            Damp_Param_Init();
            Damp_Init(&DPD);
            Damp_Init(&DPQ);

            QPR_Param_Init();
            QPR_Init(&QPRA_50hz);
            QPR_Init(&QPRB_50hz);
            QPR_Init(&QPRC_50hz);

            ctrl_clr_flag = 0;
        }
    }
}
