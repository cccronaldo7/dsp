/*********************************************
-- 版权(copyright)：国家电能变换与控制工程技术研究中心(NECC)
-- 项目名:
-- 模块名:
-- 文件名:
-- 作者:  张凯
-- 功能和特点概述: 中断程序 AD采样中断
-- 初始版本和发布时间: 1.00，2022-03-15
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

volatile Uint16 *sys_opera_addr = (Uint16 *)0x280011;
volatile Uint16 sys_opera_flag = 0xAAAA;
volatile Uint16 *svc_alpha_addr = (Uint16 *)0x280006;

volatile Uint16 pwm_flag = 0;
volatile Uint16 pwm_flag_s = 0;
volatile Uint16 ctrl_clr_flag = 0;
volatile Uint16 ovr_cur_flag = 0;
volatile Uint16 protect_flag = 0;

float ad_ch[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float ad_disp1[400];
float ad_disp2[400];
Uint16 disp_cnt = 0;
Uint16 MAF_cnt = 0;
Uint16 RC_cnt1 = 0;
Uint16 RC_cnt2 = 0;

static float table_tcr[90] = {
     0.90286, 0.90859, 0.91432, 0.91718, 0.92291, 0.92864, 0.93437, 0.93724, 0.94297, 0.94870,\
     0.95443, 0.95729, 0.96302, 0.96875, 0.97448, 0.97734, 0.98307, 0.98880, 0.99453, 1.00026,\
     1.00313, 1.00886, 1.01459, 1.02032, 1.02605, 1.02891, 1.03464, 1.04037, 1.04610, 1.05183,\
     1.05756, 1.06042, 1.06615, 1.07188, 1.07761, 1.08334, 1.08907, 1.09480, 1.10053, 1.10626,\
     1.11199, 1.11772, 1.12345, 1.12918, 1.13491, 1.14064, 1.14637, 1.15210, 1.15783, 1.16642,\
     1.17215, 1.17788, 1.18361, 1.18934, 1.19793, 1.20366, 1.20939, 1.21799, 1.22372, 1.23231,\
     1.23804, 1.24663, 1.25236, 1.26096, 1.26955, 1.27528, 1.28388, 1.29247, 1.30107, 1.30966,\
     1.31825, 1.32685, 1.33544, 1.34690, 1.35550, 1.36696, 1.37841, 1.38987, 1.40133, 1.41279,\
     1.42712, 1.43858, 1.45576, 1.47009, 1.49014, 1.50733, 1.53025, 1.55890, 1.59041, 1.64198};

interrupt void ISRAdc(void)
{

    // tp1 = 1;
    /**************************************************************************
     *********************************** 采样 ***********************************
     **************************************************************************/
    // 第一组电流(APF电流)
    ad_ch[0] = (float) (AdcRegs.ADCRESULT0 >> 4)*3.0/4096;  //0.000244140625 = 1/4096
    ad_ch[1] = (float) (AdcRegs.ADCRESULT1 >> 4)*3.0/4096;
    ad_ch[2] = (float) (AdcRegs.ADCRESULT2 >> 4)*3.0/4096;

    // 第二组电流(注入支路电流)
    ad_ch[3] = (float) (AdcRegs.ADCRESULT3 >> 4)*3.0/4096;
    ad_ch[4] = (float) (AdcRegs.ADCRESULT4 >> 4)*3.0/4096;
    ad_ch[5] = (float) (AdcRegs.ADCRESULT5 >> 4)*3.0/4096;

    // 第三组电流(负载电流)
    ad_ch[6] = (float) (AdcRegs.ADCRESULT6 >> 4)*3.0/4096;
    ad_ch[7] = (float) (AdcRegs.ADCRESULT7 >> 4)*3.0/4096;
    ad_ch[8] = (float) (AdcRegs.ADCRESULT8 >> 4)*3.0/4096;

    // 第一组电压(逆变器PCC电压)
    ad_ch[9] = (float) (AdcRegs.ADCRESULT9 >> 4)*3.0/4096;
    ad_ch[10] = (float) (AdcRegs.ADCRESULT10 >> 4)*3.0/4096;
    ad_ch[11] = (float) (AdcRegs.ADCRESULT11 >> 4)*3.0/4096;

    // 第二组电压(注入支路PCC电压)
    ad_ch[12] = (float) (AdcRegs.ADCRESULT12 >> 4)*3.0/4096;
    ad_ch[13] = (float) (AdcRegs.ADCRESULT13 >> 4)*3.0/4096;
    ad_ch[14] = (float) (AdcRegs.ADCRESULT14 >> 4)*3.0/4096;

    // DC电压
    ad_ch[15] = (float) (AdcRegs.ADCRESULT15 >> 4)*3.0/4096;

    /**************************************************************************
     *********************************** 开环 ***********************************
     **************************************************************************/
    // 无源逆变正弦索引
    if(sin_cnt < 399)
    {
        sin_cnt = sin_cnt + 1;
    }
    else
    {
        sin_cnt = 0;
    }

    /**************************************************************************
     *********************************** 闭环 ***********************************
     **************************************************************************/
    // 电网电压相位读取
    volatile Uint16 *theta_addr = (Uint16 *)0x280010;
    float theta = 0;
    // theta = (float32)(2*PI*sin_cnt/400);
    theta = (float)(*theta_addr) /31250*2*PI -PI/6 + PI * 0.002;  //线电压 - 30° + 0.36°延时补偿

    // 线电压计算
    float Usab = 0.0;
    float Usbc = 0.0;
    float Usca = 0.0;
    Usab = (ad_ch[12] - 1.504) *  251;
    Usbc = (ad_ch[13] - 1.508) *  251;
    Usca = (ad_ch[14] - 1.504) *  251;

    // 相电压计算
    float Usa = 0.0;
    float Usb = 0.0;
    float Usc = 0.0;
    Usa = (Usab - Usca)/3;
    Usb = (Usbc - Usab)/3;
    Usc = (Usca - Usbc)/3;

    // 负载电流
    float iLa = 0.0;
    float iLb = 0.0;
    float iLc = 0.0;

    iLa = (ad_ch[6] - 1.5075) * 22.727;  // *36/24/0.066 注意电流方向
    iLb = (ad_ch[7] - 1.5085) * 22.727;
    iLc = (ad_ch[8] - 1.51) * 22.727;

    // 注入支路电流
    float iia = 0.0;
    float iib = 0.0;
    float iic = 0.0;

    iia = (1.5125 - ad_ch[3]) * 22.727;  // *36/24/0.066 注意电流方向
    iib = (1.5095 - ad_ch[4]) * 22.727;
    iic = (1.5175 - ad_ch[5]) * 22.727;

    // 逆变器电流
    float ica = 0.0;
    float icb = 0.0;
    float icc = 0.0;

    ica = (ad_ch[0] - 1.5125) * 26.667;  // *24/36/0.025 注意电流方向
    icb = (ad_ch[1] - 1.506) * 26.667;
    icc = (ad_ch[2] - 1.506) * 26.667;

    /****************** 无功功率计算 ******************/
    Ins_Power_Load.ua = Usa;
    Ins_Power_Load.ub = Usb;
    Ins_Power_Load.uc = Usc;
    Ins_Power_Load.ia = iLa;
    Ins_Power_Load.ib = iLb;
    Ins_Power_Load.ic = iLc;
    Ins_Power_Cal(&Ins_Power_Load);

	float alpha = 0.0;
	int alpha_ind = 0;
    if(Ins_Power_Load.Qdc < -2300)
    {
        alpha = 90.0;
    }
	else if(Ins_Power_Load.Qdc > 3351.5)
	{
		alpha = 180.0;
	}
	else
	{
		alpha_ind = round(Ins_Power_Load.Qdc + 2300);
		alpha = 100.0 * table_tcr[alpha_ind];
	}

	*svc_alpha_addr = (Uint16)(alpha*31250/360);  // alpha*31250/360
    /****************** 谐波电流计算 ******************/
    //负载谐波电流计算
    NFA_Load.r_in = iLa;
    NFB_Load.r_in = iLb;
    NFC_Load.r_in = iLc;
    NF_Cal(&NFA_Load);
    NF_Cal(&NFB_Load);
    NF_Cal(&NFC_Load);

    //注入支路谐波电流计算
    NFA_Inj.r_in = iia;
    NFB_Inj.r_in = iib;
    NFC_Inj.r_in = iic;
    NF_Cal(&NFA_Inj);
    NF_Cal(&NFB_Inj);
    NF_Cal(&NFC_Inj);

    /****************** RC+DAMP ******************/
    Park_Param Park_ic = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    Park_ic.theta = theta;

    iPark_Param iPark_ic = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    iPark_ic.theta = theta;

    // ABC-DQ
    Park_ic.phaseA = ica;
    Park_ic.phaseB = icb;
    Park_ic.phaseC = icc;
    Park(&Park_ic);

    // MAF
    MAFD.k = MAF_cnt;
    MAFQ.k = MAF_cnt;
    MAFD.x_in = Park_ic.Ds;
    MAFQ.x_in = Park_ic.Qs;

    MAF_Cal(&MAFD);
    MAF_Cal(&MAFQ);

    if(MAF_cnt < 199)  //199 49
    {
        MAF_cnt = MAF_cnt + 1;
    }
    else
    {
        MAF_cnt = 0;
    }

    // 外环电流参考值反馈值
    RCA.ref = NFA_Load.y_out;
    RCB.ref = NFB_Load.y_out;
    RCC.ref = NFC_Load.y_out;

    RCA.fb = NFA_Inj.y_out;
    RCB.fb = NFB_Inj.y_out;
    RCC.fb = NFC_Inj.y_out;

    // 阻尼电流参考值
    DPD.x_in1 = Park_ic.Ds;
    DPQ.x_in1 = Park_ic.Qs;
    DPD.x_in2 = MAFD.y_out;
    DPQ.x_in2 = MAFQ.y_out;
    /*DPD.x_in = Park_ic.Ds;
    DPQ.x_in = Park_ic.Qs;*/

    // 内环电流反馈值
    QPRA_50hz.fb = ica;
    QPRB_50hz.fb = icb;
    QPRC_50hz.fb = icc;

    QPRA_50hz.us = 0.0;
    QPRB_50hz.us = 0.0;
    QPRC_50hz.us = 0.0;

    /*******************************************************************************
     *********************************** 运行状态控制 ***********************************
     *******************************************************************************/

    /********************* 上位机系统运行控制 *********************/
    // 读取FPGA系统运行标志位
    sys_opera_flag = *sys_opera_addr;
    if(sys_opera_flag == 0x5555)
    {
        pwm_flag = 1;
    }
    else if(sys_opera_flag == 0x00AA)
    {
        pwm_flag = 2;
    }
    else if(sys_opera_flag == 0x3333)
    {
        pwm_flag = 3;
    }
    else if(sys_opera_flag == 0xAAAA)
    {
        pwm_flag = 0;
    }

    /********************* 软件保护 *********************/
    // 过流保护标志位
    if((ica > 19) || (ica < -19))
    {
        ovr_cur_flag = 1;
    }
    else if((icb > 19) || (icb < -19))
    {
        ovr_cur_flag = 1;
    }
    else if((icc > 19) || (icc < -19))
    {
        ovr_cur_flag = 1;
    }
    else
    {
        if(pwm_flag == 0)  // 系统停机后清保护位
        {
            ovr_cur_flag = 0;
        }
    }

    // 软件保护标志位
    protect_flag = ovr_cur_flag;

    /********************* PWM生成 *********************/
    // PWM计算
    if((pwm_flag == 0) || (pwm_flag == 2) || (protect_flag == 1))  //停机
    {
        RC_cnt1 = 0;
        RC_cnt2 = 0;
        sin_cnt = 0;

        EPwm4Regs.CMPA.half.CMPA = 0;
        EPwm4Regs.CMPB = PWM_perid;
        EPwm5Regs.CMPA.half.CMPA = 0;
        EPwm5Regs.CMPB = PWM_perid;
        EPwm6Regs.CMPA.half.CMPA = 0;
        EPwm6Regs.CMPB = PWM_perid;
    }
    else  //闭环
    {
        // 外环RC控制
        RCA.k1 = RC_cnt1;
        RCB.k1 = RC_cnt1;
        RCC.k1 = RC_cnt1;

        RCA.k2 = RC_cnt2;
        RCB.k2 = RC_cnt2;
        RCC.k2 = RC_cnt2;

        RC_Ctrl(&RCA);
        RC_Ctrl(&RCB);
        RC_Ctrl(&RCC);

        if(RC_cnt1 < 399)
        {
            RC_cnt1 = RC_cnt1 + 1;
        }
        else
        {
            RC_cnt1 = 0;
        }

        if(RC_cnt2 < 397)
        {
            RC_cnt2 = RC_cnt2 + 1;
        }
        else
        {
            RC_cnt2 = 0;
        }

        // 外环二阶Butterworth
        BW_LPFA.x_in = RCA.y_out;
        BW_LPFB.x_in = RCB.y_out;
        BW_LPFC.x_in = RCC.y_out;

        BW_LPF_Cal(&BW_LPFA);
        BW_LPF_Cal(&BW_LPFB);
        BW_LPF_Cal(&BW_LPFC);

        // 阻尼控制
        Damp_Ctrl(&DPD);
        Damp_Ctrl(&DPQ);

        iPark_ic.Ds = DPD.y_out;
        iPark_ic.Qs = DPQ.y_out;
        iPark_ic.Zero = 0.0;
        iPark(&iPark_ic);

        QPRA_50hz.ref = BW_LPFA.y_out - iPark_ic.phaseA;
        QPRB_50hz.ref = BW_LPFB.y_out - iPark_ic.phaseB;
        QPRC_50hz.ref = BW_LPFC.y_out - iPark_ic.phaseC;

        /*QPRA_50hz.ref = BW_LPFA.y_out - Damp_Param_50hz.Rv * ica;
        QPRB_50hz.ref = BW_LPFB.y_out - Damp_Param_50hz.Rv * icb;
        QPRC_50hz.ref = BW_LPFC.y_out - Damp_Param_50hz.Rv * icc;*/

        /*QPRA_50hz.ref = BW_LPFA.y_out;
        QPRB_50hz.ref = BW_LPFB.y_out;
        QPRC_50hz.ref = BW_LPFC.y_out;*/

        QPR_Ctrl(&QPRA_50hz);
        QPR_Ctrl(&QPRB_50hz);
        QPR_Ctrl(&QPRC_50hz);

        // PWM参考值
        EPwm4Regs.CMPA.half.CMPA = QPRA_50hz.pr_out;
        EPwm4Regs.CMPB = QPRA_50hz.pr_out;
        EPwm5Regs.CMPA.half.CMPA = QPRB_50hz.pr_out;
        EPwm5Regs.CMPB = QPRB_50hz.pr_out;
        EPwm6Regs.CMPA.half.CMPA = QPRC_50hz.pr_out;
        EPwm6Regs.CMPB = QPRC_50hz.pr_out;

    }

    // 控制器清零
    if((pwm_flag == 2) && (pwm_flag_s != 2))
    {
        ctrl_clr_flag = 1;
    }

    pwm_flag_s = pwm_flag;

    /****************************************************************************
     *********************************** 状态监测 ***********************************
     ****************************************************************************/

    ad_disp2[disp_cnt] = Ins_Power_Load.Qdc;

    // CCS显示
    iPark_ic.Ds = MAFD.y_out;
    iPark_ic.Qs = MAFQ.y_out;
    iPark(&iPark_ic);

    ad_disp1[disp_cnt] = iPark_ic.phaseA;

    if(disp_cnt<399)
    {
        disp_cnt++;
    }
    else
    {
        disp_cnt = 0;
    }

    // tp1 = 0;

    // Reinitialize for next ADC sequence
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1; // 序列发生器复位到CONV00
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;

}
