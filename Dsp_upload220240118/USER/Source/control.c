/*********************************************
-- 版权(copyright)：国家电能变换与控制工程技术研究中心(NECC)
-- 项目名:
-- 模块名:
-- 文件名:
-- 作者:  张凯
-- 功能和特点概述: 控制算法程序
-- 初始版本和发布时间: 1.00，2022-05-23
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

/*****************************************************************************
 *********************************** 变量定义 ***********************************
 *****************************************************************************/

/******************* 开环 *******************/
Uint16 sin_cnt = 0;                        // 无源逆变器正弦索引
Pwm_CMP Spwm_CMP;                          // 开环控制器

/******************* QPR *******************/
QPR_Param QPR_Param_50hz;                  // QPR参数
QPR QPRA_50hz, QPRB_50hz, QPRC_50hz;       // QPR控制器

/******************* 陷波器 *******************/
NF_Param NF_Param_50hz;                    // 50hz陷波器参数
NF NFA_Load, NFB_Load, NFC_Load;           // 负载电流陷波器
NF NFA_Inj, NFB_Inj, NFC_Inj;              // 注入支路负载电流陷波器

/******************* MQPR *******************/
MQPR_Param MQPR_Param_Out, MQPR_Param_In;  // MQPR参数结构体 外环和内环控制器参数
MQPR_Out MQPRA_Out, MQPRB_Out, MQPRC_Out;  // 外环MQPR控制器
MQPR_In MQPRA_In, MQPRB_In, MQPRC_In;      // 内环MQPR控制器

/******************* 重复控制 *******************/
RC_Param RC_Param_out;
RC RCA, RCB, RCC;

/******************* 二阶 Butterworth LPF *******************/
BW_LPF BW_LPFA, BW_LPFB, BW_LPFC;

/******************* damping control *******************/
Damp_Param Damp_Param_50hz;
Damp DPD, DPQ;

/******************* MAF *******************/
MAF MAFD, MAFQ;
MAF MAFD_ref, MAFQ_ref;

/******************* 瞬时功率 *******************/
Ins_Power_Param Ins_Power_Param_Load;      //瞬时功率计算参数
Ins_Power Ins_Power_Load;                  //瞬时功率

/*****************************************************************************
 *********************************** 函数定义 ***********************************
 *****************************************************************************/

/******************* HAPF系统初始化 *******************/
void HAPF_Init(void)
{
    pwm_flag = 0;
    ovr_cur_flag = 0;
    protect_flag = 0;
    *svc_alpha_addr = 0x2600;  // alpha*31250/360 0x2600 0x29B5
}

/******************* 开环控制器 *******************/
void Pwm_CMP_Init(Pwm_CMP *v)
{
    v->Pwm4_CMP = 0;
    v->Pwm5_CMP = 0;
    v->Pwm6_CMP = 0;
}

/******************* QPR *******************/
void QPR_Param_Init(void)
{
    float mid = 0;
    QPR_Param_50hz.kp = 4.8;  //4.8

    QPR_Param_50hz.kr = 0;  //0.8 1.7
    QPR_Param_50hz.Ts = T_s;
    QPR_Param_50hz.w0 = 2*50*PI;
    QPR_Param_50hz.wc = 2*5*PI;

    QPR_Param_50hz.mid_a = QPR_Param_50hz.kr * QPR_Param_50hz.wc * QPR_Param_50hz.Ts;

    mid = QPR_Param_50hz.w0 * QPR_Param_50hz.Ts;
    QPR_Param_50hz.mid_b = 2*(1-mid*mid/2 + mid*mid*mid*mid/24);

    QPR_Param_50hz.u_max = 0.9;
    QPR_Param_50hz.u_min = 0.1;
}

void QPR_Init(QPR *v)
{
    // v相初始化
    v->us = 0.0;
    v->kp = QPR_Param_50hz.kp;
    v->mid_a = QPR_Param_50hz.mid_a;
    v->mid_b = QPR_Param_50hz.mid_b;
    v->ref = 0;
    v->fb = 0;
    v->err1 = 0;
    v->err2 = 0;

    v->pr_out = 0;
    v->r_out1 = 0;
    v->r_out2 = 0;

    v->u_max = QPR_Param_50hz.u_max;
    v->u_min = QPR_Param_50hz.u_min;
}

/******************* 陷波器 *******************/
void NF_Param_Init(void)
{
    float32 mid1 = 0.0;
    float32 mid2 = 0.0;
    float32 b0 = 0.0;

    NF_Param_50hz.w0 = 2*PI*50;
    NF_Param_50hz.wc = 30;  // 30
    NF_Param_50hz.Ts = T_s;

    mid1 = NF_Param_50hz.w0 * NF_Param_50hz.w0 * NF_Param_50hz.Ts * NF_Param_50hz.Ts;
    mid2 = NF_Param_50hz.wc * NF_Param_50hz.Ts;
    b0 = 4 + mid1 + 2*mid2;

    NF_Param_50hz.a0 = (4 + mid1)/b0;
    NF_Param_50hz.a1 = (2*mid1 - 8)/b0;
    NF_Param_50hz.a2 = NF_Param_50hz.a0;
    NF_Param_50hz.b1 = NF_Param_50hz.a1;
    NF_Param_50hz.b2 = (4 + mid1 - 2*mid2)/b0;
}

void NF_Init(NF *v)
{
    v->r_in = 0.0;
    v->r_in1 = 0.0;
    v->r_in2 = 0.0;

    v->y_out = 0.0;
    v->y_out1 = 0.0;
    v->y_out2 = 0.0;
}

/******************* MQPR *******************/
void MQPR_Param_Init(void)
{
    float mid_out = 0.0;
    float mid_in = 0.0;

    // 外环MQPR参数
    MQPR_Param_Out.Ts = T_s;

    MQPR_Param_Out.kp = 0.5;
    MQPR_Param_Out.kr5 = 0.05;
    MQPR_Param_Out.kr7 = 0.1;
    MQPR_Param_Out.kr11 = 0.0;
    MQPR_Param_Out.kr13 = 0.0;

    MQPR_Param_Out.w0 = 2*PI*50;
    MQPR_Param_Out.wc = 2*PI*5;

    mid_out = MQPR_Param_Out.w0 * MQPR_Param_Out.w0 * MQPR_Param_Out.Ts * MQPR_Param_Out.Ts;

    MQPR_Param_Out.mid5_a = MQPR_Param_Out.kr5 * MQPR_Param_Out.wc * MQPR_Param_Out.Ts;
    MQPR_Param_Out.mid5_b = 2*(1 - 25*mid_out/2 + 25*25*mid_out*mid_out/24);

    MQPR_Param_Out.mid7_a = MQPR_Param_Out.kr7 * MQPR_Param_Out.wc * MQPR_Param_Out.Ts;
    MQPR_Param_Out.mid7_b = 2*(1 - 49*mid_out/2 + 49*49*mid_out*mid_out/24);

    MQPR_Param_Out.mid11_a = MQPR_Param_Out.kr11 * MQPR_Param_Out.wc * MQPR_Param_Out.Ts;
    MQPR_Param_Out.mid11_b = 2*(1 - 121*mid_out/2 + 121*121*mid_out*mid_out/24);

    MQPR_Param_Out.mid13_a = MQPR_Param_Out.kr13 * MQPR_Param_Out.wc * MQPR_Param_Out.Ts;
    MQPR_Param_Out.mid13_b = 2*(1 - 169*mid_out/2 + 169*169*mid_out*mid_out/24);

    MQPR_Param_Out.out_max = 9;
    MQPR_Param_Out.out_min = -9;

    // 内环MQPR参数
    MQPR_Param_In.Ts = T_s;

    MQPR_Param_In.kp = 0.3;  //测试 20 0.3 10
    MQPR_Param_In.kr5 = 0.7;
    MQPR_Param_In.kr7 = 0.7;
    MQPR_Param_In.kr11 = 0.2;
    MQPR_Param_In.kr13 = 0.01;

    MQPR_Param_In.w0 = 2*PI*50;
    MQPR_Param_In.wc = 2*PI*5;

    mid_in = MQPR_Param_In.w0 * MQPR_Param_In.w0 * MQPR_Param_In.Ts * MQPR_Param_In.Ts;

    MQPR_Param_In.mid5_a = MQPR_Param_In.kr5 * MQPR_Param_In.wc * MQPR_Param_In.Ts;
    MQPR_Param_In.mid5_b = 2*(1 - 25*mid_in/2 + 25*25*mid_in*mid_in/24);

    MQPR_Param_In.mid7_a = MQPR_Param_In.kr7 * MQPR_Param_In.wc * MQPR_Param_In.Ts;
    MQPR_Param_In.mid7_b = 2*(1 - 49*mid_in/2 + 49*49*mid_in*mid_in/24);

    MQPR_Param_In.mid11_a = MQPR_Param_In.kr11 * MQPR_Param_In.wc * MQPR_Param_In.Ts;
    MQPR_Param_In.mid11_b = 2*(1 - 121*mid_in/2 + 121*121*mid_in*mid_in/24);

    MQPR_Param_In.mid13_a = MQPR_Param_In.kr13 * MQPR_Param_In.wc * MQPR_Param_In.Ts;
    MQPR_Param_In.mid13_b = 2*(1 - 169*mid_in/2 + 169*169*mid_in*mid_in/24);

    MQPR_Param_In.out_max = 0.9;
    MQPR_Param_In.out_min = 0.1;
}

void MQPR_Out_Init(MQPR_Out *v)
{
    v->ref = 0.0;
    v->fb = 0.0;
    v->err1 = 0.0;
    v->err2 = 0.0;

    v->r5_out1 = 0.0;
    v->r5_out2 = 0.0;

    v->r7_out1 = 0.0;
    v->r7_out2 = 0.0;

    v->pr_out = 0.0;
}

void MQPR_In_Init(MQPR_In *v)
{
    v->ref = 0.0;
    v->fb = 0.0;
    v->err1 = 0.0;
    v->err2 = 0.0;

    v->r5_out1 = 0.0;
    v->r5_out2 = 0.0;

    v->r7_out1 = 0.0;
    v->r7_out2 = 0.0;

    v->r11_out1 = 0.0;
    v->r11_out2 = 0.0;

    v->r13_out1 = 0.0;
    v->r13_out2 = 0.0;

    v->pr_out = 0;
}

/******************* 重复控制 *******************/
void RC_Param_Init(void)
{
    RC_Param_out.Kp = 1;  //1.5

    RC_Param_out.Qz = 0.98;
    RC_Param_out.Kr = 5.1;  //5.1
}

void RC_Init(RC *v)
{
    Uint16 i = 0;
    v->k1 = 0;
    v->k2 = 0;
    v->ref = 0.0;
    v->fb = 0.0;
    v->y_out = 0.0;

    for(i = 0; i<400; i++)
    {
        if(i<398)
        {
            v->x[i] = 0.0;
            v->y[i] = 0.0;
        }
        else
        {
            v->y[i] = 0.0;
        }
    }
}

/******************* 二阶 Butterworth LPF *******************/
void BW_LPF_Init(BW_LPF *v)
{
    v->x_in = 0.0;
    v->x1 = 0.0;
    v->x2 = 0.0;

    v->y_out = 0.0;
    v->y1 = 0.0;
    v->y2 = 0.0;
}

/******************* damping control *******************/
void Damp_Param_Init(void)
{
    Damp_Param_50hz.Rv = 20.5;  // 23 20.5
    Damp_Param_50hz.Cv = 0.9;  // 0.9
}

void Damp_Init(Damp *v)
{
    v->x_in1 = 0.0;
    v->x_in2 = 0.0;
    v->mid_c1 = 0.0;

    v->y_out = 0.0;
}

/******************* MAF *******************/
void MAF_Init(MAF *v)
{
    Uint16 i = 0;
    v->k = 0;
    v->x_in = 0.0;
    v->y_out = 0.0;
    v->y1 = 0.0;

    for(i = 0; i<200; i++)
    {
        v->x[i] = 0.0;
    }
}

/******************* 瞬时功率 *******************/
void Ins_Power_Param_Init(void)
{
    Ins_Power_Param_Load.wc = 2*PI*10;
    Ins_Power_Param_Load.Ts = T_s;

    Ins_Power_Param_Load.a_out = Ins_Power_Param_Load.wc * Ins_Power_Param_Load.Ts/(1 + Ins_Power_Param_Load.wc * Ins_Power_Param_Load.Ts);
    Ins_Power_Param_Load.b_out = 1 - Ins_Power_Param_Load.a_out;
}

void Ins_Power_Init(Ins_Power *v)
{
    v->ua = 0.0;
    v->ub = 0.0;
    v->uc = 0.0;

    v->ia = 0.0;
    v->ib = 0.0;
    v->ic = 0.0;

    v->Qins = 0.0;
    v->Qdc = 0.0;
    v->Qdc1 = 0.0;
}
