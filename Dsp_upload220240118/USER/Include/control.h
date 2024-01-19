/*********************************************
-- ��Ȩ(copyright)�����ҵ��ܱ任����ƹ��̼����о�����(NECC)
-- ��Ŀ��:
-- ģ����:
-- �ļ���:
-- ����:  �ſ�
-- ���ܺ��ص����: ����ͷ�ļ�
-- ��ʼ�汾�ͷ���ʱ��: 1.00��2022-03-16
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

#ifndef CONTROL_H_
#define CONTROL_H_

#include "math.h"

#define dc_inverter 50  // Udc
#define PI 3.1415926
//#define Q_up 600.0       // �����л����޹�����(����)
#define T_s 0.00005     // ��������

/******************************************************************************
 *********************************** �ṹ�嶨�� ***********************************
 ******************************************************************************/

/******************* ����任 *******************/
// park
typedef struct{
    float32 theta;
    float32 phaseA;
    float32 phaseB;
    float32 phaseC;

    float32 Ds;
    float32 Qs;
    float32 Zero;
} Park_Param;

// ipark
typedef struct{
    float32 theta;
    float32 Ds;
    float32 Qs;
    float32 Zero;

    float32 phaseA;
    float32 phaseB;
    float32 phaseC;
} iPark_Param;

/******************* ���������� *******************/
// PWM�Ƚ�ֵ
typedef struct{
    Uint16 Pwm4_CMP;
    Uint16 Pwm5_CMP;
    Uint16 Pwm6_CMP;
} Pwm_CMP;


/******************* QPR *******************/
//QPR����
typedef struct{
    float32 kp;     // ����ϵ��
    float32 kr;     // г��ϵ��
    float32 Ts;     // ��������
    float32 w0;     // г��Ƶ��
    float32 wc;     // ��ֹƵ��

    float32 mid_a;
    float32 mid_b;

    float32 u_max;   // �޷�
    float32 u_min;
} QPR_Param;

//QPR������
typedef struct{
    float32 us;     // �����ѹ
    float32 kp;     // ����ϵ��
    float32 mid_a;
    float32 mid_b;

    float32 ref;     // �ο�ֵ
    float32 fb;      // ����ֵ
    float32 err1;    // t-1ʱ�����
    float32 err2;    // t-2ʱ�����

    Uint16 pr_out;   // PR����Ƚ�ֵ
    float32 r_out1;  // t-1ʱ��г�������
    float32 r_out2;  // t-2ʱ��г�������

    float32 u_max;   // �޷�
    float32 u_min;
} QPR;

/******************* �ݲ��� *******************/
//�ݲ�������
typedef struct{
    float32 w0;    // �ݲ�Ƶ��
    float32 wc;    // ��ֹƵ��
    float32 Ts;    // ����Ƶ��

    float32 a0;
    float32 a1;
    float32 a2;

    float32 b1;
    float32 b2;
} NF_Param;

//�ݲ���
typedef struct{
    float32 r_in;       // tʱ������ֵ
    float32 r_in1;      // t-1ʱ������ֵ
    float32 r_in2;      // t-2ʱ������ֵ

    float32 y_out;      // tʱ�����ֵ
    float32 y_out1;     // t-1ʱ�����ֵ
    float32 y_out2;     // t-2ʱ�����ֵ
} NF;

/******************* MQPR *******************/
//MQPR����
typedef struct{
    float32 kp;       // ����ϵ��
    float32 kr5;      // 5��г��ϵ��
    float32 kr7;      // 7��г��ϵ��
    float32 kr11;     // 11��г��ϵ��
    float32 kr13;     // 13��г��ϵ��
    float32 Ts;       // ��������
    float32 w0;       // ��Ƶ
    float32 wc;       // ��ֹƵ��

    float32 mid5_a;
    float32 mid5_b;

    float32 mid7_a;
    float32 mid7_b;

    float32 mid11_a;
    float32 mid11_b;

    float32 mid13_a;
    float32 mid13_b;

    float32 out_max;  // ����޷�
    float32 out_min;
} MQPR_Param;

//MQPR�⻷������
typedef struct{
    float32 ref;       // �ο�ֵ
    float32 fb;        // ����ֵ
    float32 err1;      // t-1ʱ�����
    float32 err2;      // t-2ʱ�����

    float32 r5_out1;   // t-1ʱ��5��г�������
    float32 r5_out2;   // t-2ʱ��5��г�������

    float32 r7_out1;   // t-1ʱ��7��г�������
    float32 r7_out2;   // t-2ʱ��7��г�������

    float32 pr_out;     // �⻷PR���ֵ
} MQPR_Out;

//MQPR�ڻ�������
typedef struct{
    float32 ref;       // �ο�ֵ
    float32 fb;        // ����ֵ
    float32 err1;      // t-1ʱ�����
    float32 err2;      // t-2ʱ�����

    float32 r5_out1;   // t-1ʱ��5��г�������
    float32 r5_out2;   // t-2ʱ��5��г�������

    float32 r7_out1;   // t-1ʱ��7��г�������
    float32 r7_out2;   // t-2ʱ��7��г�������

    float32 r11_out1;  // t-1ʱ��11��г�������
    float32 r11_out2;  // t-2ʱ��11��г�������

    float32 r13_out1;  // t-1ʱ��13��г�������
    float32 r13_out2;  // t-2ʱ��13��г�������

    Uint16 pr_out;    // PR����Ƚ�ֵ
} MQPR_In;

/******************* �ظ����� *******************/
typedef struct{
    float32 Kp;

    float32 Qz;
    float32 Kr;
} RC_Param;

typedef struct{
    Uint16 k1;
    Uint16 k2;
    float32 ref;
    float32 fb;

    float32 x[398];
    float32 y[400];
    float32 y_out;
} RC;

/******************* ���� Butterworth LPF *******************/
typedef struct{
    float32 x_in;
    float32 x1;
    float32 x2;

    float32 y_out;
    float32 y1;
    float32 y2;
} BW_LPF;

/******************* damping control *******************/
typedef struct{
    float32 Rv;
    float32 Cv;
} Damp_Param;

typedef struct{
    float32 x_in1;
    float32 x_in2;
    float32 mid_c1;  //��һʱ��������ݼ�����

    float32 y_out;
} Damp;

/******************* MAF *******************/
typedef struct{
    Uint16 k;
    float32 x_in;
    float32 x[200];

    float32 y_out;
    float32 y1;
} MAF;

/******************* ˲ʱ���� *******************/
//˲ʱ���ʼ������
typedef struct{
    float32 wc;        // ��ֹƵ��
    float32 Ts;        // ��������

    float32 a_out;
    float32 b_out;     // b = 1-a
} Ins_Power_Param;

//˲ʱ����
typedef struct{
    float32 ua;        // ��ѹ
    float32 ub;
    float32 uc;

    float32 ia;        // ����
    float32 ib;
    float32 ic;

    float32 Qins;      // ˲ʱ�޹�����
    float32 Qdc1;      // t-1ʱ�̵�ͨ�˲�������޹�����
    float32 Qdc;
} Ins_Power;

/*****************************************************************************
 *********************************** �������� ***********************************
 *****************************************************************************/

extern Uint16 sin_cnt;                            // ��Դ�������������
extern Pwm_CMP Spwm_CMP;                          // ����������

/******************* QPR *******************/
extern QPR_Param QPR_Param_50hz;                  // QPR����
extern QPR QPRA_50hz, QPRB_50hz, QPRC_50hz;       // QPR������

/******************* �ݲ��� *******************/
extern NF_Param NF_Param_50hz;                    // 50hz�ݲ�������
extern NF NFA_Load, NFB_Load, NFC_Load;           // ���ص����ݲ���
extern NF NFA_Inj, NFB_Inj, NFC_Inj;              // ע��֧·���ص����ݲ���

/******************* MQPR *******************/
extern MQPR_Param MQPR_Param_Out, MQPR_Param_In;  // MQPR���� �⻷���ڻ�����������
extern MQPR_Out MQPRA_Out, MQPRB_Out, MQPRC_Out;  // �⻷MQPR������
extern MQPR_In MQPRA_In, MQPRB_In, MQPRC_In;      // �ڻ�MQPR������

/******************* �ظ����� *******************/
extern RC_Param RC_Param_out;
extern RC RCA, RCB, RCC;

/******************* ���� Butterworth LPF *******************/
extern BW_LPF BW_LPFA, BW_LPFB, BW_LPFC;

/******************* damping control *******************/
extern Damp_Param Damp_Param_50hz;
extern Damp DPD, DPQ;

/******************* MAF *******************/
extern MAF MAFD, MAFQ;
extern MAF MAFD_ref, MAFQ_ref;

/******************* ˲ʱ���� *******************/
extern Ins_Power_Param Ins_Power_Param_Load;      //˲ʱ���ʼ������
extern Ins_Power Ins_Power_Load;                  //˲ʱ����

/*****************************************************************************
 *********************************** �������� ***********************************
 *****************************************************************************/

/******************* HAPFϵͳ��ʼ�� *******************/
void HAPF_Init(void);

/******************* ���������� *******************/
void Pwm_CMP_Init(Pwm_CMP *v);

/******************* QPR *******************/
void QPR_Param_Init(void);
void QPR_Init(QPR *v);

/******************* �ݲ��� *******************/
void NF_Param_Init(void);
void NF_Init(NF *v);

/******************* MQPR *******************/
void MQPR_Param_Init(void);
void MQPR_Out_Init(MQPR_Out *v);
void MQPR_In_Init(MQPR_In *v);

/******************* �ظ����� *******************/
void RC_Param_Init(void);
void RC_Init(RC *v);

/******************* ���� Butterworth LPF *******************/
void BW_LPF_Init(BW_LPF *v);

/******************* damping control *******************/
void Damp_Param_Init(void);
void Damp_Init(Damp *v);

/******************* MAF *******************/
void MAF_Init(MAF *v);

/******************* ˲ʱ���� *******************/
void Ins_Power_Param_Init(void);
void Ins_Power_Init(Ins_Power *v);

/*****************************************************************************
 *********************************** �������� ***********************************
 *****************************************************************************/

/******************* ����任 *******************/
static inline void Park(Park_Param *v)
{
    float32 k = 0.0;
    float32 theta_p = 0.0;

    k = 2.0/3;
    theta_p = v->theta - PI/2;

    v->Ds =  k*(v->phaseA * cos(theta_p) + v->phaseB * cos(theta_p - 2.0943951) + v->phaseC * cos(theta_p + 2.0943951));
    v->Qs = -k*(v->phaseA * sin(theta_p) + v->phaseB * sin(theta_p - 2.0943951) + v->phaseC * sin(theta_p + 2.0943951));
    v->Zero = k*0.5*(v->phaseA + v->phaseB + v->phaseC);
}

static inline void iPark(iPark_Param *v)
{
    float32 theta_p = 0.0;
    theta_p = v->theta - PI/2;

    v->phaseA = v->Ds * cos(theta_p) - v->Qs * sin(theta_p) + v->Zero;
    v->phaseB = v->Ds * cos(theta_p - 2.0943951) - v->Qs * sin(theta_p - 2.0943951) + v->Zero;
    v->phaseC = v->Ds * cos(theta_p + 2.0943951) - v->Qs * sin(theta_p + 2.0943951) + v->Zero;
}

/******************* ���������� *******************/
static inline void Open_Loop(Pwm_CMP *CMP)
{
    float32 open_cnt = 0;
    open_cnt = (float32)(2*PI*sin_cnt/400);
    CMP->Pwm4_CMP = (Uint16)(PWM_perid*0.5*0.9*sin(open_cnt) + PWM_perid*0.5);
    CMP->Pwm5_CMP = (Uint16)(PWM_perid*0.5*0.9*sin(open_cnt - 2.0943951) + PWM_perid*0.5);
    CMP->Pwm6_CMP = (Uint16)(PWM_perid*0.5*0.9*sin(open_cnt + 2.0943951) + PWM_perid*0.5);
}

/******************* QPR *******************/
static inline void QPR_Ctrl(QPR *v)
{
    float32 err = 0.0;
    float32 r_out = 0.0;
    float32 pr_mid1 = 0.0;
    float32 pr_mid2 = 0.0;

    err = v->ref - v->fb;

    // R���������
    r_out = (v->mid_a)*(v->err1 - v->err2) + (v->mid_b)*(v->r_out1) - v->r_out2;
    // PR���
    // pr_mid1 = ((v->kp)*err + r_out + dc_inverter/2 + v -> us)/(dc_inverter);  // ��һ��
    pr_mid1 = ((v->kp)*err + dc_inverter/2 + v -> us)/(dc_inverter);

    if(pr_mid1 >= v->u_max)
    {
        pr_mid2 = v->u_max;
    }
    else if(pr_mid1 <= v->u_min)
    {
        pr_mid2 = v->u_min;
    }
    else
    {
        pr_mid2 = pr_mid1;
    }

    v->pr_out = (Uint16)(PWM_perid*pr_mid2);

    // ����t-1 t-2ʱ��ֵ
    v->r_out2 = v->r_out1;
    v->r_out1 = r_out;
    v->err2 = v->err1;
    v->err1 = err;
}

/******************* �ݲ��� *******************/
static inline void NF_Cal(NF *v)
{
    v->y_out = NF_Param_50hz.a0 * v->r_in + NF_Param_50hz.a1 * v->r_in1 + NF_Param_50hz.a2 * v->r_in2 - NF_Param_50hz.b1 * v->y_out1 - NF_Param_50hz.b2 * v->y_out2;

    // ����t-1 t-2ʱ��ֵ
    v->r_in2 = v->r_in1;
    v->r_in1 = v->r_in;
    v->y_out2 = v->y_out1;
    v->y_out1 = v->y_out;
}

/******************* MQPR *******************/
static inline void MQPR_Out_Ctrl(MQPR_Out *v)
{
    float32 err = 0.0;
    float32 r5_out = 0.0;
    float32 r7_out = 0.0;
    float32 pr_mid1 = 0.0;
    float32 pr_mid2 = 0.0;

    err = v->ref - v->fb;

    // 5��R���������
    r5_out = MQPR_Param_Out.mid5_a*(v->err1 - v->err2) + MQPR_Param_Out.mid5_b*(v->r5_out1) - v->r5_out2;

    // 7��R���������
    r7_out = MQPR_Param_Out.mid7_a*(v->err1 - v->err2) + MQPR_Param_Out.mid7_b*(v->r7_out1) - v->r7_out2;

    // MQPR���
    //pr_mid1 = (MQPR_param_out.kp)*err + r5_out + r7_out;
    pr_mid1 = (MQPR_Param_Out.kp)*err;

    if(pr_mid1 >= MQPR_Param_Out.out_max)
    {
        pr_mid2 = MQPR_Param_Out.out_max;
    }
    else if(pr_mid1 <=MQPR_Param_Out.out_min)
    {
        pr_mid2 = MQPR_Param_Out.out_min;
    }
    else
    {
        pr_mid2 = pr_mid1;
    }

    v->pr_out = pr_mid2;

    // ����t-1 t-2ʱ��ֵ
    v->r5_out2 = v->r5_out1;
    v->r5_out1 = r5_out;

    v->r7_out2 = v->r7_out1;
    v->r7_out1 = r7_out;

    v->err2 = v->err1;
    v->err1 = err;
}

static inline void MQPR_In_Ctrl(MQPR_In *v)
{
    float32 err = 0;
    float32 r5_out = 0;
    float32 r7_out = 0;
    float32 r11_out = 0;
    float32 r13_out = 0;
    float32 pr_mid1 = 0;
    float32 pr_mid2 = 0;

    err = v->ref - v->fb;

    // 5��R���������
    r5_out = MQPR_Param_In.mid5_a*(v->err1 - v->err2) + MQPR_Param_In.mid5_b*(v->r5_out1) - v->r5_out2;

    // 7��R���������
    r7_out = MQPR_Param_In.mid7_a*(v->err1 - v->err2) + MQPR_Param_In.mid7_b*(v->r7_out1) - v->r7_out2;

    // 11��R���������
    r11_out = MQPR_Param_In.mid11_a*(v->err1 - v->err2) + MQPR_Param_In.mid11_b*(v->r11_out1) - v->r11_out2;

    // 13��R���������
    r13_out = MQPR_Param_In.mid13_a*(v->err1 - v->err2) + MQPR_Param_In.mid13_b*(v->r13_out1) - v->r13_out2;

    // MQPR���
    // pr_mid1 = ((MQPR_param_in.kp)*err + r5_out + r7_out + r11_out + r13_out + dc_inverter/2)/(dc_inverter);  //��һ��
    // pr_mid1 = ((MQPR_param_in.kp)*err + r5_out + r7_out + dc_inverter/2)/(dc_inverter);
    pr_mid1 = ((MQPR_Param_In.kp)*err + dc_inverter/2)/(dc_inverter);

    if(pr_mid1 >= MQPR_Param_In.out_max)
    {
        pr_mid2 = MQPR_Param_In.out_max;
    }
    else if(pr_mid1 <=MQPR_Param_In.out_min)
    {
        pr_mid2 = MQPR_Param_In.out_min;
    }
    else
    {
        pr_mid2 = pr_mid1;
    }

    v->pr_out = (Uint16)(PWM_perid*pr_mid2);

    // ����t-1 t-2ʱ��ֵ
    v->r5_out2 = v->r5_out1;
    v->r5_out1 = r5_out;

    v->r7_out2 = v->r7_out1;
    v->r7_out1 = r7_out;

    v->r11_out2 = v->r11_out1;
    v->r11_out1 = r11_out;

    v->r13_out2 = v->r13_out1;
    v->r13_out1 = r13_out;

    v->err2 = v->err1;
    v->err1 = err;
}

/******************* �ظ����� *******************/
static inline void RC_Ctrl(RC *v)
{
    float32 err = 0.0;
    float32 mid_rc = 0.0;

    err = v->ref - v->fb;
    mid_rc = v->x[v->k2] + RC_Param_out.Qz * v->y[v->k1];
    v->x[v->k2] = err;
    v->y[v->k1] = mid_rc;

    v->y_out = RC_Param_out.Kp * err + RC_Param_out.Kr * mid_rc;

    // v->y_out = 5.1 * err;  // ���� 5.1
}

/******************* ���� Butterworth LPF *******************/
static inline void BW_LPF_Cal(BW_LPF *v)
{
    v->y_out = 0.4151*v->x_in + 0.8302*v->x1 + 0.4151*v->x2 - 0.4525*v->y1 - 0.208*v->y2;
    v->x2 = v->x1;
    v->x1 = v->x_in;

    v->y2 = v->y1;
    v->y1 = v->y_out;

    //v->y_out = v->x_in;  // ����
}

/******************* damping control *******************/
static inline void Damp_Ctrl(Damp *v)
{
    float32 mid_c = 0.0;  // ������ݽ��

    mid_c = Damp_Param_50hz.Cv * (v->x_in1 + v->mid_c1);
    // mid_c = Damp_Param_50hz.Cv * (0.000749 * v->x_in1 + 0.999251 * v->mid_c1);

    v->y_out = Damp_Param_50hz.Rv * v->x_in2 + mid_c;

    v->mid_c1 = mid_c;

    // v->y_out = Damp_Param_50hz.Rv * v->x_in;  //����
}

/******************* MAF *******************/
static inline void MAF_Cal(MAF *v)
{
    float32 mid = 0.0;
    float32 mid_out = 0.0;
    mid = v->x_in - v->x[v->k];
    v->x[v->k] = v->x_in;

    mid_out = mid + v->y1;
    v->y1 = mid_out;
    v->y_out = mid_out*0.005; // 1/200
}

/******************* ˲ʱ���� *******************/
static inline void Ins_Power_Cal(Ins_Power *v)
{
    float32 ualpha = 0.0;
    float32 ubeta = 0.0;
    float32 ialpha = 0.0;
    float32 ibeta = 0.0;

    // ���ֵclark
    ualpha = 2*v->ua/3 - v->ub/3 - v->uc/3;
    ubeta = 0.577350269*v->ub - 0.577350269*v->uc;
    ialpha = 2*v->ia/3 - v->ib/3 - v->ic/3;
    ibeta = 0.577350269*v->ib - 0.577350269*v->ic;

    // �޹����ʼ���
    v->Qins = (ubeta*ialpha - ualpha*ibeta) * 1.5;
    // v->Qins = (ualpha*ialpha + ubeta*ibeta) * 1.5; // �й����ʲ���
    // ��ͨ�˲�����ȡֱ������
    v->Qdc = Ins_Power_Param_Load.a_out * v->Qins + Ins_Power_Param_Load.b_out * v->Qdc1;

    // ����t-1ʱ��ֵ
    v->Qdc1 = v->Qdc;
}

#endif /* CONTROL_H_ */
