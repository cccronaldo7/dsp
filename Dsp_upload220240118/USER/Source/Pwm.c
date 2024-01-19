/*********************************************
-- ��Ȩ(copyright)�����ҵ��ܱ任����ƹ��̼����о�����(NECC)
-- ��Ŀ��:
-- ģ����:
-- �ļ���:
-- ����:  �ſ�
-- ���ܺ��ص����: PWM���� EPWM4 5 6����
-- ��ʼ�汾�ͷ���ʱ��: 1.00��2022-03-21
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

void Pwm_Init(void)
{
    InitEPwm4Gpio();
    InitEPwm5Gpio();
    InitEPwm6Gpio();

    // Disable TBCLK within the ePWM
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    // ����ģʽλ��Ĭ��Ϊ0
    //0��TBCTR����ֹͣ��1��TBCTR��������������ں�ֹͣ��2��3��free run
    EPwm4Regs.TBCTL.bit.FREE_SOFT = 0x3;
    EPwm5Regs.TBCTL.bit.FREE_SOFT = 0x3;
    EPwm6Regs.TBCTL.bit.FREE_SOFT = 0x3;

    /*************** EPWM4 setup ***************/
    // Setup TBCLK
    EPwm4Regs.TBPRD = PWM_perid;  // setup pwm period TBCLK/(3750*2)
    EPwm4Regs.TBPHS.half.TBPHS = 0x0000;  // ��λ�Ĵ�������
    EPwm4Regs.TBCTR = 0x0000;  // ���������

    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // ��������
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;  // TBCLK = SYSCLKOUT/(HSPCLKDIV*CLKDIV)
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;  // ��ֹ��λװ��
    EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;  // 0:��CTR=0ʱ����ӳ��Ĵ���������װ�ص����ڼĴ����У�1������ӳ��Ĵ���
    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;  // 0������ͬ���źţ�1����CTR����0ʱ�̷���ͬ���źţ�2����CTR����CMPBʱ�̷���ͬ���źţ�3����ֹEPWMxSYNCO

    // ��CTR����0ʱ��װ�رȽϼĴ�����ֵ
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;  // 0��ӳ��ģʽ��1������ģʽ
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // 0:CTR=0ʱ��װ�أ�1��CTR=PRD(���ڼĴ���)ʱ��װ�أ�:2������CTR=0ʱ������CTR=PRD(���ڼĴ���)ʱ��װ�أ�3����ֹװ��
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //Setup Action
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;  // ����������
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;  // ��������λ
    EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm4Regs.AQCTLB.bit.CBD = AQ_SET;

    //Setup  Deadband
    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // 0����ֹȫ����ʱ��1��EPWMxAֱ��ͨ����2��EPWMxBֱ��ͨ����3��ʹ��ȫ����ʱ
    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;  // 0��EPWMxA��EPWMxB������ת���ԣ�1��EPWMxA��ת���ԣ�2��EPWMxB��ת���ԣ�3��EPWMxA��EPWMxB����ת����
    EPwm4Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;  // 0��EPWMxA�������½�����ʱ�ź�Դ��1��EPWMxB��������ʱ��EPWMxA�½�����ʱ��2��EPWMxA��������ʱ��EPWMxB�½�����ʱ��3��EPWMxB�������½�����ʱ
    EPwm4Regs.DBRED = PWM_DB;  // 150 ����ʱ��1us
    EPwm4Regs.DBFED = PWM_DB;

    //Setup Event and INT
    //EPwm4Regs.ETSEL.bit.INTEN = 0;
    EPwm4Regs.ETSEL.bit.SOCAEN = 1;
    EPwm4Regs.ETSEL.bit.SOCASEL = 1;  // EPWMxSOCA�������� 1
    EPwm4Regs.ETPS.bit.SOCAPRD = 1;

    //Setup Compare
    EPwm4Regs.CMPA.half.CMPA = 0;  //ռ�ձ�Ϊ0%
    EPwm4Regs.CMPB = PWM_perid;  //ռ�ձ�Ϊ0%


    /************** EPWM5 setup **************/
    // Setup TBCLK
    EPwm5Regs.TBPRD = PWM_perid;  // setup pwm period TBCLK/(3750*2)
    EPwm5Regs.TBPHS.half.TBPHS = 0x0000;  // ��λ�Ĵ�������
    EPwm5Regs.TBCTR = 0x0000;  // ���������

    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // ��������
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;  // TBCLK = SYSCLKOUT/(HSPCLKDIV*CLKDIV)
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;  // ʹ����λװ��
    EPwm5Regs.TBCTL.bit.PRDLD = TB_SHADOW;  // 0:��CTR=0ʱ����ӳ��Ĵ���������װ�ص����ڼĴ����У�1������ӳ��Ĵ���
    EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;  // 0������ͬ���źţ�1����CTR����0ʱ�̷���ͬ���źţ�2����CTR����CMPBʱ�̷���ͬ���źţ�3����ֹEPWMxSYNCO

    // ��CTR����0ʱ��װ�رȽϼĴ�����ֵ
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;  // 0��ӳ��ģʽ��1������ģʽ
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // 0:CTR=0ʱ��װ�أ�1��CTR=PRD(���ڼĴ���)ʱ��װ�أ�:2������CTR=0ʱ������CTR=PRD(���ڼĴ���)ʱ��װ�أ�3����ֹװ��
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //Setup Action
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;  // ����������
    EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;  //��������λ
    EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm5Regs.AQCTLB.bit.CBD = AQ_SET;

    //Setup  Deadband
    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // 0����ֹȫ����ʱ��1��EPWMxAֱ��ͨ����2��EPWMxBֱ��ͨ����3��ʹ��ȫ����ʱ
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;  // 0��EPWMxA��EPWMxB������ת���ԣ�1��EPWMxA��ת���ԣ�2��EPWMxB��ת���ԣ�3��EPWMxA��EPWMxB����ת����
    EPwm5Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;  // 0��EPWMxA�������½�����ʱ�ź�Դ��1��EPWMxB��������ʱ��EPWMxA�½�����ʱ��2��EPWMxA��������ʱ��EPWMxB�½�����ʱ��3��EPWMxB�������½�����ʱ
    EPwm5Regs.DBRED = PWM_DB;  // ����ʱ��1us
    EPwm5Regs.DBFED = PWM_DB;

    //Setup Compare
    EPwm5Regs.CMPA.half.CMPA = 0;  //ռ�ձ�Ϊ0%
    EPwm5Regs.CMPB = PWM_perid;  //ռ�ձ�Ϊ0%

    /************** EPWM6 setup **************/
    // Setup TBCLK
    EPwm6Regs.TBPRD = PWM_perid;  // setup pwm period TBCLK/(3750*2)
    EPwm6Regs.TBPHS.half.TBPHS = 0x0000;  // ��λ�Ĵ�������
    EPwm6Regs.TBCTR = 0x0000;  // ���������

    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // ��������
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;  // TBCLK = SYSCLKOUT/(HSPCLKDIV*CLKDIV)
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;  // ʹ����λװ��
    EPwm6Regs.TBCTL.bit.PRDLD = TB_SHADOW;  // 0:��CTR=0ʱ����ӳ��Ĵ���������װ�ص����ڼĴ����У�1������ӳ��Ĵ���
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;  // 0������ͬ���źţ�1����CTR����0ʱ�̷���ͬ���źţ�2����CTR����CMPBʱ�̷���ͬ���źţ�3����ֹEPWMxSYNCO

    // ��CTR����0ʱ��װ�رȽϼĴ�����ֵ
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;  // 0��ӳ��ģʽ��1������ģʽ
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // 0:CTR=0ʱ��װ�أ�1��CTR=PRD(���ڼĴ���)ʱ��װ�أ�:2������CTR=0ʱ������CTR=PRD(���ڼĴ���)ʱ��װ�أ�3����ֹװ��
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //Setup Action
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;  // ����������
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;  // ��������λ
    EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm6Regs.AQCTLB.bit.CBD = AQ_SET;

    //Setup  Deadband
    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // 0����ֹȫ����ʱ��1��EPWMxAֱ��ͨ����2��EPWMxBֱ��ͨ����3��ʹ��ȫ����ʱ
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;  // 0��EPWMxA��EPWMxB������ת���ԣ�1��EPWMxA��ת���ԣ�2��EPWMxB��ת���ԣ�3��EPWMxA��EPWMxB����ת����
    EPwm6Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;  // 0��EPWMxA�������½�����ʱ�ź�Դ��1��EPWMxB��������ʱ��EPWMxA�½�����ʱ��2��EPWMxA��������ʱ��EPWMxB�½�����ʱ��3��EPWMxB�������½�����ʱ
    EPwm6Regs.DBRED = PWM_DB;  // ����ʱ��1us
    EPwm6Regs.DBFED = PWM_DB;

    //Setup Compare
    EPwm6Regs.CMPA.half.CMPA = 0;  //ռ�ձ�Ϊ0%
    EPwm6Regs.CMPB = PWM_perid;  //ռ�ձ�Ϊ0%

    // Start all the timers synced
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

}
