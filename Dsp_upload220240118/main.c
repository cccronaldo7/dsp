/*********************************************
-- ��Ȩ(copyright)�����ҵ��ܱ任����ƹ��̼����о�����(NECC)
-- ��Ŀ��:
-- ģ����:
-- �ļ���:
-- ����:  �ſ�
-- ���ܺ��ص����:
-- ��ʼ�汾�ͷ���ʱ��: 1.00��2022-03-14
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
#include "math.h"

Uint16 j = 0;

void main(void)
{

    /************* CPU&�����ʼ�� *************/
    InitSysCtrl();
    InitGpio();

    // Tp_Init();

    /************* �����㷨��ʼ�� *************/
    // ˲ʱ���ʳ�ʼ��
    Ins_Power_Param_Init();
    Ins_Power_Init(&Ins_Power_Load);

    // �ݲ�����ʼ��
    NF_Param_Init();
    NF_Init(&NFA_Load);
    NF_Init(&NFB_Load);
    NF_Init(&NFC_Load);
    NF_Init(&NFA_Inj);
    NF_Init(&NFB_Inj);
    NF_Init(&NFC_Inj);

    // �ظ����Ƴ�ʼ��
    RC_Param_Init();
    RC_Init(&RCA);
    RC_Init(&RCB);
    RC_Init(&RCC);

    // ���� Butterworth LPF��ʼ��
    BW_LPF_Init(&BW_LPFA);
    BW_LPF_Init(&BW_LPFB);
    BW_LPF_Init(&BW_LPFC);

    // MAF��ʼ��
    MAF_Init(&MAFD);
    MAF_Init(&MAFQ);

    // damping control��ʼ��
    Damp_Param_Init();
    Damp_Init(&DPD);
    Damp_Init(&DPQ);

    // QPR��ʼ��
    QPR_Param_Init();

    QPR_Init(&QPRA_50hz);
    QPR_Init(&QPRB_50hz);
    QPR_Init(&QPRC_50hz);

    /************* �����ʼ�� *************/
    Pwm_Init();
    Zone7_Init();
    Adc_Init();

    /************* HAPF��ʼ�� *************/
    HAPF_Init();

    /************* �жϳ�ʼ�� *************/
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCINT = &ISRAdc;
    EDIS;

    // ʹ��CPU�ж�INTx
    IER |= M_INT1;

    // ʹ���ж�������
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;  // Enable ADCINT in the PIE: Group 1 interrupt 6
    EINT;
    ERTM;

    // ��AD�ж�
    /*AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1; // ���з�������λ��CONV00
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;*/

    /************* ������ʼ�� *************/
    pwm_flag = 0;
    for (j = 0; j<=399; j++)
    {
        ad_disp1[j] = 0.0;
        ad_disp2[j] = 0.0;
    }

    while (1)
    {
        if(ctrl_clr_flag == 1)  //���������
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
