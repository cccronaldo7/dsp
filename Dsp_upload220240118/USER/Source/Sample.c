/*********************************************
-- ��Ȩ(copyright)�����ҵ��ܱ任����ƹ��̼����о�����(NECC)
-- ��Ŀ��:
-- ģ����:
-- �ļ���:
-- ����:  �ſ�
-- ���ܺ��ص����: AD������ʼ�� ��ȡ
-- ��ʼ�汾�ͷ���ʱ��: 1.00��2022-03-24
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

void Adc_Init(void)
{
    InitAdc();

    // ADʱ�� SYSCLKOUT/(2*HISPCP*(2*ADCLKPS)*(CPS+1)) = 12.5M
    EALLOW;
    SysCtrlRegs.HISPCP.all = ADC_MODCLK;
    EDIS;

    AdcRegs.ADCTRL3.bit.ADCCLKPS = ADC_CKPS;
    AdcRegs.ADCTRL1.bit.CPS = 0;

    // ˳��ģʽ�������ٶ� 1/[(2+ACQ_PS)*ADC clock in ns]
    AdcRegs.ADCTRL1.bit.ACQ_PS = ADC_SHCLK; // ���������� ACQ_PS+1��ADʱ�����ڣ�ת�����1��ʱ������

    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;  // ���з����������ڼ���ģʽ
    AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 15;  // �������ת������MAX_CONV1 + 1����SEQ1��MAX_CONV1[2:0]�����ã���SEQ2��MAX_CONV2[2:0]�����ã���SEQ��MAX_CONV1[3:0]������.14

    AdcRegs.ADCTRL3.bit.SMODE_SEL = 0;  // 0:˳�������1����������
    AdcRegs.ADCTRL1.bit.CONT_RUN = 0;  // 0����/ͣģʽ��1����������ģʽ

    // ���ò���˳��
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x7;  //��һ�����(APF����)
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x6;
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x1;

    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x9;  //�ڶ������(ע��֧·����) d c 8
    AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0xf;
    AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0xe;

    AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0xd;  //���������(���ص���) 9 f e
    AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0xc;
    AdcRegs.ADCCHSELSEQ3.bit.CONV08 = 0x8;

    AdcRegs.ADCCHSELSEQ3.bit.CONV09 = 0x5;  //��һ���ѹ(�����PCC��ѹ) 5 4 2
    AdcRegs.ADCCHSELSEQ3.bit.CONV10 = 0x4;
    AdcRegs.ADCCHSELSEQ3.bit.CONV11 = 0x2;

    AdcRegs.ADCCHSELSEQ4.bit.CONV12 = 0x3;  //�ڶ����ѹ(ע��֧·PCC��ѹ) 3 a b
    AdcRegs.ADCCHSELSEQ4.bit.CONV13 = 0xa;
    AdcRegs.ADCCHSELSEQ4.bit.CONV14 = 0xb;

    AdcRegs.ADCCHSELSEQ4.bit.CONV15 = 0x0;  //DC��ѹ

    // ����AD�������ж�
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 0x1; // ����EPWM_SOCA����SEQ1/SEQ
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 0x1;  // ����INT_SEQ1�ж� (every EOS)
    // AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0x0;  // 0��ÿ��SEQ1����ʱ��λ��1�����һ��SEQ1����ʱ��λ

}

void adc_read(void)
{
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1; // ���з�������λ��CONV00
}
