/*********************************************
-- ��Ȩ(copyright)�����ҵ��ܱ任����ƹ��̼����о�����(NECC)
-- ��Ŀ��:
-- ģ����:
-- �ļ���:
-- ����:  �ſ�
-- ���ܺ��ص����: AD��������ͷ�ļ�
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

#ifndef SAMPLE_H
#define SAMPLE_H

#if (CPU_FRQ_150MHZ)     // Ĭ��ϵͳʱ�� SYSCLKOUT = 150 MHz
  #define ADC_MODCLK 0x3 // ��������ʱ�� HSPCLK = SYSCLKOUT/2*ADC_MODCLK=150/(2*3)=25.0 MHz
#endif
#if (CPU_FRQ_100MHZ)
  #define ADC_MODCLK 0x2 // ��������ʱ�� HSPCLK = SYSCLKOUT/2*ADC_MODCLK2=100/(2*2)=25.0 MHz
#endif
#define ADC_CKPS   0x1   // ADʱ��    ADCmodule clock = HSPCLK/(2*ADC_CKPS)=25MHz/(2)=12.5 MHz

#define ADC_SHCLK  0x0   // �������������S/H width in ADC module periods = (ADC_SHCLK+1) ADC cycle

void Adc_Init(void);

#endif /* SAMPLE_H */
