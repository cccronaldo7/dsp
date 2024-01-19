/*********************************************
-- ��Ȩ(copyright)�����ҵ��ܱ任����ƹ��̼����о�����(NECC)
-- ��Ŀ��:
-- ģ����:
-- �ļ���:
-- ����:  �ſ�
-- ���ܺ��ص����: FPGA�������߳���ͷ�ļ�
-- ��ʼ�汾�ͷ���ʱ��: 1.00��2022-03-20
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
