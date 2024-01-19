/*********************************************
-- ��Ȩ(copyright)�����ҵ��ܱ任����ƹ��̼����о�����(NECC)
-- ��Ŀ��:
-- ģ����:
-- �ļ���:
-- ����:  �ſ�
-- ���ܺ��ص����: �жϳ���ͷ�ļ�
-- ��ʼ�汾�ͷ���ʱ��: 1.00��2022-03-15
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

#ifndef ISR_H
#define ISR_H

interrupt void ISRAdc(void);

extern volatile Uint16 *sys_opera_addr;
extern volatile Uint16 sys_opera_flag;
extern volatile Uint16 *svc_alpha_addr;

extern volatile Uint16 pwm_flag;
extern volatile Uint16 pwm_flag_s;
extern volatile Uint16 ctrl_clr_flag;
extern volatile Uint16 ovr_cur_flag;
extern volatile Uint16 protect_flag;

extern float ad_ch[16];
extern float ad_disp1[400];
extern float ad_disp2[400];

#endif /* ISR_H */
