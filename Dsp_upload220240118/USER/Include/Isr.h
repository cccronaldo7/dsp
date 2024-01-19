/*********************************************
-- 版权(copyright)：国家电能变换与控制工程技术研究中心(NECC)
-- 项目名:
-- 模块名:
-- 文件名:
-- 作者:  张凯
-- 功能和特点概述: 中断程序头文件
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
