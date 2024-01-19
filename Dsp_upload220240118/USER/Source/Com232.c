/*********************************************
-- 版权(copyright)：国家电能变换与控制工程技术研究中心(NECC)
-- 项目名:
-- 模块名:
-- 文件名:
-- 作者:  张凯
-- 功能和特点概述: 串口程序 SCI初始化 接收 发送
-- 初始版本和发布时间: 1.00，2022-03-16
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

Uint16 com232_rx_data[5] = {0,0,0,0,0};

void com232_init(void)
{

    // 1 stop bit, No loopback, No parity,8 char bits
    ScicRegs.SCICCR.all = 0x0007;

    // 设置波特率 SCIHBAUD:SCILBAUD = 37.5M/(115200*8) - 1
    ScicRegs.SCIHBAUD = 0x0000;
    ScicRegs.SCILBAUD = 0x0028;

    ScicRegs.SCICTL1.all = 0x0003;  // 软件复位，使能发送，接收
    ScicRegs.SCICTL2.all = 0x0003;  //使能发送/间断中断，接收中断
    //ScibRegs.SCICTL2.bit.TXINTENA = 1;
    //ScibRegs.SCICTL2.bit.RXBKINTENA = 1;

    // FIFO
    ScicRegs.SCIFFTX.all = 0xc045;  // 使能SCI FIFO，复位TX FIFO，清TX FIFO FLAG，禁止TX FIFO中断
    ScicRegs.SCIFFRX.all = 0x4065;  // 复位RX FIFO，清RX FIFO FLAG，使能RX FIFO中断，中断深度5字节
    ScicRegs.SCIFFCT.all = 0x0000;

    ScicRegs.SCICTL1.all = 0x0023;  // 使能SCI
    ScicRegs.SCIFFTX.bit.TXFIFOXRESET = 1;  // 使能TX FIFO
    ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;  // 使能RX FIFO

}

void com232_cmd(void)  //指令解析
{
}

void com232_tx(void)
{
    Uint16 i;
    for(i=0;i<=4;i++)
    {
        ScicRegs.SCITXBUF = com232_rx_data[i];
    }
}
