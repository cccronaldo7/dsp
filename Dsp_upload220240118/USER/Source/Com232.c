/*********************************************
-- ��Ȩ(copyright)�����ҵ��ܱ任����ƹ��̼����о�����(NECC)
-- ��Ŀ��:
-- ģ����:
-- �ļ���:
-- ����:  �ſ�
-- ���ܺ��ص����: ���ڳ��� SCI��ʼ�� ���� ����
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

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "User.h"

Uint16 com232_rx_data[5] = {0,0,0,0,0};

void com232_init(void)
{

    // 1 stop bit, No loopback, No parity,8 char bits
    ScicRegs.SCICCR.all = 0x0007;

    // ���ò����� SCIHBAUD:SCILBAUD = 37.5M/(115200*8) - 1
    ScicRegs.SCIHBAUD = 0x0000;
    ScicRegs.SCILBAUD = 0x0028;

    ScicRegs.SCICTL1.all = 0x0003;  // �����λ��ʹ�ܷ��ͣ�����
    ScicRegs.SCICTL2.all = 0x0003;  //ʹ�ܷ���/����жϣ������ж�
    //ScibRegs.SCICTL2.bit.TXINTENA = 1;
    //ScibRegs.SCICTL2.bit.RXBKINTENA = 1;

    // FIFO
    ScicRegs.SCIFFTX.all = 0xc045;  // ʹ��SCI FIFO����λTX FIFO����TX FIFO FLAG����ֹTX FIFO�ж�
    ScicRegs.SCIFFRX.all = 0x4065;  // ��λRX FIFO����RX FIFO FLAG��ʹ��RX FIFO�жϣ��ж����5�ֽ�
    ScicRegs.SCIFFCT.all = 0x0000;

    ScicRegs.SCICTL1.all = 0x0023;  // ʹ��SCI
    ScicRegs.SCIFFTX.bit.TXFIFOXRESET = 1;  // ʹ��TX FIFO
    ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;  // ʹ��RX FIFO

}

void com232_cmd(void)  //ָ�����
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
