/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : radio_defs.h
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : ?
*******************************************************************************/

/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __RADIO_DEFS_H
#define __RADIO_DEFS_H

/* INCLUDES ------------------------------------------------------------------*/

/* EXPORTED TYPES ------------------------------------------------------------*/

/* EXPORTED CONSTANTS --------------------------------------------------------*/

/* EXPORTED MACROS -----------------------------------------------------------*/

/* EXPORTED DEFINES ----------------------------------------------------------*/

//#define RSSI_OFFSET               76
//#define RSSI_OFFSET_LNA_HIGHGAIN  ( RSSI_OFFSET + 3 )
//#define RSSI_OFFSET_LNA_LOWGAIN   ( RSSI_OFFSET + 14 )

/* FREG (BSET/BCLR supported) */
#define CC2520_FRMFILT0                 0x000
#define CC2520_FRMFILT1                 0x001
#define CC2520_SRCMATCH                 0x002
#define CC2520_SRCSHORTEN0              0x004
#define CC2520_SRCSHORTEN1              0x005
#define CC2520_SRCSHORTEN2              0x006
#define CC2520_SRCEXTEN0                0x008
#define CC2520_SRCEXTEN1                0x009
#define CC2520_SRCEXTEN2                0x00A
#define CC2520_FRMCTRL0                 0x00C
#define CC2520_FRMCTRL1                 0x00D
#define CC2520_RXENABLE0                0x00E
#define CC2520_RXENABLE1                0x00F
#define CC2520_EXCFLAG0                 0x010
#define CC2520_EXCFLAG1                 0x011
#define CC2520_EXCFLAG2                 0x012
#define CC2520_EXCMASKA0                0x014
#define CC2520_EXCMASKA1                0x015
#define CC2520_EXCMASKA2                0x016
#define CC2520_EXCMASKB0                0x018
#define CC2520_EXCMASKB1                0x019
#define CC2520_EXCMASKB2                0x01A
#define CC2520_EXCBINDX0                0x01C
#define CC2520_EXCBINDX1                0x01D
#define CC2520_EXCBINDY0                0x01E
#define CC2520_EXCBINDY1                0x01F
#define CC2520_GPIOCTRL0                0x020
#define CC2520_GPIOCTRL1                0x021
#define CC2520_GPIOCTRL2                0x022
#define CC2520_GPIOCTRL3                0x023
#define CC2520_GPIOCTRL4                0x024
#define CC2520_GPIOCTRL5                0x025
#define CC2520_GPIOPOLARITY             0x026
#define CC2520_GPIOCTRL                 0x028
#define CC2520_DPUCON                   0x02A
#define CC2520_DPUSTAT                  0x02C
#define CC2520_FREQCTRL                 0x02E
#define CC2520_FREQTUNE                 0x02F
#define CC2520_TXPOWER                  0x030
#define CC2520_TXCTRL                   0x031
#define CC2520_FSMSTAT0                 0x032
#define CC2520_FSMSTAT1                 0x033
#define CC2520_FIFOPCTRL                0x034
#define CC2520_FSMCTRL                  0x035
#define CC2520_CCACTRL0                 0x036
#define CC2520_CCACTRL1                 0x037
#define CC2520_RSSI                     0x038
#define CC2520_RSSISTAT                 0x039
#define CC2520_TXFIFO_BUF               0x03A
#define CC2520_RXFIRST                  0x03C
#define CC2520_RXFIFOCNT                0x03E
#define CC2520_TXFIFOCNT                0x03F

/* SREG (BSET/BCLR unsupported) */
#define CC2520_CHIPID                   0x040
#define CC2520_VERSION                  0x042
#define CC2520_EXTCLOCK                 0x044
#define CC2520_MDMCTRL0                 0x046
#define CC2520_MDMCTRL1                 0x047
#define CC2520_FREQEST                  0x048
#define CC2520_RXCTRL                   0x04A
#define CC2520_FSCTRL                   0x04C
#define CC2520_FSCAL0                   0x04E
#define CC2520_FSCAL1                   0x04F
#define CC2520_FSCAL2                   0x050
#define CC2520_FSCAL3                   0x051
#define CC2520_AGCCTRL0                 0x052
#define CC2520_AGCCTRL1                 0x053
#define CC2520_AGCCTRL2                 0x054
#define CC2520_AGCCTRL3                 0x055
#define CC2520_ADCTEST0                 0x056
#define CC2520_ADCTEST1                 0x057
#define CC2520_ADCTEST2                 0x058
#define CC2520_MDMTEST0                 0x05A
#define CC2520_MDMTEST1                 0x05B
#define CC2520_DACTEST0                 0x05C
#define CC2520_DACTEST1                 0x05D
#define CC2520_ATEST                    0x05E
#define CC2520_DACTEST2                 0x05F
#define CC2520_PTEST0                   0x060
#define CC2520_PTEST1                   0x061
#define CC2520_RESERVED                 0x062
#define CC2520_DPUBIST                  0x07A
#define CC2520_ACTBIST                  0x07C
#define CC2520_RAMBIST                  0x07E

/* RAM Addresses */
#define CC2520_RAM_START_ADDR           0x100
#define CC2520_RAM_TXBUF                0x100
#define CC2520_RAM_RXBUF                0x180
#define CC2520_RAM_CBCTEMPL             0x360
#define CC2520_RAM_CBCTEMPH             0x370
#define CC2520_RAM_SRCTABLEBASE         0x380
#define CC2520_RAM_SRCRESMASK0          0x3E0
#define CC2520_RAM_SRCRESMASK1          0x3E1
#define CC2520_RAM_SRCRESMASK2          0x3E2
#define CC2520_RAM_SRCRESINDEX          0x3E3
#define CC2520_RAM_SRCEXTPENDEN0        0x3E4
#define CC2520_RAM_SRCEXTPENDEN1        0x3E5
#define CC2520_RAM_SRCEXTPENDEN2        0x3E6
#define CC2520_RAM_SRCSHORTPENDEN0      0x3E7
#define CC2520_RAM_SRCSHORTPENDEN1      0x3E8
#define CC2520_RAM_SRCSHORTPENDEN2      0x3E9
#define CC2520_RAM_EXTADDR              0x3EA
#define CC2520_RAM_PANID                0x3F2
#define CC2520_RAM_SHORTADDR            0x3F4

/* Command Strobes (identifiers to be used with GPIO and exception binding) */
#define CC2520_STR_SIBUFEX              0x00
#define CC2520_STR_SRXMASKBITCLR        0x01
#define CC2520_STR_SRXMASKBITSET        0x02
#define CC2520_STR_SRXON                0x03
#define CC2520_STR_SSAMPLECCA           0x04
#define CC2520_STR_SACK                 0x05
#define CC2520_STR_SACKPEND             0x06
#define CC2520_STR_SNACK                0x07
#define CC2520_STR_STXON                0x08
#define CC2520_STR_STXONCCA             0x09
#define CC2520_STR_SFLUSHRX             0x0A
#define CC2520_STR_SFLUSHTX             0x0B
#define CC2520_STR_SRXFIFOPOP           0x0C
#define CC2520_STR_STXCAL               0x0D
#define CC2520_STR_SRFOFF               0x0E
#define CC2520_STR_SXOSCOFF             0x0F

/* Exceptions Numbers [Table 14] */
#define CC2520_EXC_RF_IDLE              0x00
#define CC2520_EXC_TX_FRM_DONE          0x01
#define CC2520_EXC_TX_ACK_DONE          0x02
#define CC2520_EXC_TX_UNDERFLOW         0x03
#define CC2520_EXC_TX_OVERFLOW          0x04
#define CC2520_EXC_RX_UNDERFLOW         0x05
#define CC2520_EXC_RX_OVERFLOW          0x06
#define CC2520_EXC_RXENABLE_ZERO        0x07
#define CC2520_EXC_RX_FRM_DONE          0x08
#define CC2520_EXC_RX_FRM_ACCEPTED      0x09
#define CC2520_EXC_SRC_MATCH_DONE       0x0A
#define CC2520_EXC_SRC_MATCH_FOUND      0x0B
#define CC2520_EXC_FIFOP                0x0C
#define CC2520_EXC_SFD                  0x0D
#define CC2520_EXC_DPU_DONE_L           0x0E
#define CC2520_EXC_DPU_DONE_H           0x0F
#define CC2520_EXC_MEMADDR_ERROR        0x10
#define CC2520_EXC_USAGE_ERROR          0x11
#define CC2520_EXC_OPERAND_ERROR        0x12
#define CC2520_EXC_SPI_ERROR            0x13
#define CC2520_EXC_RF_NO_LOCK           0x14
#define CC2520_EXC_RX_FRM_ABORTED       0x15
#define CC2520_EXC_RXBUFMOV_TIME        0x16

/* Exception BitMasks */
#define CC2520_EXC0_RF_IDLE_BM          0x01
#define CC2520_EXC0_TX_FRM_DONE_BM      0x02
#define CC2520_EXC0_TX_ACK_DONE_BM      0x04
#define CC2520_EXC0_TX_UNDERFLOW_BM     0x08
#define CC2520_EXC0_TX_OVERFLOW_BM      0x10
#define CC2520_EXC0_RX_UNDERFLOW_BM     0x20
#define CC2520_EXC0_RX_OVERFLOW_BM      0x40
#define CC2520_EXC0_RXENABLE_ZERO_BM    0x80
#define CC2520_EXC1_RX_FRM_DONE_BM      0x01
#define CC2520_EXC1_RX_FRM_ACCEPTED_BM  0x02
#define CC2520_EXC1_SRC_MATCH_DONE_BM   0x04
#define CC2520_EXC1_SRC_MATCH_FOUND_BM  0x08
#define CC2520_EXC1_FIFOP_BM            0x10
#define CC2520_EXC1_SFD_BM              0x20
#define CC2520_EXC1_DPU_DONE_L_BM       0x40
#define CC2520_EXC1_DPU_DONE_H_BM       0x80
#define CC2520_EXC2_MEMADDR_ERROR_BM    0x01
#define CC2520_EXC2_USAGE_ERROR_BM      0x02
#define CC2520_EXC2_OPERAND_ERROR_BM    0x04
#define CC2520_EXC2_SPI_ERROR_BM        0x08
#define CC2520_EXC2_RF_NO_LOCK_BM       0x10
#define CC2520_EXC2_RX_FRM_ABORTED_BM   0x20
#define CC2520_EXC2_RXBUFMOV_TIME_BM    0x40

/* FSM States */
#define CC2520_FSM_IDLE                 0
#define CC2520_FSM_FS_RX_CAL            2
#define CC2520_FSM_RX_RESET             3
#define CC2520_FSM_RX_S_CLK             4
#define CC2520_FSM_RX_ON                5
#define CC2520_FSM_RX_W_SFD             6
#define CC2520_FSM_RD_LEN               7
#define CC2520_FSM_RX_RD_FCF0           8
#define CC2520_FSM_RX_RD_FCF1           9
#define CC2520_FSM_RX_RD_SEQ            10
#define CC2520_FSM_RX_PKT               12
#define CC2520_FSM_RX_FINAL             13
#define CC2520_FSM_RXRX_WAIT            14
#define CC2520_FSM_FIFO_RESET           16
#define CC2520_FSM_RX_OVERFLOW          17
#define CC2520_FSM_FS_LB_CAL_WAIT       20
#define CC2520_FSM_LB_RESET             21
#define CC2520_FSM_LB_S_CLK             22
#define CC2520_FSM_LB_PRE               23
#define CC2520_FSM_LB_PKT               24
#define CC2520_FSM_LB_FINAL             25
#define CC2520_FSM_RX_INF               31
#define CC2520_FSM_FS_TX_CAL            32
#define CC2520_FSM_FS_TX_CAL_WAIT       33
#define CC2520_FSM_TX_RESET             34
#define CC2520_FSM_TX_S_CLK             35
#define CC2520_FSM_TX_PRE               36
#define CC2520_FSM_LEN                  37
#define CC2520_FSM_PKT                  38
#define CC2520_FSM_TX_FINAL             39
#define CC2520_FSM_TXRX_TRANSIT         40
#define CC2520_FSM_PA_SETTLE            41
#define CC2520_FSM_TX_MOD_DOWNRAMP      42
#define CC2520_FSM_FS_ACK_CAL           48
#define CC2520_FSM_TX_ACK_RESET         49
#define CC2520_FSM_TX_ACK_S_CLK         50
#define CC2520_FSM_TX_ACK_PRE           51
#define CC2520_FSM_TX_ACK_LEN           52
#define CC2520_FSM_TX_ACK_PKT           53
#define CC2520_FSM_TX_ACK_FIN_PKT       54
#define CC2520_FSM_TX_ACK_DELAY         55
#define CC2520_FSM_TX_UNDERFLOW         56
#define CC2520_FSM_TX_TURNOFF           57
#define CC2520_FSM_TX_INF               63

/* FSMSTAT0/1 BitMask */
#define CC2520_FSMSTAT0_FSM_STATE_BM    0x3F
#define CC2520_FSMSTAT0_CAL_RUNNING_BM  0x40
#define CC2520_FSMSTAT0_CAL_DONE_BM     0x80
#define CC2520_FSMSTAT1_RX_ACTIVE_BM    0x01
#define CC2520_FSMSTAT1_TX_ACTIVE_BM    0x02
#define CC2520_FSMSTAT1_LOCK_STATUS_BM  0x04
#define CC2520_FSMSTAT1_SAMPLED_CCA_BM  0x08
#define CC2520_FSMSTAT1_CCA_BM          0x10
#define CC2520_FSMSTAT1_SFD_BM          0x20
#define CC2520_FSMSTAT1_FIFOP_BM        0x40
#define CC2520_FSMSTAT1_FIFO_BM         0x80

/* Status Byte BitMask */
#define CC2520_STB_XOSC_STABLE_BM       0x80
#define CC2520_STB_RSSI_VALID_BM        0x40
#define CC2520_STB_EXC_CHB_BM           0x20
#define CC2520_STB_EXC_CHA_BM           0x10
#define CC2520_STB_DPUH_ACTIVE_BM       0x08
#define CC2520_STB_DPUL_ACTIVE_BM       0x04
#define CC2520_STB_TX_ACTIVE_BM         0x02
#define CC2520_STB_RX_ACTIVE_BM         0x01

/* GPIO Output Type */
#define CC2520_GPIO_CLOCK               0x00
#define CC2520_GPIO_RF_IDLE             0x01
#define CC2520_GPIO_TX_FRM_DONE         0x02
#define CC2520_GPIO_TX_ACK_DONE         0x03
#define CC2520_GPIO_TX_UNDERFLOW        0x04
#define CC2520_GPIO_TX_OVERFLOW         0x05
#define CC2520_GPIO_RX_UNDERFLOW        0x06
#define CC2520_GPIO_RX_OVERFLOW         0x07
#define CC2520_GPIO_RXENABLE_ZERO       0x08
#define CC2520_GPIO_RX_FRM_DONE         0x09
#define CC2520_GPIO_RX_FRM_ACCEPTED     0x0A
#define CC2520_GPIO_SRC_MATCH_DONE      0x0B
#define CC2520_GPIO_SRC_MATCH_FOUND     0x0C
#define CC2520_GPIO_FIFOP               0x0D
#define CC2520_GPIO_SFD                 0x0E
#define CC2520_GPIO_DPU_DONE_L          0x0F
#define CC2520_GPIO_DPU_DONE_H          0x10
#define CC2520_GPIO_MEMADDR_ERROR       0x11
#define CC2520_GPIO_USAGE_ERROR         0x12
#define CC2520_GPIO_OPERAND_ERROR       0x13
#define CC2520_GPIO_SPI_ERROR           0x14
#define CC2520_GPIO_RF_NO_LOCK          0x15
#define CC2520_GPIO_RX_FRM_ABORTED      0x16
#define CC2520_GPIO_RXBUFMOV_TIME       0x17
#define CC2520_GPIO_EXC_CH_A            0x21
#define CC2520_GPIO_EXC_CH_B            0x22
#define CC2520_GPIO_EXC_CH_INVA         0x23
#define CC2520_GPIO_EXC_CH_INVB         0x24
#define CC2520_GPIO_EXC_CH_RX           0x25
#define CC2520_GPIO_EXC_CH_ERR          0x26
#define CC2520_GPIO_FIFO                0x27
#define CC2520_GPIO_FIFOP_NOTEXC        0x28
#define CC2520_GPIO_CCA                 0x29
#define CC2520_GPIO_SFD_NOTEXC          0x2A
#define CC2520_GPIO_RSSI_VALID          0x2C
#define CC2520_GPIO_SAMPLED_CCA         0x2D
#define CC2520_GPIO_SNIFFER_CLK         0x31
#define CC2520_GPIO_SNIFFER_DATA        0x32
#define CC2520_GPIO_RX_ACTIVE           0x43
#define CC2520_GPIO_TX_ACTIVE           0x44
#define CC2520_GPIO_LOW                 0x7E
#define CC2520_GPIO_HIGH                0x7F

/* Instructions */
#define CC2520_INS_SNOP                 0x00
#define CC2520_INS_IBUFLD               0x02
#define CC2520_INS_SIBUFEX              0x03
#define CC2520_INS_SSAMPLECCA           0x04
#define CC2520_INS_SRES                 0x0F
#define CC2520_INS_MEMRD                0x10
#define CC2520_INS_MEMWR                0x20
#define CC2520_INS_RXBUF                0x30
#define CC2520_INS_RXBUFCP              0x38
#define CC2520_INS_RXBUFMOV             0x32
#define CC2520_INS_TXBUF                0x3A
#define CC2520_INS_TXBUFCP              0x3E
#define CC2520_INS_RANDOM               0x3C
#define CC2520_INS_SXOSCON              0x40
#define CC2520_INS_STXCAL               0x41
#define CC2520_INS_SRXON                0x42
#define CC2520_INS_STXON                0x43
#define CC2520_INS_STXONCCA             0x44
#define CC2520_INS_SRFOFF               0x45
#define CC2520_INS_SXOSCOFF             0x46
#define CC2520_INS_SFLUSHRX             0x47
#define CC2520_INS_SFLUSHTX             0x48
#define CC2520_INS_SACK                 0x49
#define CC2520_INS_SACKPEND             0x4A
#define CC2520_INS_SNACK                0x4B
#define CC2520_INS_SRXMASKBITSET        0x4C
#define CC2520_INS_SRXMASKBITCLR        0x4D
#define CC2520_INS_RXMASKAND            0x4E
#define CC2520_INS_RXMASKOR             0x4F
#define CC2520_INS_MEMCP                0x50
#define CC2520_INS_MEMCPR               0x52
#define CC2520_INS_MEMXCP               0x54
#define CC2520_INS_MEMXWR               0x56
#define CC2520_INS_BCLR                 0x58
#define CC2520_INS_BSET                 0x59
#define CC2520_INS_CTR                  0x60
#define CC2520_INS_CBCMAC               0x64
#define CC2520_INS_UCBCMAC              0x66
#define CC2520_INS_CCM                  0x68
#define CC2520_INS_UCCM                 0x6A
#define CC2520_INS_ECB                  0x70
#define CC2520_INS_ECBO                 0x72
#define CC2520_INS_ECBX                 0x74
#define CC2520_INS_ECBXO                0x76
#define CC2520_INS_INC                  0x78
#define CC2520_INS_ABORT                0x7F
#define CC2520_INS_REGRD                0x80
#define CC2520_INS_REGWR                0xC0

/* EXPORTED FUNCTIONS --------------------------------------------------------*/

/* EXTERNAL VARIABLES --------------------------------------------------------*/

#endif  /*__RADIO_DEFS_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/