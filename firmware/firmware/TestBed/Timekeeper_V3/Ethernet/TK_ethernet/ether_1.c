/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
// ether_1.c Set up for ST 32F107 MAC and connected phy
// Inits MAC periferal and MII interface to PHY
//
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/


#include "includes.h"
#include "hardware.h"
#include <yfuns.h>
#include "ether_1.h"

unsigned int PhyAddr;

int Ethernet_Test(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	ETH_InitTypeDef ETH_InitStructure;
        
volatile   clock_time_t wait_time;
struct timer PHY_Delay;

	printf("Starting MAC/PHY setupt\n\r");


	/* Enable ETHERNET clock  */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC | RCC_AHBPeriph_ETH_MAC_Tx |
	RCC_AHBPeriph_ETH_MAC_Rx, ENABLE);

	/* Enable GPIOs clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |	RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
	RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE| RCC_APB2Periph_AFIO, ENABLE);

	GPIO_ETH_MediaInterfaceConfig(GPIO_ETH_MediaInterface_RMII);

	/* Get HSE clock = 25MHz on PA8 pin(MCO) */
	/* set PLL3 clock output to 50MHz (25MHz /5 *10 =50MHz) */
	RCC_PLL3Config(RCC_PLL3Mul_10);
	/* Enable PLL3 */
	RCC_PLL3Cmd(ENABLE);
	/* Wait till PLL3 is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_PLL3RDY) == RESET)
	{}

	/* Get clock PLL3 clock on PA8 pin */
	RCC_MCOConfig(RCC_MCO_PLL3CLK);

	/* ETHERNET pins configuration */
	/* AF Output Push Pull:
	- ETH_MII_MDIO / ETH_RMII_MDIO: PA2
	- ETH_MII_MDC / ETH_RMII_MDC: PC1
	- ETH_MII_TXD2: PC2
	- ETH_MII_TX_EN / ETH_RMII_TX_EN: PB11
	- ETH_MII_TXD0 / ETH_RMII_TXD0: PB12
	- ETH_MII_TXD1 / ETH_RMII_TXD1: PB13
	- ETH_MII_PPS_OUT / ETH_RMII_PPS_OUT: PB5
	- ETH_MII_TXD3: PB8 */

	/* Configure PA2 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PC1, PC2 and PC3 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure PB5, PB8, PB11, PB12 and PB13 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 |
	GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/**************************************************************/
	/*               For Remapped Ethernet pins                   */
	/*************************************************************/
	/* Input (Reset Value):
	- ETH_MII_CRS CRS: PA0
	- ETH_MII_RX_CLK / ETH_RMII_REF_CLK: PA1
	- ETH_MII_COL: PA3
	- ETH_MII_RX_DV / ETH_RMII_CRS_DV: PD8
	- ETH_MII_TX_CLK: PC3
	- ETH_MII_RXD0 / ETH_RMII_RXD0: PD9
	- ETH_MII_RXD1 / ETH_RMII_RXD1: PD10
	- ETH_MII_RXD2: PD11
	- ETH_MII_RXD3: PD12
	- ETH_MII_RX_ER: PB10 */

	/* ETHERNET pins remapp in STM3210C-EVAL board: RX_DV and RxD[3:0] */
	GPIO_PinRemapConfig(GPIO_Remap_ETH, DISABLE);

	/* Configure PA0, PA1 and PA3 as input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PB10 as input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure PC3 as input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure PD8, PD9, PD10, PD11 and PD12 as input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure); /**/

	/* MCO pin configuration------------------------------------------------- */
	/* Configure MCO (PA8) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Reset ETHERNET on AHB Bus */
	ETH_DeInit();

	/* Software reset */
	ETH_SoftwareReset();

	/* Wait for software reset */
	while(ETH_GetSoftwareResetStatus()==SET);

	/* ETHERNET Configuration ------------------------------------------------------*/
	/* Call ETH_StructInit if you don't like to configure all ETH_InitStructure parameter */
	ETH_StructInit(&ETH_InitStructure);

	/* Fill ETH_InitStructure parametrs */
	/*------------------------   MAC   -----------------------------------*/
	ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Disable  ;
	//ETH_InitStructure.ETH_Speed = ETH_Speed_100M;
	ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;
	//ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
	ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable;
	ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;
	ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Enable;
	ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Disable;
	ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;
	ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;
	ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
	ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
	ETH_InitStructure.ETH_Speed = ETH_Speed_100M;
        
//#ifdef TK_V2_hdwr     // delay and reset for hardware reset pin on V2 hardware
        
 // reset..200ms
        timer_set(&PHY_Delay, 10);
        while(!(timer_expired(&PHY_Delay))){
        }
        HwGPOLow(GPO_PHY_RST);
        timer_set(&PHY_Delay, 10);
        while(!(timer_expired(&PHY_Delay))){
        }
  // set and wait 200 ms        
        HwGPOHigh(GPO_PHY_RST);
        timer_set(&PHY_Delay, 10);
        while(!(timer_expired(&PHY_Delay))){
        }
//#else        
//    Lets wait for 1 sec for the Phy to come ready.....
 //       timer_set(&PHY_Delay, 100);
 //       while(!(timer_expired(&PHY_Delay))){
 //       }
//#endif        

/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
// Phy address and Manufacturers ID
// Olimex ST Phy address:  Man ID reg 2 : 0x0006 Man ID Reg3: 0x1C50       
// TK SMSC 8700 Phy addr:0x1F  Man ID reg 2 : 0x0007 Man ID Reg3:0xC0C4
//  TK SMSC 8710A       
        
        for(PhyAddr = 1; 32 >= PhyAddr; PhyAddr++)
	{

#ifdef OLIMEX          
          if((0x0006 == ETH_ReadPHYRegister(PhyAddr,2))
				&& (0x1c50 == (ETH_ReadPHYRegister(PhyAddr,3)&0xFFF0))) break;
#endif
#ifdef TimeKeeper   
#ifdef TK_V2_hdwr       

            if((0x0007 == ETH_ReadPHYRegister(PhyAddr,2))
                                 && (0xc0c0 == (ETH_ReadPHYRegister(PhyAddr,3)&0xFFF0))) break;
#endif
#ifdef TK_V3_hdwr 
            
                   // printf("PhyAddr 2:%x PhyAddr 3:%x \n\r",ETH_ReadPHYRegister(PhyAddr,2),ETH_ReadPHYRegister(PhyAddr,3));
                if((0x0007 == ETH_ReadPHYRegister(PhyAddr,2))
                                 && (0xc0F0 == (ETH_ReadPHYRegister(PhyAddr,3)&0xFFF0))) break;
#endif
                
#endif                
	}

	if(32 < PhyAddr)
	{
		printf("Ethernet Phy Not Found\n\r");
		return 1;
	}
	/* Configure Ethernet */
	if(0 == ETH_Init(&ETH_InitStructure, PhyAddr))
	{
		printf("Ethernet Initialization Failed\n\r");
		return 1;
	}

	printf("MAC and PHY Su Comp Num:%d \n\r",PhyAddr);
        


        
        return 1;
}

