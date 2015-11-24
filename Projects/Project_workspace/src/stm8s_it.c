/**
  ******************************************************************************
  * @file     stm8s_it.c
  * @author   MCD Application Team
  * @version  V2.0.1
  * @date     18-November-2011
  * @brief    Main Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm8s_it.h"
#include "stdio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define INPUT_RFID GPIO_ReadInputPin(GPIOC,GPIO_PIN_4)
#define RF433_STATUS GPIO_ReadInputPin(GPIOA,GPIO_PIN_3)
/*
u16 Min_Low_433 = 0;
u16 Max_Low_433 = 0;
u16 Min_High_433 = 0;
u16 Max_High_433 = 0; */

u16 Min_Low_433 = 5;
u16 Max_Low_433 = 60;

u16 Min_High_433 = 80;
u16 Max_High_433 = 200;
//u16 Min_Low_Sys = 30;//30
//u16 Max_Low_Sys  =200;//50
//u16 Min_High_Sys=  500;//1330
//u16 Max_High_Sys = 1355;//1355//xung chuan nen 150:700 va bit la 3,4,dao toan bo tin hieu luon.

u16 Min_High	= 0;//30
u16 Max_High	= 0;//60
u16 Min_Low		= 0;//15
u16 Max_Low		= 0;//29

extern u8 Data_RF[6];//Store Data
extern u32 Time_Sleep;
extern u32 Time_ACCL_ON;
extern u32 Time_Check_Accl;
extern u32 Time_Wdt_Soft;
extern bool Have_Accl;
u8 Count_Data_RF=0;//Number of bit RF
volatile u16 Timing_RF433;
bool Status_Bit[4];
u8 Local_Bit =0;//Get bit detail
bool Get_RF433=FALSE;
bool Sys_Long_Bit = FALSE;


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u32 Counter1;
volatile u16 De_Noise;
bool First_Header = FALSE;
bool Buff_RFID[60];
u8 Point_RFID_Data= 0x00;
//bool Logic_RFID[20];
//u8 Point_Logic_Data = 0x00;
bool Get_Data_OK=FALSE;
bool HEADER_OK = FALSE;
u32 TimingDelay;
u8 Temp_Logic_Change;
u8 Header_RFID;
u16 IWDT_Timing=0;
u8 calibration = 0;
u8 Chuc=0,DV=0,CRC_Row=0,change =0;
u8 i,Header = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
typedef enum _RF433{
	WAIT_SHORT_BIT,
	WAIT_HEADER_BIT,
	WAIT_SHORT_BIT_2,
	GET_DATA_RF433,
	FINISH_RF433
}ID433SM;



typedef enum _RFID{
	WAIT_STOP_BIT,
	WAIT_HEADER,
	WAIT_SYS_CLK,
	GET_DATA,
	FINISH
}IDSM;

/* Private functions ---------------------------------------------------------*/
IDSM SMRFID = WAIT_STOP_BIT;
ID433SM SMRF433 = WAIT_SHORT_BIT;

/** @addtogroup FLASH_ByteReadWriteOperation
  * @{
  */
#ifdef _COSMIC_
/**
  * @brief  Dummy interrupt routine
  * @param  None
  * @retval None
  */
@far @interrupt void NonHandledInterrupt(void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief  TRAP interrupt routine
  * @param  None
  * @retval None
  */
@far @interrupt void TRAP_IRQHandler(void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#else /*_RAISONANCE_*/

/**
  * @brief  TRAP interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER_TRAP(TRAP_IRQHandler)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#endif /*_COSMIC_*/

/**
  * @brief  Top Level Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(TLI_IRQHandler, 0)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief  Auto Wake Up Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(AWU_IRQHandler, 1)	
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief  Clock Controller Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(CLK_IRQHandler, 2)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief  External Interrupt PORTA Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
 switch(SMRF433)
	{
		case WAIT_SHORT_BIT:
			if((Timing_RF433 > Min_Low_433) && (Timing_RF433 < Max_Low_433) && !RF433_STATUS)//Muc 0 nhe
			{
				SMRF433 = WAIT_HEADER_BIT;
				Header = 0;
				Count_Data_RF =0;
				Chuc=0;DV=0;CRC_Row =0;change =0;
				for(i=0;i<6;i++)Data_RF[i] = 0;
			}		
			break;
		case WAIT_HEADER_BIT://	wait header
			if(Timing_RF433 > Min_High_433 && Timing_RF433 < Max_High_433)
			{
				Header++;
				if(Header>=9) SMRF433 = WAIT_SHORT_BIT_2;
			}
			else
			{
				SMRF433 = WAIT_SHORT_BIT;
			}
			break;
		case WAIT_SHORT_BIT_2://	Get data in RFID,
			if(Timing_RF433 > Min_Low_433 && Timing_RF433 < Max_Low_433)//Muc 0 nhe
			SMRF433 = GET_DATA_RF433;
			else	
			SMRF433 = WAIT_SHORT_BIT;	
			
			break;
		case GET_DATA_RF433://	Get data in RFID,
			if(DV!=4) Data_RF[Chuc]=Data_RF[Chuc]<<1;
			
			if(Timing_RF433 > Min_Low_433 && Timing_RF433 < Max_Low_433)//Muc 0 nhe
			{
				i=0;
				//Data_RF[Count_Data_RF] = 0;
			}
			else if(Timing_RF433 > Min_High_433 && Timing_RF433 < Max_High_433)//muc 1 nhe.
			{
				//Data_RF[Count_Data_RF] = 1;
				if(DV!=4)Data_RF[Chuc]++;
				CRC_Row++;
			}
			else	SMRF433 = WAIT_SHORT_BIT;
			
			Count_Data_RF++;
			DV++;//Chuc=0,
			if(DV == 5)
			{
				if(CRC_Row == 1 || CRC_Row ==3 ||CRC_Row ==5) SMRF433 = WAIT_SHORT_BIT;//false crc
				DV = 0;change++;
				if(change==2){Chuc++;change=0;}
				CRC_Row = 0;
			}
			if(Count_Data_RF >=60)
			{
				Get_RF433 = TRUE;
				SMRF433 = FINISH_RF433;
			}
			break;
		case FINISH_RF433://	WAIT_HEADER
			if(!Get_RF433)SMRF433 = WAIT_SHORT_BIT;
			break;
		default: break;				
	}//*/	
	Timing_RF433=0;						
}	
	/*
INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3)
{
	 switch(SMRF433)
		{
			case WAIT_SHORT_BIT:
			if(Timing_RF433>25) calibration = 15;
			if(Timing_RF433>100) calibration = 30;
			if(Timing_RF433<15) calibration = 5;
				//if(Timing_RF433 > Min_Low_Sys && Timing_RF433 < Max_Low_Sys && !RF433_STATUS)//Muc 0 nhe
				if(Timing_RF433 > calibration && !RF433_STATUS)//Muc 0 nhe
				{
					Min_Low_433 = Timing_RF433 - calibration;
					Max_Low_433 = Timing_RF433 + calibration;
					SMRF433 = WAIT_LONG_BIT;
				}//Min_High_Sys			
				break;
			case WAIT_LONG_BIT://	wait header
				if(Timing_RF433 > 10*Max_Low_433 && Timing_RF433 < 45*Max_Low_433 && RF433_STATUS)
				{
					SMRF433 = GET_DATA_RF433;
					Local_Bit = 0;
					Count_Data_RF=0;
					Sys_Long_Bit = TRUE;
				}
				else
				{
					SMRF433 = WAIT_SHORT_BIT;
				}
				break;
			case GET_DATA_RF433://	Get data in RFID,
				if(Timing_RF433>Min_Low_433 && Timing_RF433 < Max_Low_433)//Muc 1 nhe
				{						
					Status_Bit[Local_Bit] = FALSE;
					Local_Bit++;
					if(Local_Bit==4)
					{
						Local_Bit=0;
						if(Status_Bit[0] == TRUE) Data_RF[Count_Data_RF] = 1;//HIGH
						else Data_RF[Count_Data_RF] = 2;//FLOAT		
						Count_Data_RF++;								
					}
				}
				else
				{
					if(Sys_Long_Bit)
					{
						if(Timing_RF433 > 2*Min_Low_433 && Timing_RF433 < 5*Max_Low_433)
						{
							Sys_Long_Bit = FALSE;
							Min_High_433 = Timing_RF433 - calibration;
							Max_High_433 = Timing_RF433 + calibration;
						}
						else SMRF433 = WAIT_SHORT_BIT;//nhieu thi quay lai tu dau nhe 
					}
					if(Timing_RF433 > Min_High_433 && Timing_RF433 < Max_High_433)//Muc 0 nhe
					{
						Status_Bit[Local_Bit] = TRUE;
						Local_Bit++;
						if(Local_Bit==4)
						{
							Local_Bit=0;
							if(Status_Bit[0] == TRUE) Data_RF[Count_Data_RF] = 3;//1001
							else if(Status_Bit[1] == TRUE) Data_RF[Count_Data_RF] = 0;//		DPAK footprint
							else Data_RF[Count_Data_RF] = 4;//0011
							Count_Data_RF++;								
						}
					}
					else SMRF433 = WAIT_SHORT_BIT;//nhieu thi quay lai tu dau nhe
				}
				if(Count_Data_RF >=12)
				{
					Get_RF433 = TRUE;
					SMRF433 = FINISH_RF433;
				}
				break;
			case FINISH_RF433://	WAIT_HEADER
				if(!Get_RF433)SMRF433 = WAIT_SHORT_BIT;
				break;
			default: break;				
		}
		Timing_RF433 = 0;
}
*/
/**
  * @brief  External Interrupt PORTB Interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
	if(GPIO_ReadInputPin(BUTTON_PORT,BUTTON_PIN))
	GPIO_WriteHigh(LED_PORT, LED_PIN);		
	else GPIO_WriteLow(LED_PORT, LED_PIN);  */
	/*	
	if(INPUT_RFID)Buff_RFID[Point_RFID_Data] = 0;
	else Buff_RFID[Point_RFID_Data] = 1;
	Point_RFID_Data++;
	if (Point_RFID_Data>54)
	{
		Point_RFID_Data =0;
		Get_Data_OK =TRUE;
	}
	*/
	
}

/**
  * @brief  External Interrupt PORTC Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
				switch(SMRFID)
				{
					case WAIT_STOP_BIT:
						//else	SMRFID = WAIT_STOP_BIT;
						if(((De_Noise/2) - 15) > 0 && !INPUT_RFID)
						{
							Min_High = De_Noise - 15;
							Max_High = De_Noise + 15;
							Min_Low  = ((De_Noise/2) - 15);
							Max_Low  = ((De_Noise/2) + 15);
							Header_RFID =0;
							First_Header = TRUE;
							SMRFID = WAIT_HEADER;
						}
						break;
					case WAIT_HEADER://	wait header
						if(De_Noise> Min_Low && De_Noise < Max_Low) 
						{
							Header_RFID++;
							if(First_Header)
							{
								First_Header = FALSE;
								if(De_Noise - 15 > 0) 
								{
									Min_Low = De_Noise - 15;
									Max_Low = De_Noise + 15;
								}
								else
								SMRFID = WAIT_STOP_BIT;
							}
						}
						else
						{
							Header_RFID = 0;
							SMRFID = WAIT_STOP_BIT;	
						}
						if(Header_RFID == 16)
						{
							HEADER_OK = TRUE;
							SMRFID = WAIT_SYS_CLK;							
						}
						break;
					case WAIT_SYS_CLK://	Wait sys clock,
						if(De_Noise > Min_High && De_Noise < Max_High && INPUT_RFID)//Muc 0 nhe
						{
							Point_RFID_Data = 0;	
							Buff_RFID[0] = 0;
							Min_High = De_Noise - 15;
							Max_High = De_Noise + 15;
							SMRFID = GET_DATA;
						}
						else SMRFID = WAIT_STOP_BIT;
						break;
					case GET_DATA://	Get data in RFID,
						if(De_Noise>Min_Low && De_Noise < Max_Low)//Muc 1 nhe
						{						
							Temp_Logic_Change++;
							if(Temp_Logic_Change == 2)
							{
								Point_RFID_Data++;
									if(INPUT_RFID)Buff_RFID[Point_RFID_Data] = 0;
									else Buff_RFID[Point_RFID_Data] = 1;
								//Buff_RFID[Point_RFID_Data]=  !INPUT_RFID;
								Temp_Logic_Change=0;
							}//
						}
						else if(De_Noise > Min_High && De_Noise < Max_High)//Muc 0 nhe
						{
							Point_RFID_Data++;
							//Buff_RFID[Point_RFID_Data]	= !INPUT_RFID;	
								if(INPUT_RFID)Buff_RFID[Point_RFID_Data] = 0;
								else Buff_RFID[Point_RFID_Data] = 1;
							Temp_Logic_Change = 0;
						}
						else 
						{
							SMRFID = WAIT_HEADER;//nhieu thi quay lai tu dau nhe
							Point_RFID_Data = 0;
							Temp_Logic_Change = 0;
						}
						if(Point_RFID_Data >54)
						{
							Get_Data_OK = TRUE;
							SMRFID = FINISH;
						}
						break;
					case FINISH://	WAIT_HEADER
						Header_RFID =0;
						HEADER_OK =FALSE;
						if(!Get_Data_OK)SMRFID = WAIT_STOP_BIT;
						break;
					default: break;				
				}
				De_Noise=0;		
}

/**
  * @brief  External Interrupt PORTD Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(EXTI_PORTD_IRQHandler, 6)
{
	Have_Accl = TRUE;
	
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief  External Interrupt PORTE Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(EXTI_PORTE_IRQHandler, 7)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#ifdef STM8S903
/**
  * @brief  External Interrupt PORTF Interrupt routine
  * @param None
  * @retval
  * None
  */
 INTERRUPT_HANDLER(EXTI_PORTF_IRQHandler, 8)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#endif /*STM8S903*/

#ifdef STM8S208
/**
  * @brief  CAN RX Interrupt routine
  * @param None
  * @retval
  * None
  */
 INTERRUPT_HANDLER(CAN_RX_IRQHandler, 8)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief  CAN TX Interrupt routine
  * @param None
  * @retval
  * None
  */
 INTERRUPT_HANDLER(CAN_TX_IRQHandler, 9)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#endif /*STM8S208 || STM8AF52Ax */

/**
  * @brief  SPI Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(SPI_IRQHandler, 10)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief  Timer1 Update/Overflow/Trigger/Break Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief  Timer1 Capture/Compare Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(TIM1_CAP_COM_IRQHandler, 12)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

#ifdef STM8S903
/**
  * @brief  Timer5 Update/Overflow/Break/Trigger Interrupt routine
  * @param None
  * @retval
  * None
  */
 INTERRUPT_HANDLER(TIM5_UPD_OVF_BRK_TRG_IRQHandler, 13)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
/**
  * @brief  Timer5 Capture/Compare Interrupt routine
  * @param None
  * @retval
  * None
  */
 INTERRUPT_HANDLER(TIM5_CAP_COM_IRQHandler, 14)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

#else /*STM8S208, STM8S207, STM8S105 or STM8S103 or STM8AF62Ax or STM8AF52Ax or STM8AF626x */
/**
  * @brief  Timer2 Update/Overflow/Break Interrupt routine
  * @param None
  * @retval
  * None
  */
 INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief  Timer2 Capture/Compare Interrupt routine
  * @param None
  * @retval
  * None
  */
 INTERRUPT_HANDLER(TIM2_CAP_COM_IRQHandler, 14)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#endif /*STM8S903*/

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S105) || \
    defined(STM8S005) ||  defined (STM8AF62Ax) || defined (STM8AF52Ax) || defined (STM8AF626x)
/**
  * @brief  Timer3 Update/Overflow/Break Interrupt routine
  * @param None
  * @retval
  * None
  */
 INTERRUPT_HANDLER(TIM3_UPD_OVF_BRK_IRQHandler, 15)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief  Timer3 Capture/Compare Interrupt routine
  * @param None
  * @retval
  * None
  */
 INTERRUPT_HANDLER(TIM3_CAP_COM_IRQHandler, 16)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#endif /*STM8S208, STM8S207 or STM8S105 or STM8AF62Ax or STM8AF52Ax or STM8AF626x */

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S103) || \
    defined(STM8S003) ||  defined (STM8AF62Ax) || defined (STM8AF52Ax) || defined (STM8S903)
/**
  * @brief  UART1 TX Interrupt routine
  * @param None
  * @retval
  * None
  */
 INTERRUPT_HANDLER(UART1_TX_IRQHandler, 17)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief  UART1 RX Interrupt routine
  * @param None
  * @retval
  * None
  */
 INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#endif /*STM8S105*/

/**
  * @brief  I2C Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(I2C_IRQHandler, 19)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

#if defined(STM8S105) || defined(STM8S005) ||  defined (STM8AF626x)
/**
  * @brief  UART2 TX interrupt routine.
  * @param None
  * @retval
  * None
  */
 INTERRUPT_HANDLER(UART2_TX_IRQHandler, 20)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief  UART2 RX interrupt routine.
  * @param None
  * @retval
  * None
  */
 INTERRUPT_HANDLER(UART2_RX_IRQHandler, 21)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
#endif /* STM8S105*/

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
/**
  * @brief  UART3 TX interrupt routine.
  * @param None
  * @retval
  * None
  */
 INTERRUPT_HANDLER(UART3_TX_IRQHandler, 20)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief  UART3 RX interrupt routine.
  * @param None
  * @retval
  * None
  */
 INTERRUPT_HANDLER(UART3_RX_IRQHandler, 21)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#endif /*STM8S208 or STM8S207 or STM8AF52Ax or STM8AF62Ax */

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
/**
  * @brief  ADC2 interrupt routine.
  * @param None
  * @retval
  * None
  */
 INTERRUPT_HANDLER(ADC2_IRQHandler, 22)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
#else /*STM8S105, STM8S103 or STM8S903 or STM8AF626x */
/**
  * @brief  ADC1 interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(ADC1_IRQHandler, 22)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
#endif /*STM8S208 or STM8S207 or STM8AF52Ax or STM8AF62Ax */

#ifdef STM8S903
/**
  * @brief  Timer6 Update/Overflow/Trigger Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(TIM6_UPD_OVF_TRG_IRQHandler, 23)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#else /*STM8S208, STM8S207, STM8S105 or STM8S103 or STM8AF62Ax or STM8AF52Ax or STM8AF626x */
/**
  * @brief  Timer4 Update/Overflow Interrupt routine
  * @param None
  * @retval
  * None
  */
 INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
	Counter1++;
	Time_Wdt_Soft++;
	Time_Sleep++;
	De_Noise++;
	Time_ACCL_ON++;
	Timing_RF433++;
	TimingDelay++;
	if(IWDT_Timing++>20000 && Time_Wdt_Soft < 2000000)//<5s ko reset. >5s reset
	{
		IWDG_ReloadCounter();
		IWDT_Timing=0;
	}
  TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
}
#endif /*STM8S903*/

/**
  * @brief  Eeprom EEC Interrupt routine
  * @param None
  * @retval
	* None
  */
INTERRUPT_HANDLER(EEPROM_EEC_IRQHandler, 24)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/