/**
  ******************************************************************************
  * @file    FLASH_ByteReadWriteOperation\main.c
  * @author  MCD Application Team
  * @version  V2.0.1
  * @date     18-November-2011
  * @brief   This file contains the main function for FLASH byte read write operation example.
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
#include "stm8s.h"
#include "stm8s_clk.h"

/**
  * @addtogroup FLASH_ByteReadWriteOperation
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;
/* Private define ------------------------------------------------------------*/
#define CCR1_Val  ((u16)8)
#define BUTTON_PORT GPIOC
#define BUTTON_PIN GPIO_PIN_4

#define RF433_PORT GPIOA
#define RF433_PIN GPIO_PIN_3


#define	XOUT_REG		0x00
#define YOUT_REG		0x01
#define ZOUT_REG		0x02
#define TILT_REG		0x03
#define SRST_REG		0x04
#define SPCNT_REG		0x05
#define INTSU_REG		0x06
#define	MODE_REG		0x07
#define	SR_REG			0x08
#define PDET_REG		0x09
#define PD_REG			0x0A

#define TIME_ACCL    60
#define ACCL_INT_PORT	 GPIOD
#define ACCL_INT_PIN	 GPIO_PIN_3

#define ACCL_EN_PORT	 GPIOA
#define ACCL_EN_PIN	 GPIO_PIN_1

#define SCL_PORT GPIOB//b4
#define SCL_PIN  GPIO_PIN_4

#define SDA_PORT GPIOB//b5
#define SDA_PIN  GPIO_PIN_5
//SCL
#define SCL1 GPIO_WriteHigh(SCL_PORT,SCL_PIN)
#define SCL0 GPIO_WriteLow(SCL_PORT,SCL_PIN)
//SDA
#define SDA1 GPIO_WriteHigh(SDA_PORT,SDA_PIN)
#define SDA0 GPIO_WriteLow(SDA_PORT,SDA_PIN)

#define SDAIN  GPIO_Init(SDA_PORT,SDA_PIN, GPIO_MODE_IN_FL_NO_IT);
#define SDAOUT GPIO_Init(SDA_PORT,SDA_PIN,GPIO_MODE_OUT_PP_HIGH_FAST);




#define SDA  GPIO_ReadInputPin(SDA_PORT,SDA_PIN)


#define RFID_EN_PORT GPIOC
#define RFID_EN_PIN  GPIO_PIN_3

#define ACC_PORT GPIOD
#define ACC_PIN GPIO_PIN_6

#define EN_PWR_PORT GPIOD
#define EN_PWR_PIN GPIO_PIN_2

#define ENGINE_PORT GPIOC//B
#define ENGINE_PIN  GPIO_PIN_6//4

#define SIGNAL_PORT GPIOC  
#define SIGNAL_PIN GPIO_PIN_5


#define SIG1IN_PORT GPIOC  
#define SIG1IN_PIN GPIO_PIN_7


#define SIG2IN_PORT GPIOA  
#define SIG2IN_PIN GPIO_PIN_2

#define INPUT_1 GPIO_ReadInputPin(SIG1IN_PORT,SIG1IN_PIN)
#define INPUT_2 GPIO_ReadInputPin(SIG2IN_PORT,SIG2IN_PIN)

#define BELL_PORT GPIOD//B
#define BELL_PIN GPIO_PIN_5//5

#define ACC GPIO_ReadInputPin(ACC_PORT,ACC_PIN)

#define RFID_ON  GPIO_WriteHigh(RFID_EN_PORT,RFID_EN_PIN)

#define RFID_OFF GPIO_WriteLow(RFID_EN_PORT,RFID_EN_PIN)


#ifdef _RAISONANCE_
#define PUTCHAR_PROTOTYPE int putchar (char c)
#define GETCHAR_PROTOTYPE int getchar (void)
#elif defined (_COSMIC_)
#define PUTCHAR_PROTOTYPE char putchar (char c)
#define GETCHAR_PROTOTYPE char getchar (void)
#else /* _IAR_ */
#define PUTCHAR_PROTOTYPE int putchar (int c)
#define GETCHAR_PROTOTYPE int getchar (void)
#endif /* _RAISONANCE_ */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO TestStatus OperationStatus;
extern u32 Counter1;
extern u32 TimingDelay;
extern bool Get_Data_OK;
extern bool Buff_RFID[60];
//extern bool Logic_RFID[20];
u32 Time_Sleep;
u32 Time_Off;
u32 Time_Ring;
u32 Time_Reg_Tag;
u32 Time_Wdt_Soft=0;
u32 Time_ACCL_ON= 0;
u32 Time_Check_Accl = 0;
u32 Time_Signal = 0; //En_Signal
//u8 Data_RFID[5]= {0,0,0,0,0};
u16 Data_RFID[5]= {0,0,0,0,0};
uint8_t Sig_Left=0,Sig_Right=0;
bool Tag_Admin[5] = {TRUE,TRUE,TRUE,TRUE,TRUE};
bool Tag_Master = TRUE;
bool Wait_RF = FALSE;
bool Tag_User[2] = {TRUE,TRUE};
bool Enable_Ring = TRUE;
bool Have_Ring = TRUE;
bool Chan_bell = TRUE;
bool Off_Tag = FALSE;
u8   Count_Ring = 0;
u8 Count2Sleep=0;
const u8 Admin[5][5]= {7,0,166,71,86,
								 8,0,91,58,58,
								 5,0,135,86,36,
								 5,0,117,156,52,
								 83,0,132,217,38};
u8 Master[5]= {0,0,0,0,0};
u8 User[2][5]= {0,0,0,0,0,
								0,0,0,0,0};
uint8_t On_Off_Bell = 0x00;

const u8 Databit[8] ={0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
const u8 Admin_433[3]={0x1D,0x75,0x30};//0-H;2-L;3-L. tu thiet ke lay. 48 1d 75 30
u8 User_433[4][3]={0,0,0,
									 0,0,0,
									 0,0,0,
									 0,0,0};
bool En_Signal = FALSE;
bool Tag_Admin_433 = TRUE;
bool tem1=FALSE,tem2=FALSE;
bool Tag_User_433[4] = {TRUE,TRUE,TRUE,TRUE};
bool Exit_Learning_tag= FALSE;
bool Have_Accl = FALSE;
bool Wdt_Reset = FALSE;
u8  ACCL_EN = 0;
uint8_t Check_Dat;
//bool Data_Accl[8]= {FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE};
//u8 Data_Accl=0;

//u8 Data_RF[12];
u16 Data_RF[12];
u8 Data_433[3];

u8 State_433 = 0;
extern bool Get_RF433;
typedef enum _SMMAIN{
	ENG_ON_NO_TAG,
	ENG_ON_TAG,
	ENG_OFF,
	ENG_OFF_NO_TAG,
	WAIT_CHANGE
}MAINSM;
//MAINSM SMSTATUS = ENG_OFF;
MAINSM SMSTATUS = ENG_ON_NO_TAG;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void TIMER_Configuration(void);
//void Bell_Ring(u8 on,u8 off,u8 time);
void Bell_Ring(void);
int check_parity(void);
bool Data_2_Number (void);
bool Get_Id (void);
void Delay(u32 nTime);
uint32_t LSIMeasurment(void);
void IWDG_Config(void);
void Learing_Tag_RFID(uint16_t Address);//address store data.
bool Get_Data_RF433(void);
bool Check_SM_Status(void);
bool Process_Tag(void);
u8 Process_433(void);
void Leaning_RF433(uint16_t Address);
bool Receive_433(void);

void MMA7660_Write(u8 Regis,u8 data);
void Delay_MMA7660(void);
u8 MMA7660_Init(void);
uint8_t MMA7660_Read(u8 Regis);
void i2c_start (void);
void i2c_stop (void);
void i2c_write (uint8_t output_data);
uint8_t i2c_read (void);
//bool Writer_Eeprom(u32 Adress, u8 Data[],Number)
/* Public functions ----------------------------------------------------------*/

/**
  * @brief How to Read / Write / Erase one Byte on FLASH memory.
  * @par   Examples description
  *        - Read one byte at address 0x40A5
  *        - Write its complement value at adress + 1
  *        - Check programed value
  *        - Erase 2 byte (address 40A5 & 40A6)
  *        - Check the 2 bytes value is 0x00.
  * @param  None
  * @retval None
  */
void main(void)
{

		int i,j;
		CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1); 
		CLK_FastHaltWakeUpCmd (ENABLE);
		FLASH_Lock(FLASH_MEMTYPE_PROG);
		/*UART1_Init((uint32_t)9600, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,
              UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);							
	 	Output a message on Hyperterminal using printf function */
		//printf("V: 14-04\n\r");
		//printf("Athor: TDX\n\r");
		//printf("Create:07-04-2014\n\r");
		//printf("Version: 2.0\n\r");
//		GPIO_DeInit(GPIOA);  		
//		GPIO_DeInit(GPIOB); 
//		GPIO_DeInit(GPIOC);		
//		GPIO_DeInit(GPIOD); 	
		
  /* I2C Initialize */
  //I2C_Init(I2C_SPEED, 0xA0, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, 16);


  /* Enable Buffer and Event Interrupt*/
  //I2C_ITConfig((I2C_IT_TypeDef)(I2C_IT_EVT | I2C_IT_BUF) , ENABLE);
	

		/* get measured LSI frequency */
		// LsiFreq = LSIMeasurment(); 
		//printf("%lu\n\r",LsiFreq);		
		
		/* IWDG Configuration */
		IWDG_Config();
		
		GPIO_Init(EN_PWR_PORT,EN_PWR_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);	//RFID_EN_PORT

		/* Configure  (ENGINE) as output push-pull high */
		GPIO_Init(ENGINE_PORT, ENGINE_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
		
		
		/* Configure  (SIGNAL) as output push-pull low  */
		GPIO_Init(SIGNAL_PORT, SIGNAL_PIN, GPIO_MODE_OUT_PP_LOW_FAST);//ACCL_EN_PORT
		
		GPIO_Init(ACCL_EN_PORT, ACCL_EN_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
		
		/* Configure  (BELL) as output push-pull high*/		
		
		
		GPIO_Init(SCL_PORT, SCL_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
		
		GPIO_Init(SDA_PORT, SDA_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
		
		
		GPIO_Init(BELL_PORT,BELL_PIN, GPIO_MODE_OUT_PP_LOW_FAST);		
		
		

		
		GPIO_Init(RFID_EN_PORT,RFID_EN_PIN, GPIO_MODE_OUT_PP_LOW_FAST);	//RFID EN			

		/* Configure PB6 (push buton) as input floating */
		GPIO_Init(ACC_PORT,ACC_PIN, GPIO_MODE_IN_PU_NO_IT);
		
		GPIO_Init(SIG1IN_PORT,SIG1IN_PIN, GPIO_MODE_IN_PU_NO_IT);
		GPIO_Init(SIG2IN_PORT,SIG2IN_PIN, GPIO_MODE_IN_PU_NO_IT);		
		
		
		GPIO_Init(BUTTON_PORT, BUTTON_PIN, GPIO_MODE_IN_PU_IT);// ACCL_INT_PORT
		GPIO_Init(ACCL_INT_PORT, ACCL_INT_PIN, GPIO_MODE_IN_PU_IT);
		GPIO_Init(RF433_PORT, RF433_PIN, GPIO_MODE_IN_PU_IT);//GPIO_MODE_IN_FL_IT.GPIO_MODE_IN_FL_NO_IT	
		
		/* Reload IWDG counter */
		IWDG_ReloadCounter();
		TIMER_Configuration();
		EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOC, EXTI_SENSITIVITY_RISE_FALL);
		EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA, EXTI_SENSITIVITY_RISE_FALL);
		EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_FALL_ONLY);
		enableInterrupts();

    /* Define FLASH programming time */
    FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);

    /* Unlock Data memory */
    FLASH_Unlock(FLASH_MEMTYPE_DATA);
    /* Read a byte at a specified address */
    //add = 0x4000;
		for(i = 0;i<5;i++)
		{
			Master[i] = FLASH_ReadByte(0x4000+i);
			User[0][i] = FLASH_ReadByte(0x4010+i);
			User[1][i] = FLASH_ReadByte(0x4020+i);
		}
		On_Off_Bell = FLASH_ReadByte(0x4030);
		for(j=0;j<3;j++) User_433[0][j] = FLASH_ReadByte(0x4040+j);	
		for(j=0;j<3;j++) User_433[1][j] = FLASH_ReadByte(0x4050+j);	
		
		ACCL_EN = FLASH_ReadByte(0x406A);		
		FLASH_Lock(FLASH_MEMTYPE_DATA); 
		//printf("Data EEPROM: %d\n\r",Read_Data_Eeprom);		
		Time_Ring = Counter1;
		Get_Data_OK = FALSE;//Lay the moi
	if(RST_GetFlagStatus(RST_FLAG_EMCF)!=RESET) 
	{
		Wdt_Reset = TRUE;
		//printf("EMC reset flag\r\n");
		RST_ClearFlag(RST_FLAG_EMCF);
	}
	if(RST_GetFlagStatus(RST_FLAG_ILLOPF)!=RESET)
	{
		Wdt_Reset = TRUE;
		//printf("Illigal opcode reset flag\r\n");
		RST_ClearFlag(RST_FLAG_ILLOPF);
	}
	if(RST_GetFlagStatus(RST_FLAG_IWDGF)!=RESET)
	{
		Wdt_Reset = TRUE;
		//printf("Independent watchdog reset flag\r\n");
		RST_ClearFlag(RST_FLAG_IWDGF);
	}
	if(Wdt_Reset)
	{
		Enable_Ring = FALSE;
		SMSTATUS = ENG_ON_TAG;
		GPIO_WriteHigh(ENGINE_PORT, ENGINE_PIN);
		RFID_OFF;
	}
	else
	{
		Enable_Ring = TRUE;
		SMSTATUS = ENG_ON_NO_TAG;
		GPIO_Init(ENGINE_PORT, ENGINE_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
		RFID_ON;
	}


		
		/*while(1)
	{
			if(ACC)
			{
			GPIO_WriteHigh(BELL_PORT, BELL_PIN);// On  Bell
			GPIO_WriteHigh(ENGINE_PORT, ENGINE_PIN);// On  Bell
			GPIO_WriteHigh(SIGNAL_PORT, SIGNAL_PIN);// On  Bell
			}
			else
			{
				GPIO_WriteLow(BELL_PORT, BELL_PIN);// Off Bell
				GPIO_WriteLow(ENGINE_PORT, ENGINE_PIN);// Off Bell
				GPIO_WriteLow(SIGNAL_PORT, SIGNAL_PIN);// Off Bell
			}
	}// */
	/*
		GPIO_WriteLow(ACCL_EN_PORT, ACCL_EN_PIN);// bat nguon accl
		MMA7660_Init();
		ACCL_EN = 1;
			if(Have_Accl)
			{
				//printf("Have ACCL");
				Have_Accl = 0;
				GPIO_WriteHigh(SIGNAL_PORT, SIGNAL_PIN);// Off Bell
				Delay(50000); 
				GPIO_WriteLow(SIGNAL_PORT, SIGNAL_PIN);// On  Bell	
				Delay(50000);				
				MMA7660_Read(TILT_REG);
			}
			while(1)
		{
			if(Get_Data_RF433())
			{
					GPIO_WriteHigh(SIGNAL_PORT, SIGNAL_PIN);// Off Bell
					Delay(50000); 
					GPIO_WriteLow(SIGNAL_PORT, SIGNAL_PIN);// On  Bell	
					Delay(50000);		
			}
		}//*/
		/*
		them ham if reset vao day la okie nhe.
		neu wdt thi Enable_Ring = false va 	SMSTATUS = 	ENG_ON_TAG
		neu do bat nguon thi Enable_Ring = true va SMSTATUS = 	ENG_ON_NO_TAG
		/// */
		
		while(1)
		{
			Time_Wdt_Soft = 0;
			// /*
			switch(SMSTATUS)
			{
				case ENG_ON_NO_TAG://bat may chua quet the
					if(Process_Tag())
					{
						//Bell_Ring(1,10,1);//Ring bell	
						if(Tag_Admin[0]||Tag_Admin[1]||Tag_Admin[2]||Tag_Admin[3]||Tag_Admin[4]) //admin tag
						{
							ACCL_EN = 1;
							Exit_Learning_tag = FALSE;
							Learing_Tag_RFID(0x4000);	//Learning Master tag				
							tem1 = TRUE;
							goto __Regis_Tag;
						}
						if(Tag_Master)
						{						
							Counter1=0;
							Time_Reg_Tag = Counter1;
							while((Counter1 - Time_Reg_Tag)<200000)
							{
								if(ACC)
								{
									Delay(500);
									if(ACC)
									{
										goto __Exit;
									}
								}
								Time_Wdt_Soft = 0;
							}
							//Delay(300000);//delay 3s
							Get_Data_OK = FALSE;//Lay the moi
							//printf("Confirm\r\n");
							Counter1=0;
							Time_Reg_Tag = Counter1;
							Delay(50000);
							while(!Get_Id())// Ghi the User 1 
							{
								if(Counter1-Time_Reg_Tag >100000)
								{
									TIM2_Cmd(DISABLE);	//	
									goto __Exit;
								}						
							}
							Tag_Master = TRUE;
							for(i=0;i<5;i++)
							{
								if(Data_RFID[i] != Master[i])Tag_Master = FALSE;
							}
							if(Tag_Master==FALSE){goto __Exit;}
							
							Bell_Ring();//Ring bell		
							Bell_Ring();//Ring bel4l		
							
							Exit_Learning_tag = FALSE;							
							tem1 = FALSE;tem2 =FALSE;
							__Regis_Tag: 
							Get_Data_OK = FALSE;
							Get_RF433 = FALSE;
							Exit_Learning_tag = FALSE;
							Time_Reg_Tag = Counter1;
							while(!tem1 && !tem2 && ((Counter1-Time_Reg_Tag)<300000))
							{
								tem1 = Get_Id();
								tem2 = Get_Data_RF433();
							}
							if(tem1==TRUE)
							{
								Learing_Tag_RFID(0x4010);	//Learn Tag User 1
								Learing_Tag_RFID(0x4020);//Learn Tag User 2	
							}
							if(tem2==TRUE)
							{
								Leaning_RF433(0x4040);
								Leaning_RF433(0x4050);	
							}							
						}
	__Exit:		
						GPIO_Init(BUTTON_PORT, BUTTON_PIN, GPIO_MODE_IN_PU_NO_IT);
						TIM2_Cmd(DISABLE);
						RFID_OFF;
						SMSTATUS = ENG_ON_TAG;// ENG_ON_NO_TAG 
					}
					else
					{
						if(Receive_433())
						{
							SMSTATUS = ENG_ON_TAG;
							GPIO_Init(BUTTON_PORT, BUTTON_PIN, GPIO_MODE_IN_PU_NO_IT);
							RFID_OFF;
							TIM2_Cmd(DISABLE);
							State_433  = 0;
							Enable_Ring = FALSE;
							GPIO_WriteHigh(ENGINE_PORT, ENGINE_PIN);// On engine	
							GPIO_WriteLow(SIGNAL_PORT, SIGNAL_PIN);
							Bell_Ring();//Bell_Ring(1,10,1);//Ring bell	
						}					
					} //
					break;
				case ENG_ON_TAG://Bat may da quet the	
					if((!INPUT_1) && (Chan_bell == TRUE))
					{
						Delay(500);//100 000
						if(!INPUT_1)
						Sig_Left++;
					}
					if((!INPUT_2) && (Chan_bell == TRUE))
					{
						Delay(500);//100 000
						if(!INPUT_2)
						Sig_Right++;
					}
					if((Counter1<600000) && (Sig_Left>= 1) && (Sig_Right>=1) && (Chan_bell == TRUE))
					{
						Chan_bell = FALSE;
						if(On_Off_Bell == 0x00) On_Off_Bell = 0x01;
						else On_Off_Bell = 0x00;
						FLASH_Unlock(FLASH_MEMTYPE_DATA);
						IWDG_ReloadCounter();
						FLASH_ProgramByte(0x4030,On_Off_Bell);// Luu the vao bo nho		
						IWDG_ReloadCounter();	
						FLASH_Lock(FLASH_MEMTYPE_DATA);				
						for(i=0;i<On_Off_Bell+2;i++) Bell_Ring();//Ring bell		
						Sig_Left =0;Sig_Right=0;
					}
					
					/* turn off engine
					j = Process_433();
					if(j==2)
					{
						Time_Off = Counter1;
						Wait_RF=TRUE;
					}
					if(Wait_RF)
					{
						if(Counter1- Time_Off< 500000)
						{
							if(j==3) //
							{
								Wait_RF = FALSE;
								GPIO_WriteLow(ENGINE_PORT, ENGINE_PIN);
								SMSTATUS = ENG_OFF;
								Off_Tag = TRUE;
							}
						}
						else Wait_RF = FALSE;
					}// */				
					Time_Sleep = 0;
					Count2Sleep = 0;
					// bat tat coi o day ne.
					if((!INPUT_1 || ! INPUT_2) && (On_Off_Bell == 0x00))// || Have_Signal>60))
					GPIO_WriteHigh(BELL_PORT, BELL_PIN);// On  Bell
					else
					GPIO_WriteLow(BELL_PORT, BELL_PIN);// Off Bell	
// /*					
					break;
				case ENG_OFF_NO_TAG://Tat may chua quet the
					if(Process_Tag()) 
					{
						SMSTATUS = ENG_OFF; //ENG_OFF_NO_TAG
						GPIO_WriteLow(ENGINE_PORT, ENGINE_PIN);// On engine	
						GPIO_WriteLow(BELL_PORT, BELL_PIN);// Off Bell
						RFID_OFF;
						TIM2_Cmd(DISABLE);
					}
					if(Receive_433())//Process_433  . Receive_433()
					{
						SMSTATUS = ENG_OFF; //ENG_OFF_NO_TAG
						TIM2_Cmd(DISABLE);
						RFID_OFF;
						GPIO_Init(BUTTON_PORT, BUTTON_PIN, GPIO_MODE_IN_PU_NO_IT);
						State_433  = 0;
						Enable_Ring = FALSE;
						GPIO_WriteLow(ENGINE_PORT, ENGINE_PIN);// On engine	
						GPIO_WriteLow(SIGNAL_PORT, SIGNAL_PIN);
						Bell_Ring();//Bell_Ring(1,10,1);//Ring bell	
						Get_RF433=FALSE;
					}
					break;
				case ENG_OFF://Tat may da quet the
					if(Have_Accl)
					{
						Have_Accl = 0;
						Delay(40000);
						MMA7660_Read(TILT_REG);
						Counter1=0;
						Time_Reg_Tag = Counter1;
						while(Counter1 - Time_Reg_Tag<300000)
						{
							if(!ACC)
							{
								Delay(500);
								if(!ACC)
								{
									goto __Thoat;
								}
							}
							if(Have_Accl) break;
							Time_Wdt_Soft = 0;
							j = Process_433();
							if(j>0) goto __Thoat;
						}
						if(Time_ACCL_ON > 1000000 && ACCL_EN && Have_Accl)
						{
							for(i=0;i<2;i++)
							{
								GPIO_WriteHigh(BELL_PORT, BELL_PIN);// On  Bell											
								GPIO_WriteHigh(SIGNAL_PORT, SIGNAL_PIN);// On  Bell
								Delay(30000);
								GPIO_WriteLow(SIGNAL_PORT, SIGNAL_PIN);// Off Bell
								GPIO_WriteLow(BELL_PORT, BELL_PIN);// Off Bell
								Delay(30000);
							}
							Time_ACCL_ON = 0;
						}
						MMA7660_Read(TILT_REG);
					}
					j = Process_433();
__Thoat: 						
					if(j==2)//user tag 433
					{
						for(i=0;i<3;i++)
						{
							GPIO_WriteHigh(BELL_PORT, BELL_PIN);// On  Bell											
							GPIO_WriteHigh(SIGNAL_PORT, SIGNAL_PIN);// On  Bell
							Delay(30000);
							GPIO_WriteLow(SIGNAL_PORT, SIGNAL_PIN);// Off Bell
							GPIO_WriteLow(BELL_PORT, BELL_PIN);// Off Bell
							Delay(30000);
						}
						Get_RF433 = FALSE;
					}
					if(j==3)//bat tat coi bao dong nhe
					{
						FLASH_Unlock(FLASH_MEMTYPE_DATA);		
						if(ACCL_EN == 0)
						{
							ACCL_EN = 1;
						}
						else  ACCL_EN = 0;
						IWDG_ReloadCounter();
						FLASH_ProgramByte(0x406A,ACCL_EN);// Luu the vao bo nho		
						IWDG_ReloadCounter();	
						FLASH_Lock(FLASH_MEMTYPE_DATA);		//FLASH_Unlock(FLASH_MEMTYPE_DATA);
						for(i=0;i<(ACCL_EN+1);i++)	
						Bell_Ring();	
						Get_RF433 = FALSE;						
					}
					if(Time_ACCL_ON - Time_Check_Accl > 2000000 && ACCL_EN)
					{
						Time_Check_Accl = Time_ACCL_ON;
						Check_Dat = MMA7660_Read(TILT_REG);
						if(Check_Dat==0xFF||Check_Dat==0x00)
						{
							MMA7660_Init();						
						}
					}
					if(Time_ACCL_ON > 6000000)
					{
						Time_ACCL_ON = 1000000;
						Time_Check_Accl = 1000000;
					}
					break;
				default:break;
			}
			Check_SM_Status();// */
			 /*
			while(1)
			{
				// if(Get_Id()) printf("Have tag\r\n");
				
			
					if(Get_Data_RF433())
{					printf("Have RF433\r\n");
							for(i=0;i<2;i++)
							{
								GPIO_WriteHigh(BELL_PORT, BELL_PIN);// On  Bell											
								GPIO_WriteHigh(SIGNAL_PORT, SIGNAL_PIN);// On  Bell
								Delay(30000);
								GPIO_WriteLow(SIGNAL_PORT, SIGNAL_PIN);// Off Bell
								GPIO_WriteLow(BELL_PORT, BELL_PIN);// Off Bell
								Delay(30000);
							}

} 
			
				if(Get_RF433)
				{
					for(i=0;i<10;i++)
					printf("%d|",Data_RF[i]);
					printf("\r\n");
					Delay(100000);
					Get_RF433 = FALSE;
				} 
				
			}//*/	
		}
	}
PUTCHAR_PROTOTYPE
{
  /* Write a character to the UART1 */
  UART1_SendData8(c);
  /* Loop until the end of transmission */
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);

  return (c);
}

/**
  * @brief Retargets the C library scanf function to the USART.
  * @param None
  * @retval char Character to Read
  */
GETCHAR_PROTOTYPE
{
#ifdef _COSMIC_
  char c = 0;
#else
  int c = 0;
#endif
  /* Loop until the Read data register flag is SET */
  while (UART1_GetFlagStatus(UART1_FLAG_RXNE) == RESET);
    c = UART1_ReceiveData8();
  return (c);
}

void TIMER_Configuration(void)
{
	/* TIM2 Peripheral Configuration: TIM2 tao PWM 125KHZ tai PIN 1 PD4*/ 
  TIM2_DeInit();
	/* cai dat time 1ms, nghia la TIM4 se tao ra ngat sau moi 1mS */
  TIM4_DeInit();
	
  /* Set TIM2 Frequency to 125Khz=2.000.000/16 */ //16.000.000
  TIM2_TimeBaseInit(TIM2_PRESCALER_8, 15);
	TIM2_OC1Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE,CCR1_Val, TIM2_OCPOLARITY_LOW);
	//TIM2_OC2Init(TIM2_OCMODE_TIMING, TIM2_OUTPUTSTATE_ENABLE,CCR2_Val, TIM2_OCPOLARITY_HIGH);
  /* Time base configuration */
  TIM4_TimeBaseInit(TIM4_PRESCALER_16, 0x0A );// 16.000.000/16/10 = 100KHZ

  /* Enables TIM2 peripheral Preload register on ARR */
  TIM2_ARRPreloadConfig(ENABLE);
  /* Enable TIM4 IT UPDATE */
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
 
 /* Enable TIM2 */
  TIM2_Cmd(ENABLE);
  /* Enable TIM4 */
  TIM4_Cmd(ENABLE);
}
bool Check_SM_Status(void)
{
	char i;
	if(SMSTATUS == ENG_OFF)//Dang tat ma bat may len ne
	{
		if(!ACC)
		{
			Delay(500);//100 000
			if(!ACC)
			{
				if(Off_Tag == FALSE)
				RFID_ON;
				Off_Tag = FALSE;
				State_433 = 0;
				GPIO_WriteHigh(ACCL_EN_PORT, ACCL_EN_PIN);// tat nguon accl
				Count2Sleep = 0;
				Time_Sleep = 0;
				GPIO_Init(BUTTON_PORT, BUTTON_PIN, GPIO_MODE_IN_PU_IT);
				TIM2_Cmd(ENABLE);
				SMSTATUS = ENG_ON_NO_TAG;
				Get_Data_OK = FALSE;
				Counter1 = 0;
				Time_Ring= 0;
				Count_Ring = 0;
				Enable_Ring = TRUE;
				Sig_Left=0;
				Sig_Right=0;
				Chan_bell = TRUE;
				Wait_RF = FALSE;
				// Bell_Ring();
			}
		}		
	}
	if(SMSTATUS == ENG_ON_TAG)//Dang bat may da quet the ma tat may
	{
		if(ACC)
		{
			if(!Wdt_Reset)
			{
				for(i=0;i<50;i++)
				{
					if(!ACC)//mo khoa
					goto ExitCheck;
					Delay(2000);//100 000
				}
			}
			else
			Delay(500);//100 000
			if(ACC)
			{
				Wdt_Reset = TRUE;
				Count2Sleep = 0;
				Time_Sleep = 0;
				GPIO_Init(BUTTON_PORT, BUTTON_PIN, GPIO_MODE_IN_PU_NO_IT);
				TIM2_Cmd(DISABLE);
				//TIM2_Cmd(ENABLE);
				SMSTATUS = ENG_OFF;
				GPIO_WriteLow(ENGINE_PORT, ENGINE_PIN);// On  Bell
				Get_RF433 = FALSE;
				Enable_Ring = FALSE;
				//Bell_Ring();//Bell_Ring(1,1,2);
				if(ACCL_EN)
				{
					Time_ACCL_ON = 0;
					Time_Check_Accl = 0;
					GPIO_WriteLow(ACCL_EN_PORT, ACCL_EN_PIN);// bat nguon accl
					MMA7660_Init();
				}
				RFID_OFF;
			}
		}	
	}	
	if(SMSTATUS == ENG_OFF_NO_TAG)//dang tat may ko quet the ma bat may
	{
		if(!ACC)//bat may khi chua quet the
		{
			for(i=0;i<20;i++)
			{
				if(ACC)//tat khoa
				goto ExitCheck;
				Delay(2000);//100 000
			}	
			if(!ACC)
			{
				SMSTATUS = ENG_ON_NO_TAG;
			}
		}	
	}
	if(SMSTATUS == ENG_ON_NO_TAG)//Dang bat the ko quet the ma tat may
	{
		if(ACC)//bat may khi chua quet the
		{
			for(i=0;i<20;i++)
			{
				if(!ACC)//mo khoa
				goto ExitCheck;
				Delay(2000);//100 000
			}			
			if(ACC)
			{
				SMSTATUS = ENG_OFF_NO_TAG;
				GPIO_WriteLow(ENGINE_PORT, ENGINE_PIN);// On  Bell
			}
		}	
	}
ExitCheck:	/// Time_Signal = 0; //En_Signal
	if(Enable_Ring)
	{
		if(Counter1 - Time_Ring > 500000 && Counter1 - Time_Ring < 6000000)//Lon hon 5s chua co the dung
		{
			if(Have_Ring)
			{
				GPIO_WriteHigh(BELL_PORT, BELL_PIN);// On  Bell
				En_Signal = TRUE;
				Count_Ring++;
				Have_Ring = FALSE;
			}			
		}
		else
		{
			if(Count_Ring<3)
			{
				Have_Ring=TRUE;
			}
			else 
			{
				Have_Ring = FALSE;
			}
			GPIO_WriteLow(BELL_PORT, BELL_PIN);// Off Bell		
			En_Signal = FALSE;			
		}
		if(En_Signal)
		{
			if((Counter1 - Time_Signal) > 30000)
			{
				GPIO_WriteReverse(SIGNAL_PORT, SIGNAL_PIN);// On  Bell
				Time_Signal = Counter1;
			}
		}
		else
		GPIO_WriteLow(SIGNAL_PORT, SIGNAL_PIN);// Off Bell
		
		if(Counter1>7000000)//70s
		{
			Counter1=0;
			Time_Ring=0;
		}		
	}
	//if(Time_Sleep>60*60*100000)//1 hour
	if(Time_Sleep>60*60*100000)//1 hour
	{
		Count2Sleep++;
		Time_Sleep=0;
		if(Count2Sleep>80)//x48h Count2Sleep
		{
			GPIO_WriteLow(EN_PWR_PORT, EN_PWR_PIN); //khoi 
			//while(1);
		}
	}
	if(Time_ACCL_ON > 60*TIME_ACCL*150000)Time_ACCL_ON = TIME_ACCL*150000;
	return 0;
}
bool Get_Id (void)
{
	char i;
	if(Get_Data_OK)
	{
		//disableInterrupts();
		if(check_parity())
		{
			Data_2_Number();
			//printf("\r\n");
			/*for(i=0;i<55;i++)
			{
				printf("%d|",Buff_RFID[i]);
			}*/
			//printf("\r\nData:\r\n");	
			//for(i = 0;i<5;i++)
			//{
			//	printf("%d|",Data_RFID[i]);		
			//}
			//printf("\r\n");	
			//enableInterrupts();
			Get_Data_OK = FALSE;
			return TRUE;		
		}
		else
		{
			//printf("Loi parity\r\n");
			//enableInterrupts();
			Get_Data_OK = FALSE;
			return FALSE;
		}
	}
	else return FALSE;
}

int check_parity (void)//check parity data.
{
	char i=0,colParity1=0,colParity2= 0,colParity3 =0,colParity4=0,Total_Tag=0;
	char row_parity[11]={0,0,0,0,0,0,0,0,0,0,0};
	bool Wrong_Parity = 0;
	for(i=0;i<11;i++)
	{
		colParity1 ^= Buff_RFID[5*i];
		colParity2 ^= Buff_RFID[5*i+1];
		colParity3 ^= Buff_RFID[5*i+2];
		colParity4 ^= Buff_RFID[5*i+3];
		row_parity[i] = Buff_RFID[5*i] ^ Buff_RFID[5*i+1]^Buff_RFID[5*i+2]^Buff_RFID[5*i+3]^Buff_RFID[5*i+4];
	}
	for(i=0;i<50;i++) Total_Tag = Buff_RFID[i] + Total_Tag;
	if(Total_Tag==0x00) return 0;
	
	for(i=0;i<10;i++)
	{
		if(row_parity[i] != 0) Wrong_Parity = 1;
	}
  if(colParity1 || colParity2 ||   colParity3 || colParity4 || Wrong_Parity)
  {
     return 0;
  }
	else 
	{
		//logger("okie");
		return 1;
	}
}



bool Data_2_Number (void)//doi sang so cho de luu
{
	int i;
	for(i=0;i<5;i++) Data_RFID[i] =0x00;// xoa du lieu het
	for(i=0;i<5;i++)
	{	
		Data_RFID[i] <<= 1;
		if(Buff_RFID[10*i]) 	Data_RFID[i] |= (1<<0);
		else Data_RFID[i] &=~(1<<0);	
		
		Data_RFID[i] <<= 1;		
		if(Buff_RFID[10*i+1]) 	Data_RFID[i] |= (1<<0);	
		else Data_RFID[i] &=~(1<<0);
		
		Data_RFID[i] <<= 1;		
		if(Buff_RFID[10*i+2]) 	Data_RFID[i] |= (1<<0);	
		else Data_RFID[i] &=~(1<<0);	

		Data_RFID[i] <<= 1;
		if(Buff_RFID[10*i+3]) 	Data_RFID[i] |= (1<<0);	
		else Data_RFID[i] &=~(1<<0);	

		Data_RFID[i] <<= 1;
		if(Buff_RFID[10*i+5]) 	Data_RFID[i] |= (1<<0);	
		else Data_RFID[i] &=~(1<<0)	;

		Data_RFID[i] <<= 1;
		if(Buff_RFID[10*i+6]) 	Data_RFID[i] |= (1<<0);	
		else Data_RFID[i] &=~(1<<0);	

		Data_RFID[i] <<= 1;
		if(Buff_RFID[10*i+7]) 	Data_RFID[i] |= (1<<0);	
		else Data_RFID[i] &=~(1<<0)	;

		Data_RFID[i] <<= 1;
		if(Buff_RFID[10*i+8]) 	Data_RFID[i] |= (1<<0);	
		else Data_RFID[i] &=~(1<<0)	;
	}
return TRUE;
}

//void Bell_Ring(u8 on,u8 off,u8 time)
void Bell_Ring(void)
{

		GPIO_WriteHigh(BELL_PORT, BELL_PIN);// On  Bell
		Delay(10000);
		GPIO_WriteLow(BELL_PORT, BELL_PIN);// Off Bell
		Delay(100000);

}
u8 MMA7660_Init(void)
{
		u8 retry = 5;
		Have_Accl = 0;

		GPIO_WriteHigh(ACCL_EN_PORT, ACCL_EN_PIN);// tat nguon accl
		Delay(50000);
		GPIO_WriteLow(ACCL_EN_PORT, ACCL_EN_PIN);// bat nguon accl
		Delay(50000);
	while(retry--)
	{
		MMA7660_Write(MODE_REG, 0x00);


		MMA7660_Write(MODE_REG, 0x00);
		Check_Dat = MMA7660_Read(MODE_REG);
		if(Check_Dat != 0) continue;


		MMA7660_Write(SPCNT_REG, 0x00);
		Check_Dat = MMA7660_Read(SPCNT_REG);
		if(Check_Dat != 0) continue;


		MMA7660_Write(INTSU_REG, 0xe7);
		Check_Dat = MMA7660_Read(INTSU_REG);
		if(Check_Dat != 0xe7) continue;


		MMA7660_Write(PDET_REG, 0x01);
		Check_Dat = MMA7660_Read(PDET_REG);
		if(Check_Dat != 0x01) continue;


		MMA7660_Write(SR_REG, 0x01);
		Check_Dat = MMA7660_Read(SR_REG);
		if(Check_Dat != 0x01) continue;


		MMA7660_Write(PD_REG, 0x00);
		Check_Dat = MMA7660_Read(PD_REG);
		if(Check_Dat != 0x00) continue;


		MMA7660_Write(MODE_REG,0x01);
		Check_Dat = MMA7660_Read(MODE_REG);
		if(Check_Dat != 0x01) continue;


		Check_Dat = MMA7660_Read(TILT_REG);			
		Delay(50000);
		ACCL_EN = 1;
		Time_ACCL_ON = 0;
		Have_Accl = 0;
		return 1;
	}
	ACCL_EN = 0;
	return 0;
}
void MMA7660_Write(u8 Regis,u8 data)
{
	
	//void i2c_start (void)//void i2c_stop (void)//void i2c_write (uint8_t output_data)//uint8_t i2c_read (void)
	i2c_start();
	i2c_write(0x98);
	i2c_write(Regis);
	i2c_write(data);
	i2c_stop();
}

uint8_t MMA7660_Read(u8 Regis)
{
	uint8_t Data;
	i2c_start();
	i2c_write(0x98);
	i2c_write(Regis);
	i2c_start();
	i2c_write(0x99);
	Data = i2c_read();
	i2c_stop();
	return Data;
}
//------------------------------------------------------------------------------
// 	Routine:	i2c_start
//	Inputs:		none
//	Outputs:	none
//	Purpose:	Sends I2C Start Trasfer - State "B"
//------------------------------------------------------------------------------
void i2c_start (void)//void i2c_stop (void)//void i2c_write (uint8_t output_data)//uint8_t i2c_read (void)
{
	SDAOUT;
	//SDATA = HIGH;							// Set data line high
	SDA1;	
	Delay_MMA7660();	
	//SCLK = HIGH;							// Set clock line high
	SCL1;
	Delay_MMA7660();	
	//SDATA = LOW;							// Set data line low (START SIGNAL)
	SDA0;	
	Delay_MMA7660();	
	//SCLK = LOW;								// Set clock line low
	SCL0;
	Delay_MMA7660();	
}

//------------------------------------------------------------------------------
// 	Routine:	i2c_stop
//	Inputs:		none
//	Outputs:	none
//	Purpose:	Sends I2C Stop Trasfer - State "C"
//------------------------------------------------------------------------------
void i2c_stop (void)//void i2c_write (uint8_t output_data)//uint8_t i2c_read (void)
{
	unsigned char input_var;
	SDAOUT;
//	SCLK = LOW;								// Set clock line low
	SCL0;
	Delay_MMA7660();	
	//SDATA = LOW;							// Set data line low
	SDA0;
	Delay_MMA7660();	
	//SCLK = HIGH;							// Set clock line high
	SCL1;
	Delay_MMA7660();	
	//SDATA = HIGH;							// Set data line high (STOP SIGNAL)
	SDA1;
	Delay_MMA7660();	
	//input_var = SDATA;						// Put port pin into HiZ
}

//------------------------------------------------------------------------------
// 	Routine:	i2c_write
//	Inputs:		output byte
//	Outputs:	none
//	Purpose:	Writes data over the I2C bus
//------------------------------------------------------------------------------
void i2c_write (uint8_t output_data)//uint8_t i2c_read (void)
{
	uint8_t index;
	SDAOUT;	
	for(index=0;index<8;index++)
	{
		if(Databit[index]&output_data)SDA1;
		else SDA0;	
		Delay_MMA7660();
		SCL1;
		Delay_MMA7660();
		SCL0;	
	}	
	//index = SDATA;							// Put data pin into read mode
	SDAIN;
	//	SCLK = HIGH;   		        			// Clock the ACK from the I2C Bus
	SCL1;
	Delay_MMA7660();//DOC ACK O SAU DAY NE.
//	if(!SDA)
//	printf("Read ACk 3 \r\n");
//	else printf("No Read ACk 3\r\n");		
	//SCLK = LOW;		
	SCL0;// den day okie roi nhe.
	Delay_MMA7660();
}

//------------------------------------------------------------------------------
// 	Routine:	i2c_read
//	Inputs:		none
//	Outputs:	input byte
//	Purpose:	Reads data from the I2C bus
//------------------------------------------------------------------------------
uint8_t i2c_read (void)
{
	uint8_t index,Data_I2C;

//	index = SDATA;							// Put data pin into read mode
	SDAIN;
	Data_I2C = 0x00;
	for(index = 0; index < 8; index++)  	// Send 8 bits to the I2C Bus
	{
		Data_I2C <<= 1;					// Shift the byte by one bit
		//SCLK = HIGH;           				// Clock the data into the I2C Bus
    SCL1;
		Delay_MMA7660();  
		if(SDA)		Data_I2C++;
		// Data_I2C |= SDA; 		   		// Input the data from the I2C Bus
		//SCLK = LOW;		
		SCL0;
		Delay_MMA7660();
	}
   return Data_I2C;
}

void Delay_MMA7660(void)
{
	//u8 i =0,k=0;
	//for(i=0;i<100;i++)k++;
	Delay(2);
}
void Delay(u32 nTime)
{ 
	TimingDelay = 0;
	while(TimingDelay < nTime);
}
/**
  * @brief  Configures the IWDG to generate a Reset if it is not refreshed at the
  *         correct time. 
  * @param  None
  * @retval None
  */
void IWDG_Config(void)
{
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable(); 
  
  /* IWDG timeout equal to 214 ms (the timeout may varies due to LSI frequency
     dispersion) */
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  
  /* IWDG configuration: IWDG is clocked by LSI = 38KHz */
  IWDG_SetPrescaler(IWDG_Prescaler_256);
  
  /* IWDG timeout equal to 214.7 ms (the timeout may varies due to LSI frequency dispersion) */
  /* IWDG timeout = (RELOAD_VALUE + 1) * Prescaler / LSI 
                  = (254 + 1) * 256 / 38 000 
                  = 1.7s */
  IWDG_SetReload((uint8_t)254);
  
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
}
/**
  * @brief  Measure the LSI frequency using timer IC1 and update the calibration registers.
  * @note   It is recommended to use a timer clock frequency of at least 10MHz in order 
  *         to obtain a better in the LSI frequency measurement.
  * @param  None
  * @retval None
  */

void Leaning_RF433(uint16_t Address)
{
	u8 i=0,Tag_Ok = 1;
	Get_RF433 = FALSE;//Lay the moi
	
	Counter1=0;
	Time_Reg_Tag = Counter1;
	if(!Exit_Learning_tag)
	{ 
		while(!Get_Data_RF433())// Ghi the User 1 
		{
			if(Counter1-Time_Reg_Tag >500000)
			{
				Tag_Ok = 0;
				Exit_Learning_tag =TRUE;
				Bell_Ring();//Bell_Ring(1,1,2);
				break;
			}
			/*
			if(!ACC)//tat may
			{
				Delay(500);
				if(!ACC)
				{
					Tag_Ok = 0;
					Exit_Learning_tag = TRUE;
					//SMSTATUS == ENG_OFF
					break;
				}
			}*/
		}
	}
	else
	{
		Tag_Ok = 0;		
	}
	if(Tag_Ok)
	{
		FLASH_Unlock(FLASH_MEMTYPE_DATA);
		for(i=0;i<3;i++)
		{
			IWDG_ReloadCounter();
			FLASH_ProgramByte(Address+i, Data_433[i]);// Luu the vao bo nho		
			IWDG_ReloadCounter();
		}
		for(i=0;i<3;i++) User_433[0][i] = FLASH_ReadByte(0x4040+i);	
		for(i=0;i<3;i++) User_433[1][i] = FLASH_ReadByte(0x4050+i);	
		FLASH_Lock(FLASH_MEMTYPE_DATA);
		//printf("OK\r\n");
		//Bell_Ring(1,10,1);
		Bell_Ring();
	}	
}
bool Receive_433(void)
{
	u8 i=0;
	i = Process_433();
	if(i)
	{
		switch(State_433)
		{
			case 0:
				if(i == 2)State_433  = 1;
				break;
			case 1:
				State_433  = 0;
				if(i == 2)State_433  = 1;
				if(i == 3)State_433  = 2;
				break;
			case 2:
				State_433  = 0;
				if(i == 2)State_433  = 2;
				if(i == 3)State_433  = 3;							
				break;
			case 3:
				State_433  = 0;
				if(i == 2)State_433  = 3;
				if(i == 3)//tat coi bao dong. Tat chuong
				{
					State_433  = 0;
					return 1;
				}								
				break;
			default: break;
		}
	}
return 0;
}

void Learing_Tag_RFID(uint16_t Address)//address store data.
{
	u8 i=0,Tag_Ok = 1;
	Get_Data_OK = FALSE;//Lay the moi
	
	Counter1=0;
	Time_Reg_Tag = Counter1;
	if(!Exit_Learning_tag)
	{ 
		while(! Get_Id())// Ghi the User 1 
		{
			if(Counter1-Time_Reg_Tag >500000)
			{
				Tag_Ok = 0;
				Exit_Learning_tag = TRUE;
				//Bell_Ring(1,1,2);
				// TIM2_Cmd(DISABLE);	//ENABLE
				//printf("False\r\n");
				//while(1);
				//printf("Qua thoi gian hoc the\r\n");
				break;
			}
			if(ACC)//tat may
			{
				Delay(500);
				if(ACC)
				{
					Tag_Ok = 0;
					Exit_Learning_tag = TRUE;
					//SMSTATUS == ENG_OFF
					break;
				}
			}	
			Time_Wdt_Soft = 0;			
		}
	}
	else 
	{
		Tag_Ok = 0;
	}
	if(Tag_Ok)
	{
		FLASH_Unlock(FLASH_MEMTYPE_DATA);
		for(i=0;i<5;i++)
		{
			IWDG_ReloadCounter();
			FLASH_ProgramByte(Address+i, Data_RFID[i]);// Luu the vao bo nho		
			IWDG_ReloadCounter();
		}
		for(i = 0;i<5;i++)
		{
			Master[i] = FLASH_ReadByte(0x4000+i);
			User[0][i] = FLASH_ReadByte(0x4010+i);
			User[1][i] = FLASH_ReadByte(0x4020+i);
		}
		FLASH_Lock(FLASH_MEMTYPE_DATA);
		//printf("OK\r\n");
		//Bell_Ring();
		Bell_Ring();//Bell_Ring(1,10,1);
	}
}
bool Get_Data_RF433(void)
{
	u8 k = 0,i=0;
	if(Get_RF433)
	{
		//disableInterrupts();
		/*
		for(i=0;i<12;i++)
		{
			printf("%d|",(u16)Data_RF[i]);
		}
		printf("\r\n");
		*/
		for(i=0;i<3;i++)
		{
			Data_433[i]=0x00;
		}
		for(i=0;i<12;i++)
		{
			k=i/4;
			switch(Data_RF[i])
			{
				case 0://00
					Data_433[k] <<= 1;
					Data_433[k] &=~(1<<0);
					Data_433[k] <<= 1;
					Data_433[k] &=~(1<<0);							
					break;
				case 1://11
					Data_433[k] <<= 1;
					Data_433[k] |= (1<<0);
					Data_433[k] <<= 1;
					Data_433[k] |= (1<<0);							
					break;
				case 2://01
					Data_433[k] <<= 1;
					Data_433[k] &=~(1<<0);	
					Data_433[k] <<= 1;
					Data_433[k] |= (1<<0);							
					break;
				case 3://10	
					Data_433[k] <<= 1;
					Data_433[k] |= (1<<0);	
					Data_433[k] <<= 1;
					Data_433[k] &=~(1<<0);					
					break;
				case 4://11 Giai ma ra bao nhieu ko quan trong. Quan trong la Ma hoa tin hieu
					Data_433[k] <<= 1;
					Data_433[k] |= (1<<0);
					Data_433[k] <<= 1;
					Data_433[k] |= (1<<0);	
					break;
				default:break;
			}					
		}
		for(i=0;i<3;i++)
		{
		//	printf("%d\r\n",(u16)Data_433[i]);
		}
		Get_RF433 = FALSE;
		//enableInterrupts();
		return TRUE;
	}
return FALSE;
}
bool Process_Tag(void)
{
	u8 i,j;
	if(Get_Id())
	{
		for(i=0;i<5;i++)//check admin tag
		{
			Tag_Admin[i] = TRUE;
			for(j=0;j<5;j++) 
			{
				if(Data_RFID[j] != Admin[i][j]) Tag_Admin[i] = FALSE;
			}
		}
		Tag_Master = TRUE;
		Tag_User[0] = TRUE;
		Tag_User[1] = TRUE;
		for(j=0;j<5;j++)
		{
			if(Data_RFID[j] != Master[j]) Tag_Master = FALSE;
			if(Data_RFID[j] != User[0][j]) Tag_User[0] = FALSE;
			if(Data_RFID[j] != User[1][j]) Tag_User[1] = FALSE;
		}
		if(Tag_Admin[0]||Tag_Admin[1]||Tag_Admin[2]||Tag_Admin[3]||Tag_Admin[4]||Tag_Master||Tag_User[0]||Tag_User[1]) 
		{
			Enable_Ring = FALSE;	
			GPIO_WriteHigh(ENGINE_PORT, ENGINE_PIN);// On engine	
			GPIO_WriteLow(BELL_PORT, BELL_PIN);// Off Bell
			if(SMSTATUS == ENG_OFF_NO_TAG ) SMSTATUS = ENG_OFF; //ENG_OFF_NO_TAG
			if(SMSTATUS == ENG_ON_NO_TAG ) SMSTATUS = ENG_ON_TAG; //ENG_OFF_NO_TAG
			//Bell_Ring(1,10,1);//Ring bell	
			GPIO_WriteHigh(SIGNAL_PORT, SIGNAL_PIN);// On  Bell
			Delay(30000);
			GPIO_WriteLow(SIGNAL_PORT, SIGNAL_PIN);// Off Bell
			return 1; //admin tag
		}
		else return 0;
		return 0;
	}
	else
	return 0;
}
u8 Process_433(void)
{
	char i,j;
	if(Get_Data_RF433())
	{
		for(i=0;i<4;i++)
		{
			Tag_User_433[i] = TRUE;
			for(j=0;j<3;j++)
			{
				if(Data_433[j]!=User_433[i][j]) Tag_User_433[i] = FALSE;
			}
		}
		Tag_Admin_433 = TRUE;
		for(j=0;j<3;j++)
		{
			if(Data_433[j]!=Admin_433[j]) Tag_Admin_433 = FALSE;
		}
		if(Tag_Admin_433) return 1;
		for(i=0;i<4;i++)
		{
			if(Tag_User_433[i]) return i+2;
		}
		return 0;
	}	
}
/*bool Writer_Eeprom(u32 Adress, u8 Data[5],Number)
{
	char i;
	for(i=0;i<Number;i++)
	{
		FLASH_ProgramByte(Adress+i,Data[i]);	
	}
	return TRUE;
}*/
#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
