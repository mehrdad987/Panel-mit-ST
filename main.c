/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "stdio.h"
//#include "MY_FLASH.h"
#include "dwt_stm32_delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Water_Led_OFF()				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);					
#define Water_Led_ON()				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);			
#define Oil_Led_OFF()					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);						//lamp1	
#define Oil_Led_ON()					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);					//lamp1
#define Oil_Led_Toggle()			HAL_GPIO_TogglePin (GPIOE, GPIO_PIN_1);												//lamp1
#define No_Name_Led_OFF()			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);					
#define No_Name_Led_ON()			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);		
#define Buzzer_ON()						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);					
#define Buzzer_OFF()					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);	
#define Light_Level1_Low()		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);					
#define Light_Level1_High()		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);					
#define Light_Level2_Low()		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);					
#define Light_Level2_High()	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);					
#define Light_Level3_Low()		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);					
#define Light_Level3_High()		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);	
#define Auto_Stop_ON()				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);					
#define Auto_Stop_OFF()				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	
#define TC1_Type_J()					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);					
#define TC1_Type_K()					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
#define TC2_Type_J()					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);					
#define TC2_Type_K()					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
#define AL1_Led_OFF()					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);							//Gbox Oil	
#define AL1_Led_ON()					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);						//Gbox Oil
#define AL3_Led_ON()					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);							//water level
#define AL3_Led_OFF()					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);						//water level
#define Alt_Led_ON()					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);							//water level
#define Alt_Led_OFF()					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);						//water level

#define Batlow_Led_ON()				BatLed = 0x00000002;
#define Batlow_Led_OFF()			BatLed = 0x00000000;
#define Achar_Led_ON()				AcharLed = 0x00000001;
#define Achar_Led_OFF()				AcharLed = 0x00000000;
#define Spin_Led_ON()					SpinLed =  0x00000004;
#define Spin_Led_OFF()				SpinLed =  0x00000000;
#define AL2_ON()							AL2Led = 0x00000002;
#define AL2_OFF()							AL2Led = 0x00000000;
//**********************************	RTC Defines	********************************//
#define DS3231_ADDRESS 0xD0

//**********************************	Data Defines	*****************************//
#define Serial_Data_Set()			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
#define Serial_Data_Reset()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
#define Serial_Clk_Set()			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
#define Serial_Clk_Reset()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
#define Serial2_Data_Set()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
#define Serial2_Data_Reset()	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
#define Serial2_Clk_Set()			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
#define Serial2_Clk_Reset()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//**********************************	ADC Defines	*******************************//
#define ADC_Average						30 
#define Vref									3.23
#define Battery_Lower_Alarm_Limit		23
#define Battery_Upper_Alarm_Limit		34
//*******************************	eeprom Defines	*******************************//
#define Device_Address				0xA0
//*******************************	Buzzer Defines	*******************************//
#define Mute_Time							12000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
void SendToSegment5Digit(unsigned char SegSTR[5],unsigned char ROW);
void SendToSegment4Digit(unsigned char SegSTR[4],unsigned char ROW);
void SendToSegment5Digit2(unsigned char SegSTR[5],unsigned char ROW);
void SendToSegment4Digit2(unsigned char SegSTR[4],unsigned char ROW);
void Init(void);
void Panel_Switch_ON(void);
void Init_EEPROM(void);
void Get_Time(void);
void Setting_Time (uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year);

///////////////////////////  Global Variables  ////////////////////////////
//7segments Variables
uint32_t SEG_TABLE[4][10] = {0x80000718,0x00000108,0xC0000608,0xC0000308,0x40000118,0xC0000310,0xC0000710,0x80000108,0xC0000718,0xC0000118,
                             0x200C4060,0x00080020,0x30044020,0x30084020,0x10080060,0x30084040,0x300C4040,0x20080020,0x300C4060,0x30080060,
                             0x00139080,0x00008080,0x00033080,0x0002B080,0x0010A080,0x0012B000,0x0013B000,0x00009080,0x0013B080,0x0010B080,
                             0x07E00000,0x00C00000,0x0B600000,0x09E00000,0x0CC00000,0x0DA00000,0x0FA00000,0x00E00000,0x0FE00000,0x0CE00000};

uint32_t SegValue;
uint32_t num = 0;
uint8_t LoopControl = 0;
uint8_t Segment_Cnt = 0;
uint8_t Show_Segment_Cnt = 0;														 
uint16_t Contrast = 1500;			//contrast of 7segments
//General Flags
uint8_t Flag_50hz = 0;
uint8_t One_Second_Cnt = 0;
uint8_t TgasPturb1_Flag = 0;
uint8_t TgasPturb2_Flag = 0;
uint8_t TengPgbox_Flag = 0;
uint8_t TgasPturb11_Flag = 0;
uint8_t TgasPturb21_Flag = 0;
uint8_t TengPgbox1_Flag = 0;
//key Flags
uint8_t Down_B_Flag_R = 0;
uint8_t Down_B_Flag_L = 0;
uint8_t Up_B_Flag_R = 0;
uint8_t Up_B_Flag_L = 0;
uint8_t Mute_B_Flag_R = 0;
uint8_t Mute_B_Flag_L = 0;
uint8_t Buzzer_Mute_Flag = 0;
uint16_t Buzzer_Cnt = 0;

//key variables
unsigned long int BatLed = 0;
unsigned long int AcharLed = 0;
unsigned long int SpinLed = 0;
unsigned char ACHAR = 0;
unsigned long int AL2Led = 0;
unsigned char BAT_LOW = 0;
unsigned char SelKey1 = 0;
unsigned char SelKey2 = 0;
unsigned char SelKey3 = 0;
unsigned char SelKey4 = 0;
unsigned char SelKey5 = 0;
unsigned char SelKey6 = 0;

//Alarm Flags and limits
uint8_t Alarm1 = 0;
uint8_t Alarm2 = 0;
uint8_t Alarm3 = 0;
uint8_t Alarm4 = 0;
uint8_t Alarm5 = 0;
uint8_t Alarm6 = 0;
uint8_t Alarm7 = 0;
uint8_t Alarm77 = 0;
uint8_t Alarm8 = 0;
uint8_t Alarm9 = 0;
uint8_t Alarm10 = 0;
uint8_t Alarm11 = 0;
uint8_t Alarm12 = 0;
uint8_t Alarm13 = 0;
uint8_t Alarm14 = 0;
uint8_t Alarm15 = 0;
uint8_t Alarm88 = 0;
uint8_t New_Alarm = 0;
uint8_t Automatic_Stop_Flag = 0;	
uint16_t Alarm_Decide = 0;
float Oil_Press_Upper_Limit = 30;
float Oil_Press_Lower_Limit = 1.2;
float Oil_Gbox_Upper_Limit = 30;
float Oil_Gbox_Lower_Limit = 10.5;
float Gbox_Oil_Temp_Limit = 105;
float Water_Temp_Upper_Limit = 92;
float Water_Press_Lower_Limit = 30;
float TurboL_Press_Lower_Limit = 2;
float TurboR_Press_Lower_Limit = 2;
float AI2_Lower_Limit = 3;
float AI3_Lower_Limit = 3;
uint8_t Blinking = 0;							//0: Normal			1: Abnormal
uint16_t Blink_Cnt = 0;						// 0 ~ 400
uint16_t Auto_Stop_cnt = 0;
uint8_t Stop_Off = 0;

//LED Variables
uint8_t PLEDS_Cnt = 0;
//ADC Variables
uint16_t Adc_Value[10];  		//  adc buffer
float Water_Temp_F = 0;			//float variable
float Water_Temp = 0;	
char Water_Temp_C[5];				//Ascii variable
float A_Cof_Water_Temp = 1;	//A coeficient
float B_Cof_Water_Temp = 0;	//B coeficient
uint16_t Low_Press_Water_Cnt = 0;
float Oil_Pressure_F = 0;
float Oil_Pressure = 0;
char Oil_Pressure_C[5];
float A_Cof_Oil_Pressure = 1;
float B_Cof_Oil_Pressure = 0;
uint16_t Low_Press_Oil_Cnt = 0;
float Water_Pressure_F = 0;
float Water_Pressure = 0;
char Water_Pressure_C[5];
float A_Cof_Water_Pressure = 1;
float B_Cof_Water_Pressure = 0;
uint16_t High_temp_Water_Cnt = 0;
float Battery_Voltage_F = 0;
float Battery_Voltage = 0;
unsigned char Battery_Voltage_C[5];
float A_Cof_Battery_Voltage = 10.0906;		//10.344;
float B_Cof_Battery_Voltage = -0.1034;
uint16_t Low_Bat = 0;
uint8_t Low_Bat_Cnt = 0;
float Gbox_Oil_Pressure_F = 0;
float Gbox_Oil_Pressure = 0;
char Gbox_Oil_Pressure_C[5];
float A_Cof_Gbox_Oil_Pressure = 1;
float B_Cof_Gbox_Oil_Pressure = 0;
float Reserve_Voltage_F = 0;
float Reserve_Voltage = 0;
float A_Cof_Reserve_Voltage = 1;
float B_Cof_Reserve_Voltage = 0;
float AI2_F = 0;
float AI2 = 0;
char AI2_C[5];
float A_Cof_AI2 = 1;
float B_Cof_AI2 = 0;
float AI3_F = 0;
float AI3 = 0;
char AI3_C[5];
float A_Cof_AI3 = 1;
float B_Cof_AI3 = 0;
float TC2_F = 0;
char TC2_C[5];
float A_Cof_TC2 = 1;
float B_Cof_TC2 = 0;
float TC1_F = 0;
char TC1_C[5];
float A_Cof_TC1 = 1;
float B_Cof_TC1 = 0;

uint16_t Row1_F;
float Row2_F;
float Row3_F;
float Row4_F;
float Row5_F;
float Row6_F;
unsigned char Row1_C[5];
unsigned char Row2_C[5];
unsigned char Row3_C[5];
unsigned char Row4_C[5];
unsigned char Row5_C[5];
unsigned char Row6_C[5];
uint8_t Row1_Select = 0;
uint8_t Row2_Select = 0;
uint8_t Row3_Select = 0;
uint8_t Row4_Select = 0;
uint8_t Row5_Select = 0;
uint8_t Row6_Select = 0;
uint8_t ADC_Cnt = 0;

uint8_t data_valid = 0;
//RPM Variables
uint32_t RPM = 0;
unsigned char RPM_C[5];
uint8_t RPM_Cnt = 0;
uint16_t HRS = 0;
unsigned char HRS_C[5];
uint16_t Maint = 100;
unsigned char Maint_C[5];
uint16_t Maint_Time = 100;
uint16_t Maint_Counter_Reset;
uint8_t Maint_L = 100;
uint8_t Maint_H = 0x00;
uint16_t Interval = 0;
uint8_t Interval_L = 50;
uint8_t Interval_H = 0x00;
uint16_t Interval_Time = 50;
uint16_t Interval_Counter_Reset;
unsigned char Interval_C[5];
//Timer Variables
uint8_t Cnt_4second = 0; 
uint8_t Flag_4second = 0;
//Time Variables
uint16_t Second = 0;
uint8_t Min = 0;
uint8_t HRS_L;
uint8_t HRS_H;
//eeprom Variables
uint8_t EEPROM_Var[50];
uint8_t Water_Temp_Threshold1 = 80;
uint8_t Water_Temp_Threshold2 = 90;
uint16_t rit = 10;
uint8_t Low_Bat_Threshold = 11;
float Oil_Press_Threshold1 = 1;
float Oil_Press_Threshold2 = 0.8;
uint8_t TC1_Type = 0;							//0:type k			1:type j
uint8_t TC2_Type = 0;							//0:type k			1:type j
uint16_t Oil_Change_Interval_Time = 1000;
uint8_t Light_Intensity = 3;			//3:high				2:mid			1:low
float RPM_Cof = 7;
typedef union {
   float f;
   char bytes[4];
} my_union;
my_union Oil1;
my_union Oil2;

uint16_t Time_Flash_Read[2];
uint16_t Time_Flash_Write[2];
//usart1 variables
uint8_t Rx1_Data[50];
uint8_t Tx1_Data[50];
uint8_t Pc_Data_Ready = 0;
//switch on variables
uint16_t Pre_Start_Delay = 1000;

//RTC Variables
uint8_t SECOND = 0x00;
uint8_t MINUTE = 0x00;
uint8_t HOUR = 0x00;
uint8_t DAY = 0x00;
uint8_t DATE = 0x00;
uint8_t MONTH = 0x00;
uint8_t YEAR = 0x00;
uint8_t Set_Time = 0;

typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour;
	uint8_t dayofweek;
	uint8_t dayofmonth;
	uint8_t month;
	uint8_t year;
} TIME;

TIME time;

//logger
uint8_t Logger[32];
uint16_t Send_Ready = 0;
uint8_t Logger_Cnt = 0;

///////////////////////////// Coefficient //////////////////////////////
//1
float Oil_A1=101.84;				float Oil_A2=17.56;		float Oil_A3=16.87;		float Oil_A4=13.43;		float Oil_A5=13.80;
float Oil_B1=-85.67;				float Oil_B2=-7.21;		float Oil_B3=-6.59;		float Oil_B4=-3.62;		float Oil_B5=-3.90;
float Oil_A6=13.00;					float Oil_A7=11.23;		float Oil_A8=9.60;		float Oil_A9=9.30;		float Oil_A10=10.55;	float Oil_A11=4.01;	float Oil_A12=11.47;
float Oil_B6=-3.34;					float Oil_B7=-2.23;		float Oil_B8=-1.40;		float Oil_B9=-1.28;		float Oil_B10=-1.65;	float Oil_B11=-0.25;float Oil_B12=-0.54;

float Gb_A1=88.26;				float Gb_A2=80.31;		float Gb_A3=43.06;		float Gb_A4=63.66;		float Gb_A5=37.50;	
float Gb_B1=-58.85;				float Gb_B2=-51.38;		float Gb_B3=-16.08;		float Gb_B4=-35.48;		float Gb_B5=-12.44;	
float Gb_A6=51.24;				float Gb_A7=38.99;		float Gb_A8=27.87;		float Gb_A9=26.35;		float Gb_A10=23.40;		float Gb_A11=17.54;	float Gb_A12=11.47;	
float Gb_B6=-23.99;				float Gb_B7=-14.24;		float Gb_B8=-6.41;		float Gb_B9=-5.47;		float Gb_B10=-4.05;		float Gb_B11=-2.00;	float Gb_B12=-0.54;	


float AI2_A1=32.27;					float AI2_A2=10.22;		float AI2_A3=7.10;		float AI2_A4=6.69;		float AI2_A5=6.27;
float AI2_B1=-23.15;				float AI2_B2=-4.02;		float AI2_B3=-1.52;		float AI2_B4=-1.25;		float AI2_B5=-1.02;
float AI2_A6=7.16;					float AI2_A7=2.46;		float AI2_A8=16.91;		float AI2_A9=6.88;		
float AI2_B6=-1.42;					float AI2_B7=0.24;		float AI2_B8=-2.68;		float AI2_B9=-0.77;		
		
float AI3_A1=32.27;					float AI3_A2=10.22;		float AI3_A3=7.10;		float AI3_A4=6.69;		float AI3_A5=6.27;
float AI3_B1=-23.15;				float AI3_B2=-4.02;		float AI3_B3=-1.52;		float AI3_B4=-1.25;		float AI3_B5=-1.02;
float AI3_A6=7.16;					float AI3_A7=2.46;		float AI3_A8=16.91;		float AI3_A9=6.88;		
float AI3_B6=-1.42;					float AI3_B7=0.24;		float AI3_B8=-2.68;		float AI3_B9=-0.77;		

float WTemp_A = 125;	
float WTemp_B = -361.25;

float OTemp_A = 125;	
float OTemp_B = -361.25;

//2
float Oil2_A1=3.929;		float Oil2_A2=4.64;		float Oil2_A3=6.12;		float Oil2_A4=9.22;		float Oil2_A5=13.7;
float Oil2_B1=0;				float Oil2_B2=0.36;		float Oil2_B3=1.75;		float Oil2_B4=5.68;		float Oil2_B5=12.33;

float Gb2_A1=3.929;		float Gb2_A2=4.64;		float Gb2_A3=6.12;		float Gb2_A4=9.22;		float Gb2_A5=13.7;
float Gb2_B1=0;				float Gb2_B2=0.36;		float Gb2_B3=1.75;		float Gb2_B4=5.68;		float Gb2_B5=12.33;
	
float AI2_2A1=3.929;		float AI2_2A2=4.64;		float AI2_2A3=6.12;		float AI2_2A4=9.22;		float AI2_2A5=13.7;
float AI2_2B1=0;				float AI2_2B2=0.36;		float AI2_2B3=1.75;		float AI2_2B4=5.68;		float AI2_2B5=12.33;
		
float AI3_2A1=3.929;		float AI3_2A2=4.64;		float AI3_2A3=6.12;		float AI3_2A4=9.22;		float AI3_2A5=13.7;
float AI3_2B1=0;				float AI3_2B2=0.36;		float AI3_2B3=1.75;		float AI3_2B4=5.68;		float AI3_2B5=12.33;
	
float WTemp1_A = -20.75;	float WTemp2_A = -19.20;		float WTemp3_A = -35.57;	float WTemp4_A = -38.26;			float WTemp5_A = -64.19;	float WTemp6_A = -102.4;	float WTemp7_A = -190.71;
float WTemp1_B = 59.219;	float WTemp2_B = 55.53;			float WTemp3_B = 85.83;		float WTemp4_B = 89.30;				float WTemp5_B = 109.15;	float WTemp6_B = 126.50;	float WTemp7_B = 149.36;	

float OTemp1_A = -20.75;		float OTemp2_A = -19.20;	float OTemp3_A = -35.57;	float OTemp4_A = -38.26;	float OTemp5_A = -64.19;	float OTemp6_A = -102.4;	float OTemp7_A = -190.71;	//float OTemp8_A = -230.89;
float OTemp1_B = 59.219;			float OTemp2_B = 55.53;		float OTemp3_B = 85.83;	float OTemp4_B = 89.30;		float OTemp5_B = 109.15;	float OTemp6_B = 126.50;	float OTemp7_B = 149.36;	//float OTemp8_B = 178.10;


//float OTemp2_A = 181.81;	
//float OTemp2_B = -556.36;	

//decide
uint8_t Oil_Select = 0;
uint8_t Gb_Select = 0;
uint8_t WTemp_Select = 0;
uint8_t OTemp_Select = 0;
uint8_t TurboL_Select = 0;
uint8_t TurboR_Select = 0;			
////////////////////////////////////////////////////////////////////////
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
///////////// timer1 Interrupt //////////////////// 133.333 Hz///////
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if(htim->Instance == TIM2)
		{
			Flag_50hz = 1;
			Send_Ready++;
			if(Send_Ready == 133)
				{
					Send_Ready = 0;
					//-----------------------------------  Logger			 ----------------------------------------------------//
			Logger[0] = SECOND;
			Logger[1] = MINUTE;
			Logger[2] = HOUR;
			Logger[3] = DAY;
			Logger[4] = DATE;
			Logger[5] = MONTH;
			Logger[6] = YEAR;
			Logger[7] = Adc_Value[0];
			Logger[8] = Adc_Value[0] >> 8;
			Logger[9] = Adc_Value[1];
			Logger[10] = Adc_Value[1] >> 8;
			Logger[11] = Adc_Value[2];
			Logger[12] = Adc_Value[2] >> 8;
			Logger[13] = Adc_Value[3];
			Logger[14] = Adc_Value[3] >> 8;
			Logger[15] = Adc_Value[4];
			Logger[16] = Adc_Value[4] >> 8;
			Logger[17] = Adc_Value[5];
			Logger[18] = Adc_Value[5] >> 8;
			Logger[19] = Adc_Value[6];
			Logger[20] = Adc_Value[6] >> 8;
			Logger[21] = Adc_Value[7];
			Logger[22] = Adc_Value[7] >> 8;
			Logger[23] = Adc_Value[8];
			Logger[24] = Adc_Value[8] >> 8;
			Logger[25] = Adc_Value[9];
			Logger[26] = Adc_Value[9] >> 8;
			Logger[27] = RPM;
			Logger[28] = RPM >> 8;
			Logger[29] = Logger_Cnt;
			Logger[30] = 0xEB; 
			Logger[31] = 0x90;
	
					HAL_UART_Transmit(&huart3,Logger, 32, 100 );
					Logger_Cnt++;
				}
			if(RPM >= 100)
			{
				Second++;				
				if(Second == 8000)		// 1 min
				{	
					Second = 0;
					Min++;
					HAL_I2C_Mem_Write(&hi2c1,Device_Address,47,0xFF,&Min,1,10);				
					if(Min >= 60)
					{
						HRS_L++;
						if(HRS_L == 0) HRS_H++;
						Min = 0;		
						HAL_I2C_Mem_Write(&hi2c1,Device_Address,48,0xFF,&HRS_L,1,100);	
						HAL_I2C_Mem_Write(&hi2c1,Device_Address,49,0xFF,&HRS_H,1,100);	
						HRS = ((HRS_H<<8) & 0xFF00) | HRS_L;
					
						if(Maint != 0x00)
						{
							Maint_L--;
							HAL_I2C_Mem_Write(&hi2c1,Device_Address,45,0xFF,&Maint_L,1,100);	
							Maint =  Maint_L;
						}
					
						if(Interval != 0x00)
						{
							Interval_L--;
							HAL_I2C_Mem_Write(&hi2c1,Device_Address,42,0xFF,&Interval_L,1,100);	
							Interval = Interval_L;
						}
					}
				}
			}			
		}
}

///////////// USART Interrupt ////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{	
	/////////////  USART1 Interrupt ////////////////////
	if(huart->Instance == USART1)
	{
		Pc_Data_Ready = 1;
		HAL_UART_Receive_IT(&huart1,Rx1_Data,50);	
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
   /* Unlock the Flash Program Erase controller */
  //FLASH_Unlock();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
	//----------------------------------- RTC Section -----------------------
					
  //------------------------------------------------------------------------------
	Init();
	Init_EEPROM();
	Panel_Switch_ON();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(Pc_Data_Ready == 1)
		{
			Pc_Data_Ready = 0;
			HAL_I2C_Mem_Write(&hi2c1,Device_Address,0,0xFF,Rx1_Data,32,10);				//50byte old ebteda ba 50prg mishavad sepas fh 32 prg mishavad
			HAL_Delay(1000);
		}
		else if(Flag_50hz == 1)
		{
			Flag_50hz = 0;
			Get_Time();
			//------------------------------  Read Sensor Value From ADC_DMA ------------------------------------//
			ADC_Cnt++;
			Water_Temp_F			  +=( (Adc_Value[0]*Vref)/4095 * A_Cof_Water_Temp      + B_Cof_Water_Temp);
			Oil_Pressure_F      +=( (Adc_Value[1]*Vref)/4095 * A_Cof_Oil_Pressure    + B_Cof_Oil_Pressure);
			Water_Pressure_F    +=( (Adc_Value[2]*Vref)/4095 * A_Cof_Water_Pressure  + B_Cof_Water_Pressure);
			Battery_Voltage_F   +=( (Adc_Value[3]*Vref)/4095 * A_Cof_Battery_Voltage + B_Cof_Battery_Voltage);
			Reserve_Voltage_F   +=( (Adc_Value[4]*Vref)/4095 * A_Cof_Reserve_Voltage + B_Cof_Reserve_Voltage);
			Gbox_Oil_Pressure_F +=( (Adc_Value[5]*Vref)/4095 * A_Cof_Gbox_Oil_Pressure + B_Cof_Gbox_Oil_Pressure);
			AI2_F 							+=( (Adc_Value[6]*Vref)/4095 * A_Cof_AI2 + B_Cof_AI2);		//right turbine
			AI3_F 							+=( (Adc_Value[7]*Vref)/4095 * A_Cof_AI3 + B_Cof_AI3);		//left turbine
			TC2_F								+=( (Adc_Value[8]*Vref)/4095 * A_Cof_TC2 + B_Cof_TC2);
			TC1_F								+=( (Adc_Value[9]*Vref)/4095 * A_Cof_TC1 + B_Cof_TC1);
			
			//-----------------------------------  RPM Value ------------------------------------------------------//
			RPM_Cnt++;
			
			if(RPM_Cnt == 67)		// 133.33/2 = 0.5second counting
			{
				RPM_Cnt = 0;
				RPM = __HAL_TIM_GET_COUNTER(&htim1) * 1.6 *3.13 ;//RPM_Cof;			//	/6
				if(RPM >= 9999 ) RPM = 9999;
				__HAL_TIM_SET_COUNTER(&htim1, 0);		//Reset counter value
			}		
			
			//-----------------------------------  ADC Average ----------------------------------------------------//
			if(ADC_Cnt == ADC_Average)
			{
				ADC_Cnt = 0;
				
				
				
				//------------------	Oil_Pressure ------------------//
				if(Oil_Select == 0)
				{
					
				Oil_Pressure_F = Oil_Pressure_F / ADC_Average;		
						if( Oil_Pressure_F < 1.5 )
					{
						Oil_Pressure_F = 8.582 * Oil_Pressure_F * Oil_Pressure_F + 3.205 * Oil_Pressure_F + 0.005 ; 
						if(Oil_Pressure_F < 0.4) Oil_Pressure_F = 0;
					}
					
//				if((Oil_Pressure_F > 0.930  && Oil_Pressure_F <= 1.5 )) 
//					Oil_Pressure_F = Oil_Pressure_F * Oil_A1 + Oil_B1;
//				else if((Oil_Pressure_F > 0.897 && Oil_Pressure_F <= 0.930 ))
//					Oil_Pressure_F = Oil_Pressure_F * Oil_A2 + Oil_B2;
//				else if((Oil_Pressure_F > 0.864 && Oil_Pressure_F <= 0.897 ))
//					Oil_Pressure_F = Oil_Pressure_F * Oil_A3 + Oil_B3;
//				else if((Oil_Pressure_F > 0.786 && Oil_Pressure_F <= 0.864 ))
//					Oil_Pressure_F = Oil_Pressure_F * Oil_A4 + Oil_B4;
//				else if((Oil_Pressure_F > 0.707 && Oil_Pressure_F <= 0.786 ))
//					Oil_Pressure_F = Oil_Pressure_F * Oil_A5 + Oil_B5;
//				else if((Oil_Pressure_F > 0.625 && Oil_Pressure_F <= 0.707 ))
//					Oil_Pressure_F = Oil_Pressure_F * Oil_A6 + Oil_B6;
//				else if((Oil_Pressure_F > 0.507 && Oil_Pressure_F <= 0.625 ))
//					Oil_Pressure_F = Oil_Pressure_F * Oil_A7 + Oil_B7;
//				else if((Oil_Pressure_F > 0.409 && Oil_Pressure_F <= 0.507 ))
//					Oil_Pressure_F = Oil_Pressure_F * Oil_A8 + Oil_B8;
//				else if((Oil_Pressure_F > 0.293 && Oil_Pressure_F <= 0.409 ))
//					Oil_Pressure_F = Oil_Pressure_F * Oil_A9 + Oil_B9;
//				else if((Oil_Pressure_F > 0.213 && Oil_Pressure_F <= 0.293 ))
//					Oil_Pressure_F = Oil_Pressure_F * Oil_A10 + Oil_B10;
//				else if((Oil_Pressure_F > 0 && Oil_Pressure_F <= 0.213 ))
//					Oil_Pressure_F = Oil_Pressure_F * Oil_A11 + Oil_B11;
				else 
					Oil_Pressure_F = 10000;				//float
			}
				else
				{
					Oil_Pressure_F = Oil_Pressure_F / ADC_Average;			
				if(Oil_Pressure_F <= 0.509) 
					Oil_Pressure_F = Oil_Pressure_F * Oil2_A1;
				else if((Oil_Pressure_F > 0.509 && Oil_Pressure_F <= 0.940 ))
					Oil_Pressure_F = Oil_Pressure_F * Oil2_A2 - Oil2_B2;
				else if((Oil_Pressure_F > 0.940 && Oil_Pressure_F <= 1.267 ))
					Oil_Pressure_F = Oil_Pressure_F * Oil2_A3 - Oil2_B3;
				else if((Oil_Pressure_F > 1.267 && Oil_Pressure_F <= 1.484 ))
					Oil_Pressure_F = Oil_Pressure_F * Oil2_A4 - Oil2_B4;
				else if((Oil_Pressure_F > 1.484 && Oil_Pressure_F <= 2.2 ))																																				//if(Oil_Pressure_F > 1.484 )
					Oil_Pressure_F = Oil_Pressure_F * Oil2_A5 - Oil2_B5;
				else
					Oil_Pressure_F = 10000;				//float
				}
				Oil_Pressure = Oil_Pressure_F;
				
				//------------------	Fuel Level ------------------//	
				Water_Pressure_F = Water_Pressure_F / ADC_Average;	
				if(Water_Pressure_F < 0.5)
					Water_Pressure_F = 10000;
				else
					Water_Pressure_F = Water_Pressure_F * 61.34 ;									//ful tank%
				Water_Pressure = Water_Pressure_F;
				
				
				//------------------	Battery_Voltage ------------------//
				Battery_Voltage_F = Battery_Voltage_F / ADC_Average;
				Battery_Voltage = Battery_Voltage_F;
				
				//-------------	water temperature ------------------//
				Water_Temp_F = Water_Temp_F / ADC_Average;
				if(Water_Temp_F <=2.5  && Water_Temp_F > 2.37 )
						Water_Temp_F = Water_Temp_F * WTemp1_A + WTemp1_B;
				else if(Water_Temp_F <= 2.37 && Water_Temp_F > 1.85 )
						Water_Temp_F = Water_Temp_F * WTemp2_A + WTemp2_B;
				else if(Water_Temp_F <= 1.85 && Water_Temp_F > 1.28 )
						Water_Temp_F = Water_Temp_F * WTemp3_A + WTemp3_B;
				else if(Water_Temp_F <= 1.28 && Water_Temp_F > 0.76 )
						Water_Temp_F = Water_Temp_F * WTemp4_A + WTemp4_B;
				else if(Water_Temp_F <= 0.76 && Water_Temp_F > 0.45 )
						Water_Temp_F = Water_Temp_F * WTemp5_A + WTemp5_B;
				else if(Water_Temp_F <= 0.45 && Water_Temp_F > 0.25 )
						Water_Temp_F = Water_Temp_F * WTemp6_A + WTemp6_B;
				else if(Water_Temp_F <= 0.25 && Water_Temp_F > 0.01 )
						Water_Temp_F = Water_Temp_F * WTemp7_A + WTemp7_B;
				else
					Water_Temp_F = 10000;
				Water_Temp = Water_Temp_F;
				
				//------------------	Oil Temperature ------------------//
				Reserve_Voltage_F = Reserve_Voltage_F / ADC_Average;
				if(OTemp_Select == 0)
				{
				if(Reserve_Voltage_F <= 2.5 && Reserve_Voltage_F > 2.37)
					Reserve_Voltage_F = Reserve_Voltage_F * OTemp1_A + OTemp1_B;
				else if(Reserve_Voltage_F <= 2.37 && Reserve_Voltage_F > 1.85)
					Reserve_Voltage_F = Reserve_Voltage_F * OTemp2_A + OTemp2_B;
				else if(Reserve_Voltage_F <= 1.85 && Reserve_Voltage_F > 1.28)
					Reserve_Voltage_F = Reserve_Voltage_F * OTemp3_A + OTemp3_B;
				else if(Reserve_Voltage_F <= 1.28 && Reserve_Voltage_F > 0.76)
					Reserve_Voltage_F = Reserve_Voltage_F * OTemp4_A + OTemp4_B;
				else if(Reserve_Voltage_F <= 0.76 && Reserve_Voltage_F > 0.45)
					Reserve_Voltage_F = Reserve_Voltage_F * OTemp5_A + OTemp5_B;
				else if(Reserve_Voltage_F <= 0.45 && Reserve_Voltage_F > 0.25)
					Reserve_Voltage_F = Reserve_Voltage_F * OTemp6_A + OTemp6_B;
				else if(Reserve_Voltage_F <= 0.25 && Reserve_Voltage_F > 0.01 )
						Reserve_Voltage_F = Reserve_Voltage_F * OTemp7_A + OTemp7_B;
				else
					Reserve_Voltage_F = 10000;				//float sensor
				}
				else
				{
					if(Reserve_Voltage_F <= 2.44)
					Reserve_Voltage_F = Reserve_Voltage_F * -OTemp2_A + OTemp2_B;
				else
					Reserve_Voltage_F = 10000;				//float sensor
				}
				
				Reserve_Voltage = Reserve_Voltage_F;
				
				//---------------------	Gbox_Oil_Pressure ---------------------//
				if(Gb_Select == 0)
				{
					
				Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F / ADC_Average;
					if( Gbox_Oil_Pressure_F <= 1.5)
					{
						Gbox_Oil_Pressure_F = 16.54 * (Gbox_Oil_Pressure_F * Gbox_Oil_Pressure_F) + 11.2 * Gbox_Oil_Pressure_F - 0.494 ;
						if( Gbox_Oil_Pressure_F <= 0.4 ) Gbox_Oil_Pressure_F = 0; 
					}
//				if(Gbox_Oil_Pressure_F > 0.938  && Gbox_Oil_Pressure_F <= 1.4 )
//					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F * Gb_A1 + Gb_B1;
//				else if((Gbox_Oil_Pressure_F > 0.928 && Gbox_Oil_Pressure_F <= 0.938 ))
//					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F * Gb_A2 + Gb_B2;
//				else if((Gbox_Oil_Pressure_F > 0.906 && Gbox_Oil_Pressure_F <= 0.928 ))
//					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F * Gb_A3 + Gb_B3;
//				else if((Gbox_Oil_Pressure_F > 0.880 && Gbox_Oil_Pressure_F <= 0.906 ))
//					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F * Gb_A4 + Gb_B4;
//				else if((Gbox_Oil_Pressure_F > 0.839 && Gbox_Oil_Pressure_F <= 0.880 )) 																																				//if(Gbox_Oil_Pressure_F > 1.484 )
//					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F * Gb_A5  + Gb_B5;
//				else if((Gbox_Oil_Pressure_F > 0.795 && Gbox_Oil_Pressure_F <= 0.839 )) 																																				//if(Gbox_Oil_Pressure_F > 1.484 )
//					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F * Gb_A6  + Gb_B6;
//				else if((Gbox_Oil_Pressure_F > 0.703 && Gbox_Oil_Pressure_F <= 0.795 )) 																																				//if(Gbox_Oil_Pressure_F > 1.484 )
//					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F * Gb_A7  + Gb_B7;
//				else if((Gbox_Oil_Pressure_F > 0.621 && Gbox_Oil_Pressure_F <= 0.703 )) 																																				//if(Gbox_Oil_Pressure_F > 1.484 )
//					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F * Gb_A8  + Gb_B8;
//				else if((Gbox_Oil_Pressure_F > 0.478 && Gbox_Oil_Pressure_F <= 0.621 )) 																																				//if(Gbox_Oil_Pressure_F > 1.484 )
//					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F * Gb_A9  + Gb_B9;
//				else if((Gbox_Oil_Pressure_F > 0.350 && Gbox_Oil_Pressure_F <= 0.478 )) 																																				//if(Gbox_Oil_Pressure_F > 1.484 )
//					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F * Gb_A10  + Gb_B10;
//				else if((Gbox_Oil_Pressure_F > 0.239 && Gbox_Oil_Pressure_F <= 0.350 )) 																																				//if(Gbox_Oil_Pressure_F > 1.484 )
//					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F * Gb_A11  + Gb_B11;
//				else if((Gbox_Oil_Pressure_F > 0 && Gbox_Oil_Pressure_F <= 0.239 )) 																																				//if(Gbox_Oil_Pressure_F > 1.484 )
//					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F * Gb_A12  + Gb_B12;
				//else if(Gbox_Oil_Pressure_F > 1.5)
				else
					Gbox_Oil_Pressure_F = 10000;
			}
				else
				{
					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F / ADC_Average;
				if(Gbox_Oil_Pressure_F <= 0.509) 
					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F * Gb2_A1;
				else if((Gbox_Oil_Pressure_F > 0.509 && Gbox_Oil_Pressure_F <= 0.940 ))
					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F * Gb2_A2 - Gb2_B2;
				else if((Gbox_Oil_Pressure_F > 0.940 && Gbox_Oil_Pressure_F <= 1.267 ))
					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F * Gb2_A3 - Gb2_B3;
				else if((Gbox_Oil_Pressure_F > 1.267 && Gbox_Oil_Pressure_F <= 1.484 ))
					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F * Gb2_A4 - Gb2_B4;
				else if((Gbox_Oil_Pressure_F > 1.484 && Gbox_Oil_Pressure_F <= 2.2 )) 																																				//if(Gbox_Oil_Pressure_F > 1.484 )
					Gbox_Oil_Pressure_F = Gbox_Oil_Pressure_F * Gb2_A5 - Gb2_B5;
				else
					Gbox_Oil_Pressure_F = 10000;
				}
				Gbox_Oil_Pressure = Gbox_Oil_Pressure_F;
				
				//---------------------	Right Turbine_Pressure ---------------------//
				if(TurboR_Select == 0)
				{
				AI2_F = AI2_F / ADC_Average;
					if( AI2_F <= 1 )
				{
					AI2_F = (3.35 * AI2_F * AI2_F ) + ( 2.747 * AI2_F ) - 0.147;
				}
//				if((AI2_F > 0.867 && AI2_F <=1.4  )) 
//					AI2_F = AI2_F * AI2_A1 + AI2_B1 ;
//				else if((AI2_F > 0.798 && AI2_F <= 0.867 ))
//					AI2_F = AI2_F * AI2_A2 + AI2_B2;
//				else if((AI2_F > 0.671 && AI2_F <= 0.798 ))
//					AI2_F = AI2_F * AI2_A3 + AI2_B3;
//				else if((AI2_F > 0.546 && AI2_F <= 0.671 ))
//					AI2_F = AI2_F * AI2_A4 + AI2_B4;
//				else if((AI2_F > 0.450 && AI2_F <= 0.546 ))
//					AI2_F = AI2_F * AI2_A5 + AI2_B5;
//				else if((AI2_F > 0.356 && AI2_F <= 0.450 ))
//					AI2_F = AI2_F * AI2_A6 + AI2_B6;
//				else if((AI2_F > 0.202 && AI2_F <= 0.356 ))
//					AI2_F = AI2_F * AI2_A7 + AI2_B7;
//				else if((AI2_F > 0.190 && AI2_F <= 0.202 ))
//					AI2_F = AI2_F * AI2_A8 + AI2_B8;
//				else if((AI2_F > 0 && AI2_F <= 0.190 ))
//					AI2_F = AI2_F * AI2_A9 + AI2_B9;
//				else if(AI2_F > 1.5)
				else
					AI2_F = 10000;				//float
			}
				else
				{
					AI2_F = AI2_F / ADC_Average;
				if(AI2_F <= 0.94 ) 
					AI2_F = AI2_F * AI2_A1;
				else if((AI2_F > 0.94 && AI2_F <= 1.05 ))
					AI2_F = AI2_F * AI2_A2 - AI2_B2;
				else if((AI2_F > 1.05 && AI2_F <= 1.11 ))
					AI2_F = AI2_F * AI2_A3 - AI2_B3;
				else if((AI2_F > 1.11 && AI2_F <= 1.42 ))
					AI2_F = AI2_F * AI2_A4 - AI2_B4;
				else if((AI2_F > 1.42 && AI2_F <= 2.2 ))																																			//if(AI2_F > 1.484 )
					AI2_F = AI2_F * AI2_A5 - AI2_B5;
				else
					AI2_F = 10000;				//float
			}
				AI2 = AI2_F ;
				//---------------------	Left Turbine_Pressure ---------------------//
			if(TurboL_Select == 0)
			{
				AI3_F = AI3_F / ADC_Average;
				if( AI3_F <= 1 )
				{
					AI3_F = (3.35 * AI3_F * AI3_F ) + ( 2.747 * AI3_F ) - 0.147;
				}
//				if((AI3_F > 0.867 && AI3_F <=1.4  )) 
//					AI3_F = AI3_F * AI3_A1 + AI3_B1 ;
//				else if((AI3_F > 0.798 && AI3_F <= 0.867 ))
//					AI3_F = AI3_F * AI3_A2 + AI3_B2;
//				else if((AI3_F > 0.671 && AI3_F <= 0.798 ))
//					AI3_F = AI3_F * AI3_A3 + AI3_B3;
//				else if((AI3_F > 0.546 && AI3_F <= 0.671 ))
//					AI3_F = AI3_F * AI3_A4 + AI3_B4;
//				else if((AI3_F > 0.450 && AI3_F <= 0.546 ))
//					AI3_F = AI3_F * AI3_A5 + AI3_B5;
//				else if((AI3_F > 0.356 && AI3_F <= 0.450 ))
//					AI3_F = AI3_F * AI3_A6 + AI3_B6;
//				else if((AI3_F > 0.202 && AI3_F <= 0.356 ))
//					AI3_F = AI3_F * AI3_A7 + AI3_B7;
//				else if((AI3_F > 0.190 && AI3_F <= 0.202 ))
//					AI3_F = AI3_F * AI3_A8 + AI3_B8;
//				else if((AI3_F > 0 && AI3_F <= 0.190 ))
//					AI3_F = AI3_F * AI3_A9 + AI3_B9;
//				else  if(AI3_F > 1.5)
				else
					AI3_F = 10000;				//float
			}
			else
			{
				AI3_F = AI3_F / ADC_Average;
				if(AI3_F <= 0.94) 
					AI3_F = AI3_F * AI3_A1;
				else if((AI3_F > 0.94 && AI3_F <= 1.05 ))
					AI3_F = AI3_F * AI3_A2 - AI3_B2;
				else if((AI3_F > 1.05 && AI3_F <= 1.11 ))
					AI3_F = AI3_F * AI3_A3 - AI3_B3;
				else if((AI3_F > 1.11 && AI3_F <= 1.42 ))
					AI3_F = AI3_F * AI3_A4 - AI3_B4;
				else if((AI3_F > 1.42 && AI3_F <= 2.2 ))																																				//if(AI2_F > 1.484 )
					AI3_F = AI3_F * AI3_A5 - AI3_B5;
				else
					AI3_F = 10000;
			}
				AI3 = AI3_F;
				
				////---------------------	TC2 ---------------------//
				TC2_F = TC2_F / ADC_Average;
				TC2_F = 493.9 * TC2_F + 68.46;//right
				if(TC2_F > 999 ) TC2_F = 999;
			
//				if(TC2_F <= 0.589) 
//					TC2_F = TC2_F * 491.15 + 0.707;
//				else if((TC2_F > 0.589 && TC2_F <= 1.01 ))
//					TC2_F = TC2_F * 471.69 + 13.584;
//				else if((TC2_F > 1.01 && TC2_F <= 1.745 ))
//					TC2_F = TC2_F * 476.19 + 9.04;
//				else if((TC2_F > 1.745 && TC2_F <= 1.945 ))
//					TC2_F = TC2_F * 500 - 32.5;
//				else 																																				//if(AI2_F > 1.484 )
//					TC2_F = TC2_F * 524.19 - 79.55;
				//
				TC1_F = TC1_F / ADC_Average;
				TC1_F = 493.9 * TC1_F + 8.46;//left
				if(TC1_F > 999 ) TC1_F = 999;
//				if(TC1_F <= 0.589) 
//					TC1_F = TC1_F * 491.15 + 0.707;
//				else if((TC1_F > 0.589 && TC1_F <= 1.01 ))
//					TC1_F = TC1_F * 471.69 + 13.584;
//				else if((TC1_F > 1.01 && TC1_F <= 1.745 ))
//					TC1_F = TC1_F * 476.19 + 9.04;
//				else if((TC1_F > 1.745 && TC1_F <= 1.945 ))
//					TC1_F = TC1_F * 500 - 32.5;
//				else 																																				//if(AI2_F > 1.484 )
//					TC1_F = TC1_F * 524.19 - 79.55;
				//
				if(Row1_Select == 0)
				{
					Row1_F = RPM   ;
					SelKey1 = 0;
				}
				else if(Row1_Select == 1)
				{
					Row1_F =  Maint;
					SelKey1 = 1;
				}
				else if(Row1_Select == 2)
				{
					Row1_F = HRS;
					SelKey1 = 2;
				}
				else
				{
					Row1_F = Interval;
					SelKey1 = 3;
				}
				/////////////////////////////
				if(Row2_Select == 0)
				{
					Row2_F =Water_Pressure_F;
					SelKey2 = 1;
				}
				else
				{
					Row2_F = ( Water_Temp_F);
					SelKey2 = 0;
				}
				/////////////////////////////
				if(Row3_Select == 0)
				{	
					Row3_F = Oil_Pressure_F;
					SelKey3 = 0;
				}
				else
				{
					Row3_F = Battery_Voltage_F ;
					SelKey3 = 1;
				}
				///////////////////////////
				if(Row4_Select == 0)
				{
					Row4_F = AI3_F ;
					SelKey4 = 1;
				}
				else
				{
					Row4_F = TC1_F ;
					SelKey4 = 0;
				}
				///////////////////////////
				if(Row5_Select == 0)
				{
					Row5_F = AI2_F ;
					SelKey5 = 1;
				}
				else
				{
					Row5_F = TC2_F ;
					SelKey5 = 0;
				}
				///////////////////////////
				if(Row6_Select == 0)
				{
					Row6_F = Gbox_Oil_Pressure_F;
					SelKey6 = 1;
				}
				else
				{
					Row6_F = Reserve_Voltage_F;
					SelKey6 = 0;
				}
				///////////////////////////			
				
				Water_Temp_F = 0;
				Oil_Pressure_F = 0;
				Water_Pressure_F = 0;
				Battery_Voltage_F = 0;
				Gbox_Oil_Pressure_F = 0;
				Reserve_Voltage_F = 0;
				AI2_F = 0;
				AI3_F = 0;
				TC2_F = 0;
				TC1_F = 0;
			}
			//------------------------  show data on 7segments-------------------------------------------//
			
			Show_Segment_Cnt++;
			if(Show_Segment_Cnt == 1)
			{
				sprintf((char*)Row1_C, "%d", Row1_F);
				SendToSegment5Digit(Row1_C,1);
				DWT_Delay_us(Contrast);
				SendToSegment5Digit(Row1_C,4);
				
				if(SelKey4 == 0)
					sprintf((char*)Row4_C, "%3.0f", Row4_F);
				else
				{
					sprintf((char*)Row4_C, "%3.1f", Row4_F);
					if(Row4_F == 10000)
					{
						Row4_C[0] = '\0';Row4_C[1] = '\0';Row4_C[2] = '\0';Row4_C[3] = '\0';Row4_C[4] = '\0';
					}
				}
				SendToSegment4Digit2(Row4_C,1);
				DWT_Delay_us(Contrast);
				SendToSegment4Digit2(Row4_C,4);
				
			}
			if(Show_Segment_Cnt == 2)
			{
				if(SelKey2 == 0)
				{
					 sprintf((char*)Row2_C, "%3.0f", Row2_F);
					if(Row2_F == 10000)
					{
						Row2_C[0] = '\0';Row2_C[1] = '\0';Row2_C[2] = '\0';Row2_C[3] = '\0';Row2_C[4] = '\0';
					}
				}
				else
				{
					sprintf((char*)Row2_C, "%3.0f", Row2_F);
					if(Row2_F == 10000)
					{
						Row2_C[0] = '\0';Row2_C[1] = '\0';Row2_C[2] = '\0';Row2_C[3] = '\0';Row2_C[4] = '\0';
					}
				}
				SendToSegment4Digit(Row2_C,2);
				DWT_Delay_us(Contrast);
				SendToSegment4Digit(Row2_C,4);
				
				if(SelKey5 == 0)
				{
					sprintf((char*)Row5_C, "%3.0f", Row5_F);
					if(Row5_F == 10000)
					{
						Row5_C[0] = '\0';Row5_C[1] = '\0';Row5_C[2] = '\0';Row5_C[3] = '\0';Row5_C[4] = '\0';
					}
				}
				else
				{
					sprintf((char*)Row5_C, "%3.1f", Row5_F);
					if(Row5_F == 10000)
					{
						Row5_C[0] = '\0';Row5_C[1] = '\0';Row5_C[2] = '\0';Row5_C[3] = '\0';Row5_C[4] = '\0';
					}
				}
				SendToSegment4Digit2(Row5_C,2);
				DWT_Delay_us(Contrast);
				SendToSegment4Digit2(Row5_C,4);
			}
			if(Show_Segment_Cnt == 3)
			{
				if(SelKey3 == 0)
				{
					sprintf((char*)Row3_C, "%3.1f", Row3_F);
					if(Row3_F == 10000)
					{
						Row3_C[0] = '\0';Row3_C[1] = '\0';Row3_C[2] = '\0';Row3_C[3] = '\0';Row3_C[4] = '\0';
					}
				}
				else
				{
					sprintf((char*)Row3_C, "%3.1f", Row3_F);
				}
				SendToSegment4Digit(Row3_C,3);
				DWT_Delay_us(Contrast);
				SendToSegment4Digit(Row3_C,4);
				
				if(SelKey6 == 0)
				{
					
					if(Row6_F == 10000)
					{
						Row6_C[0] = '\0';Row6_C[1] = '\0';Row6_C[2] = '\0';Row6_C[3] = '\0';Row6_C[4] = '\0';
					}
					else
						sprintf((char*)Row6_C, "%3.0f", Row6_F);
				}
				else
				{
					if(Row6_F == 10000)
					{
						Row6_C[0] = '\0';Row6_C[1] = '\0';Row6_C[2] = '\0';Row6_C[3] = '\0';Row6_C[4] = '\0';
					}
					else
						sprintf((char*)Row6_C, "%3.1f", Row6_F);
				}
				//sprintf((char*)Row6_C, "%3.0f", Row6_F);
				Show_Segment_Cnt = 0;
				SendToSegment4Digit2(Row6_C,3);
				DWT_Delay_us(Contrast);
				SendToSegment4Digit2(Row6_C,4);
			}
			if(Show_Segment_Cnt == 4)
			{
				if(SelKey4 == 0)
					sprintf((char*)Row4_C, "%3.0f", Row4_F);
				else
				{
					sprintf((char*)Row4_C, "%3.1f", Row4_F);
					if(Row4_F == 10000)
					{
						Row4_C[0] = '\0';Row4_C[1] = '\0';Row4_C[2] = '\0';Row4_C[3] = '\0';Row4_C[4] = '\0';
					}
				}
				SendToSegment4Digit2(Row4_C,1);
				DWT_Delay_us(Contrast);
				SendToSegment4Digit2(Row4_C,4);
				
				sprintf((char*)Row1_C, "%d", Row1_F);
				SendToSegment5Digit(Row1_C,1);
				DWT_Delay_us(Contrast);
				SendToSegment5Digit(Row1_C,4);
			}
			if(Show_Segment_Cnt == 5)
			{
				if(SelKey5 == 0)
					sprintf((char*)Row5_C, "%3.0f", Row5_F);
				else
				{
					sprintf((char*)Row5_C, "%3.1f", Row5_F);
					if(Row5_F == 10000)
					{
						Row5_C[0] = '\0';Row5_C[1] = '\0';Row5_C[2] = '\0';Row5_C[3] = '\0';Row5_C[4] = '\0';
					}
				}
				SendToSegment4Digit2(Row5_C,2);
				DWT_Delay_us(Contrast);
				SendToSegment4Digit2(Row5_C,4);
				
				sprintf((char*)Row2_C, "%3.1f", Row2_F);
				SendToSegment4Digit(Row2_C,2);
				DWT_Delay_us(Contrast);
				SendToSegment4Digit(Row2_C,4);
			}
			if(Show_Segment_Cnt == 6)
			{
				if(SelKey6 == 0)
				{
					sprintf((char*)Row6_C, "%3.0f", Row6_F);
					if(Row6_F == 10000)
					{
						Row6_C[0] = '\0';Row6_C[1] = '\0';Row6_C[2] = '\0';Row6_C[3] = '\0';Row6_C[4] = '\0';
					}
				}
				else
				{
					sprintf((char*)Row6_C, "%3.1f", Row6_F);
					if(Row6_F == 10000)
					{
						Row6_C[0] = '\0';Row6_C[1] = '\0';Row6_C[2] = '\0';Row6_C[3] = '\0';Row6_C[4] = '\0';
					}
				}
				sprintf((char*)Row6_C, "%3.1f", Row6_F);
				Show_Segment_Cnt = 0;
				SendToSegment4Digit2(Row6_C,3);
				DWT_Delay_us(Contrast);
				SendToSegment4Digit2(Row6_C,4);
				
				sprintf((char*)Row3_C, "%3.1f", Row3_F);
				SendToSegment4Digit(Row3_C,3);
				DWT_Delay_us(Contrast);
				SendToSegment4Digit(Row3_C,4);
			}
			
						
			//-----------------------------------  Alarm Limits-----------------------------------------//
			
			//************************************ sensor alarms Contacts ***************************************//	
			New_Alarm = 0;
			//BPO alarm (low pressure of oil )
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11) == 0  )
			{
					if((Alarm_Decide & 0x0001) == 0)
						New_Alarm = 1;
					Alarm_Decide |= 0x0001;
					Oil_Led_ON()	
					Alarm1 = 1;		
			}
			else
			{
				Alarm_Decide &= 0xFFFE;
				Oil_Led_OFF()
				Low_Press_Oil_Cnt = 0;
				Alarm1 = 0;
			}
			
			//ATA (high temperature of water )
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12) == 0)
			{
					if((Alarm_Decide & 0x0002) == 0)
						New_Alarm = 1;
					Alarm_Decide |= 0x0002;
					Water_Led_ON()
					Alarm2 = 1;
			}
			else
			{
				Alarm_Decide &= 0xFFFD;
				High_temp_Water_Cnt = 0;
				Water_Led_OFF()
				Alarm2 = 0;
			}
			
			//AL (high speed)
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2) == 0)
			{
				if((Alarm_Decide & 0x0004) == 0)
						New_Alarm = 1;
				Alarm_Decide |= 0x0004;
				Spin_Led_ON()				//On out33 led (spin)
				Alarm3 = 1;
			}
			else 
			{
				Alarm_Decide &= 0xFFFB;
				Spin_Led_OFF()		 //Off out33 led (spin)
				Alarm3 = 0;
			}
			
			//AL1 (low press Gbox oil)
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8) == 0)					//GPIOA,GPIO_PIN_8
			{
				if((Alarm_Decide & 0x0008) == 0)
						New_Alarm = 1;
				Alarm_Decide |= 0x0008;
				AL1_Led_ON()	
				Alarm4 = 1;
			}
			else 
			{
				Alarm_Decide &= 0xFFF7;
				AL1_Led_OFF()
				Alarm4 = 0;
			}
			
			//AL2 (high temp Gbox )
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15) == 0)
			{
				if((Alarm_Decide & 0x0010) == 0)
						New_Alarm = 1;
				Alarm_Decide |= 0x0010;
				AL2_ON()	
				Alarm5 = 1;	
			}
			else 
			{
				Alarm_Decide &= 0xFFEF;
				AL2_OFF()
				Alarm5 = 0;
			}
			
			//AL3 (low water level)
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) == 0)
			{
				if((Alarm_Decide & 0x0020) == 0)
						New_Alarm = 1;
				Alarm_Decide |= 0x0020;
				AL3_Led_ON()	
				Alarm6 = 1;
			}
			else 
			{
				Alarm_Decide &= 0xFFDF;
				AL3_Led_OFF()
				Alarm6 = 0;
			}		
			//***************************************	Analog Alarms	**********************************//
			Blink_Cnt++;
			if(Blink_Cnt == 400) 
				Blink_Cnt = 0;
			
			//water temp
			if(Water_Temp >= Water_Temp_Upper_Limit && Water_Temp <= 9999)
			{
				if((Alarm_Decide & 0x0100) == 0)
						New_Alarm = 1;
				//Water_Led_ON()
				Alarm_Decide |= 0x0100;
				Alarm7 = 1;
				if(Water_Temp >= 96)
					Alarm77 = 1;
			}
			else
			{
				//Water_Led_OFF()
				Alarm_Decide &= 0xFEFF;
				Alarm7 = 0;
				Alarm77 = 0;
			}
			
			//oil pressure
			if(Oil_Pressure <= Oil_Press_Lower_Limit)		//1.2
			{
				Alarm8 = 1;
				if((Alarm_Decide & 0x0200) == 0)
						New_Alarm = 1;
				//Oil_Led_ON()
				Alarm_Decide |= 0x0200;
				if(Oil_Pressure <= 0.8)
					Alarm88 = 1;
			}
			else
			{
				//Oil_Led_OFF()
				Alarm_Decide &= 0xFDFF;
				Alarm8 = 0;
				Alarm88 = 0;
			}
			
//			//Gbox iol press
			if(Gbox_Oil_Pressure <= Oil_Gbox_Lower_Limit)		//10.5
			{
				if((Alarm_Decide & 0x0400) == 0)
						New_Alarm = 1;
				Alarm_Decide |= 0x0400;
				//AL1_Led_ON()
				Alarm9 = 1;
			}
			else
			{
				Alarm_Decide &= 0xFBFF;
				//AL1_Led_OFF()
				Alarm9 = 0;
			}
			
			//turboL press
			if(AI2 >= AI2_Lower_Limit && AI2 <= 9999)
			{
				if((Alarm_Decide & 0x0800) == 0)
						New_Alarm = 1;
				Alarm_Decide |= 0x0800;
				Alarm10 = 1;
			}
			else
			{
				Alarm_Decide &= 0xF7FF;
				Alarm10 = 0;
			}
			
			//turboR press
			if(AI3 >= AI3_Lower_Limit && AI3 <= 9999)
			{
				if((Alarm_Decide & 0x1000) == 0)
						New_Alarm = 1;
				Alarm_Decide |= 0x1000;
				Alarm11 = 1;
			}
			else
			{
				Alarm_Decide &= 0xEFFF;
				Alarm11 = 0;
			}
			
			//Battery Voltage alarm
			if(Battery_Voltage <= Battery_Lower_Alarm_Limit || Battery_Voltage >= Battery_Upper_Alarm_Limit)
			{
					if((Alarm_Decide & 0x2000) == 0)
						New_Alarm = 1;
				  Alarm_Decide |= 0x2000;
					Batlow_Led_ON()						//led out34	
					Alarm12 = 1;
			}
			else
			{
				Alarm_Decide &= 0xDFFF;
				Low_Bat = 0;
				Batlow_Led_OFF()
				Alarm12 = 0;
			}
			
			//oil temp alarm
			if(Reserve_Voltage >= Gbox_Oil_Temp_Limit && Reserve_Voltage <= 9999 )
			{
				if((Alarm_Decide & 0x4000) == 0)
						New_Alarm = 1;
				Alarm_Decide |= 0x4000;
				Alarm14 = 1;
			}
			else
			{
				Alarm_Decide &= 0xBFFF;
				Alarm14 = 0;
			}
			
			//high speed alarm
			if(RPM >= 3000) 
			{
				
				if((Alarm_Decide & 0x8000) == 0)
						New_Alarm = 1;
				  Alarm_Decide |= 0x8000;
					Alarm15 = 1;
			}
			else
			{
				//Spin_Led_OFF()	
				Alarm_Decide &= 0x7FFF;
				Alarm15 = 0;
			}
			
			
			//fuil tank
			if(Water_Pressure <= Water_Press_Lower_Limit)
			{
				Alarm13 = 1;
			}
			else
			{
				Alarm13 = 0;
			}
			
			//maintenace alarm
			if(Maint == 0)
				Achar_Led_ON()
			else
				Achar_Led_OFF()
			
			
			//***************************************	 Alarms Decides	**********************************//			
				if(Alarm_Decide != 0)
			{
				if(Buzzer_Mute_Flag == 1 )
				{
					if(New_Alarm == 1)
					{
						Buzzer_Mute_Flag = 0;
						Buzzer_ON()
					}
				}
				else
					Buzzer_ON()
				
				if(Alarm1 == 1 || Alarm4 == 1 || Alarm88 == 1 || Alarm77 == 1 || Alarm9 == 1 )
				{
					Auto_Stop_cnt ++;
					if(Stop_Off == 0)
					{
						Auto_Stop_ON()
						if( Auto_Stop_cnt == 800 )
							Stop_Off = 1;
					}
					else
						Auto_Stop_OFF()
				}
				else
					Auto_Stop_OFF()
			}
			else
			{
				Buzzer_OFF()
				Auto_Stop_OFF()
				Buzzer_Mute_Flag = 0;
			}
			
						
			//--------------------------------  Input Keys  ---------------------------------------------//
			
			//TgasPturb1 (select RPM  or HRS or Maint or Interval )
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0) == 0)
			{
				TgasPturb1_Flag = 1;
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0) == 1 && TgasPturb1_Flag == 1)
			{
				TgasPturb1_Flag = 0;
				Row1_Select++;
				if(Row1_Select == 4) Row1_Select = 0;
			}
			
			//TgasPturb2 (select temperature of water or Level )
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1) == 0)
			{
				TgasPturb2_Flag = 1;
				Interval_Counter_Reset++;
				if(Interval_Counter_Reset >= 2400)					//200*12=2400
				{
					Interval_Counter_Reset = 0;
					Buzzer_ON()
					//Interval = Interval_Time;										//Reset Maint
					Interval_L = 50;
					Interval_H = 0x00;						
					HAL_I2C_Mem_Write(&hi2c1,Device_Address,42,0xFF,&Interval_L,1,10);						
					Interval = Interval_L;
					Buzzer_OFF()
				}
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1) == 1 && TgasPturb2_Flag == 1)
			{
				TgasPturb2_Flag = 0;
				Row2_Select = ~Row2_Select;
			}
			
			//TengPgbox (select pressure of engine oil or Battery voltage )
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2) == 0)
			{
				TengPgbox_Flag = 1;
				Maint_Counter_Reset++;
				if(Maint_Counter_Reset == 2400)					//200*12=2400
				{
					Maint_Counter_Reset = 0;
					Buzzer_ON()
					//Maint = Maint_Time;										//Reset Maint
					Maint_L = 100;
					Maint_H = 0x00;
					HAL_I2C_Mem_Write(&hi2c1,Device_Address,45,0xFF,&Maint_L,1,100);		
					Maint = Maint_L;
					Buzzer_OFF()
				}
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2) == 1 && TengPgbox_Flag == 1)
			{
				TengPgbox_Flag = 0;
				Maint_Counter_Reset = 0;
				Row3_Select = ~Row3_Select;
			}
			//**************************** right keys ******************************
			//TgasPturb11 (select TURBL  or GAS1 )
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6) == 0)
			{
				TgasPturb11_Flag = 1;
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6) == 1 && TgasPturb11_Flag == 1)
			{
				TgasPturb11_Flag = 0;
				Row4_Select = ~Row4_Select;
			}
			
			//TgasPturb21 (select TURBR  or GAS2 )
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7) == 0)
			{
				TgasPturb21_Flag = 1;
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7) == 1 && TgasPturb21_Flag == 1)
			{
				TgasPturb21_Flag = 0;
				Row5_Select = ~Row5_Select;
			}
			
			//TengPgbox1 (select TEMP of engine oil or pressure of GEARBOX )
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8) == 0)
			{
				TengPgbox1_Flag = 1;
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8) == 1 && TengPgbox1_Flag == 1)
			{
				TengPgbox1_Flag = 0;
				Row6_Select = ~Row6_Select;
			}	
			
			//Down_B (light level )
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3) == 0 )
			{
				Down_B_Flag_L = 1;
			}
			if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3) == 1 && Down_B_Flag_L == 1) )
			{
				Down_B_Flag_L = 0;
				Contrast = Contrast - 200;
				if(Contrast <= 199) Contrast = 200;	
				Light_Intensity = Contrast / 200;
				HAL_I2C_Mem_Write(&hi2c1,Device_Address,16,0xFF,&Light_Intensity,1,10);
			}
			if( HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) == 0)						//GPIOC,GPIO_PIN_9
			{
				Down_B_Flag_R = 1;
			}
			if( (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) == 1 && Down_B_Flag_R == 1))			//GPIOC,GPIO_PIN_9
			{
				Down_B_Flag_R = 0;
				Contrast = Contrast - 200;
				if(Contrast <= 199) Contrast = 200;	
				Light_Intensity = Contrast / 200;
				HAL_I2C_Mem_Write(&hi2c1,Device_Address,16,0xFF,&Light_Intensity,1,10);
			}
			
			
			//Up_b (light level )
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4) == 0)
			{
				Up_B_Flag_R = 1;
			}
			if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4) == 1 && Up_B_Flag_R == 1) )
			{
				Up_B_Flag_R = 0;
				Contrast += 200;
				if(Contrast >= 1801) Contrast = 1800;
				Light_Intensity = Contrast / 200;
				HAL_I2C_Mem_Write(&hi2c1,Device_Address,16,0xFF,&Light_Intensity,1,10);
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == 0)
			{
				Up_B_Flag_L = 1;
			}
			if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == 1 && Up_B_Flag_L == 1))
			{
				Up_B_Flag_L = 0;
				Contrast += 200;
				if(Contrast >= 1501) Contrast = 1800;
				Light_Intensity = Contrast / 200;
				HAL_I2C_Mem_Write(&hi2c1,Device_Address,16,0xFF,&Light_Intensity,1,10);
			}
			
			//Mute_B
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5) == 0 )
			{
				Mute_B_Flag_R = 1;
			}
			if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5) == 1 && Mute_B_Flag_R == 1))
			{
				Mute_B_Flag_R = 0;					
				Buzzer_OFF()			
				Buzzer_Mute_Flag = 1;	
				Buzzer_Cnt = 0;
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == 0 )
			{
				Mute_B_Flag_L = 1;
			}
			if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == 1 && Mute_B_Flag_L == 1))
			{
				Mute_B_Flag_L = 0;					
				Buzzer_OFF()			
				Buzzer_Mute_Flag = 1;	
				Buzzer_Cnt = 0;
			}
			Buzzer_Cnt++;
			if(Buzzer_Cnt == Mute_Time && Buzzer_Mute_Flag == 1 )
			{
				Buzzer_Cnt = 0;
				Buzzer_Mute_Flag = 0;
			}
			
//*******************************************************************************************************//
			
			
			
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System 
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 10;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 74;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12 
                          |GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE6 PE7 PE8 PE10 
                           PE11 PE12 PE13 PE14 
                           PE15 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC0 PC1 PC2 
                           PC3 PC4 PC5 PC6 
                           PC7 PC8 PC10 PC11 
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB15 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD14 
                           PD0 PD1 PD2 PD3 
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14 
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/////////////////////// Functions	//////////////////////////////////////////////
void Init()
{	
	/////////////  Initialize Flash memory ////////////////////
	//MY_FLASH_SetSectorAddrs(11,0x080E0000);
	//MY_FLASH_ReadN(0, Time_Flash_Read,2, DATA_TYPE_16);
	if(Time_Flash_Read[0] >= 9999 || Time_Flash_Read[1] >= 9999)
	{
		Time_Flash_Read[0] = 0;
		Time_Flash_Read[1] = 0;
	}		
	Min = Time_Flash_Read[0];
	HRS = Time_Flash_Read[1];			
	/////////////////	Initilize Prepherals	//////////////////////////
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*) Adc_Value, 10);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start(&htim1);
	HAL_UART_Receive_IT(&huart1,Rx1_Data,50);
	DWT_Delay_Init();
	///////////////////// initial Variables ///////////////////////
	Flag_4second = 1;	
	Row1_F = RPM;
	Row2_F = Water_Temp_F;
	Row3_F = Battery_Voltage_F;
	///////////////////// initial LED ////////////////////////////
	SelKey1 = 0;
	SelKey2 = 1;
	SelKey3 = 0;
}
void Init_EEPROM()
{
	HAL_I2C_Mem_Read(&hi2c1,Device_Address,0,0xFF,EEPROM_Var,50,10);
	 Water_Temp_Threshold1 = EEPROM_Var[0];
	 Water_Temp_Threshold2 = EEPROM_Var[1];
	 rit = EEPROM_Var[2] * 200;							//200 is for 200Hz
	 Low_Bat_Threshold = EEPROM_Var[3];
	 Oil1.bytes[0] = EEPROM_Var[4];
	 Oil1.bytes[1] = EEPROM_Var[5];
	 Oil1.bytes[2] = EEPROM_Var[6];
	 Oil1.bytes[3] = EEPROM_Var[7];
	 Oil_Press_Threshold1 = Oil1.f;

	 Oil2.bytes[0] = EEPROM_Var[8];
	 Oil2.bytes[1] = EEPROM_Var[9];
	 Oil2.bytes[2] = EEPROM_Var[10];
	 Oil2.bytes[3] = EEPROM_Var[11];
	 Oil_Press_Threshold2 = Oil2.f;
	
	 TC1_Type = EEPROM_Var[12];							//0:type k			0x01:type j
	 TC2_Type = EEPROM_Var[13];							//0:type k			0x01:type j
	 Oil_Change_Interval_Time = 1000;
	 Light_Intensity = EEPROM_Var[16];			//1:high				2:mid			3:low
	 //RPM_Cof = EEPROM_Var[17];				//deactive for cte coef
	 Set_Time = EEPROM_Var[18];
	 if(Set_Time == 0xFF)
	 {
		 Set_Time = 0;
		 HAL_I2C_Mem_Write(&hi2c1,Device_Address,18,0xFF,&Set_Time,1,10);
		 SECOND = EEPROM_Var[19];
		 MINUTE = EEPROM_Var[20];
		 HOUR = EEPROM_Var[21];
		 DAY = EEPROM_Var[22];
		 DATE = EEPROM_Var[23];
		 MONTH = EEPROM_Var[24];
		 YEAR = EEPROM_Var[25];
		 //InitializeRTC();
		 Setting_Time (SECOND, MINUTE, HOUR, DAY, DATE, MONTH, YEAR);
	 }
	
	 ///////////////////  select coefficient	///////////////////
	 if(EEPROM_Var[26] == 0x00)
		 Oil_Select = 0;
	 else
		 Oil_Select = 1;
	 
	 if(EEPROM_Var[27] == 0x00)
		 Gb_Select = 0;
	 else
		 Gb_Select = 1;
	 
	 if(EEPROM_Var[28] == 0x00)
		 WTemp_Select = 0;
	 else
		 WTemp_Select = 1;
	 
	 if(EEPROM_Var[29] == 0x00)
		 OTemp_Select = 0;
	 else
		 OTemp_Select = 1;
	 
	  if(EEPROM_Var[30] == 0x00)
		 TurboL_Select = 0;
	 else
		 TurboL_Select = 1;
	 
	 if(EEPROM_Var[31] == 0x00)
		  TurboR_Select = 0;
	 else
		 TurboR_Select = 1;
	 
	 ///////////////////////////////////////////////////////////
	 
	////// initial value after reading eeprom ///////////////////
	Contrast = Light_Intensity * 200;	
	 ///////////////////	HRS	////////////////////////
	 Min = EEPROM_Var[47];
	 HRS_L = EEPROM_Var[48];
	 HRS_H = EEPROM_Var[49];
	 HRS = ((HRS_H<<8) & 0xFF00) | HRS_L;
	 ///////////////////	Maint	////////////////////////
	 Maint_L = EEPROM_Var[45];
	 Maint_H = EEPROM_Var[46];
	 Maint =  Maint_L;
	 ///////////////////	Interval	////////////////////////
	 Interval_L = EEPROM_Var[42];
	 Interval_H = EEPROM_Var[43];
	 Interval =  Interval_L;
	 
}

void Panel_Switch_ON()
{
	//ON water temp led2
	Water_Led_ON()
	//ON low pressure oil led1
	Oil_Led_ON()
	//ON Bat low led4 (out34)
	//ON spin led6 (out33)
	//ON ACHAR led (out35)
	//ON alternator lamp3(by hardware)  
	HAL_Delay(Pre_Start_Delay);				//1 second delay to ON leds
	//ON water temp led2
	Water_Led_OFF()
	//ON low pressure oil led1
	Oil_Led_OFF()
}

//----------------------- INITIALIZE RTC --------------------------
// Convert normal decimal numbers to binary coded decimal
uint8_t decToBcd(int val)
{
  return (uint8_t)( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
int bcdToDec(uint8_t val)
{
  return (int)( (val/16*10) + (val%16) );
}


// function to set time

void Setting_Time (uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year)
{
	uint8_t set_time[7];
	set_time[0] = decToBcd(sec);
	set_time[1] = decToBcd(min);
	set_time[2] = decToBcd(hour);
	set_time[3] = decToBcd(dow);
	set_time[4] = decToBcd(dom);
	set_time[5] = decToBcd(month);
	set_time[6] = decToBcd(year);
	
	HAL_I2C_Mem_Write(&hi2c3, DS3231_ADDRESS, 0x00, 1, set_time, 7, 1000);
}

void Get_Time()
{
	uint8_t get_time[7];
	HAL_I2C_Mem_Read(&hi2c3, DS3231_ADDRESS, 0x00, 1, get_time, 7, 1000);
	time.seconds = bcdToDec(get_time[0]);
	SECOND = time.seconds;
	time.minutes = bcdToDec(get_time[1]);
	MINUTE = time.minutes;
	time.hour = bcdToDec(get_time[2]);
	HOUR = time.hour;
	time.dayofweek = bcdToDec(get_time[3]);
	DAY = time.dayofweek;
	time.dayofmonth = bcdToDec(get_time[4]);
	DATE = time.dayofmonth;
	time.month = bcdToDec(get_time[5]);
	MONTH = time.month;
	time.year = bcdToDec(get_time[6]);
	YEAR = time.year;
}

////////////////////////////////////////////////////////////////////

void SendToSegment5Digit(unsigned char SegSTR[5],unsigned char ROW)
{
    unsigned char LoopCounter = 0;
    unsigned char PointLoc = 5;
    unsigned char NewSegValue[4] = "\0";
    
    while(SegSTR[LoopCounter] != '\0')
    {
        if(SegSTR[LoopCounter] == '.')
        {
            PointLoc = LoopCounter;            
        }
        else
        {
            if(PointLoc == 5)
                NewSegValue[LoopCounter] = (SegSTR[LoopCounter] - 0x30);
            else
                NewSegValue[LoopCounter - 1] = (SegSTR[LoopCounter] - 0x30);    
        };
        LoopCounter ++;          
    }
    if(PointLoc == 5)
    {
        if(LoopCounter == 0)
        {
            SegValue = SEG_TABLE[3][0] | SEG_TABLE[2][0] | SEG_TABLE[1][0] | SEG_TABLE[0][0]; 
        }
        else if(LoopCounter == 1)
        {
            SegValue = SEG_TABLE[3][0] | SEG_TABLE[2][0] | SEG_TABLE[1][0] | SEG_TABLE[0][NewSegValue[LoopCounter - 1]]; 
        }
        else if(LoopCounter == 2)
        {
            SegValue = SEG_TABLE[3][0] | SEG_TABLE[2][0] | SEG_TABLE[1][NewSegValue[LoopCounter - 2]] | SEG_TABLE[0][NewSegValue[LoopCounter - 1]]; 
        }
        else if(LoopCounter == 3)
        {
            SegValue = SEG_TABLE[3][0] | SEG_TABLE[2][NewSegValue[LoopCounter - 3]] | SEG_TABLE[1][NewSegValue[LoopCounter - 2]] | SEG_TABLE[0][NewSegValue[LoopCounter - 1]]; 
        }
        else
        {
            SegValue = SEG_TABLE[3][NewSegValue[LoopCounter - 4]] | SEG_TABLE[2][NewSegValue[LoopCounter - 3]] | SEG_TABLE[1][NewSegValue[LoopCounter - 2]] | SEG_TABLE[0][NewSegValue[LoopCounter - 1]]; 
        };    
    }
    else
    {
        if(LoopCounter == 4)
        {
            SegValue = SEG_TABLE[3][0] | SEG_TABLE[2][NewSegValue[LoopCounter - 4]] | SEG_TABLE[1][NewSegValue[LoopCounter - 3]] | SEG_TABLE[0][NewSegValue[LoopCounter - 2]];    
						SegValue |= 0x00000800;
        }
        else
        {
            SegValue = SEG_TABLE[3][NewSegValue[LoopCounter - 5]] | SEG_TABLE[2][NewSegValue[LoopCounter - 4]] | SEG_TABLE[1][NewSegValue[LoopCounter - 3]] | SEG_TABLE[0][NewSegValue[LoopCounter - 2]];
						SegValue |= 0x00000800;
        };
    };   
		
    Serial_Data_Reset()    //data = 0
    Serial_Clk_Reset()    //clock = 0
		
		SegValue |= SpinLed;
    SegValue |= AcharLed;
		SegValue |= BatLed;

    Serial_Data_Set()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    
    
    if((SegValue & 0x00000001) == 0x00000001)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000002) == 0x00000002)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000004) == 0x00000004)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000008) == 0x00000008)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000010) == 0x00000010)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000020) == 0x00000020)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000040) == 0x00000040)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000080) == 0x00000080)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000100) == 0x00000100)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000200) == 0x00000200)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000400) == 0x00000400)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000800) == 0x00000800)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00001000) == 0x00001000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00002000) == 0x00002000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00004000) == 0x00004000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00008000) == 0x00008000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00010000) == 0x00010000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00020000) == 0x00020000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00040000) == 0x00040000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00080000) == 0x00080000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    


    if((SegValue & 0x00100000) == 0x00100000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00200000) == 0x00200000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00400000) == 0x00400000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00800000) == 0x00800000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x01000000) == 0x01000000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x02000000) == 0x02000000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x04000000) == 0x04000000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x08000000) == 0x08000000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

//            if(ROW == 1)
//            {
//                Serial_Data_Set()
//            DWT_Delay_us(2);Serial_Clk_Set()
//            DWT_Delay_us(2);Serial_Clk_Reset()                    

//                Serial_Data_Reset()
//            DWT_Delay_us(2);Serial_Clk_Set()
//            DWT_Delay_us(2);Serial_Clk_Reset()                    

//                Serial_Data_Reset()
//            DWT_Delay_us(2);Serial_Clk_Set()
//            DWT_Delay_us(2);Serial_Clk_Reset()                    
//            }
						if((ROW == 1&& Alarm15 == 0 && SelKey1 == 0 ) || (ROW == 1 && Alarm15 == 1 && Blink_Cnt <= 300 ))
            {
                Serial_Data_Set()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                     
            }
						else if((ROW == 1 && SelKey1 != 0 ) )
            {
								Serial_Data_Set()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()  
            }
            else if(ROW == 2)
            {
                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Set()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    
            }
            else if(ROW == 3)
            {
                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Set()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    
            }
            else
            {
                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    
            };

    if((SegValue & 0x10000000) == 0x10000000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x20000000) == 0x20000000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x40000000) == 0x40000000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x80000000) == 0x80000000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    
}

//----------- send to segment 4 digit -------
void SendToSegment4Digit(unsigned char SegSTR[4],unsigned char ROW)
{
    unsigned char LoopCounter = 0;
    unsigned char PointLoc = 5;
    unsigned char NewSegValue[3] = "\0";
    SegValue = 0;
    
    while(SegSTR[LoopCounter] != '\0')
    {
        if(SegSTR[LoopCounter] == '.')
        {
            PointLoc = LoopCounter;            
        }
        else
        {
            if(PointLoc == 5)
                NewSegValue[LoopCounter] = (SegSTR[LoopCounter] - 0x30);
            else
                NewSegValue[LoopCounter - 1] = (SegSTR[LoopCounter] - 0x30);    
        };
        LoopCounter ++;          
    }
    if(PointLoc == 5)
    {
        if(LoopCounter == 0)
        {
            SegValue = 0x00002000 | 0x10000000 | 0x40000000 ; //SEG_TABLE[2][0] | SEG_TABLE[1][0] | SEG_TABLE[0][0]; 
        }
        else if(LoopCounter == 1)
        {
            SegValue = SEG_TABLE[2][0] | SEG_TABLE[1][0] | SEG_TABLE[0][NewSegValue[0]]; 
        }
        else if(LoopCounter == 2)
        {
            SegValue = SEG_TABLE[2][0] | SEG_TABLE[1][NewSegValue[0]] | SEG_TABLE[0][NewSegValue[1]]; 
        }
        else
        {
            SegValue = SEG_TABLE[2][NewSegValue[0]] | SEG_TABLE[1][NewSegValue[1]] | SEG_TABLE[0][NewSegValue[2]]; 
        };    
    }
    else
    {
        if(LoopCounter == 3)
        {
            SegValue = SEG_TABLE[2][0] | SEG_TABLE[1][NewSegValue[0]] | SEG_TABLE[0][NewSegValue[1]];
            SegValue |= 0x00000800;            
        }
        else
        {
            SegValue = SEG_TABLE[2][NewSegValue[0]] | SEG_TABLE[1][NewSegValue[1]] | SEG_TABLE[0][NewSegValue[2]];
            SegValue |= 0x00000800;        
        };
    };

		 if(ROW == 2)
    {
        if(SelKey1 < 2)
            SegValue |= 0x00200000;
        else
            SegValue |= 0x00400000;    
        if(SelKey2 == 0)
            SegValue |= 0x00800000;
        else
            SegValue |= 0x01000000;    
    }
    if(ROW == 3)
    {
        if(SelKey3 == 0)
            SegValue |= 0x00200000;
        else
            SegValue |= 0x00400000;    
    };
		
    Serial_Data_Reset()    //data = 0
    Serial_Clk_Reset()    //clock = 0

    Serial_Data_Set()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    
    
    if((SegValue & 0x00000001) == 0x00000001)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000002) == 0x00000002)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000004) == 0x00000004)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000008) == 0x00000008)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000010) == 0x00000010)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000020) == 0x00000020)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000040) == 0x00000040)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000080) == 0x00000080)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000100) == 0x00000100)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000200) == 0x00000200)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000400) == 0x00000400)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00000800) == 0x00000800)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00001000) == 0x00001000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00002000) == 0x00002000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00004000) == 0x00004000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00008000) == 0x00008000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00010000) == 0x00010000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00020000) == 0x00020000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00040000) == 0x00040000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00080000) == 0x00080000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    


    if((SegValue & 0x00100000) == 0x00100000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00200000) == 0x00200000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00400000) == 0x00400000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x00800000) == 0x00800000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x01000000) == 0x01000000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x02000000) == 0x02000000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x04000000) == 0x04000000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x08000000) == 0x08000000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

            if(ROW == 1)
            {
                Serial_Data_Set()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    
            }
            else if((ROW == 2&& Alarm13 == 0 && SelKey2 == 1 ) || (ROW == 2 && Alarm13 == 1 && Blink_Cnt <= 300 ))
            {
                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Set()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    
            }
						else if((ROW == 2&& Alarm7 == 0 && SelKey2 == 0 ) || (ROW == 2 && Alarm7 == 1 && Blink_Cnt <= 300 ))
            {
                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Set()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    
            }
            else if((ROW == 3 && Alarm8 == 0 && SelKey3 == 0 ) || (ROW == 3 && Alarm8 == 1 && Blink_Cnt <= 300 ))
            {
                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Set()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    
            }
						else if((ROW == 3 && Alarm12 == 0 && SelKey3 == 1 ) || (ROW == 3 && Alarm12 == 1 && Blink_Cnt <= 300 ))
            {
                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Set()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    
            }
            else
            {
                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    

                Serial_Data_Reset()
            DWT_Delay_us(2);Serial_Clk_Set()
            DWT_Delay_us(2);Serial_Clk_Reset()                    
            };

    if((SegValue & 0x10000000) == 0x10000000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x20000000) == 0x20000000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x40000000) == 0x40000000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    if((SegValue & 0x80000000) == 0x80000000)
        Serial_Data_Set()
    else
        Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    

    Serial_Data_Reset()
    DWT_Delay_us(2);Serial_Clk_Set()
    DWT_Delay_us(2);Serial_Clk_Reset()                    
}




/////////////////////// Functions2	//////////////////////////////////////////////

void SendToSegment5Digit2(unsigned char SegSTR[5],unsigned char ROW)
{
    unsigned char LoopCounter = 0;
    unsigned char PointLoc = 5;
    unsigned char NewSegValue[4] = "\0";
    
    while(SegSTR[LoopCounter] != '\0')
    {
        if(SegSTR[LoopCounter] == '.')
        {
            PointLoc = LoopCounter;            
        }
        else
        {
            if(PointLoc == 5)
                NewSegValue[LoopCounter] = (SegSTR[LoopCounter] - 0x30);
            else
                NewSegValue[LoopCounter - 1] = (SegSTR[LoopCounter] - 0x30);    
        };
        LoopCounter ++;          
    }
    if(PointLoc == 5)
    {
        if(LoopCounter == 0)
        {
            SegValue = SEG_TABLE[3][0] | SEG_TABLE[2][0] | SEG_TABLE[1][0] | SEG_TABLE[0][0]; 
        }
        else if(LoopCounter == 1)
        {
            SegValue = SEG_TABLE[3][0] | SEG_TABLE[2][0] | SEG_TABLE[1][0] | SEG_TABLE[0][NewSegValue[LoopCounter - 1]]; 
        }
        else if(LoopCounter == 2)
        {
            SegValue = SEG_TABLE[3][0] | SEG_TABLE[2][0] | SEG_TABLE[1][NewSegValue[LoopCounter - 2]] | SEG_TABLE[0][NewSegValue[LoopCounter - 1]]; 
        }
        else if(LoopCounter == 3)
        {
            SegValue = SEG_TABLE[3][0] | SEG_TABLE[2][NewSegValue[LoopCounter - 3]] | SEG_TABLE[1][NewSegValue[LoopCounter - 2]] | SEG_TABLE[0][NewSegValue[LoopCounter - 1]]; 
        }
        else
        {
            SegValue = SEG_TABLE[3][NewSegValue[LoopCounter - 4]] | SEG_TABLE[2][NewSegValue[LoopCounter - 3]] | SEG_TABLE[1][NewSegValue[LoopCounter - 2]] | SEG_TABLE[0][NewSegValue[LoopCounter - 1]]; 
        };    
    }
    else
    {
        if(LoopCounter == 4)
        {
            SegValue = SEG_TABLE[3][0] | SEG_TABLE[2][NewSegValue[LoopCounter - 4]] | SEG_TABLE[1][NewSegValue[LoopCounter - 3]] | SEG_TABLE[0][NewSegValue[LoopCounter - 2]];    
						SegValue |= 0x00000800;
        }
        else
        {
            SegValue = SEG_TABLE[3][NewSegValue[LoopCounter - 5]] | SEG_TABLE[2][NewSegValue[LoopCounter - 4]] | SEG_TABLE[1][NewSegValue[LoopCounter - 3]] | SEG_TABLE[0][NewSegValue[LoopCounter - 2]];
						SegValue |= 0x00000800;
        };
    };    
    Serial2_Data_Reset()    //data = 0
    Serial2_Clk_Reset()    //clock = 0

    Serial2_Data_Set()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    
    
    if((SegValue & 0x00000001) == 0x00000001)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000002) == 0x00000002)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000004) == 0x00000004)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000008) == 0x00000008)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000010) == 0x00000010)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000020) == 0x00000020)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000040) == 0x00000040)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000080) == 0x00000080)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000100) == 0x00000100)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000200) == 0x00000200)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000400) == 0x00000400)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000800) == 0x00000800)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00001000) == 0x00001000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00002000) == 0x00002000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00004000) == 0x00004000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00008000) == 0x00008000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00010000) == 0x00010000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00020000) == 0x00020000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00040000) == 0x00040000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00080000) == 0x00080000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    


    if((SegValue & 0x00100000) == 0x00100000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00200000) == 0x00200000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00400000) == 0x00400000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00800000) == 0x00800000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x01000000) == 0x01000000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x02000000) == 0x02000000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x04000000) == 0x04000000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x08000000) == 0x08000000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

            if(ROW == 1)
            {
                Serial2_Data_Set()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    
            }
            else if(ROW == 2)
            {
                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Set()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    
            }
            else if(ROW == 3)
            {
                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Set()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    
            }
            else
            {
                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    
            };

    if((SegValue & 0x10000000) == 0x10000000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x20000000) == 0x20000000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x40000000) == 0x40000000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x80000000) == 0x80000000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    
}

//----------- send to segment 4 digit -------
void SendToSegment4Digit2(unsigned char SegSTR[4],unsigned char ROW)
{
    unsigned char LoopCounter = 0;
    unsigned char PointLoc = 5;
    unsigned char NewSegValue[3] = "\0";
    SegValue = 0;
    
    while(SegSTR[LoopCounter] != '\0')
    {
        if(SegSTR[LoopCounter] == '.')
        {
            PointLoc = LoopCounter;            
        }
        else
        {
            if(PointLoc == 5)
                NewSegValue[LoopCounter] = (SegSTR[LoopCounter] - 0x30);
            else
                NewSegValue[LoopCounter - 1] = (SegSTR[LoopCounter] - 0x30);    
        };
        LoopCounter ++;          
    }
    if(PointLoc == 5)
    {
        if(LoopCounter == 0)
        {
            SegValue = 0x00002000 | 0x10000000 | 0x40000000 ; //SEG_TABLE[2][0] | SEG_TABLE[1][0] | SEG_TABLE[0][0];
        }
        else if(LoopCounter == 1)
        {
            SegValue = SEG_TABLE[2][0] | SEG_TABLE[1][0] | SEG_TABLE[0][NewSegValue[0]]; 
        }
        else if(LoopCounter == 2)
        {
            SegValue = SEG_TABLE[2][0] | SEG_TABLE[1][NewSegValue[0]] | SEG_TABLE[0][NewSegValue[1]]; 
        }
        else
        {
            SegValue = SEG_TABLE[2][NewSegValue[0]] | SEG_TABLE[1][NewSegValue[1]] | SEG_TABLE[0][NewSegValue[2]]; 
        };    
    }
    else
    {
        if(LoopCounter == 3)
        {
            SegValue = SEG_TABLE[2][0] | SEG_TABLE[1][NewSegValue[0]] | SEG_TABLE[0][NewSegValue[1]];
            SegValue |= 0x00000800;            
        }
        else
        {
            SegValue = SEG_TABLE[2][NewSegValue[0]] | SEG_TABLE[1][NewSegValue[1]] | SEG_TABLE[0][NewSegValue[2]];
            SegValue |= 0x00000800;        
        };
    };    
		
				 if(ROW == 2)
    {
        if(SelKey4 == 0)
            SegValue |= 0x00200000;
        else
            SegValue |= 0x00400000;    
        if(SelKey5 == 0)
            SegValue |= 0x00800000;
        else
            SegValue |= 0x01000000;    
    }
    else
    {
        if(SelKey6 == 0)
            SegValue |= 0x00200000;
        else
            SegValue |= 0x00400000;    
    };
		
		SegValue |= AL2Led;
		
    Serial2_Data_Reset()    //data = 0
    Serial2_Clk_Reset()    //clock = 0

    Serial2_Data_Set()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    
    
    if((SegValue & 0x00000001) == 0x00000001)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000002) == 0x00000002)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000004) == 0x00000004)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000008) == 0x00000008)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000010) == 0x00000010)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000020) == 0x00000020)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000040) == 0x00000040)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000080) == 0x00000080)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000100) == 0x00000100)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000200) == 0x00000200)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000400) == 0x00000400)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00000800) == 0x00000800)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00001000) == 0x00001000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00002000) == 0x00002000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00004000) == 0x00004000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00008000) == 0x00008000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00010000) == 0x00010000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00020000) == 0x00020000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00040000) == 0x00040000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00080000) == 0x00080000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    


    if((SegValue & 0x00100000) == 0x00100000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00200000) == 0x00200000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00400000) == 0x00400000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x00800000) == 0x00800000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x01000000) == 0x01000000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x02000000) == 0x02000000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x04000000) == 0x04000000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x08000000) == 0x08000000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

            if((ROW == 1 &&  SelKey4 == 0 ) || (ROW == 1 && Alarm11 == 1 &&  SelKey4 == 1 && Blink_Cnt <= 300 ) || (ROW == 1 &&  Alarm11 == 0 ))
            {
                Serial2_Data_Set()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    
            }
            else if((ROW == 2 &&  SelKey5 == 0 ) || (ROW == 2 && Alarm10 == 1 &&  SelKey5 == 1 && Blink_Cnt <= 300 ) || (ROW == 2 &&  Alarm10 == 0 ))
            {
                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Set()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    
            }
            else if((ROW == 3 && Alarm14 == 0 && SelKey6 == 0 ) || (ROW == 3 && Alarm14 == 1 && Blink_Cnt <= 300 ))
            {
                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Set()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    
            }
						else if((ROW == 3  && Alarm9 == 0 && SelKey6 == 1 ) || (ROW == 3 && Alarm9 == 1 && Blink_Cnt <= 300 ))
            {
                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Set()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    
            }
            else
            {
                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    

                Serial2_Data_Reset()
            DWT_Delay_us(2);Serial2_Clk_Set()
            DWT_Delay_us(2);Serial2_Clk_Reset()                    
            };

    if((SegValue & 0x10000000) == 0x10000000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x20000000) == 0x20000000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x40000000) == 0x40000000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    if((SegValue & 0x80000000) == 0x80000000)
        Serial2_Data_Set()
    else
        Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    

    Serial2_Data_Reset()
    DWT_Delay_us(2);Serial2_Clk_Set()
    DWT_Delay_us(2);Serial2_Clk_Reset()                    
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
