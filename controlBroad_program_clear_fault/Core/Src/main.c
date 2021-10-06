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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//command torque define

//torque upper bound
#define TORQUE_UB 700

//pedal box parameter setting
#define BrakeAct 1800
#define APPSAct 300
#define APPSLeast 200   //for the APPS plausibility check  => motor must shut down until the APPS signals less than 5% peadal travel
#define APPSDIFF 2000


//if out of this range will generate error
#define APPSRUPPEST 4000
#define APPSLUPPEST 4000
#define BPPSUPPEST 4000
#define APPSRLOWEST 500
#define APPSLLOWEST 500
#define BPPSLOWEST 0
#define DISCONNECT 4050
//deine max and offset this parameter offset will slight bigger than the lowest and max will slight smaller than the uppest depends on  the pedal box
//this parameter is used to determine the output torque
#define APPSRMAX 3740
#define APPSLMAX 3500
#define APPSROFFSET 1860
#define APPSLOFFSET 1500
#define BPPSOFFSET 1500
#define BPPSMAX 2500
#define ReadyTime 1000   //ms

//dma scan
#define APPSR adcArr[0] //min 1740   max 3740
#define APPSL adcArr[1] //min 1450   max 3500
#define BPPS   adcArr[2]
//#define APPS ((adcArr[0] - APPSROFFSET) + (adcArr[1] - APPSLOFFSET))/2


//daul algorithm
#define RAD 180.0/3.14159
#define RAD_REC 3.14159/180.0 
#define SPEED (double)adcArr[4]
#define STEER (double)adcArr[5]

#define MASS 300.0
#define POWER_UB 40000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adcArr[5];
int16_t APPS = 0;
static CAN_TxHeaderTypeDef TxMessage_right;
static CAN_TxHeaderTypeDef TxMessage_left;
static CAN_TxHeaderTypeDef TxMessage_R_clear;
static CAN_TxHeaderTypeDef TxMessage_L_clear;
static CAN_RxHeaderTypeDef RxMessage;

//Rx Txdata
uint8_t TxData_R[8]={0};
uint8_t TxData_L[8]={0};
uint8_t RxData_R[8]={0};
uint8_t RxData_L[8]={0};
uint8_t TxData_clear[8]={0};
uint32_t TxMailbox;
uint8_t RxData[8]={0};

//address
uint16_t OwnID=0x123;
uint16_t RemoteID =0x0A0;
uint16_t Received_ID;
//sensor
uint32_t accMeter =0;
uint16_t wheelSpeed[4];
uint16_t steerDegree =0;

//inverter
uint16_t rpm_right =0;
uint16_t rpm_left = 0;

bool readyButton;
uint8_t errorNumber;
const uint16_t fixed_torque_ub = 1400;
uint16_t torque_ub = 0;
uint16_t torque_right=0;
uint16_t torque_left=0;
uint32_t startTime;
uint32_t duration=0;
uint8_t cycle = 0;
bool pedals=0;
bool rtd_io=0;
bool error =0;
bool inverter_error_L = 0;
bool inverter_error_R = 0;
bool inverter_connect_L = 0;
bool inverter_connect_R = 0;
uint16_t inverter_alive_counter_R = 0;
uint16_t inverter_alive_counter_L = 0;
bool rtd_start=0; //if precharge&&reset&&readyToDrive io are all on this parameter will be true
//bool ready_io=0;
bool precharge_io=0;
bool reset_io=1;
bool clear_fault_io=0;
bool direction=0;

//SPI variable
uint8_t data_rec[6];
int16_t x, y, z;
float xg, yg, zg;
//dual algorithm const
const double Cr=(300*180)/3.14159;
const double Cf=(300*180)/3.14159;
const double Lf=1.503*0.55;
const double Lr=1.503*0.45;
const double Iz=25.0;

//dual algorithm var
double beta_diff_cur=0.0;
double beta_diff_pre=0.0;
double gamma_diff_cur=0.0;
double gamma_diff_pre=0.0;
double beta=0.0;
double gamma=0.0;

double a11_var;
double a12_var;
double a21_var;
double a22_var;
double b11_var;
double b12_var;

double steer_map=0.0;
double speed_map=0.0;


//time
double counter_cur=0;
double counter_pre=0;
double period=0;

//const double mass=300.0;


//PWM input
uint32_t IC_Val1;
uint32_t IC_Val2;

uint32_t Frequency_3;
uint32_t Duty_Cycle_3;

uint32_t Frequency_3_;
uint32_t Duty_Cycle_3_;

uint32_t Frequency_4;
uint32_t Duty_Cycle_4;

uint32_t Frequency_4_;
uint32_t Duty_Cycle_4_;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
//APPS
void driving_mode(void);
void led_blink(void);
void pedals_mode(void);
uint8_t check_safety(void);
uint16_t map(int16_t value,int16_t inputL,int16_t inputH,int16_t outputL ,int16_t outputH);
void torque_command(void);
void torque_to_can(void);

//CAN
void CAN_filterConfig(void);
void CAN_Txsetup(void);

//SPI function
void adxl_write(uint8_t address, uint8_t value);
void adxl_read(uint8_t address);
void adxl_init();

//buzzer
void setBuzzer(uint8_t value);

double map_double(double value,double inputL,double inputH,double outputL ,double outputH);
double a11(void);
double a12(void);
double a21(void);
double a22(void);
double b11(void);
double b12(void);
double beta_diff(void);
double gamma_diff(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adcArr,5);
	HAL_TIM_Base_Start_IT(&htim2);
	CAN_filterConfig();
	CAN_Txsetup();
	
	precharge_io=HAL_GPIO_ReadPin(precharge_SW_GPIO_Port,precharge_SW_Pin);
	if(precharge_io)
		HAL_GPIO_WritePin(precharge_LED_GPIO_Port,precharge_LED_Pin,GPIO_PIN_SET);
	reset_io=1;
	
	adxl_init();
	//PWM timer
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
	
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  //waiting for precharge reset_switch, ready to drive to activate the driving mode
		if(rtd_io==0 ){
				torque_right=0;
				torque_left=0;
				HAL_GPIO_WritePin(readyToDrive_LED_GPIO_Port,readyToDrive_LED_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(pedals_LED_GPIO_Port,pedals_LED_Pin,GPIO_PIN_RESET);
				duration=0;
				do{
						if(rtd_start==1 && errorNumber == 5){
								startTime=HAL_GetTick();
								setBuzzer(200);
								while(duration<ReadyTime &&   rtd_start==1){
										duration = HAL_GetTick()-startTime;
								}
						}else{
							setBuzzer(0);
						}
				}while(duration<ReadyTime);
				 rtd_io=1;
				 setBuzzer(50);
				 HAL_Delay(100);
				 setBuzzer(0);
				 HAL_Delay(100);
				 setBuzzer(100);
				 HAL_Delay(100);
				 setBuzzer(0);
			}
			//driving stage
			if(rtd_io==1){
					HAL_GPIO_WritePin(fault_LED_GPIO_Port,fault_LED_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(readyToDrive_LED_GPIO_Port,readyToDrive_LED_Pin,GPIO_PIN_SET);
					
				  //entering drive mode
					driving_mode();		
					
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  /* EXTI2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 2);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* EXTI4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
	/*
	a11_var=a11();
	a12_var=a12();
	a21_var=a21();
	a22_var=a22();
	b11_var=b11();
	b12_var=b12();
	//speed_map=1;
	//steer_map=0.0872664626;
	
	speed_map=map_double((double)adcArr[4],0.0,4095.0,-20.0,20.0);
	steer_map=map_double((double)adcArr[3],0.0,4095.0,-30*RAD_REC,30*RAD_REC);
	beta_diff_cur=beta_diff();
	gamma_diff_cur = gamma_diff();
	counter_cur=HAL_GetTick();
	period = counter_cur-counter_pre;
	beta+=period*beta_diff()/1000;
	gamma+=period*gamma_diff()/1000;
	counter_pre=counter_cur;
	if((cycle % 30 ) == 0){
	//Gyro read
	adxl_read(0x32);
	x = ((data_rec[1] << 8) | data_rec[0]);
	y = ((data_rec[3] << 8) | data_rec[2]);
	z = ((data_rec[5] << 8) | data_rec[4]);
		
	xg = x * 0.0078;
	yg = y * 0.0078;
	zg = z * 0.0078;
	}
	*/
	APPS = ((adcArr[0] - APPSROFFSET) + (adcArr[1] - APPSLOFFSET))/2;
	if((cycle%20) == 0){
	//CAN transmit
		if(clear_fault_io){
			HAL_CAN_AddTxMessage(&hcan1,&TxMessage_R_clear,TxData_clear,&TxMailbox);
			HAL_CAN_AddTxMessage(&hcan1,&TxMessage_L_clear,TxData_clear,&TxMailbox);
			clear_fault_io = 0;
			inverter_error_L = 0;
			inverter_error_R = 0;
			HAL_GPIO_WritePin(CAN_fault_LED_GPIO_Port,CAN_fault_LED_Pin,GPIO_PIN_RESET);
		}
		torque_to_can();
		HAL_CAN_AddTxMessage(&hcan1,&TxMessage_right,TxData_R,&TxMailbox);
		HAL_CAN_AddTxMessage(&hcan1,&TxMessage_left ,TxData_L,&TxMailbox);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_1);    
		
		++inverter_alive_counter_L;
		++inverter_alive_counter_R;
		if(inverter_alive_counter_L > 10){
			inverter_connect_L = 0;
		}
		if(inverter_alive_counter_R > 10){
			inverter_connect_R = 0;
		}
		
		if(!inverter_connect_R || !inverter_connect_L){
			rtd_io=0;
			HAL_GPIO_WritePin(readyToDrive_LED_GPIO_Port,readyToDrive_LED_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(CAN_fault_LED_GPIO_Port,CAN_fault_LED_Pin,GPIO_PIN_SET);
		}
	}
	
	//check apps sensor range
	if(!rtd_io){
		errorNumber = check_safety();
		if(!(errorNumber == 0 || errorNumber == 5)){
			error = 1;
			HAL_GPIO_WritePin(fault_LED_GPIO_Port,fault_LED_Pin,GPIO_PIN_SET);
		}else{
			error = 0;
			HAL_GPIO_WritePin(fault_LED_GPIO_Port,fault_LED_Pin,GPIO_PIN_RESET);
		}
	}
	
	
	++cycle;
}

void CAN_filterConfig(void){
	CAN_FilterTypeDef filterConfig;
	
	filterConfig.FilterBank=0;
	filterConfig.FilterActivation=ENABLE;
	filterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;
	filterConfig.FilterIdHigh=0x0000;
	filterConfig.FilterIdLow=0x0000;
	filterConfig.FilterMaskIdHigh=0x0000;
	filterConfig.FilterMaskIdLow=0x0000;
	filterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
	filterConfig.FilterScale= CAN_FILTERSCALE_32BIT;
	filterConfig.SlaveStartFilterBank=14;
	
	if(HAL_CAN_ConfigFilter(&hcan1,&filterConfig)!=HAL_OK){
		Error_Handler();
	}
	
	if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK){
		Error_Handler();
	}		//enable interrupt
	
}
void CAN_Txsetup(){
		if(HAL_CAN_Start(&hcan1)!=HAL_OK){
		Error_Handler();
		}
		
		TxMessage_right.StdId=0x0C0;
		TxMessage_right.ExtId=0x01;
		TxMessage_right.RTR=CAN_RTR_DATA;
		TxMessage_right.IDE=CAN_ID_STD;
		TxMessage_right.DLC=8;
		TxMessage_right.TransmitGlobalTime=DISABLE;   //time trigger must be turned ON
		
		TxMessage_left.StdId=0x070;
		TxMessage_left.ExtId=0x01;
		TxMessage_left.RTR=CAN_RTR_DATA;
		TxMessage_left.IDE=CAN_ID_STD;
		TxMessage_left.DLC=8;
		TxMessage_left.TransmitGlobalTime=DISABLE;   //time trigger must be turned ON
		
		
		TxData_R[0]=0;
		TxData_R[1]=0;
		TxData_R[2]=0;
		TxData_R[3]=0;
		TxData_R[4]=!direction;
		TxData_R[5]=0;
		TxData_R[6]=0;
		TxData_R[7]=0;
		
		
		TxData_L[0]=0;
		TxData_L[1]=0;
		TxData_L[2]=0;
		TxData_L[3]=0;
		TxData_L[4]=direction;
		TxData_L[5]=0;
		TxData_L[6]=0;
		TxData_L[7]=0;
		
		TxMessage_R_clear.StdId=0x0C1;
		TxMessage_R_clear.ExtId=0x01;
		TxMessage_R_clear.RTR=CAN_RTR_DATA;
		TxMessage_R_clear.IDE=CAN_ID_STD;
		TxMessage_R_clear.DLC=8;
		TxMessage_R_clear.TransmitGlobalTime=DISABLE; 
		
		TxMessage_L_clear.StdId=0x071;
		TxMessage_L_clear.ExtId=0x01;
		TxMessage_L_clear.RTR=CAN_RTR_DATA;
		TxMessage_L_clear.IDE=CAN_ID_STD;
		TxMessage_L_clear.DLC=8;
		TxMessage_L_clear.TransmitGlobalTime=DISABLE;   
		
		
		TxData_clear[0]=20;
		TxData_clear[1]=0;
		TxData_clear[2]=1;
		TxData_clear[3]=0;
		TxData_clear[4]=0;
		TxData_clear[5]=0;
		TxData_clear[6]=0;
		TxData_clear[7]=0;
		
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxMessage,RxData)!=HAL_OK){
		Error_Handler();
	}
	Received_ID=RxMessage.StdId;
	
	switch(Received_ID){
		case 0xAB:
			inverter_alive_counter_R = 0;
			inverter_connect_R = 1;
			inverter_error_R = 0;
			for(int i=0;i<8;++i){
				RxData_R[i]=RxData[i];
				if(RxData[i]){
					inverter_error_R = 1;
					error = 1;
					rtd_io=0;
					HAL_GPIO_WritePin(CAN_fault_LED_GPIO_Port,CAN_fault_LED_Pin,GPIO_PIN_SET);
				}
			}
			break;
		case 0x5B:
			inverter_alive_counter_L = 0;
			inverter_connect_L = 1;
			
			for(int i=0;i<8;++i){
				RxData_L[i]=RxData[i];
				if(RxData[i]){
					inverter_error_L = 1;
					error = 1;
			    rtd_io=0;
					HAL_GPIO_WritePin(CAN_fault_LED_GPIO_Port,CAN_fault_LED_Pin,GPIO_PIN_SET);
				}
			}
			break;
		case 0xA5:
			rpm_right = RxData[2]+RxData[3]*256;
			break;
		case 0x55:
			rpm_left  = RxData[2]+RxData[3]*256;
			break;
		}
	
	/*
	if(Received_ID == 0xAB){
		inverter_error_R = 0;
		for(int i=0;i<8;++i){
			RxData_R[i]=RxData[i];
			if(RxData[i]){
				inverter_error_R = 1;
		    HAL_GPIO_WritePin(CAN_fault_LED_GPIO_Port,CAN_fault_LED_Pin,GPIO_PIN_SET);
			}
		}
		if(!inverter_error_R){
			HAL_GPIO_WritePin(CAN_fault_LED_GPIO_Port,CAN_fault_LED_Pin,GPIO_PIN_RESET);
		}
	}
	
	if(Received_ID == 0x5B){
		inverter_error_L = 0;
		for(int i=0;i<8;++i){
			RxData_L[i]=RxData[i];
			if(RxData[i]){
				inverter_error_L = 1;
		    HAL_GPIO_WritePin(CAN_fault_LED_GPIO_Port,CAN_fault_LED_Pin,GPIO_PIN_SET);
			}
		}
		if(!inverter_error_L){
			HAL_GPIO_WritePin(CAN_fault_LED_GPIO_Port,CAN_fault_LED_Pin,GPIO_PIN_RESET);
		}
	}
	*/
}



void driving_mode(void){
  errorNumber= check_safety();
	switch(errorNumber){
		case 1:{
			//wiring disconnection
			HAL_GPIO_WritePin(fault_LED_GPIO_Port,fault_LED_Pin,GPIO_PIN_SET);
			torque_right=0;
			torque_left=0;
			error = 1;
			rtd_io=0;
			break;
		}
		case 2:{
			//apps throttle differ over
			HAL_GPIO_WritePin(fault_LED_GPIO_Port,fault_LED_Pin,GPIO_PIN_SET);
			torque_right=0;
			torque_left=0;
			error = 1;
			rtd_io=0;
			break;
		}
		case 3:{
			//bpps || apps out of range
			HAL_GPIO_WritePin(fault_LED_GPIO_Port,fault_LED_Pin,GPIO_PIN_SET);
			torque_right=0;
			torque_left=0;
			error = 1;
			rtd_io=0;
			break;
		}
		case 4:{
			//pedals =>enter pedals mode
			HAL_GPIO_WritePin(pedals_LED_GPIO_Port,pedals_LED_Pin,GPIO_PIN_SET);
			torque_right=0;
			torque_left=0;
			pedals=1;
			break;
		}
		case 5:{
			//brake acting
			torque_right=0;
			torque_left=0;
			break;
		}
		case 0:{
			//accelaration mode
			torque_command();
			break;
		}
			
	}
	if(pedals==1){
		if(APPS<APPSLeast){
			pedals=0;
			HAL_GPIO_WritePin(pedals_LED_GPIO_Port,pedals_LED_Pin,GPIO_PIN_RESET);
		}
	}
}
uint8_t check_safety(void){
		//detect disconnect
		if(APPSR>DISCONNECT || APPSL>DISCONNECT || BPPS>DISCONNECT){
			return 1;
		}
		//apps difference over APPS different  limit
		else if( __fabs ((APPSR - APPSROFFSET) - (APPSL - APPSLOFFSET))> APPSDIFF){
			return 2;
		}
		//check if sensor in the correct zone
		//BPPS 0~100 APPS 0~200
		else if(APPSR>APPSRUPPEST || APPSL>APPSLUPPEST || BPPS>BPPSUPPEST||APPSR<APPSRLOWEST || APPSL<APPSLLOWEST ||BPPS<BPPSLOWEST){
			return 3;
		}
		//braking >20
		
		else if(BPPS>BrakeAct){
			if(APPS > APPSAct){
				 pedals=1;	
				return 4;	 //braking with pedal not safe
			}
			return 5;   //braking safely
		}
		return 0;   //accelaration mode
}

uint16_t map(int16_t value, int16_t inputL,int16_t inputH,int16_t outputL ,int16_t outputH){
	if(value < inputL){
		return 0;
	}
	uint16_t returnVal=(value-inputL)*(outputH-outputL)/(inputH-inputL)+outputL;
	if(returnVal>outputH){
		return outputH;
	}else {
		return returnVal;
	}
}

void torque_command(void){
		if(pedals == 0){
			torque_right=map(APPS,0,(APPSRMAX+APPSLMAX-APPSROFFSET-APPSLOFFSET)/2,0,TORQUE_UB);
		}else{
			torque_right = 0;
		}
		//torque_ub = POWER_UB/(((float)rpm_left)*0.105);
		/*
		if (torque_right > torque_ub ){
			torque_right = torque_ub;
		}
		*/
		//torque_left= torque_right;
		
}

void torque_to_can(void){
	/*01 torque command (torque = ([0]+[1]*256)/10)
  23 speed command (angular speed = [2]+[3]*256
  4   Direction (0:reverse 1:forwards rd).  *further note: if the direction command is changed suddenly when the inverter is still enable, inverter is disable without triggering any fault
  And the Lockout mechanism is set again which will force the user to re-enable it
  5   5.0 inverter enable (0:off 1:ON)
       5.1 inverter discharge (0 disable 1 enable)
       5.2 speed mode
  67  commanded torque limit (0 default)
  this message should be continuously broadcast at least 500 milliseconds
	*/
	
	
	if(rtd_io==0){
		TxData_R[0]=0;
		TxData_R[1]=0;
		TxData_R[5]=0;
		
		TxData_L[0]=0;
		TxData_L[1]=0;
		TxData_L[5]=0;
	}else{
		//regenerate mode
		//torque_right *= -1; 
		// it will trans to 65536 + torque automatically
		
		TxData_R[0]=(torque_right%256);
		TxData_R[1]=(torque_right/256);
		TxData_R[5]=1;
			
		TxData_L[0] = (torque_right%256);
		TxData_L[1] = (torque_right/256);
		TxData_L[5]=1;
	}
}

void adxl_write(uint8_t address, uint8_t value)
{
	uint8_t data[2];
	data[0] = address|0x40;	//Multibyte write enabled
	data[1] = value;
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);	//Pull CS pin low to enable transmission
	HAL_SPI_Transmit(&hspi2, data, 2, 100);	//Transmit address and data
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);	//Pull CS pin high  to close transmission
}



void adxl_read(uint8_t address)
{
	address |= 0x80;	//read operation
	address |= 0x40;	//multibyte read
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &address, 1, 100);	//**
	HAL_SPI_Receive(&hspi2, data_rec, 6, 100);
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
}



void adxl_init()
{
	adxl_write(0x31, 0x01);
	adxl_write(0x2d, 0x00);
	adxl_write(0x2d, 0x08);
}


double map_double(double value, double inputL,double inputH,double outputL ,double outputH){
	
	double returnVal=(value-inputL)*(outputH-outputL)/(inputH-inputL)+outputL;
	if(returnVal>outputH){
		return outputH;
	}else {
		return returnVal;
	}
}

double a11(void){
	return -2*(Cr+Cf)/(MASS*speed_map);
}

double a12(void){
	return -1-2*(Lf*Cf-Lr*Cr)/(MASS*speed_map*speed_map);
}

double a21(void){
	return -2*(Lf*Cf-Lr*Cr)/Iz;
}

double a22(void){
	return -2*(Lf*Lf*Cf+Lr*Lr*Cr)/(Iz*speed_map);
}

double b11(void){
	return 2*Cf/(MASS*speed_map);
}

double b12(void){
	return 2*Lf*Cf/Iz;
}

double beta_diff(void){
	return a11()*beta+a12()*gamma+b11()*steer_map;
}

double gamma_diff(void){
	return a21()*beta+a22()*gamma+b12()*steer_map; 
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_IC_CaptureCallback could be implemented in the user file
   */
	
	
	//Timer 3 PWM interrupt
	if(htim->Instance == TIM3){
      if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
	 //Read Captured Value
	 //First Value - time period of the pulse
	 IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
	 
	 if(IC_Val1 != 0)
	 {
		 //Second Vlaue - time for the pulse remain high
		 IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		 
		 //Calculate Duty Cycle - set to integer = IC_Val2_2 / IC_Val1_2 * 100
		 Duty_Cycle_3 = IC_Val2 * 100 / IC_Val1;
		 
		 //Calculated Frequency - Timer Clock / IC_Val1_2, which timerclock = 2 * pclk1
		 Frequency_3 = 2 * HAL_RCC_GetPCLK1Freq() / IC_Val1;
	 }
	  else
		{
			Duty_Cycle_3 = 0;
			Frequency_3 = 0;
		}
	 }
			
	 if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
	 //Read Captured Value
	 //First Value - time period of the pulse
	
	 IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
	 
	 if(IC_Val1 != 0)
	 {
		 //Second Vlaue - time for the pulse remain high
		 IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		 
		 //Calculate Duty Cycle - set to integer = IC_Val2_2 / IC_Val1_2 * 100
		 Duty_Cycle_3_ = IC_Val2 * 100 / IC_Val1;
		 
		 //Calculated Frequency - Timer Clock / IC_Val1_2, which timerclock = 2 * pclk1
		 Frequency_3_ = 2 * HAL_RCC_GetPCLK1Freq() / IC_Val1;
	 }
	  else
		{
			Duty_Cycle_3_ = 0;
			Frequency_3_ = 0;
		}
	 }
	}
	
	//Timer 4 PWM interrupt
	if(htim->Instance == TIM4){
      if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
	 //Read Captured Value
	 //First Value - time period of the pulse
	 IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
	 
	 if(IC_Val1 != 0)
	 {
		 //Second Vlaue - time for the pulse remain high
		 IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		 
		 //Calculate Duty Cycle - set to integer = IC_Val2_2 / IC_Val1_2 * 100
		 Duty_Cycle_4 = IC_Val2 * 100 / IC_Val1;
		 
		 //Calculated Frequency - Timer Clock / IC_Val1_2, which timerclock = 2 * pclk1
		 Frequency_4 = 2 * HAL_RCC_GetPCLK1Freq() / IC_Val1;
	 }
	  else
		{
			Duty_Cycle_4 = 0;
			Frequency_4 = 0;
		}
	 }
			
	 if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
	 //Read Captured Value
	 //First Value - time period of the pulse
	
	 IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
	 
	 if(IC_Val1 != 0)
	 {
		 //Second Vlaue - time for the pulse remain high
		 IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		 
		 //Calculate Duty Cycle - set to integer = IC_Val2_2 / IC_Val1_2 * 100
		 Duty_Cycle_4_ = IC_Val2 * 100 / IC_Val1;
		 
		 //Calculated Frequency - Timer Clock / IC_Val1_2, which timerclock = 2 * pclk1
		 Frequency_4_ = 2 * HAL_RCC_GetPCLK1Freq() / IC_Val1;
	 }
	  else
		{
			Duty_Cycle_4_ = 0;
			Frequency_4_ = 0;
		}
	 }
	}
}


void setBuzzer(uint8_t value){
	TIM1 -> CCR1 = value;
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
