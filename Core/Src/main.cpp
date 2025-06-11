/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include "screen.h"
#include <vector>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// pins for warning lights
#define WARN_REG GPIOB
#define OIL_WARN_PIN GPIO_PIN_1
#define ECT_WARN_PIN GPIO_PIN_0

// pins for LED driver
#define LED_DRIVER_REG GPIOA
#define LE_PIN GPIO_PIN_3
#define OE_PIN GPIO_PIN_4
#define CLK_PIN GPIO_PIN_5
#define SDI_PIN GPIO_PIN_7

#define SCLK_PIN GPIO_PIN_5
#define MOSI_PIN GPIO_PIN_7

// Pins for input buttons
#define BTN_REG GPIOB
#define NEXT_BTN GPIO_PIN_10
#define PREV_BTN GPIO_PIN_11

#define LED_MASK 0b111111111111111111111111111111

#define RPM_MAX 14000
#define ONE_LED RPM_MAX / 30

#define LED_BRIGHTNESS 0b00110100


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxReader; // i dont think I need this -> I in fact don't need this
CAN_RxHeaderTypeDef RxHeader;



class ScreenHandler;
class ScreenPanel;
class Rx_TypeDef;
class Rx_LapStats;

Rx_TypeDef gear(0x60E);
Rx_TypeDef limpMode(0x61E);
Rx_TypeDef EOP(0x608);
Rx_TypeDef vBat(0x60C);
Rx_TypeDef RPM(0x600);
Rx_TypeDef oilTemp(0x604);
Rx_TypeDef ECT(0x605);
Rx_TypeDef lambda(0x602);
Rx_TypeDef speed(0x779);
Rx_LapStats laptime(0x777);
Rx_TypeDef brakeBias(0x779);
Rx_TypeDef diffBest(0x778);
Rx_TypeDef rollTime(0x780);
Rx_TypeDef sessTime(0x780);
Rx_TypeDef bestTime(0x778);

void LEDSeq(uint16_t segSeq, uint32_t ledBar);

volatile int RX_FLAG = 0;
volatile int BAM_MASK = 0b00000001;
int NUM_LEDS = 0;

//Rx_TypeDef rpmRx; // LED bar
//Rx_TypeDef oilRx; // oil pressure
//Rx_TypeDef ECTRx; // engine cooling temp

// Rx_TypeDef vBattRx -> battery voltage
// Rx_TypeDef LimpRx -> if in limp mode
// Rx_TypeDef oilTempRx
// Rx_TypeDef LambdaRx air-fuel ratio (different from perfect air fuel ratio) (perhaps figure out how to graph it? or calculating past 5 sec
// maybe - Rx_TypeDef LaptimeRx

// preset screens that driver can scroll through with buttons ?

// oil pressure is a function of rpm (they should be proportional), need to make a function wth rpm and oil pressure to know when to turn on warning light

uint8_t RxData[8]; //tag

uint16_t startnum[8] = {0b1110111, 0b0100010, 0b1101101, 0b1011101, 0b0011110, 0b1011011, 0b1111011, 0b0011111};
uint16_t gears[7] = {0b0111000, 0b0100010, 0b1101101, 0b1011101, 0b0011110, 0b1011011,0b1111011};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
extern void set_hi2c(I2C_HandleTypeDef hi2c);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	RX_FLAG = 1;
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) Error_Handler();
	  if(RxHeader.StdId ==  gear.getID())
			  gear._Update(RxData[3]);
	  else if(RxHeader.StdId == limpMode.getID())
			  limpMode._Update(RxData[0] << 8 | RxData[1]);
	  else if(RxHeader.StdId == EOP.getID())
			  EOP._Update((RxData[0] << 8 | RxData[1])); // oil pressure in psi
	  else if(RxHeader.StdId == vBat.getID())
			  vBat._Update(RxData[0] << 8 | RxData[1]);
	  else if(RxHeader.StdId ==  RPM.getID()){ // gear and speed have same id
			  RPM._Update(RxData[0] << 8 | RxData[1]);
			  NUM_LEDS = (RPM.getValue() * 30) / RPM_MAX;
			  if(gear.getValue() >= 0 && gear.getValue() < 7)
				  LEDSeq(gears[gear.getValue()-2], ~(LED_MASK >> NUM_LEDS));

	  }
	  else if(RxHeader.StdId == oilTemp.getID())
			  oilTemp._Update(RxData[2] << 8 | RxData[3]);
	  else if(RxHeader.StdId == ECT.getID())
			  ECT._Update((RxData[2] << 8 | RxData[3]) / 10);
	  else if(RxHeader.StdId == lambda.getID())
			  lambda._Update(RxData[0] << 8 | RxData[1]);
	  else if(RxHeader.StdId == laptime.getID()){
		  uint32_t temp = 0b0;
		  temp = RxData[0] | RxData[1] << 8 | RxData[2] << 16 | RxData[3] << 24;
		  laptime._Update(temp, RxData[4]);
	  }
	  else if(RxHeader.StdId == brakeBias.getID()){
		  brakeBias._Update(RxData[4] << 8 | RxData[5]);
	  	  speed._Update(RxData[6] << 8 | RxData[7]);
	  }
	  else if(RxHeader.StdId == diffBest.getID()){
		  uint32_t temp = 0b0;
		  temp = RxData[4] | RxData[5] << 8 | RxData[6] << 16 | RxData[7] << 24;
		  diffBest._Update(temp);
		  uint32_t temp2 = 0b0;
		  temp2 = RxData[0] | RxData[1] << 8 | RxData[2] << 16 | RxData[3] << 24;
		  bestTime._Update(temp2);
	  }
	  else if(RxHeader.StdId == rollTime.getID()){
		  uint32_t temp = 0b0;
		  temp = RxData[0] | RxData[1] << 8 | RxData[2] << 16 | RxData[3] << 24;
		  rollTime._Update(temp);
		  uint32_t temp2 = 0b0;
		  temp = RxData[4] | RxData[5] << 8 | RxData[6] << 16 | RxData[7] << 24;
		  diffBest._Update(temp2);

	  }
}

volatile int flg = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
#if 1
	if ((LED_BRIGHTNESS & BAM_MASK) == 0) HAL_GPIO_WritePin(LED_DRIVER_REG, OE_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LED_DRIVER_REG, OE_PIN, GPIO_PIN_RESET);
	BAM_MASK <<= 1; // shift mask to next bit pos
	TIM3->ARR <<= 1; // increase timer period by power of 2
	TIM3->CNT = 0; // reset count
	if (BAM_MASK == 0){
		BAM_MASK = 0b00000001;
		TIM3->ARR = 1;
	}
#endif
#if 0
	if(flg) {
		HAL_GPIO_WritePin(WARN_REG, ECT_WARN_PIN, GPIO_PIN_SET);
		flg = 0;
	}
	else{
		HAL_GPIO_WritePin(WARN_REG, ECT_WARN_PIN, GPIO_PIN_RESET);
		flg = 1;
	}
#endif

}


void LEDSeq(uint16_t segSeq, uint32_t ledBar)
{
	uint8_t n_led_drivers = 3;
	uint16_t bm_15 = 0b0111111111111111;
	uint16_t firstHalf = bm_15 & ledBar;
	uint16_t secondHalf = ledBar >> 15;
	uint16_t data[n_led_drivers];
	data[0] = segSeq;
	data[1] = secondHalf;
	data[2] = firstHalf;
	//. ok chat so basically there are three LED drivers each with 16 outputs, one set is for
	// the 7-seg number display (first 7 are the ones that light it up, last 8 are fake)
	// next 16 are for the second half (green/orange half) of the bar, 16th bit is fake
	// final 16 are for the first half (red/orange) of the bar, 1st bit is fake?


	HAL_SPI_Transmit(&hspi1, (uint8_t*) &data, n_led_drivers * 2, HAL_MAX_DELAY);

	// pull LE signal back low
	HAL_GPIO_WritePin(LED_DRIVER_REG, LE_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(LED_DRIVER_REG, LE_PIN, GPIO_PIN_RESET);

	// pull OE pin low (do i need this?)
//	HAL_GPIO_WritePin(LED_DRIVER_REG, OE_PIN, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(LED_DRIVER_REG, OE_PIN, GPIO_PIN_SET); // i don't think i need these
//	HAL_GPIO_WritePin(LED_DRIVER_REG, OE_PIN, GPIO_PIN_RESET);

}

void LEDEnable() // enable LEDs to be on
{
	HAL_GPIO_WritePin(LED_DRIVER_REG, OE_PIN, GPIO_PIN_RESET);
}
void LEDOff() // turn off all LEDs
{
	HAL_GPIO_WritePin(LED_DRIVER_REG, OE_PIN, GPIO_PIN_SET);
}

void startupSeq()
{
	  HAL_Delay(500);
	 for(int i = 1; i <= 30; i++){
		 LEDSeq(startnum[i/10], ~(LED_MASK>>i));
		 HAL_Delay(50);
	 }
	 for(int i = 1; i <= 30; i++){
		 LEDSeq(startnum[3 + i/10], LED_MASK<<i);
		 HAL_Delay(50);
	 }
	 HAL_Delay(50);
	 LEDSeq(0b0011111,0);

	HAL_Delay(500);
	LEDSeq(0,0);

}

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
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  set_hi2c(hi2c1);

  HAL_Delay(200);
  HAL_GPIO_WritePin(LED_DRIVER_REG, OE_PIN, GPIO_PIN_SET);

  // do boot before activating can interrupts so not interrupted?
  //reboot();
  clear_display();
  display_FSAE_bootscreen();
  startupSeq();
  HAL_Delay(500);

  clear_display();
  set_funBitmap();



  CAN_FilterTypeDef canfilterconfig;
  /* LIMP MODE */
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 1; // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x60E << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x60E << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  /* EOP */
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 2;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x608 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x608 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  /* VBAT */
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 3;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x60C << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0X60C << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  /* RPM AND SPEED */
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 4;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x600 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0X600 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  /* OIL TEMP */
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 5;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x604 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0X604 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  /* ECT */
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 6;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x605 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0X605 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  /* LAMBDA */
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 7;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x602 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0X602 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  /* GEAR */
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 8;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x60E << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0X60E << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 9;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x777 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x777 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 10;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x779 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x779 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 11;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x778 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x778 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 12;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x780 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x780 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);


  HAL_CAN_Start(&hcan);
  HAL_TIM_Base_Start_IT(&htim3);


  WarmupScreen warmupScreen(&ECT, &EOP, &vBat, &oilTemp, &lambda);
  DrivingScreen drivingScreen(&speed, &ECT, &limpMode, &RPM);
  ScreenHandler dashScreen;
  LapTimeScreen laptimeScreen(&laptime, &diffBest, &rollTime, &bestTime, &sessTime);

  dashScreen.addScreen(&warmupScreen);
  dashScreen.addScreen(&drivingScreen);
  dashScreen.addScreen(&laptimeScreen);

  dashScreen.startScreen();

  TIM3->ARR = 1;

  //dashScreen.nextScreen();
  if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler(); // enable CAN interrupts



  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(RX_FLAG) {

		  dashScreen.handle();
		  if(ECT.getValue() >= 105) HAL_GPIO_WritePin(WARN_REG, ECT_WARN_PIN, GPIO_PIN_SET);
		  if((RPM.getValue() <= 4000 && EOP.getValue() < 6)
		  	 | ((RPM.getValue() * 0.0058) > EOP.getValue())) HAL_GPIO_WritePin(WARN_REG, OIL_WARN_PIN, GPIO_PIN_SET);
		  else HAL_GPIO_WritePin(WARN_REG, OIL_WARN_PIN, GPIO_PIN_RESET);
		  RX_FLAG = 0;

	  }

	  //__disable_irq();

	  if(!HAL_GPIO_ReadPin(BTN_REG, NEXT_BTN)) dashScreen.nextScreen();
	  if(!HAL_GPIO_ReadPin(BTN_REG, PREV_BTN)) dashScreen.prevScreen();

	  //__enable_irq();


    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */


  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}


static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 256;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 255;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ECT_Warning_Output_Pin|OilP_Warning_Output_Pin|Debug_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ECT_Warning_Output_Pin OilP_Warning_Output_Pin */
  GPIO_InitStruct.Pin = ECT_Warning_Output_Pin|OilP_Warning_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : UserInput1_Pin UserInput2_Pin */
  GPIO_InitStruct.Pin = UserInput1_Pin|UserInput2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Debug_LED_Pin */
  GPIO_InitStruct.Pin = Debug_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Debug_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
