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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "File_Handling.h"
#include "stdio.h"
#include "stdlib.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SD_HandleTypeDef hsd;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
/* USER CODE BEGIN PFP */
//CAN_TxHeaderTypeDef TxHeader1;
//CAN_RxHeaderTypeDef RxHeader1;
//CAN_TxHeaderTypeDef TxHeader2;
//CAN_RxHeaderTypeDef RxHeader2;
//CAN_FilterTypeDef canFilter0;

uint8_t TxMessage[8];
uint8_t RxMessage[8];
uint32_t TxMailbox;

// Debugging
short enablePrintf = 1;
short enableTimers = 0;
short enableTimerPrints = 1;
short magnetometerPrints = 0;
short accelerometerPrints = 1;
short angularPrints = 0;
// Driver config
short enableDisplay = 0;
short enableWriteSD = 0;
short enableDiagnosticSD = 0;
short startupAnimation = 0; // A nice startup animation, if it's ever implemented.
// short dynamicBrightness = 0; // 0 is Off, 1 is On. Battery voltage-based brightness.
short dispBrightness[2] = {0x09,0x09}; // Startup Display Brightness 8-7SD || Gear+LEDs
short displayState[2] = {0,0}; // Startup Display Configuration

// CAN Bus variable declarations
unsigned int RPM = 0;
double TPS = 0; // %
double fuelOpenTime = 0; // ms
double ignitionAngle = 0; // Degrees
double barometer = 0; // PSI
double MAP = 0; // PSI
double lambda = 0; // ECU mapped Lambda #1 (A/F R) (DO NOT PRINT)
unsigned int pressureType = 0;
double AnIn1 = 0; // Radiator Air Temp (Post) (F/C)
double AnIn2 = 0; // Radiator Air Temp (Pre) (F/C)
double AnIn3 = 0; // Lambda #1 (A/F R)
double AnIn4 = 0; // Lambda #2 (A/F R)
double AnIn5 = 0; // Radiator Coolant Temp (Pre)
double AnIn6 = 0; // Radiator Coolant Temp (Post)
double AnIn7 = 0; // Oil Pressure (PSI - Display, kPa - SD Card)
double AnIn8 = 0; // MAF Sensor (?)
double freq1 = 0; // Wheel Speed Sensor (FR) (mph)
double freq2 = 0; // Wheel Speed Sensor (FL) (mph)
double freq3 = 0; // Wheel Speed Sensor (RR) (mph)
double freq4 = 0; // Wheel Speed Sensor (RL) (mph)
double batteryVoltage = 0;
double airTemp = 0; // (F)
double coolantTemp = 0; // Coolant temp at Engine
unsigned int tempType = 0;
double AnInT5 = 0; // (DO NOT PRINT SD)
double AnInT7 = 0; // (DO NOT PRINT SD)
int versionMajor = 0;
int versionMinor = 0;
int versionBuild = 0;
int TBD = 0;

// GPS variable declarations
int day, month, year, hour, minute, second, statFix, gSpeed;
float latitude, longitude, height_Ellipsoid, height_SeaLvl;
int uniqueID[6];

// ADC1 variable declarations
int damperT_Sense_FL = 0; // ADC1_IN0
int damperT_Sense_FR = 0; // ADC1_IN1
int damperT_Sense_RL = 0; // ADC1_IN4
int damperT_Sense_RR = 0; // ADC1_IN6

// ADC2 variable declarations
int steeringA_Sense = 0; // ADC2_IN7
int brakeP_Sense1 = 0; // ADC2_IN8
int brakeP_Sense2 = 0; // ADC2_IN9
int radiatorWaterTemp1 = 0; // ADC2_IN10
int radiatorWaterTemp2 = 0; // ADC2_IN11

// Accelerometer variable declarations
uint8_t receive;
uint8_t receiveArr[12];
int16_t pitchS = 0;
int16_t rollS = 0;
int16_t yawS = 0;
int16_t x_AccelS = 0;
int16_t y_AccelS = 0;
int16_t z_AccelS = 0;

double x_LS, y_LS, z_LS, roll_LS, pitch_LS, yaw_LS; // Print these to SD Card

// ADC DMA
uint16_t DMA_Store_ADC1[4];
uint16_t DMA_Store_ADC2[5];

// Display variable declarations
double gearing[6] = {2.883,2.062, 1.647, 1.421, 1.272, 1.173};
double finalDrive = 4.55;
double tireRadius = 10; // Inches
double computedSpeed;
double wheelSpeed = 0;
double lapTime = 0;
double oilPressure = 0;

unsigned short gear;

int dispCommFail = 0; // CAN, I2C, SPI, or UART Failure
int dispMCUFail = 0; // In the event of failure, I don't even know how this would be triggered. I sure hope it never does though.

short dataMark = 0;
uint8_t current7SegL[4] = {0,0,0,0};
uint8_t current7SegR[4] = {0,0,0,0};
uint8_t currentRPMBar = 0;
uint8_t enablePins[2] = {7,6}; // Left || Right
uint8_t digitAddresses[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}; // Leftmost to Rightmost
unsigned short dispUpdateDelay = 10; // milliseconds.
unsigned short launchDisplay = 0; // Waits for first CAN message so meaningful values can be sent to display.

const int dispRPM[2] = {10000, 12500}; // RPM Indicator: Warning Light, Max (RPM)
const int dispWaterTemp[2] = {32, 230}; // Temperature Warning Indicators: Low, High (Farenheit)
const int dispOilPressure[2] = {0, 90}; // Oil Pressure Indicator: Low, High (PSI)

uint8_t tempVar1;

void CAN1_TX(void);

//void initializeDisplays();
//void updateDisplaysComplex();
//void updateDisplaysFlush();
//void initializeLSM9DS1();
//void readLSM9DS1();
//void ambientCalcs();
//void testDisplay();
void csvHeader(char* FileName, int lenF);
void csvUpdate(char* FileName, int lenF);
void SD_Check(FRESULT fresult);
int SD_Status();

// SD Card
FRESULT fresult;
short int isSaving = 0;
short int detect = 0;
short int previousDetect = 0;
char* FileName = NULL;
int lenF = 0;
char buffer[100];
int indx = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
	for(int i=0; i<len; i++)
		ITM_SendChar((*ptr++));

	return len;
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
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

	/* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SDIO_SD_Init();
    MX_FATFS_Init();
    /* USER CODE BEGIN 2 */

    //Check if the card is already inserted or not
    if(Mount_SD("/")==FR_OK) {
		detect = 1;
		printf("%d) Card is already detected\n\n",detect);
	}
	else {
		detect = 0;
		printf("%d) No card detected\n\n",detect);
	}

	Unmount_SD("/");









  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)) {
			Unmount_SD("/");
			fresult = Mount_SD("/");
			if (fresult == FR_OK) {

				detect = 1;
				if (((previousDetect == 0 && detect == 1) || !FileName)) {
					printf("Creating a new file...\n");

					//Create new file with a GPS naming convention
					second = 53;
					minute = 33;
					hour = 14;
					day = 24;
					month = 3;
					year = 2021;
					char dayS[12], monthS[12], yearS[12], hourS[12], minuteS[12], secondS[12];
					sprintf(dayS, "%d", indx);
					sprintf(monthS, "%d", month);
					sprintf(yearS, "%d", year);
					sprintf(hourS, "%d", hour);
					sprintf(minuteS, "%d", minute);
					sprintf(secondS, "%d", second);
					lenF = strlen("DFR_")+strlen(dayS)+strlen("-")+strlen(monthS)+strlen("-")+strlen(yearS)+strlen("_")+strlen(secondS)+strlen("-")+strlen(minuteS)+strlen("-")+strlen(hourS)+strlen(".csv")+1;
					FileName = (char*)malloc(lenF * sizeof(char));
					snprintf(FileName,lenF,"DFR_%s-%s-%s_%s-%s-%s.CSV", dayS, monthS, yearS, secondS, minuteS, hourS);


					SD_Check(fresult);
					Format_SD();
					fresult = Create_File(FileName);
					SD_Check(fresult);
					Unmount_SD("/");

					csvHeader(FileName, lenF);

					previousDetect = detect;

				}



				HAL_GPIO_WritePin(GPIOA, LD2_Pin, 1);
				csvUpdate(FileName, lenF);
				indx++;
				printf("\n Count: %d \n", indx);

			}
			else {
				if(FileName) {
					free(FileName);
					FileName = NULL;
				}

				detect = 0;
				HAL_GPIO_WritePin(GPIOA, LD2_Pin, 0);
				printf("ERROR - Failed to mount...\n");

				previousDetect = detect;
			}
		}
		else {
			if(FileName) {
				free(FileName);
				FileName = NULL;
			}

			detect = 0;
			HAL_GPIO_WritePin(GPIOA, LD2_Pin, 0);
			printf("ERROR - No card inserted...\n");

			previousDetect = detect;
		}


		//To do: if(FileName) free(FileName); FileName = NULL;
		// OR check realloc


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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDIO|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 16;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(USART_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void SD_Check(FRESULT fresult) {
	if(fresult != FR_OK) {
		isSaving = 0;
		if(enablePrintf)
			printf("ERROR - SD card is NOT saving!\n");
	}
	else {
		isSaving = 1;
		if(enablePrintf)
			printf("Successfully saved to SD Card\n");
	}
}

int SD_Status() { return isSaving; }

void csvHeader(char* FileName, int lenF) {

	char name[lenF];
	strcpy(name, FileName);


	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)) {
		fresult = Mount_SD("/");
		SD_Check(fresult);

		//CAN Bus
		sprintf(buffer, "Time, RPM, TPS (%%), Fuel Open Time (ms), Ignition Angle (Degrees),");
		Update_File(name, buffer);
		sprintf(buffer, "Barometer (PSI), MAP (PSI), Pressure Type,");
		Update_File(name, buffer);
		sprintf(buffer, "Pre Radiator Air Temp (C), Post Radiator Air Temp (C),");
		Update_File(name, buffer);
		sprintf(buffer, "Labmda #1 (A/F R), Lambda #2 (A/F R),");
		Update_File(name, buffer);
		sprintf(buffer, "Pre Radiator Coolant Temp (C), Post Radiator Coolant Temp (C),");
		Update_File(name, buffer);
		sprintf(buffer, "Oil Pressure (PSI), Mass Air Flow Sensor (kg/s),");
		Update_File(name, buffer);
		sprintf(buffer, "FR Wheel Speed (mph), FL Wheel Speed (mph),");
		Update_File(name, buffer);
		sprintf(buffer, "RR Wheel Speed (mph), RL Wheel Speed (mph),");
		Update_File(name, buffer);
		sprintf(buffer, "Battery Voltage (V), Air Temp (C), Coolant Temp (C),");
		Update_File(name, buffer);

		//GPS
		sprintf(buffer, "Day, Month, Year, Hour, Minute, Second, gSpeed,");
		Update_File(name, buffer);
		sprintf(buffer, "Latitude, Longitude, Height Ellipsoid, Height Sea Level,");
		Update_File(name, buffer);

		//ADC1
		sprintf(buffer, "FL Damper Sensor, FR Damper Sensor,");
		Update_File(name, buffer);
		sprintf(buffer, "RL Damper Sensor, RR Damper Sensor,");
		Update_File(name, buffer);

		//ADC2
		sprintf(buffer, "Steering Sensor, Brake Sensor #1, Brake Sensor #2,");
		Update_File(name, buffer);

		//Ask about Unused #1 and #2

		//Accelerometer & Gyroscope
		sprintf(buffer, "X, Y, Z, Roll, Pitch, Yaw\n\n");
		fresult = Update_File(name, buffer);
		SD_Check(fresult);


		Unmount_SD("/");

	}

}

void csvUpdate(char* FileName, int lenF) {

	char name[lenF];
	strcpy(name, FileName);


	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)) {
		fresult = Mount_SD("/");
		SD_Check(fresult);


		sprintf(buffer, "%d,%hu,%f,%f,%f,", indx, RPM, TPS, fuelOpenTime, ignitionAngle);
		Update_File(name, buffer);
		sprintf(buffer, "%f,%f,%i,", barometer, MAP, pressureType);
		Update_File(name, buffer);
		sprintf(buffer, "%f,%f,%f,%f,%f,%f,%f,%f,", AnIn1, AnIn2, AnIn3, AnIn4, AnIn5, AnIn6, AnIn7, AnIn8);
		Update_File(name, buffer);
		sprintf(buffer, "%f,%f,%f,%f,%f,", freq1, freq2, freq3, freq4, batteryVoltage);
		Update_File(name, buffer);
		sprintf(buffer, "%f,%f,", airTemp, coolantTemp);
		Update_File(name, buffer);

		sprintf(buffer, "%d,%d,%d,%d,%d,%d,%d,", day, month, year, hour, minute, second, gSpeed);
		Update_File(name, buffer);
		sprintf(buffer, "%f,%f,%f,%f,", latitude, longitude, height_Ellipsoid, height_SeaLvl);
		Update_File(name, buffer);

		sprintf(buffer, "%d,%d,%d,%d,", damperT_Sense_FL, damperT_Sense_FR, damperT_Sense_RL, damperT_Sense_RR);
		Update_File(name, buffer);

		sprintf(buffer, "%d,%d,%d,", steeringA_Sense, brakeP_Sense1, brakeP_Sense2);
		Update_File(name, buffer);

		sprintf(buffer, "%f,%f,%f,%f,%f,%f", x_LS, y_LS, z_LS, roll_LS, pitch_LS, yaw_LS);
		Update_File(name, buffer);

		sprintf(buffer, "\n\n");
		fresult = Update_File(name, buffer);
		SD_Check(fresult);


		Unmount_SD("/");
	}

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
