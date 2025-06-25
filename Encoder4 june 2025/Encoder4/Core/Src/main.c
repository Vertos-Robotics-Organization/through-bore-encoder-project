/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rgb_led.h"
#include "MT6835.h"
#include "vl53l0x_api.h"
#include "non_blocking_morse.h"
#include "error_blink.h"
#include "flex_encoder.h"

#include "stm32g0xx_hal_flash.h"
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
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* USER CODE BEGIN PV */
float encAngle = 0.0;
int numTimeout = 0;
int numHAL_Error = 0;

int sensorMode = 0;
const int CPR = 2097152; // 2 ^ 21
const int BASE_ID = 0xA080000;
int device_id_index = 0x40;

const int POSITION_API_ID = 0;      // API ID for position data
const int VELOCITY_API_ID = 1;  // API ID for velocity/acceleration data
const int ACCELERATION_API_ID = 2; // API ID for acceleration data

int device_id = 0;
int pooptest = 1;
int proxMode = 0;
int towr = 0;


uint32_t flashAddress = 0x0803F010;  // Example address in Flash memory
uint64_t writeData;

int errorStatus = ENCODER_STATUS_OK;

// Global variable to track the last heartbeat time (in milliseconds)
volatile uint32_t lastHeartbeatTime = 0;

// Define the timeout period (e.g., 1000 ms = 1 second)
#define HEARTBEAT_TIMEOUT 1000

const int TARGET_LOOP_TIME = 10; // ms
//Velocity calculation variables
int64_t lastMultiTurnCounts;
double currentSystemTime;
double lastSystemTimeMillisec;
double deltaRotations;
double deltaTimeSeconds;
double sensorVelocityRPS;
double lastSensorVelocityRPS;
double sensorAccel;

int flexEncoderMode = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_USB_DRD_FS_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t buffer[64] = { 0 };
uint32_t cdcRxLen = 0;

FDCAN_TxHeaderTypeDef TxHeaderPosition;
FDCAN_TxHeaderTypeDef TxHeaderVelocity;
FDCAN_TxHeaderTypeDef TxHeaderAcceleration;

FDCAN_RxHeaderTypeDef RxHeader;

//uint32_t TxMailbox[4];

uint8_t TxDataPosition[64] = { 0 };
uint8_t TxDataVelocity[8] = { 0 };
uint8_t TxDataAcceleration[8] = { 0 };
uint8_t RxData[64] = { 0 };
int indx = 0;
uint8_t count = 0;

/***************************************************************************************************
 *
 *   HAL_FDCAN_RxFifo0Callback()
 *
 **************************************************************************************************/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
		/* Retreive Rx messages from RX FIFO0 */
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData)
				!= HAL_OK) {
			Error_Handler();
		}

		if (HAL_FDCAN_ActivateNotification(hfdcan,
		FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
			Error_Handler();
		}

		// update heartbeat counter
		if (RxHeader.Identifier == ROBORIO_CAN_ID) {
			lastHeartbeatTime = HAL_GetTick();
			return;
		}

		if (RxData[0] == 41) {
			device_id = RxData[2];
			TxHeaderPosition.Identifier = BASE_ID + device_id;

			//FLASH_PageErase(FLASH_BANK_1, 0x7E);
			writeData = TxHeaderPosition.Identifier;  // Example 64-bit data
			towr = 1;
		} else if (RxData[0] == 42) {
			pooptest = 1;
		} else if (RxData[0] == 43) {
			HAL_FLASH_Unlock();
			FLASH_PageErase(FLASH_BANK_1, 0x7E);
			HAL_FLASH_Lock();
			HAL_NVIC_SystemReset();
		}

		// This is the command for query all devices
		else if (RxData[0] == 44) {
			uint32_t uid[3];

			uid[0] = HAL_GetUIDw0();
			uid[1] = HAL_GetUIDw1();
			uid[2] = HAL_GetUIDw2();
			// send as device 0 even if multiple devices with conflict
			// Declare a new 8-bit array for debug purposes
			uint8_t uidBytes[12]; // 12 bytes (since 3 * 32 bits = 96 bits = 12 bytes)

			// Populate the 8-bit array with the values from the 32-bit UID array
			uidBytes[0] = (uint8_t) (uid[0] >> 24);
			uidBytes[1] = (uint8_t) (uid[0] >> 16);
			uidBytes[2] = (uint8_t) (uid[0] >> 8);
			uidBytes[3] = (uint8_t) (uid[0]);

			uidBytes[4] = (uint8_t) (uid[1] >> 24);
			uidBytes[5] = (uint8_t) (uid[1] >> 16);
			uidBytes[6] = (uint8_t) (uid[1] >> 8);
			uidBytes[7] = (uint8_t) (uid[1]);

			uidBytes[8] = (uint8_t) (uid[2] >> 24);
			uidBytes[9] = (uint8_t) (uid[2] >> 16);
			uidBytes[10] = (uint8_t) (uid[2] >> 8);
			uidBytes[11] = (uint8_t) (uid[2]);

			TxHeaderPosition.DataLength = 8;
			TxHeaderPosition.Identifier = BASE_ID + device_id_index;

			// Now pass the 8-bit array to the Tx FIFO queue function
			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeaderPosition, uidBytes);

			TxHeaderPosition.Identifier = BASE_ID + device_id;
		}

	}

} /* Emd pf HAL_FDCAN_RxFifo0Callback */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	/* USER CODE BEGIN 1 */
	HAL_StatusTypeDef positionCanStatus = HAL_ERROR;
	HAL_StatusTypeDef velocityCanStatus = HAL_ERROR;
  HAL_StatusTypeDef accelerationCanStatus = HAL_ERROR;

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
	MX_FDCAN1_Init();
	MX_SPI1_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_I2C1_Init();
	MX_USB_DRD_FS_PCD_Init();
	MX_TIM2_Init();
	MX_SPI2_Init();
	/* USER CODE BEGIN 2 */

	// Flex Encoder
	// Initialize the encoder system
  // if (FlexEncoder_Init(&hi2c1) != HAL_OK) {
  //     Error_Handler();  // Fail-safe if setup fails
  // }

	// Call the hue cycling function
	// Start PWM on all timer channels
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  ////#define FLASH_USER_START_ADDR   0x0800F800UL
  //    // Let's store an integer (e.g. 0x12345678) in page "index = 0"
  //	HAL_FLASH_Unlock();
  //	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flashAddress, 0x12345678deadbeef);
  //	HAL_FLASH_Lock();
  //	UserFlash_WriteInt(0xdeadbeef, 0);
  //	long poodata = UserFlash_ReadInt(0);
  //
  //	HAL_FLASH_Unlock();
  //	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flashAddress+8, 0x12345678);
  //	HAL_FLASH_Lock();

	// FLASH_EraseInitTypeDef EraseInitStruct;
	// uint32_t PageError;
	// Unlock Flash

	// Read back from Flash
	uint64_t readData = *((uint64_t*) flashAddress);

  //	float hue = 0.0;
  //	const float hue_increment = 1.0 / 100.0; // Increment to cycle in 1 second (100 steps)
  //
  //	for (int step = 0; step < 100; ++step)
  //	{
  //		set_led_hue(hue, 1.0);
  //
  //		hue += hue_increment;
  //		HAL_Delay(5); // Delay 10 ms to achieve 1-second total duration
  //	}

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler();
	}
  
	if ((uint32_t) readData == 0xffffffff) {
		TxHeaderPosition.Identifier = BASE_ID;
	} else {
		TxHeaderPosition.Identifier = (uint32_t) readData;
	}

	// Configure position message header
	txHeaderConfigure(&TxHeaderPosition);  // Note the & (address-of operator)
	// Configure velocity message header
	txHeaderConfigure(&TxHeaderVelocity);  // Note the & (address-of operator)
	// Configure velocity message header
	txHeaderConfigure(&TxHeaderAcceleration);  // Note the & (address-of operator)


	// ADC
	HAL_ADC_Start(&hadc1);

	// start CAN IC
	// Enable CAN transceiver by setting EN and STB_N HIGH
	HAL_GPIO_WritePin(CAN_ENABLE_GPIO_Port, CAN_ENABLE_Pin, GPIO_PIN_SET); // EN HIGH
	HAL_GPIO_WritePin(CAN_NSTANDBY_GPIO_Port, CAN_NSTANDBY_Pin, GPIO_PIN_SET); // STB_N HIGH

	// Check if ERR_N is low, indicating an error or flag set
	int err_flag = HAL_GPIO_ReadPin(CAN_NERROR_GPIO_Port, CAN_NERROR_Pin);

	if (err_flag == GPIO_PIN_RESET) { // ERR_N is LOW
		// Clear flags by toggling STB_N or EN based on the datasheet
		HAL_GPIO_WritePin(CAN_NSTANDBY_GPIO_Port, CAN_NSTANDBY_Pin, GPIO_PIN_RESET); // STB_N LOW
		HAL_Delay(10); // Allow some stabilization time
		HAL_GPIO_WritePin(CAN_NSTANDBY_GPIO_Port, CAN_NSTANDBY_Pin, GPIO_PIN_SET);   // STB_N HIGH

		// Optionally toggle EN if STB_N toggle doesn't clear the flag
		HAL_GPIO_WritePin(CAN_ENABLE_GPIO_Port, CAN_ENABLE_Pin, GPIO_PIN_RESET); // EN LOW
		HAL_Delay(10); // Allow stabilization
		HAL_GPIO_WritePin(CAN_ENABLE_GPIO_Port, CAN_ENABLE_Pin, GPIO_PIN_SET); // EN HIGH
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		uint32_t currentTimeMillisec = HAL_GetTick();

		/* Flex Encoder Capabilities*/
    // if (flexEncoderMode == 1) {
    //     FlexEncoder_UpdateAll();

    //     int16_t data[3][MLX_COUNT];
    //     FlexEncoder_Get(data);

    //     // Access data:
    //     int16_t x0 = data[0][0];  // X from sensor 0
    //     int16_t z2 = data[2][2];  // Z from sensor 2
    //     //HAL_Delay(2000);
    // }


		if (towr == 1) {
			towr = 0;
			// my page erase thing messes this up
			HAL_FLASH_Unlock();
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
							  flashAddress,
							  writeData
			);
			HAL_FLASH_Lock();
		}

		// Read the angle and set LED hue
		//encAngle = mt6835_read_angle(&hspi1);

		// Prepare data for CAN transmission
		// Read the angle as counts
		mt6835_update_counts(&hspi1);
		get_counts_single_turn();
		//mt6835_read_counts(&hspi1);

		// Use union for safe double-to-bytes conversion
		union {
		    double d;
		    uint8_t bytes[8];
		} absoluteUnion, relativeUnion, velocityUnion, accelUnion;

		//----------------------------------------------------------------------------------------------------------
		// Read the multi-turn counts and single-turn counts
		//----------------------------------------------------------------------------------------------------------
		int64_t multiTurnCounts = get_counts_multi_turn();
		uint32_t singleTurnCounts = get_counts_single_turn();

		// Calculate absolute rotations (total rotations including multi-turn)
		double absoluteRotations = (double) multiTurnCounts / (double) CPR;

		// Calculate relative rotations (single turn position as fraction 0.0 to 1.0)
		double relativeRotations = (double) singleTurnCounts / (double) CPR;

		// Direct assignment
		absoluteUnion.d = absoluteRotations;
		relativeUnion.d = relativeRotations;


		// Pack absolute rotations (8 bytes, big-endian)
		TxDataPosition[0] = absoluteUnion.bytes[7];  // MSB
		TxDataPosition[1] = absoluteUnion.bytes[6];
		TxDataPosition[2] = absoluteUnion.bytes[5];
		TxDataPosition[3] = absoluteUnion.bytes[4];
		TxDataPosition[4] = absoluteUnion.bytes[3];
		TxDataPosition[5] = absoluteUnion.bytes[2];
		TxDataPosition[6] = absoluteUnion.bytes[1];
		TxDataPosition[7] = absoluteUnion.bytes[0];  // LSB

		//-----------------------------------------------------------------------------------------------------------
		// Velocity and Acceleration calculations
		//-----------------------------------------------------------------------------------------------------------
		// calculate the sensor velocity based on the difference in multi-turn counts and time
		deltaRotations = ((double)(multiTurnCounts - lastMultiTurnCounts) / (double) CPR);
		deltaTimeSeconds = (double) ((currentTimeMillisec - lastSystemTimeMillisec) / 1000);
		if(deltaTimeSeconds == 0) {
		  deltaTimeSeconds = 0.0001; // Prevent divide by zero scenarios and do not update velocity/accel variables
		} else {
		  //Final calculation
		  sensorVelocityRPS = deltaRotations / deltaTimeSeconds;
		  sensorAccel = ((sensorVelocityRPS - lastSensorVelocityRPS) / deltaTimeSeconds);

		  // record the last values for the next call
		  lastMultiTurnCounts = multiTurnCounts;
		  lastSystemTimeMillisec = currentTimeMillisec;
		  lastSensorVelocityRPS = sensorVelocityRPS;
		}

		velocityUnion.d = sensorVelocityRPS;
		accelUnion.d = sensorAccel;

		// Pack velocity (8 bytes)
		TxDataVelocity[0] = velocityUnion.bytes[7];
		TxDataVelocity[1] = velocityUnion.bytes[6];
		TxDataVelocity[2] = velocityUnion.bytes[5];
		TxDataVelocity[3] = velocityUnion.bytes[4];
		TxDataVelocity[4] = velocityUnion.bytes[3];
		TxDataVelocity[5] = velocityUnion.bytes[2];
		TxDataVelocity[6] = velocityUnion.bytes[1];
		TxDataVelocity[7] = velocityUnion.bytes[0];

		// Pack acceleration (8 bytes)
		TxDataAcceleration[0] = accelUnion.bytes[7];
		TxDataAcceleration[1] = accelUnion.bytes[6];
		TxDataAcceleration[2] = accelUnion.bytes[5];
		TxDataAcceleration[3] = accelUnion.bytes[4];
		TxDataAcceleration[4] = accelUnion.bytes[3];
		TxDataAcceleration[5] = accelUnion.bytes[2];
		TxDataAcceleration[6] = accelUnion.bytes[1];
		TxDataAcceleration[7] = accelUnion.bytes[0];

		// Add message to CAN Tx FIFO queue // Base id all the stuff set by first // device id can id teams are familiar with //
		// Calculate base CAN ID for this device
		uint32_t baseCanId = BASE_ID + device_id;

		// Position message: embed API ID 0 in the CAN arbitration ID
		TxHeaderPosition.Identifier = baseCanId | (POSITION_API_ID << 6);      // API 0: baseCanId | 0x000
		TxHeaderVelocity.Identifier = baseCanId | (VELOCITY_API_ID << 6);       // API 1: baseCanId | 0x400
		TxHeaderAcceleration.Identifier = baseCanId | (ACCELERATION_API_ID << 6); // API 2: baseCanId | 0x800

		// returns HAL_Error if fifo queue is full or CAN is not initialized correctly
		// HAL_OK otherwise
		positionCanStatus = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeaderPosition, TxDataPosition);
		velocityCanStatus = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeaderVelocity, TxDataVelocity);
		accelerationCanStatus = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeaderAcceleration, TxDataAcceleration);


		//this is required for auto reboot when can bus disconnects
		if (positionCanStatus != HAL_OK || 
			velocityCanStatus != HAL_OK ||
			accelerationCanStatus != HAL_OK) {
			if ((currentTimeMillisec - lastHeartbeatTime) > HEARTBEAT_TIMEOUT) {
				errorStatus = ENCODER_STATUS_NO_CANBUS;
			} else {
				errorStatus = ENCODER_STATUS_CAN_TX_FIFO_FULL;
			}
		}

		if (handle_error_blink(errorStatus) == ENCODER_STATUS_OK) {
			set_led_hue((float) singleTurnCounts / CPR, 1.0);
		}



		/* CAN Error reading */
		// if (errorStatus == ENCODER_STATUS_OK)
		// {
		// 	// normal
		// 	float hueTest = (float) singleTurnCounts / CPR;
		// }
		// else if (errorStatus == ENCODER_STATUS_NO_CANBUS) {

		// 			pooptest = 1;
		// }
		// // todo other errors

		// // Error
		// if (pooptest > 0) {
		// 	blink_led_morse_init("SOS", 0.666); // code, color
		// 	pooptest = 0;

		// }
		// if(blink_led_morse_process())
		// {
		// 	pooptest = 1;
		// }


		/* LED modes*/
		if (proxMode) {
			// Poll for ADC conversion completion
			if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
				// Get the ADC value. Between 4095 and 0
				int adcValue = HAL_ADC_GetValue(&hadc1);
				if (adcValue > 2500 || adcValue < 1500) {
					//set_led_rgb(0, 0, 255);

				} else {
					//set_led_rgb(0, 255, 0);
				}

				// Optional: Process the ADC value
			} else {

			}
		}

		// currentTime is time at START of the loop
		uint32_t loopTime = HAL_GetTick() - currentTimeMillisec;
		if (loopTime > TARGET_LOOP_TIME) {
			errorStatus = ENCODER_STATUS_LOOP_OVERRUN;
		}

	}
    /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void) {

  /* USER CODE BEGIN FDCAN1_Init 0 */
  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */
  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 31;
  hfdcan1.Init.NominalTimeSeg1 = 54;
  hfdcan1.Init.NominalTimeSeg2 = 9;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 32;
  hfdcan1.Init.DataTimeSeg2 = 3;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 2;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
	FDCAN_FilterTypeDef sFilterConfig;

	sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	sFilterConfig.FilterIndex = 0;
	//sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;

	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

	// Accept only ID 0xA080000
	sFilterConfig.FilterID1 = 0xA070000;
	sFilterConfig.FilterID2 = 0xA090000 + 32;

	//sFilterConfig.RxBufferIndex = 0;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}

	FDCAN_FilterTypeDef sFilterConfig2;

	sFilterConfig2.IdType = FDCAN_EXTENDED_ID;
	sFilterConfig2.FilterIndex = 1;
	sFilterConfig2.FilterType = FDCAN_FILTER_DUAL;

	sFilterConfig2.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

	sFilterConfig2.FilterID1 = ROBORIO_CAN_ID;
	sFilterConfig2.FilterID2 = 0x0;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig2) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}

	// everything
	// if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_ACCEPT_IN_RX_FIFO0,
	// 	                                                     FDCAN_REJECT_REMOTE,
	// 														 FDCAN_REJECT_REMOTE) != HAL_OK)
	// {
	// 	Error_Handler();
	// }

	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
									FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
		Error_Handler();
	}

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void){

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00C12166;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
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
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void) {

  /* USER CODE BEGIN SPI2_Init 0 */
	// Non encoder IC IO
  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1600-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1600-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void) {

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1600-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 255-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USB_DRD_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_DRD_FS_PCD_Init(void) {

  /* USER CODE BEGIN USB_DRD_FS_Init 0 */

  /* USER CODE END USB_DRD_FS_Init 0 */

  /* USER CODE BEGIN USB_DRD_FS_Init 1 */

  /* USER CODE END USB_DRD_FS_Init 1 */
  hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
  hpcd_USB_DRD_FS.Init.dev_endpoints = 8;
  hpcd_USB_DRD_FS.Init.Host_channels = 8;
  hpcd_USB_DRD_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.bulk_doublebuffer_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.iso_singlebuffer_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK){
    Error_Handler();
  }
  /* USER CODE BEGIN USB_DRD_FS_Init 2 */

  /* USER CODE END USB_DRD_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CAL_EN_Pin|GPIO_PIN_12|CAN_NSTANDBY_Pin|CAN_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CAL_EN_Pin PB12 CAN_NSTANDBY_Pin CAN_ENABLE_Pin */
  GPIO_InitStruct.Pin = CAL_EN_Pin|GPIO_PIN_12|CAN_NSTANDBY_Pin|CAN_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_NERROR_Pin */
  GPIO_InitStruct.Pin = CAN_NERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CAN_NERROR_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void txHeaderConfigure(FDCAN_TxHeaderTypeDef* header) {  // Note the pointer!
    header->IdType = FDCAN_EXTENDED_ID;
    header->TxFrameType = FDCAN_DATA_FRAME;
    header->DataLength = FDCAN_DLC_BYTES_8;
    header->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header->BitRateSwitch = FDCAN_BRS_OFF;
    header->FDFormat = FDCAN_CLASSIC_CAN;    //dont use fd FDCAN_FD_CAN;
    header->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header->MessageMarker = 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
	uint32_t elapsed_time = 0;  // Track elapsed time in milliseconds

	while (1) {
		// Blink the LED
		set_led_rgb(255, 0, 0);
		HAL_Delay(500);

		set_led_rgb(0, 0, 0);
		HAL_Delay(500);

		// Increment elapsed time
		elapsed_time += 1000; // 500ms + 500ms per loop iteration

		// Reset the chip after 10 seconds (10,000 milliseconds)
		if (elapsed_time >= 10000) {
			// Trigger a system reset
			NVIC_SystemReset();
		}
	}
	/* User can add his own implementation to report the HAL error return state */
//	__disable_irq();
//	while (1) {
//
//	}
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
