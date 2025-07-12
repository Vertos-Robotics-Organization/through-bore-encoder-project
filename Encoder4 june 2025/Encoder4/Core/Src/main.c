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
const int BASE_ID = 0xA110000;
int device_id_index = 0x40;

const int POSITION_API_ID = 0;          // API ID for position data (8 bytes - 64-bit counts)
const int VELOCITY_ACCEL_API_ID = 1;   // API ID for combined velocity/acceleration data (8 bytes)
const int BOOLEAN_STATUS_API_ID = 2;   // API ID for boolean status messages (1 byte)

int device_id = 0;
int pooptest = 1;
int proxMode = 0;
int towr = 0;
int encoder_ready = 0;


uint32_t flashAddress = 0x0803F010;  // Example address in Flash memory
uint64_t writeData;


// Global variable to track the last heartbeat time (in milliseconds)
volatile uint32_t lastHeartbeatTime = 0;

// Define the timeout period (e.g., 1000 ms = 1 second)
#define HEARTBEAT_TIMEOUT 1000
#define ROBORIO_CAN_ID 0x01011840


// At the top of main.c, define a backup register or flash address for the sticky flag
#define RESET_STICKY_FLAG_ADDR 0x0803F020 // Example flash address

// Constants for scaling to 16-bit values
const double VELOCITY_SCALE_FACTOR = 1000.0;     // Scale velocity to fit in 16-bit range
const double ACCELERATION_SCALE_FACTOR = 100.0;  // Scale acceleration to fit in 16-bit range

const int TARGET_LOOP_TIME = 10; // ms

// Advanced derivative calculation variables
double sensorVelocityRPS = 0.0;
double sensorAccel = 0.0;

//--------------------------------------------------------------------------------
// Dynamic Linear Regression Configuration
//--------------------------------------------------------------------------------
// Store current sample
// Simple backward difference velocity calculation with low-pass filtering
static float velocity_alpha = 0.2f; // Low-pass filter coefficient (0.0 - 1.0)
static float accel_alpha = 0.1f;    // Lower for acceleration

static int64_t prevMultiTurnCounts = 0;
static uint32_t prevTimeMillisec = 0;
static float filteredVelocityCPS = 0.0f;
static float filteredAccelCPS2 = 0.0f;

// Optimized integer-based velocity/acceleration calculation variables
int64_t lastMultiTurnCounts = 0;
uint32_t lastSystemTimeMillisec = 0;
int32_t lastVelocityCPS = 0;        // Last velocity in counts per second
int32_t lastAccelCPS2 = 0;          // Last acceleration in counts per second squared
int32_t velocityCountsScaled = 0;   // Final scaled velocity for CAN transmission
int32_t accelCountsScaled = 0;      // Final scaled acceleration for CAN transmission

// Boolean status variables (8 different boolean messages)
int isHardwareFault = 0;         // Bit 0: hardware fault status
int isLoopOverrun = 0;           // Bit 1: loop overrun status
int isCANInvalid = 0;            // Bit 2: CAN communication status
int isResetDuringEnable = 0;      // Bit 3: reset during enable status
int isMagnetWeakSignal = 0;       // Bit 4: magnet weak signal status
int isRotationOverspeed = 0;     // Bit 5: rotation overspeed status
int isCANClogged = 0;            // Bit 6: CAN bus clogged status
int isUnderVolted = 0;           // Bit 7: under-voltage status

static int mt6835_error_count = 0;

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
// Function prototypes for velocity calculation API
int setVelocityCalculationSamples(int samples);
int getVelocityCalculationSamples(void);
int getMaxVelocityCalculationSamples(void);
int getMinVelocityCalculationSamples(void);
int getVelocityCalculationInfo(int samples, uint32_t* timeSpanMs, int* noiseReduction);

// static inline const int64_t* get_timeCoeffs(void);
// static inline int64_t get_timeSquaredSum(void);

// ADD THESE MISSING PROTOTYPES:
void reset_counts(void);
void invert_encoder_direction(int direction);
void set_counts(int64_t newPosition);
void set_encoder_direction(int direction);
void reset_all_faults(void);
void txHeaderConfigure(FDCAN_TxHeaderTypeDef* header);
void txHeaderConfigureBooleanStatus(FDCAN_TxHeaderTypeDef* header);
int was_reset_during_enable(void);
void clear_reset_sticky_flag(void);
void set_reset_sticky_flag(void);
int get_weak_magnetic_field_warning(void);
int get_under_voltage_warning(void);
int get_rotation_overspeed_warning(void);
int64_t get_counts_multi_turn(void);
uint32_t get_counts_single_turn(void);
void set_led_hue(float hue, float brightness);
void set_led_rgb(uint8_t r, uint8_t g, uint8_t b);
void handle_error_blink(const char* pattern, float hue);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t buffer[64] = { 0 };
uint32_t cdcRxLen = 0;

FDCAN_TxHeaderTypeDef TxHeaderPosition;
FDCAN_TxHeaderTypeDef TxHeaderVelocityAccel;
FDCAN_TxHeaderTypeDef TxHeaderBooleanStatus;  // New header for boolean status

FDCAN_RxHeaderTypeDef RxHeader;

//uint32_t TxMailbox[4];

uint8_t TxDataPosition[8] = { 0 };
uint8_t TxDataVelocityAccel[8] = { 0 };  // Combined velocity and acceleration (8 bytes)
uint8_t TxDataBooleanStatus[1] = { 0 };  // Boolean status data (1 byte)
uint8_t RxData[64] = { 0 };
int indx = 0;
uint8_t count = 0;

uint32_t get_tx_identifier(uint32_t baseCanId, uint32_t apiId) {
    return (baseCanId + device_id) | (apiId << 10);  // Add device_id like Java does
}

// Add this helper function to unpack IEEE 754 float from big-endian bytes
float bytes_to_float(uint8_t* bytes, int offset) {
    uint32_t temp = ((uint32_t)bytes[offset] << 24) |
                    ((uint32_t)bytes[offset + 1] << 16) |
                    ((uint32_t)bytes[offset + 2] << 8) |
                    ((uint32_t)bytes[offset + 3]);

    // Convert uint32_t bits to float
    float result;
    memcpy(&result, &temp, sizeof(float));
    return result;
}


/***************************************************************************************************
 *
 *   HAL_FDCAN_RxFifo0Callback()
 *
 **************************************************************************************************/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        // 1. Retrieve the received CAN message from FIFO0
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
            Error_Handler();
        }

        // 2. Re-enable the notification for new messages
        if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
            Error_Handler();
        }

        // 3. If the message is a heartbeat (from RoboRIO), update the heartbeat timer
        if (RxHeader.Identifier == ROBORIO_CAN_ID) {
            lastHeartbeatTime = HAL_GetTick();
            return;
        }

        // 4. Handle specific command messages based on RxHeader.Identifier
        if (RxHeader.Identifier == get_tx_identifier(BASE_ID, 41)) {
            device_id = RxData[2];
            TxHeaderPosition.Identifier = BASE_ID + device_id;
            writeData = TxHeaderPosition.Identifier;
            towr = 1;
        } else if (RxHeader.Identifier == get_tx_identifier(BASE_ID, 42)) {
            pooptest = 1;
        } else if (RxHeader.Identifier == get_tx_identifier(BASE_ID, 43)) {
            HAL_FLASH_Unlock();
            FLASH_PageErase(FLASH_BANK_1, 0x7E);
            HAL_FLASH_Lock();
        } else if (RxHeader.Identifier == get_tx_identifier(BASE_ID, 10)) {  // ZERO_ENCODER_API_ID
            // Zero the encoder
            reset_counts();
        } else if (RxHeader.Identifier == get_tx_identifier(BASE_ID, 11)) {  // INVERT_DIRECTION_API_ID
            // Invert direction logic clockwise
            invert_encoder_direction(1);
        } else if (RxHeader.Identifier == get_tx_identifier(BASE_ID, 12)) {
            invert_encoder_direction(-1);  // INVERT_DIRECTION_API_ID
        } else if (RxHeader.Identifier == get_tx_identifier(BASE_ID, 13)) {  // SET_POSITION_API_ID

        } else if (RxHeader.Identifier == get_tx_identifier(BASE_ID, 14)) {  // SET_POSITION_API_ID
			set_led_hue((float)0.8, 1.0);

            if (RxHeader.DataLength == 8) {
                float newPosition = bytes_to_float(RxData, 0);  // First 4 bytes
                float timeout = bytes_to_float(RxData, 4);      // Next 4 bytes

                // Convert position from rotations to counts
                int64_t newCounts = (int64_t)(newPosition * CPR);
                set_counts(newCounts);

                // You can use the timeout value for any timeout logic you need
            }
        }

        // 5. Respond to a "query all devices" command
        else if (RxHeader.Identifier == get_tx_identifier(BASE_ID, 44)) {
            uint32_t uid[3];
            uid[0] = HAL_GetUIDw0();
            uid[1] = HAL_GetUIDw1();
            uid[2] = HAL_GetUIDw2();
            uint8_t uidBytes[12];
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
	HAL_StatusTypeDef velocityAccelCanStatus = HAL_ERROR;
	HAL_StatusTypeDef booleanStatusCanStatus = HAL_ERROR;  // New status for boolean messages

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
  //     booleanStatus2 = 0;  // Encoder initialization failed
  // } else {
  //     booleanStatus2 = 1;  // Encoder initialization successful
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
		isCANInvalid = 0;  // CAN initialization failed
	} else {
		isCANInvalid = 1;  // CAN initialization successful
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler();
	}
  
	if ((uint32_t) readData == 0xffffffff) {
		TxHeaderPosition.Identifier = BASE_ID;
	} else {
		TxHeaderPosition.Identifier = (uint32_t) readData;
	}

	// Configure position message header (8 bytes)
	txHeaderConfigure(&TxHeaderPosition);
	// Configure velocity/acceleration message header (8 bytes)
	txHeaderConfigure(&TxHeaderVelocityAccel);
	// Configure boolean status message header (1 byte)
	txHeaderConfigureBooleanStatus(&TxHeaderBooleanStatus);

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

    if (mt6835_update_counts(&hspi1) == HAL_OK) {
        // Wait for a few successful reads before declaring ready
        for (int i = 0; i < 10; i++) {
            if (mt6835_update_counts(&hspi1) == HAL_OK) {
                HAL_Delay(10);
            } else {
                break;
            }
        }
        encoder_ready = 1;
    }

    // Set the flag ONCE to detect future resets
    set_reset_sticky_flag();
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
			isRotationOverspeed = 1;  // Flash write successful
		}

		// Read the angle and set LED hue
		//encAngle = mt6835_read_angle(&hspi1);

		// Prepare data for CAN transmission
		// Read the angle as counts
		//mt6835_read_counts(&hspi1);

		//----------------------------------------------------------------------------------------------------------
		// Read the multi-turn counts and single-turn counts
		//----------------------------------------------------------------------------------------------------------
		int64_t multiTurnCounts = get_counts_multi_turn();
		uint32_t singleTurnCounts = get_counts_single_turn();

	    if (encoder_ready) {
	        // Single encoder update call per loop
	        if (mt6835_update_counts(&hspi1) == HAL_OK) {

				// Pack multi-turn counts directly as 64-bit value (8 bytes, big-endian)
				TxDataPosition[0] = (uint8_t)(multiTurnCounts >> 56);  // MSB
				TxDataPosition[1] = (uint8_t)(multiTurnCounts >> 48);
				TxDataPosition[2] = (uint8_t)(multiTurnCounts >> 40);
				TxDataPosition[3] = (uint8_t)(multiTurnCounts >> 32);
				TxDataPosition[4] = (uint8_t)(multiTurnCounts >> 24);
				TxDataPosition[5] = (uint8_t)(multiTurnCounts >> 16);
				TxDataPosition[6] = (uint8_t)(multiTurnCounts >> 8);
				TxDataPosition[7] = (uint8_t)(multiTurnCounts);        // LSB

				int flexEncoderMode = 1;

				//-----------------------------------------------------------------------------------------------------------
				// Dynamic Linear Regression velocity and acceleration calculations
				//-----------------------------------------------------------------------------------------------------------

				if (prevTimeMillisec != 0) {
					int64_t deltaCounts = multiTurnCounts - prevMultiTurnCounts;
					uint32_t deltaTime = currentTimeMillisec - prevTimeMillisec;

					if (deltaTime > 0) {
						  float velocityCPS = (float)deltaCounts * 1000.0f / (float)deltaTime;

						  // Low-pass filter for velocity
						  filteredVelocityCPS = velocity_alpha * velocityCPS + (1.0f - velocity_alpha) * filteredVelocityCPS;

						  // Acceleration (backward difference of velocity)
						  float accelCPS2 = (filteredVelocityCPS - lastVelocityCPS) * 1000.0f / (float)deltaTime;
						  filteredAccelCPS2 = accel_alpha * accelCPS2 + (1.0f - accel_alpha) * filteredAccelCPS2;

						  velocityCountsScaled = (int32_t)filteredVelocityCPS;
						  accelCountsScaled = (int32_t)filteredAccelCPS2;

						  lastVelocityCPS = filteredVelocityCPS;
						  lastAccelCPS2 = filteredAccelCPS2;
					}
				}

				prevMultiTurnCounts = multiTurnCounts;
				prevTimeMillisec = currentTimeMillisec;


				// Status 5: Proximity sensor status (based on ADC reading)
				if (proxMode) {
					if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
						int adcValue = HAL_ADC_GetValue(&hadc1);
					}
				}

				TxDataBooleanStatus[0] = 0;  // Start with empty byte: 00000000
				TxDataBooleanStatus[0] |= (isHardwareFault & 0x01) << 0;       // Add bit 0
				TxDataBooleanStatus[0] |= (isLoopOverrun & 0x01) << 1;         // Add bit 1
				TxDataBooleanStatus[0] |= (isCANInvalid & 0x01) << 2;          // Add bit 2
				TxDataBooleanStatus[0] |= (isResetDuringEnable & 0x01) << 3;   // Add bit 3
				TxDataBooleanStatus[0] |= (isMagnetWeakSignal & 0x01) << 4;    // Add bit 4
				TxDataBooleanStatus[0] |= (isRotationOverspeed & 0x01) << 5;   // Add bit 5
				TxDataBooleanStatus[0] |= (isCANClogged & 0x01) << 6;          // Add bit 6
				TxDataBooleanStatus[0] |= (isUnderVolted & 0x01) << 7;         // Add bit 7

				// Pack combined velocity and acceleration as 8 bytes (4 bytes velocity + 4 bytes acceleration, both 32-bit signed)
				// Big-endian format
				TxDataVelocityAccel[0] = (uint8_t)(velocityCountsScaled >> 24);    // Use scaled values
				TxDataVelocityAccel[1] = (uint8_t)(velocityCountsScaled >> 16);
				TxDataVelocityAccel[2] = (uint8_t)(velocityCountsScaled >> 8);
				TxDataVelocityAccel[3] = (uint8_t)(velocityCountsScaled);          // Velocity LSB
				TxDataVelocityAccel[4] = (uint8_t)(accelCountsScaled >> 24);       // Use scaled values
				TxDataVelocityAccel[5] = (uint8_t)(accelCountsScaled >> 16);
				TxDataVelocityAccel[6] = (uint8_t)(accelCountsScaled >> 8);
				TxDataVelocityAccel[7] = (uint8_t)(accelCountsScaled);             // Acceleration LSB

				// Add message to CAN Tx FIFO queue
				// Calculate base CAN ID for this device
				uint32_t baseCanId = BASE_ID + device_id;

				// Position message: embed API ID 0 in the CAN arbitration ID (8 bytes - raw counts)
				TxHeaderPosition.Identifier = baseCanId | (POSITION_API_ID << 10);           // API 0: baseCanId | 0x000
				// Combined velocity/acceleration message: embed API ID 1 in the CAN arbitration ID (8 bytes)
				TxHeaderVelocityAccel.Identifier = baseCanId | (VELOCITY_ACCEL_API_ID << 10); // API 1: baseCanId | 0x400
				// Boolean status message: embed API ID 2 in the CAN arbitration ID (1 byte)
				TxHeaderBooleanStatus.Identifier = baseCanId | (BOOLEAN_STATUS_API_ID << 10); // API 2: baseCanId | 0x800

				// returns HAL_Error if fifo queue is full or CAN is not initialized correctly
				// HAL_OK otherwise
				positionCanStatus = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeaderPosition, TxDataPosition);
				velocityAccelCanStatus = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeaderVelocityAccel, TxDataVelocityAccel);
				booleanStatusCanStatus = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeaderBooleanStatus, TxDataBooleanStatus);

			} else {
	            // Encoder read failed while ready - handle gracefully
	            mt6835_error_count++;
	            if (mt6835_error_count > 10) { // Higher threshold
	                encoder_ready = 0; // Mark as not ready
	                isHardwareFault = 1;
	            }

	            // Set error status but don't flood CAN
	            positionCanStatus = HAL_ERROR;
	            velocityAccelCanStatus = HAL_ERROR;
	            booleanStatusCanStatus = HAL_ERROR;
			}
		} else {
	        static uint32_t lastInitAttempt = 0;

	        // Only try initialization every 100ms to avoid overwhelming the SPI bus
	        if ((currentTimeMillisec - lastInitAttempt) >= 100) {
	            if (mt6835_update_counts(&hspi1) == HAL_OK) {
	                // Need multiple consecutive successful reads
	                int success_count = 0;
	                for (int i = 0; i < 5; i++) {
	                    HAL_Delay(2); // Small delay between reads
	                    if (mt6835_update_counts(&hspi1) == HAL_OK) {
	                        success_count++;
	                    } else {
	                        break;
	                    }
	                }

	                if (success_count >= 5) {
	                    encoder_ready = 1;
	                    mt6835_error_count = 0;

	                    // Reset velocity calculation state
	                    prevTimeMillisec = 0;
	                    filteredVelocityCPS = 0.0f;
	                    filteredAccelCPS2 = 0.0f;
	                }
	            }
	            lastInitAttempt = currentTimeMillisec;
	        }

	        // Set error status for CAN
	        positionCanStatus = HAL_ERROR;
	        velocityAccelCanStatus = HAL_ERROR;
	        booleanStatusCanStatus = HAL_ERROR;
		}

		//------------------------------------------------------------------------------------------------------------------------
		//  Error handling and status updates
		//--------------------------------------------------------------------------------------------------------
	    // Only check for clogging if we actually tried to send messages
	    if (encoder_ready) {
	        if (positionCanStatus != HAL_OK || velocityAccelCanStatus != HAL_OK || booleanStatusCanStatus != HAL_OK) {
	            if ((currentTimeMillisec - lastHeartbeatTime) > HEARTBEAT_TIMEOUT) {
	                isCANInvalid = 1;
	            } else {
	                isCANClogged = 1;
	            }
	        } else {
	            isCANInvalid = 0;
	            isCANClogged = 0;
	        }
	    }

		if(get_weak_magnetic_field_warning() == 1) {
		  isMagnetWeakSignal = 1;  // Magnet is not in ideal range
		} else {
		  isMagnetWeakSignal = 0;  // Magnet is in ideal range
		}

		if(get_under_voltage_warning() == 1) {
		  isUnderVolted = 1;  // System is under-volted
		} else {
		  isUnderVolted = 0;  // System is not under-volted
		}

		if(get_rotation_overspeed_warning() == 1) {
		  isRotationOverspeed = 1;  // Rotation is overspeed
		} else {
		  isRotationOverspeed = 0;  // Rotation is not overspeed
		}

		if (mt6835_update_counts(&hspi1) != HAL_OK) {
			mt6835_error_count++;
			if (mt6835_error_count > 5) { // threshold for hardware fault
				isHardwareFault = 1;
			}
		} else {
			mt6835_error_count = 0;
			isHardwareFault = 0;
		}

		// LED status logic for all boolean statuses
		if (isHardwareFault == 1) {
			handle_error_blink("SOS", 0.0f); // Red - Hardware fault (emergency) - SOS
		} else if (isLoopOverrun == 1) {
			handle_error_blink("M", 0.16f); // Yellow - Overrun - M (dash-dash)
		} else if (isUnderVolted == 1) {
			handle_error_blink("I", 0.16f); // Yellow - Under voltage - I (dot-dot)
		} else if (isRotationOverspeed == 1) {
			handle_error_blink("S", 0.16f); // Yellow - Rotation Overspeed - S (dot-dot-dot)
		} else if (isCANInvalid == 1) {
			handle_error_blink("H", 0.83f); // Purple - CAN Invalid - H (dot-dot-dot-dot)
		} else if (isCANClogged == 1) {
			handle_error_blink("5", 0.83f); // Purple - CAN Clogged - 5 (dot-dot-dot-dot-dot)
		} else if (isResetDuringEnable == 1) {
			handle_error_blink("O", 0.01f); // Orange - Reset during enable - O (dash-dash-dash)
		} else if (isMagnetWeakSignal == 1) {
			handle_error_blink("0", 0.01f); // Orange - Magnet Weak Signal - 0 (dash-dash-dash-dash-dash)
		} else {
			// Normal operation: position-based hue
			set_led_hue((float)singleTurnCounts / CPR, 1.0);
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
			isLoopOverrun = 1;  // Set loop overrun status
		} else {
			isLoopOverrun = 0;  // Clear loop overrun status
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

  sFilterConfig.FilterID1 = 0xA110000;  // Base ID for your device
  sFilterConfig.FilterID2 = 0xA120000;  // Allow wider range for all API IDs

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

//-----------------------------------------------------------------------------------------------
// Helper Functions
//-----------------------------------------------------------------------------------------------

// Function to set the sticky flag
void set_reset_sticky_flag(void) {
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, RESET_STICKY_FLAG_ADDR, 0xDEADBEEF);
    HAL_FLASH_Lock();
}

// Function to clear the sticky flag
void clear_reset_sticky_flag(void) {
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, RESET_STICKY_FLAG_ADDR, 0xFFFFFFFF);
    HAL_FLASH_Lock();
}

// Function to check the sticky flag
int was_reset_during_enable(void) {
    uint64_t flag = *((uint64_t*)RESET_STICKY_FLAG_ADDR);
    return (flag == 0xDEADBEEF);
}


//-----------------------------------------------------------------------------------------------
// Public API Functions
//-----------------------------------------------------------------------------------------------
/**
 * @brief Set the velocity low-pass filter alpha value.
 * @param alpha New alpha value (0.0f - 1.0f)
 */
void set_velocity_alpha(float alpha) {
  if (alpha >= 0.0f && alpha <= 1.0f) {
    velocity_alpha = alpha;
  }
}


/**
 * @brief Set the acceleration low-pass filter alpha value.
 * @param alpha New alpha value (0.0f - 1.0f)
 */
void set_accel_alpha(float alpha) {
  if (alpha >= 0.0f && alpha <= 1.0f) {
    accel_alpha = alpha;
  }
}

//-----------------------------------------------------------------------------------------------
// Header Configuration Functions
//-----------------------------------------------------------------------------------------------
void txHeaderConfigure(FDCAN_TxHeaderTypeDef* header) {
    header->IdType = FDCAN_EXTENDED_ID;
    header->TxFrameType = FDCAN_DATA_FRAME;
    header->DataLength = FDCAN_DLC_BYTES_8;
    header->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header->BitRateSwitch = FDCAN_BRS_OFF;
    header->FDFormat = FDCAN_CLASSIC_CAN;
    header->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header->MessageMarker = 0;
}

void txHeaderConfigureBooleanStatus(FDCAN_TxHeaderTypeDef* header) {
    header->IdType = FDCAN_EXTENDED_ID;
    header->TxFrameType = FDCAN_DATA_FRAME;
    header->DataLength = FDCAN_DLC_BYTES_1;
    header->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header->BitRateSwitch = FDCAN_BRS_OFF;
    header->FDFormat = FDCAN_CLASSIC_CAN;
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
