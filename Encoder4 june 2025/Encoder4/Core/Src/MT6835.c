#include "mt6835.h"
// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------
uint32_t singleTurnSensorCounts   = 0; // Holds the latest 21-bit absolute position
int64_t  multiTurnSensorCounts    = 0; // Tracks how many full rotations have occurred
uint8_t  rotationOverspeedWarning = 0;
uint8_t  weakMagneticFieldWarning = 0;
uint8_t  underVoltageWarning      = 0;

// Add at the top (or in a header if shared)
static uint8_t encoderDirection = 1; // 1 = normal, -1 = inverted

//extern int numTimeout; // Reference to the variable defined in main.c
//extern int numHAL_Error = 0;


void mt6835_init(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    // Set CS pin as high initially (idle state)
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

/**
 * Calculate CRC-8 SMBus using polynomial X^8 + X^2 + X + 1 (0x07)
 */

uint8_t calculate_crc_smbus(uint32_t raw_angle, uint8_t status) {
    uint32_t data = ((raw_angle & 0x1FFFFF) << 3) | (status & 0x07); // Combine angle and status
    uint8_t data_bytes[3] = {
        (data >> 16) & 0xFF,
        (data >> 8) & 0xFF,
        data & 0xFF
    };

    uint8_t crc = CRC8_INITIAL_VALUE;

    for (size_t i = 0; i < 3; i++) {
        crc ^= data_bytes[i]; // XOR the current data byte into the CRC
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

HAL_StatusTypeDef handle_spi_status(HAL_StatusTypeDef status) {
    if (status == HAL_TIMEOUT) {
        // Handle timeout (e.g., return or log an error code)
    	//numTimeout++;
        return HAL_TIMEOUT;
    } else if (status == HAL_ERROR) {
        // Handle error (e.g., return or log an error code)
    	//numHAL_Error++;

        return HAL_ERROR;
    }
    return HAL_OK;
}
//
///**
// * Read the raw counts and update warnings
// */
//uint32_t mt6835_read_counts(SPI_HandleTypeDef *hspi) {
//    uint8_t data[6] = {0}; // 48-bit transaction array
//    uint8_t command[2] = {MT6835_OP_ANGLE << 4, MT6835_REG_ANGLE1};
//
//    HAL_StatusTypeDef SPI_1_Status;
//
//    SPI_1_Status = HAL_SPI_Transmit(hspi, command, 2, SPI_1_TIMEOUT);
//    if (handle_spi_status(SPI_1_Status) != HAL_OK) {
//        // Optionally add specific actions for the status
//        return SPI_1_Status;
//    }
//
//    SPI_1_Status = HAL_SPI_Receive(hspi, data, 6, SPI_1_TIMEOUT);
//    if (handle_spi_status(SPI_1_Status) != HAL_OK) {
//        // Optionally add specific actions for the status
//        return SPI_1_Status;
//    }
//
//    // if timeout, return something,
//    // if error, do something else
//
//
//    // Combine the data into a 21-bit raw angle value
//    uint32_t raw_counts = ((data[0] << 13) & 0x1FFFFF) |
//                          ((data[1] << 5) & 0x1FFFFF) |
//                          ((data[2] >> 3) & 0x1FFFFF);
//
//    uint8_t status = (data[2] & 0x07); // Extract STATUS[2:0]
//    uint8_t crc_received = data[3];   // Extract CRC[7:0]
//
//    // Calculate CRC
//    uint8_t crc_calculated = calculate_crc_smbus(raw_counts, status);
//
//
//
//    // Check CRC
//    if (crc_received == crc_calculated) {
//        // If CRC is correct, update sensor counts
//        sensorCounts = raw_counts;
//
//        // Update warnings based on STATUS[2:0]
//        rotationOverspeedWarning = (status & 0x01) ? 1 : 0;
//        weakMagneticFieldWarning = (status & 0x02) ? 1 : 0;
//        underVoltageWarning = (status & 0x04) ? 1 : 0;
//
//        return raw_counts; // Return the raw count value
//    }
//
//    // If CRC fails, return 0 (indicating an error)
//    return 0;
//}

int mt6835_update_counts(SPI_HandleTypeDef *hspi)
{
    uint8_t data[4] = {0};
    // Example command: high nibble might be operation, low nibble might be register
    uint8_t command[2] = {
        (uint8_t)(MT6835_OP_ANGLE << 4),
        MT6835_REG_ANGLE1
    };

    // 1) Transmit the command
    HAL_StatusTypeDef spiStatus = HAL_SPI_Transmit(hspi, command, 2, SPI_1_TIMEOUT);
    if (spiStatus != HAL_OK) {
        return spiStatus; // HAL_TIMEOUT or HAL_ERROR
    }

    // 2) Receive the data (4 bytes)
    spiStatus = HAL_SPI_Receive(hspi, data, 6, SPI_1_TIMEOUT);
    if (spiStatus != HAL_OK) {
        return spiStatus;
    }

    // data layout (for 21-bit angle + 3-bit status + 8-bit CRC):
    //  data[0] => high bits of angle
    //  data[1] => middle bits of angle
    //  data[2] => low bits of angle (upper 5 bits) + status in lower 3 bits
    //  data[3] => 8-bit CRC

    // Extract raw 21-bit angle
    //   (Note: mask with 0x1FFFFF as a safeguard)
    uint32_t raw_counts = ((uint32_t)data[0] << 13) & 0x1FFFFF;
    raw_counts         |= ((uint32_t)data[1] << 5)  & 0x1FFFFF;
    raw_counts         |= (((uint32_t)data[2] >> 3) & 0x1F);

    // Extract status
    uint8_t statusBits = data[2] & 0x07; // lower 3 bits
    // Extract CRC
    uint8_t crc_received   = data[3];
    uint8_t crc_calculated = calculate_crc_smbus(raw_counts, statusBits);

    // Verify CRC
    if (crc_received != crc_calculated) {
        return HAL_ERROR;  // CRC mismatch
    }

    // Update warnings
    rotationOverspeedWarning = (statusBits & 0x01) ? 1 : 0;
    weakMagneticFieldWarning = (statusBits & 0x02) ? 1 : 0;
    underVoltageWarning      = (statusBits & 0x04) ? 1 : 0;

    // 1) Adjust 'delta' for rollovers
    static uint32_t previousSingleTurn = 0;  // track last 21-bit reading
    int32_t delta = (int32_t)raw_counts - (int32_t)previousSingleTurn;

    // If delta is too large => we rolled under
    if (delta > HALF_RANGE) {
        // e.g., new reading is near 0, old reading near max => negative rollover
        delta -= FULL_RANGE;
    }
    // If delta is too negative => we rolled over
    else if (delta < -HALF_RANGE) {
        // e.g., new reading near max, old reading near 0 => positive rollover
        delta += FULL_RANGE;
    }

    // 2) Accumulate the delta into 'multiTurnSensorCounts'
    //    so that multiTurnSensorCounts now holds the absolute total.
    multiTurnSensorCounts += delta * encoderDirection;

    previousSingleTurn = raw_counts;
    singleTurnSensorCounts = raw_counts;

    // If we got here => success
    return HAL_OK;
}

uint32_t get_counts_single_turn(void)
{
    return singleTurnSensorCounts;
}

// Getter methods for warning variables
uint8_t get_rotation_overspeed_warning(void) {
    return rotationOverspeedWarning;
}

uint8_t get_weak_magnetic_field_warning(void) {
    return weakMagneticFieldWarning;
}

uint8_t get_under_voltage_warning(void) {
    return underVoltageWarning;
}

int64_t get_counts_multi_turn(void)
{
    return multiTurnSensorCounts;
}

int64_t get_counts_full(void)
{
    // Combine multi-turn << 21 with single-turn
    return ((int64_t)multiTurnSensorCounts << 21) + (int64_t)singleTurnSensorCounts;
}

void reset_counts(void)
{
    // Reset both single-turn and multi-turn counts
    singleTurnSensorCounts = 0;
    multiTurnSensorCounts = 0;
}

void set_counts(int64_t counts)
{
    // Set the multi-turn counts and reset single-turn counts
    multiTurnSensorCounts = counts; // Extract multi-turn part
    singleTurnSensorCounts = counts; // Extract single-turn part
}

void set_encoder_direction(int dir) {
    encoderDirection = (dir >= 0) ? 1 : -1;
}

void invert_encoder_direction(int isInverted) {
    if (isInverted) {
        encoderDirection = -1;
    } else {
        encoderDirection = 1; // Reset to normal direction
    }
}



//uint32_t mt6835_read_counts(SPI_HandleTypeDef *hspi) {
//    uint8_t data[6] = {0}; // 48-bit transaction array
//    uint8_t command[2] = {MT6835_OP_ANGLE << 4, MT6835_REG_ANGLE1};
//
//    // Send the read command
//    HAL_SPI_Transmit(hspi, command, 2, HAL_MAX_DELAY);
//
//    // Receive the angle data (48 bits)
//    HAL_SPI_Receive(hspi, data, 6, HAL_MAX_DELAY);
//
//    // Combine the data into a 21-bit raw angle value
//    uint32_t raw_counts = ((data[0] << 13) & 0x1FFFFF) |
//                          ((data[1] << 5) & 0x1FFFFF) |
//                          ((data[2] >> 3) & 0x1FFFFF);
//
//    return raw_counts; // Return the raw count value
//}
