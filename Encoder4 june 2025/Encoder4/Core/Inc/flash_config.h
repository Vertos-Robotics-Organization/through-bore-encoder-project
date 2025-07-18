/**
 ******************************************************************************
 * @file           : flash_config.h
 * @brief          : Flash configuration management for STM32G0B1CC encoder
 ******************************************************************************
 * @attention
 *
 * Flash Memory Organization for STM32G0B1CC (256KB total):
 * - Application Code: 0x08000000 - 0x0803AFFF (~236KB)
 * - User Config:      0x0803B000 - 0x0803CFFF (8KB, Pages 0x76-0x79)
 * - Critical Data:    0x0803D000 - 0x0803EFFF (8KB, Pages 0x7A-0x7D)
 * - Reserved:         0x0803F000 - 0x0803FFFF (4KB, Pages 0x7E-0x7F)
 *
 ******************************************************************************
 */

#ifndef FLASH_CONFIG_H
#define FLASH_CONFIG_H

#include "stm32g0xx_hal.h"
#include <stdint.h>

//===================================================================================
// FLASH MEMORY MAP - STM32G0B1CC SPECIFIC
//===================================================================================

// STM32G0B1CC Flash specifications (from datasheet)
#define FLASH_BASE_ADDR         0x08000000
#define FLASH_SIZE_BYTES        (256 * 1024)    // 256KB for "CC" variant
#define FLASH_PAGE_SIZE         2048             // 2KB per page (datasheet confirmed)
#define FLASH_TOTAL_PAGES       128              // 256KB / 2KB = 128 pages (0x00-0x7F)

// Memory allocation (using last 20KB for configuration data)
#define USER_CONFIG_START       0x0803B000       // Page 0x76 (8KB for user settings)
#define USER_CONFIG_SIZE        (4 * FLASH_PAGE_SIZE)  // 8KB
#define CRITICAL_DATA_START     0x0803D000       // Page 0x7A (8KB for critical data)
#define CRITICAL_DATA_SIZE      (4 * FLASH_PAGE_SIZE)  // 8KB
#define RESERVED_START          0x0803F000       // Page 0x7E (4KB reserved)

//===================================================================================
// DATA STRUCTURES
//===================================================================================

/**
 * @brief User configuration structure (8KB allocated)
 * @note This structure contains user-modifiable settings that can be
 *       changed during operation via CAN commands or configuration tools
 */
typedef struct {
    uint32_t magic;                    // 0xCAFEBABE - validation marker

    // Device Identity
    uint32_t device_can_id;           // CAN bus device ID (0-255)
    char     device_name[32];         // Human readable device name
    uint32_t device_group_id;         // Group/team identifier

    // Encoder Configuration
    uint8_t  encoder_direction;       // 0=clockwise, 1=counter-clockwise
    int64_t  position_offset;         // Position zero offset (counts)
    float    gear_ratio;              // Mechanical gear ratio multiplier
    uint8_t  multi_turn_enabled;      // Enable multi-turn counting

    // CAN Bus Configuration
    uint32_t can_base_id;             // Base CAN ID (default: 0xA110000)
    uint8_t  can_heartbeat_enabled;   // Enable heartbeat monitoring
    uint16_t can_heartbeat_timeout;   // Heartbeat timeout (milliseconds)
    uint8_t  can_transmit_rate;       // CAN messages per second (1-100 Hz)

    // Sensor Configuration
    uint8_t  proximity_sensor_enabled; // Enable proximity sensor ADC
    uint16_t proximity_threshold_low;   // ADC threshold low value
    uint16_t proximity_threshold_high;  // ADC threshold high value
    uint8_t  temperature_monitoring;    // Enable temperature monitoring

    // LED Configuration
    uint8_t  led_brightness;          // LED brightness (0-255)
    uint8_t  led_mode;               // 0=position, 1=status, 2=off, 3=custom
    uint32_t led_error_colors[8];    // RGB colors for different error states

    // Signal Processing
    float    velocity_filter_alpha;   // Velocity low-pass filter coefficient (0.0-1.0)
    float    accel_filter_alpha;      // Acceleration low-pass filter coefficient (0.0-1.0)
    uint8_t  noise_filtering_level;   // 0=off, 1=low, 2=medium, 3=high

    // Runtime Statistics
    uint32_t boot_count;              // Number of successful boots
    uint32_t total_runtime_hours;     // Total runtime in hours
    uint32_t max_velocity_cps;        // Maximum velocity seen (counts/second)
    float    max_temperature_c;       // Maximum temperature recorded (Celsius)

    // User-defined boolean flags (32 custom flags)
    uint32_t user_flags;              // Bitfield for application-specific flags

    // Reserved for future expansion
    uint8_t  reserved[7900];          // Pad to ~8KB
    uint32_t checksum;                // CRC32 of entire structure
} __attribute__((packed)) flash_user_config_t;

/**
 * @brief Critical system data structure (8KB allocated)
 * @note This structure contains factory calibration data and system-critical
 *       information that should rarely change after manufacturing
 */
typedef struct {
    uint32_t magic;                   // 0xDEADBEEF - validation marker

    // System Identity
    uint32_t firmware_version;        // Firmware version (major.minor.patch)
    uint32_t hardware_revision;       // PCB/hardware revision number
    uint32_t device_serial_number;    // Unique device serial number
    uint32_t factory_calibration_date; // Unix timestamp of factory calibration

    // Factory Calibration Data
    float    magnetic_offset_degrees; // Factory magnetic offset calibration
    float    temperature_coefficient; // Temperature compensation coefficient
    int32_t  encoder_cpr;             // Counts per revolution (factory setting)
    float    linearity_correction[16]; // Linearity correction table

    // System Fault Tracking
    uint32_t reset_sticky_flag;       // Reset-during-enable detection flag
    uint32_t total_resets;            // Total system reset count
    uint32_t hard_fault_count;        // Hard fault occurrence count
    uint32_t encoder_error_count;     // MT6835 communication error count
    uint32_t can_error_count;         // CAN bus error count
    uint32_t flash_error_count;       // Flash operation error count

    // Power Management History
    uint32_t undervoltage_events;     // Count of undervoltage events
    uint32_t overvoltage_events;      // Count of overvoltage events
    float    min_voltage_recorded;    // Minimum voltage seen
    float    max_voltage_recorded;    // Maximum voltage seen

    // Reserved for future critical data
    uint8_t  reserved[7944];          // Pad to ~8KB
    uint32_t checksum;                // CRC32 of entire structure
} __attribute__((packed)) flash_critical_data_t;

//===================================================================================
// PUBLIC API FUNCTIONS
//===================================================================================

/**
 * @brief Initialize flash configuration system
 * @return HAL_OK on success, HAL_ERROR on failure
 * @note Call this once during system initialization
 */
HAL_StatusTypeDef flash_config_init(void);

//--- Device Identity Functions ---
/**
 * @brief Get device CAN ID from flash
 * @return Current device CAN ID (0-255)
 */
uint32_t flash_get_device_id(void);

/**
 * @brief Set device CAN ID and save to flash
 * @param id New device CAN ID (0-255)
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef flash_set_device_id(uint32_t id);

/**
 * @brief Get device name from flash
 * @param name Buffer to store device name (must be at least 32 bytes)
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef flash_get_device_name(char* name);

/**
 * @brief Set device name and save to flash
 * @param name New device name (max 31 characters + null terminator)
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef flash_set_device_name(const char* name);

//--- Encoder Configuration Functions ---
/**
 * @brief Get encoder direction setting
 * @return 0 for clockwise, 1 for counter-clockwise
 */
uint8_t flash_get_encoder_direction(void);

/**
 * @brief Set encoder direction and save to flash
 * @param direction 0 for clockwise, 1 for counter-clockwise
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef flash_set_encoder_direction(uint8_t direction);

/**
 * @brief Get position offset from flash
 * @return Current position offset in counts
 */
int64_t flash_get_position_offset(void);

/**
 * @brief Set position offset and save to flash
 * @param offset New position offset in counts
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef flash_set_position_offset(int64_t offset);

//--- Filter Configuration Functions ---
/**
 * @brief Get velocity filter alpha coefficient
 * @return Current velocity filter alpha (0.0-1.0)
 */
float flash_get_velocity_alpha(void);

/**
 * @brief Set velocity filter alpha and save to flash
 * @param alpha New alpha coefficient (0.0-1.0)
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef flash_set_velocity_alpha(float alpha);

/**
 * @brief Get acceleration filter alpha coefficient
 * @return Current acceleration filter alpha (0.0-1.0)
 */
float flash_get_accel_alpha(void);

/**
 * @brief Set acceleration filter alpha and save to flash
 * @param alpha New alpha coefficient (0.0-1.0)
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef flash_set_accel_alpha(float alpha);

//--- System Statistics Functions ---
/**
 * @brief Increment boot counter
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef flash_increment_boot_count(void);

/**
 * @brief Get current boot count
 * @return Number of successful boots
 */
uint32_t flash_get_boot_count(void);

/**
 * @brief Increment error counter for specific error type
 * @param error_type 0=hard_fault, 1=encoder_error, 2=can_error, 3=flash_error
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef flash_increment_error_count(uint8_t error_type);

//--- Reset and Sticky Flag Functions ---
/**
 * @brief Get reset sticky flag value
 * @return Current sticky flag value (0xDEADBEEF if reset during enable)
 */
uint32_t flash_get_reset_sticky_flag(void);

/**
 * @brief Set reset sticky flag and save to flash
 * @param flag New flag value (typically 0xDEADBEEF)
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef flash_set_reset_sticky_flag(uint32_t flag);

/**
 * @brief Clear reset sticky flag
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef flash_clear_reset_sticky_flag(void);

//--- Factory and Maintenance Functions ---
/**
 * @brief Reset user configuration to factory defaults
 * @return HAL_OK on success, HAL_ERROR on failure
 * @note This preserves critical/factory calibration data
 */
HAL_StatusTypeDef flash_factory_reset(void);

/**
 * @brief Get firmware version from critical data
 * @return Firmware version as 32-bit value (major.minor.patch)
 */
uint32_t flash_get_firmware_version(void);

/**
 * @brief Get device serial number from critical data
 * @return Device serial number
 */
uint32_t flash_get_serial_number(void);

//--- Direct Access Functions (Advanced Use) ---
/**
 * @brief Get pointer to cached user configuration
 * @return Pointer to user config structure (read-only)
 * @warning Use with caution - direct modification bypasses validation
 */
const flash_user_config_t* flash_get_user_config_ptr(void);

/**
 * @brief Get pointer to cached critical data
 * @return Pointer to critical data structure (read-only)
 * @warning Use with caution - direct modification bypasses validation
 */
const flash_critical_data_t* flash_get_critical_data_ptr(void);

/**
 * @brief Force write of cached user config to flash
 * @return HAL_OK on success, HAL_ERROR on failure
 * @note Use after direct modifications to cached data
 */
HAL_StatusTypeDef flash_save_user_config(void);

//--- Utility Functions ---
/**
 * @brief Calculate CRC32 checksum for data validation
 * @param data Pointer to data buffer
 * @param length Length of data in bytes
 * @return CRC32 checksum value
 */
uint32_t flash_crc32(const uint8_t* data, size_t length);

/**
 * @brief Validate flash data integrity
 * @return HAL_OK if both user and critical data are valid, HAL_ERROR otherwise
 */
HAL_StatusTypeDef flash_validate_integrity(void);

/**
 * @brief Get flash usage statistics
 * @param user_pages_used Output: number of user config pages used
 * @param critical_pages_used Output: number of critical data pages used
 * @param total_pages_available Output: total pages available for configuration
 */
void flash_get_usage_stats(uint32_t* user_pages_used,
                          uint32_t* critical_pages_used,
                          uint32_t* total_pages_available);

#endif // FLASH_CONFIG_H
