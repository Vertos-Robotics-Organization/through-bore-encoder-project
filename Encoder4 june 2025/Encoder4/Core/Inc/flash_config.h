/**
 ******************************************************************************
 * @file           : flash_config.h
 * @brief          : Flash configuration management for STM32G0B1CC encoder (512KB)
 ******************************************************************************
 * @attention
 *
 * Flash Memory Organization for STM32G0B1CC (512KB total):
 * - Application Code: 0x08000000 - 0x0806FFFF (~448KB)
 * - User Config:      0x08070000 - 0x08077FFF (32KB, Pages 0xE0-0xFF)
 * - Critical Data:    0x08078000 - 0x0807BFFF (16KB, Pages 0x3C-0x3F)
 * - Array Storage:    0x0807C000 - 0x0807EFFF (12KB, Pages 0x3E-0x3F)
 * - Reserved:         0x0807F000 - 0x0807FFFF (4KB, Page 0x3F)
 *
 ******************************************************************************
 */

#ifndef FLASH_CONFIG_H
#define FLASH_CONFIG_H

#include "stm32g0xx_hal.h"
#include <stdint.h>

//===================================================================================
// FLASH MEMORY MAP - STM32G0B1CC 512KB SPECIFIC
//===================================================================================

// STM32G0B1CC Flash specifications (512KB variant)
#define FLASH_BASE_ADDR         0x08000000
#define FLASH_SIZE_BYTES        (512 * 1024)    // 512KB for upgraded variant
#define FLASH_PAGE_SIZE         2048             // 2KB per page
#define FLASH_TOTAL_PAGES       256              // 512KB / 2KB = 256 pages (0x00-0xFF)

// Memory allocation (using last 64KB for configuration data)
#define USER_CONFIG_START       0x08070000       // Page 0xE0 (32KB for user settings)
#define USER_CONFIG_SIZE        (16 * FLASH_PAGE_SIZE)  // 32KB
#define CRITICAL_DATA_START     0x08078000       // Page 0x3C (16KB for critical data)
#define CRITICAL_DATA_SIZE      (8 * FLASH_PAGE_SIZE)   // 16KB
#define ARRAY_STORAGE_START     0x0807C000       // Page 0x3E (12KB for array storage)
#define ARRAY_STORAGE_SIZE      (6 * FLASH_PAGE_SIZE)   // 12KB
#define RESERVED_START          0x0807F000       // Page 0x3F (4KB reserved)

//===================================================================================
// ARRAY STORAGE CONFIGURATION
//===================================================================================

// Maximum number of user-defined arrays
#define MAX_USER_ARRAYS         32
#define MAX_ARRAY_SIZE          512      // Maximum elements per array
#define ARRAY_NAME_LENGTH       16       // Maximum array name length

// Array data types
typedef enum {
    ARRAY_TYPE_UINT8 = 0,
    ARRAY_TYPE_INT8,
    ARRAY_TYPE_UINT16,
    ARRAY_TYPE_INT16,
    ARRAY_TYPE_UINT32,
    ARRAY_TYPE_INT32,
    ARRAY_TYPE_FLOAT,
    ARRAY_TYPE_COUNT
} array_data_type_t;

// Array storage header for each array
typedef struct {
    char name[ARRAY_NAME_LENGTH];     // Array name
    array_data_type_t data_type;      // Data type of elements
    uint16_t element_count;           // Number of elements stored
    uint16_t max_elements;            // Maximum elements allocated
    uint32_t data_offset;             // Offset to data in array storage area
    uint32_t checksum;                // CRC32 of array data
    uint8_t active;                   // 1 if array is active, 0 if deleted
    uint8_t reserved[7];              // Padding for alignment
} __attribute__((packed)) array_descriptor_t;

// Array storage management structure
typedef struct {
    uint32_t magic;                   // 0xARRAY01 - validation marker
    uint32_t version;                 // Storage format version
    uint32_t array_count;             // Number of active arrays
    uint32_t next_free_offset;        // Next free data offset
    array_descriptor_t arrays[MAX_USER_ARRAYS];  // Array descriptors
    uint8_t reserved[3968];           // Pad to fill page boundary
    uint32_t checksum;                // CRC32 of management structure
} __attribute__((packed)) array_storage_header_t;

//===================================================================================
// DATA STRUCTURES
//===================================================================================

/**
 * @brief User configuration structure (32KB allocated)
 * @note Expanded with more test arrays and configuration options
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

    // User-defined boolean flags (64 custom flags)
    uint64_t user_flags;              // Bitfield for application-specific flags

    // Expanded test arrays for diagnostics and development
    uint32_t test_array_u32[64];      // 64 x 32-bit test values
    uint16_t test_array_u16[128];     // 128 x 16-bit test values  
    uint8_t  test_array_u8[256];      // 256 x 8-bit test values
    float    test_array_float[64];    // 64 x float test values
    int32_t  test_array_i32[64];      // 64 x signed 32-bit test values

    // Advanced configuration
    uint32_t advanced_config[128];    // 128 advanced configuration parameters
    
    // Calibration lookup tables
    float    position_correction_table[360]; // Per-degree position correction
    float    temperature_compensation[64];    // Temperature compensation curve
    
    // User-defined configuration blocks
    uint32_t user_config_block1[256]; // General purpose config block 1
    uint32_t user_config_block2[256]; // General purpose config block 2
    uint32_t user_config_block3[256]; // General purpose config block 3
    uint32_t user_config_block4[256]; // General purpose config block 4

    // Reserved for future expansion
    uint8_t  reserved[24576];         // Pad to ~32KB
    uint32_t checksum;                // CRC32 of entire structure
} __attribute__((packed)) flash_user_config_t;

/**
 * @brief Critical system data structure (16KB allocated)
 * @note Expanded with more diagnostic and calibration data
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
    float    linearity_correction[64]; // Expanded linearity correction table

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

    // Extended diagnostics
    uint32_t can_tx_count;            // Total CAN messages transmitted
    uint32_t can_rx_count;            // Total CAN messages received
    uint32_t spi_error_count;         // SPI communication errors
    uint32_t i2c_error_count;         // I2C communication errors
    
    // Performance metrics
    uint32_t max_loop_time_us;        // Maximum main loop execution time
    uint32_t avg_loop_time_us;        // Average main loop execution time
    uint32_t cpu_usage_percent;       // CPU usage percentage
    
    // Factory test results
    float    factory_test_results[32]; // Factory test measurements
    uint32_t factory_test_status;      // Pass/fail status of factory tests
    
    // Manufacturing data
    char     manufacturing_date[16];   // Manufacturing date string
    char     test_station_id[16];      // Test station identifier
    uint32_t manufacturing_batch;      // Manufacturing batch number

    // Reserved for future critical data
    uint8_t  reserved[15872];         // Pad to ~16KB
    uint32_t checksum;                // CRC32 of entire structure
} __attribute__((packed)) flash_critical_data_t;

//===================================================================================
// PUBLIC API FUNCTIONS
//===================================================================================

/**
 * @brief Initialize flash configuration system
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef flash_config_init(void);

//--- Device Identity Functions ---
uint32_t flash_get_device_id(void);
HAL_StatusTypeDef flash_set_device_id(uint32_t id);
HAL_StatusTypeDef flash_get_device_name(char* name);
HAL_StatusTypeDef flash_set_device_name(const char* name);

//--- Encoder Configuration Functions ---
uint8_t flash_get_encoder_direction(void);
HAL_StatusTypeDef flash_set_encoder_direction(uint8_t direction);
int64_t flash_get_position_offset(void);
HAL_StatusTypeDef flash_set_position_offset(int64_t offset);

//--- Filter Configuration Functions ---
float flash_get_velocity_alpha(void);
HAL_StatusTypeDef flash_set_velocity_alpha(float alpha);
float flash_get_accel_alpha(void);
HAL_StatusTypeDef flash_set_accel_alpha(float alpha);

//--- Test Array Functions (Enhanced) ---
HAL_StatusTypeDef flash_set_test_array_u32_value(uint8_t index, uint32_t value);
uint32_t flash_get_test_array_u32_value(uint8_t index);
HAL_StatusTypeDef flash_set_test_array_u16_value(uint8_t index, uint16_t value);
uint16_t flash_get_test_array_u16_value(uint8_t index);
HAL_StatusTypeDef flash_set_test_array_u8_value(uint8_t index, uint8_t value);
uint8_t flash_get_test_array_u8_value(uint8_t index);
HAL_StatusTypeDef flash_set_test_array_float_value(uint8_t index, float value);
float flash_get_test_array_float_value(uint8_t index);
HAL_StatusTypeDef flash_set_test_array_i32_value(uint8_t index, int32_t value);
int32_t flash_get_test_array_i32_value(uint8_t index);

//--- Dynamic Array Functions (New) ---
/**
 * @brief Create a new user array
 * @param name Array name (max 15 characters)
 * @param data_type Type of data to store
 * @param max_elements Maximum number of elements
 * @return Array ID (0-31) on success, -1 on failure
 */
int8_t flash_create_user_array(const char* name, array_data_type_t data_type, uint16_t max_elements);

/**
 * @brief Delete a user array
 * @param array_id Array ID to delete
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef flash_delete_user_array(uint8_t array_id);

/**
 * @brief Write data to user array
 * @param array_id Array ID
 * @param index Element index
 * @param data Pointer to data
 * @param count Number of elements to write
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef flash_write_user_array(uint8_t array_id, uint16_t index, const void* data, uint16_t count);

/**
 * @brief Read data from user array
 * @param array_id Array ID
 * @param index Element index
 * @param data Pointer to output buffer
 * @param count Number of elements to read
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef flash_read_user_array(uint8_t array_id, uint16_t index, void* data, uint16_t count);

/**
 * @brief Get user array information
 * @param array_id Array ID
 * @param descriptor Pointer to output descriptor
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef flash_get_array_info(uint8_t array_id, array_descriptor_t* descriptor);

/**
 * @brief List all active user arrays
 * @param array_list Output buffer for array descriptors
 * @param max_arrays Maximum arrays to return
 * @return Number of active arrays
 */
uint8_t flash_list_user_arrays(array_descriptor_t* array_list, uint8_t max_arrays);

/**
 * @brief Find array by name
 * @param name Array name to search for
 * @return Array ID on success, -1 if not found
 */
int8_t flash_find_array_by_name(const char* name);

//--- System Statistics Functions ---
HAL_StatusTypeDef flash_increment_boot_count(void);
uint32_t flash_get_boot_count(void);
HAL_StatusTypeDef flash_increment_error_count(uint8_t error_type);

//--- Reset and Sticky Flag Functions ---
uint32_t flash_get_reset_sticky_flag(void);
HAL_StatusTypeDef flash_set_reset_sticky_flag(uint32_t flag);
HAL_StatusTypeDef flash_clear_reset_sticky_flag(void);

//--- Factory and Maintenance Functions ---
HAL_StatusTypeDef flash_factory_reset(void);
uint32_t flash_get_firmware_version(void);
uint32_t flash_get_serial_number(void);

//--- Direct Access Functions ---
const flash_user_config_t* flash_get_user_config_ptr(void);
const flash_critical_data_t* flash_get_critical_data_ptr(void);
HAL_StatusTypeDef flash_save_user_config(void);

//--- Utility Functions ---
uint32_t flash_crc32(const uint8_t* data, size_t length);
HAL_StatusTypeDef flash_validate_integrity(void);
void flash_get_usage_stats(uint32_t* user_pages_used,
                          uint32_t* critical_pages_used,
                          uint32_t* array_pages_used,
                          uint32_t* total_pages_available);

//--- Array Storage Utilities ---
HAL_StatusTypeDef flash_defragment_arrays(void);
uint32_t flash_get_array_storage_free_bytes(void);
HAL_StatusTypeDef flash_backup_arrays(void);
HAL_StatusTypeDef flash_restore_arrays(void);

#endif // FLASH_CONFIG_H