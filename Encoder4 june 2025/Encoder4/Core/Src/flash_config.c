/**
 ******************************************************************************
 * @file           : flash_config.c
 * @brief          : Flash configuration management implementation
 ******************************************************************************
 * @attention
 *
 * Implementation of flash-based configuration storage for STM32G0B1CC encoder.
 * Provides structured, validated storage for user settings and critical data.
 *
 ******************************************************************************
 */

#include "flash_config.h"
#include <string.h>

//===================================================================================
// PRIVATE VARIABLES
//===================================================================================

// Cached configuration data (loaded once at startup for performance)
static flash_user_config_t g_user_config;
static flash_critical_data_t g_critical_data;

// State tracking
static uint8_t g_config_loaded = 0;
static uint8_t g_user_config_dirty = 0;  // Flag for pending writes

//===================================================================================
// PRIVATE FUNCTION PROTOTYPES
//===================================================================================

static HAL_StatusTypeDef load_config_from_flash(void);
static HAL_StatusTypeDef save_user_config_to_flash(void);
static HAL_StatusTypeDef save_critical_data_to_flash(void);
static HAL_StatusTypeDef erase_user_config_pages(void);
static HAL_StatusTypeDef erase_critical_data_pages(void);
static void init_user_config_defaults(void);
static void init_critical_data_defaults(void);

//===================================================================================
// CRC32 CALCULATION
//===================================================================================

/**
 * @brief Calculate CRC32 checksum using polynomial 0xEDB88320
 * @param data Pointer to data buffer
 * @param length Length of data in bytes
 * @return CRC32 checksum value
 */
uint32_t flash_crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }

    return ~crc;
}

//===================================================================================
// PRIVATE IMPLEMENTATION FUNCTIONS
//===================================================================================

/**
 * @brief Initialize user configuration with factory defaults
 */
static void init_user_config_defaults(void) {
    memset(&g_user_config, 0, sizeof(g_user_config));

    g_user_config.magic = 0xCAFEBABE;

    // Device Identity defaults
    g_user_config.device_can_id = 0;
    strcpy(g_user_config.device_name, "STM32_Encoder");
    g_user_config.device_group_id = 0;

    // Encoder Configuration defaults
    g_user_config.encoder_direction = 0;  // Clockwise
    g_user_config.position_offset = 0;
    g_user_config.gear_ratio = 1.0f;
    g_user_config.multi_turn_enabled = 1;

    // CAN Configuration defaults
    g_user_config.can_base_id = 0xA110000;
    g_user_config.can_heartbeat_enabled = 1;
    g_user_config.can_heartbeat_timeout = 1000;  // 1 second
    g_user_config.can_transmit_rate = 100;       // 100 Hz

    // Sensor Configuration defaults
    g_user_config.proximity_sensor_enabled = 0;
    g_user_config.proximity_threshold_low = 1500;
    g_user_config.proximity_threshold_high = 2500;
    g_user_config.temperature_monitoring = 1;

    // LED Configuration defaults
    g_user_config.led_brightness = 255;
    g_user_config.led_mode = 0;  // Position mode

    // Error colors (RGB format: 0xRRGGBB)
    g_user_config.led_error_colors[0] = 0xFF0000;  // Red - Hardware fault
    g_user_config.led_error_colors[1] = 0xFFFF00;  // Yellow - Loop overrun
    g_user_config.led_error_colors[2] = 0xFF8000;  // Orange - Under voltage
    g_user_config.led_error_colors[3] = 0xFFFF00;  // Yellow - Overspeed
    g_user_config.led_error_colors[4] = 0x8000FF;  // Purple - CAN invalid
    g_user_config.led_error_colors[5] = 0x8000FF;  // Purple - CAN clogged
    g_user_config.led_error_colors[6] = 0xFF8000;  // Orange - Reset during enable
    g_user_config.led_error_colors[7] = 0xFF8000;  // Orange - Weak magnet

    // Signal Processing defaults
    g_user_config.velocity_filter_alpha = 0.2f;
    g_user_config.accel_filter_alpha = 0.1f;
    g_user_config.noise_filtering_level = 1;  // Low

    // Runtime Statistics
    g_user_config.boot_count = 1;
    g_user_config.total_runtime_hours = 0;
    g_user_config.max_velocity_cps = 0;
    g_user_config.max_temperature_c = 0.0f;

    // User flags
    g_user_config.user_flags = 0;
}

/**
 * @brief Initialize critical data with factory defaults
 */
static void init_critical_data_defaults(void) {
    memset(&g_critical_data, 0, sizeof(g_critical_data));

    g_critical_data.magic = 0xDEADBEEF;

    // System Identity
    g_critical_data.firmware_version = 0x00010000;  // Version 1.0.0
    g_critical_data.hardware_revision = 1;
    g_critical_data.device_serial_number = HAL_GetUIDw0();  // Use STM32 UID
    g_critical_data.factory_calibration_date = 0;  // Set during manufacturing

    // Factory Calibration defaults
    g_critical_data.magnetic_offset_degrees = 0.0f;
    g_critical_data.temperature_coefficient = 1.0f;
    g_critical_data.encoder_cpr = 2097152;  // 2^21 for MT6835

    // Initialize linearity correction to unity (no correction)
    for (int i = 0; i < 16; i++) {
        g_critical_data.linearity_correction[i] = 1.0f;
    }

    // System fault tracking (start at zero)
    g_critical_data.reset_sticky_flag = 0;

    // Power management defaults
    g_critical_data.min_voltage_recorded = 3.3f;  // Assume nominal
    g_critical_data.max_voltage_recorded = 3.3f;
}

/**
 * @brief Load configuration data from flash memory
 * @return HAL_OK on success, HAL_ERROR on failure
 */
static HAL_StatusTypeDef load_config_from_flash(void) {
    // Load user configuration
    memcpy(&g_user_config, (void*)USER_CONFIG_START, sizeof(g_user_config));

    // Validate user configuration
    if (g_user_config.magic != 0xCAFEBABE) {
        // Invalid or uninitialized - use defaults
        init_user_config_defaults();
    } else {
        // Verify checksum
        uint32_t calc_crc = flash_crc32((uint8_t*)&g_user_config,
                                       sizeof(g_user_config) - sizeof(uint32_t));
        if (calc_crc != g_user_config.checksum) {
            // Corruption detected - reinitialize
            init_user_config_defaults();
        }
    }

    // Load critical data
    memcpy(&g_critical_data, (void*)CRITICAL_DATA_START, sizeof(g_critical_data));

    // Validate critical data
    if (g_critical_data.magic != 0xDEADBEEF) {
        // Invalid or uninitialized - use defaults
        init_critical_data_defaults();
    } else {
        // Verify checksum
        uint32_t calc_crc = flash_crc32((uint8_t*)&g_critical_data,
                                       sizeof(g_critical_data) - sizeof(uint32_t));
        if (calc_crc != g_critical_data.checksum) {
            // Corruption detected - reinitialize but preserve what we can
            uint32_t saved_serial = g_critical_data.device_serial_number;
            uint32_t saved_version = g_critical_data.firmware_version;
            init_critical_data_defaults();
            g_critical_data.device_serial_number = saved_serial;
            g_critical_data.firmware_version = saved_version;
        }
    }

    g_config_loaded = 1;
    return HAL_OK;
}

/**
 * @brief Erase user configuration flash pages
 * @return HAL_OK on success, HAL_ERROR on failure
 */
static HAL_StatusTypeDef erase_user_config_pages(void) {
    HAL_FLASH_Unlock();

    // Calculate starting page number for STM32G0B1CC
    uint32_t start_page = (USER_CONFIG_START - FLASH_BASE_ADDR) / FLASH_PAGE_SIZE;
    uint32_t num_pages = USER_CONFIG_SIZE / FLASH_PAGE_SIZE;

    // Set up erase structure
    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks = FLASH_BANK_1;
    erase_init.Page = start_page;
    erase_init.NbPages = num_pages;

    uint32_t page_error = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);

    HAL_FLASH_Lock();
    return status;
}

/**
 * @brief Erase critical data flash pages
 * @return HAL_OK on success, HAL_ERROR on failure
 */
static HAL_StatusTypeDef erase_critical_data_pages(void) {
    HAL_FLASH_Unlock();

    // Calculate starting page number for STM32G0B1CC
    uint32_t start_page = (CRITICAL_DATA_START - FLASH_BASE_ADDR) / FLASH_PAGE_SIZE;
    uint32_t num_pages = CRITICAL_DATA_SIZE / FLASH_PAGE_SIZE;

    // Set up erase structure
    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks = FLASH_BANK_1;
    erase_init.Page = start_page;
    erase_init.NbPages = num_pages;

    uint32_t page_error = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);

    HAL_FLASH_Lock();
    return status;
}

/**
 * @brief Save user configuration to flash
 * @return HAL_OK on success, HAL_ERROR on failure
 */
static HAL_StatusTypeDef save_user_config_to_flash(void) {
    // Calculate and store checksum
    g_user_config.checksum = flash_crc32((uint8_t*)&g_user_config,
                                        sizeof(g_user_config) - sizeof(uint32_t));

    // Erase pages first
    HAL_StatusTypeDef status = erase_user_config_pages();
    if (status != HAL_OK) {
        return status;
    }

    // Write data in 64-bit chunks
    HAL_FLASH_Unlock();

    uint64_t* src = (uint64_t*)&g_user_config;
    uint32_t addr = USER_CONFIG_START;
    size_t chunks = (sizeof(g_user_config) + 7) / 8;  // Round up to 8-byte boundary

    for (size_t i = 0; i < chunks; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
        addr += 8;
    }

    HAL_FLASH_Lock();
    g_user_config_dirty = 0;  // Clear dirty flag
    return HAL_OK;
}

/**
 * @brief Save critical data to flash
 * @return HAL_OK on success, HAL_ERROR on failure
 */
static HAL_StatusTypeDef save_critical_data_to_flash(void) {
    // Calculate and store checksum
    g_critical_data.checksum = flash_crc32((uint8_t*)&g_critical_data,
                                          sizeof(g_critical_data) - sizeof(uint32_t));

    // Erase pages first
    HAL_StatusTypeDef status = erase_critical_data_pages();
    if (status != HAL_OK) {
        return status;
    }

    // Write data in 64-bit chunks
    HAL_FLASH_Unlock();

    uint64_t* src = (uint64_t*)&g_critical_data;
    uint32_t addr = CRITICAL_DATA_START;
    size_t chunks = (sizeof(g_critical_data) + 7) / 8;  // Round up to 8-byte boundary

    for (size_t i = 0; i < chunks; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
        addr += 8;
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

//===================================================================================
// PUBLIC API IMPLEMENTATION
//===================================================================================

/**
 * @brief Initialize flash configuration system
 */
HAL_StatusTypeDef flash_config_init(void) {
    return load_config_from_flash();
}

/**
 * @brief Get device CAN ID
 */
uint32_t flash_get_device_id(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }
    return g_user_config.device_can_id;
}

/**
 * @brief Set device CAN ID
 */
HAL_StatusTypeDef flash_set_device_id(uint32_t id) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }

    if (g_user_config.device_can_id != id) {
        g_user_config.device_can_id = id;
        g_user_config_dirty = 1;
        return save_user_config_to_flash();
    }

    return HAL_OK;  // No change needed
}

/**
 * @brief Get device name
 */
HAL_StatusTypeDef flash_get_device_name(char* name) {
    if (!name) return HAL_ERROR;

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    strncpy(name, g_user_config.device_name, 31);
    name[31] = '\0';  // Ensure null termination
    return HAL_OK;
}

/**
 * @brief Set device name
 */
HAL_StatusTypeDef flash_set_device_name(const char* name) {
    if (!name) return HAL_ERROR;

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    if (strncmp(g_user_config.device_name, name, 31) != 0) {
        strncpy(g_user_config.device_name, name, 31);
        g_user_config.device_name[31] = '\0';  // Ensure null termination
        g_user_config_dirty = 1;
        return save_user_config_to_flash();
    }

    return HAL_OK;  // No change needed
}

/**
 * @brief Get encoder direction
 */
uint8_t flash_get_encoder_direction(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }
    return g_user_config.encoder_direction;
}

/**
 * @brief Set encoder direction
 */
HAL_StatusTypeDef flash_set_encoder_direction(uint8_t direction) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }

    if (g_user_config.encoder_direction != direction) {
        g_user_config.encoder_direction = direction;
        g_user_config_dirty = 1;
        return save_user_config_to_flash();
    }

    return HAL_OK;
}

/**
 * @brief Get position offset
 */
int64_t flash_get_position_offset(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }
    return g_user_config.position_offset;
}

/**
 * @brief Set position offset
 */
HAL_StatusTypeDef flash_set_position_offset(int64_t offset) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }

    if (g_user_config.position_offset != offset) {
        g_user_config.position_offset = offset;
        g_user_config_dirty = 1;
        return save_user_config_to_flash();
    }

    return HAL_OK;
}

/**
 * @brief Get velocity filter alpha
 */
float flash_get_velocity_alpha(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }
    return g_user_config.velocity_filter_alpha;
}

/**
 * @brief Set velocity filter alpha
 */
HAL_StatusTypeDef flash_set_velocity_alpha(float alpha) {
    if (alpha < 0.0f || alpha > 1.0f) {
        return HAL_ERROR;  // Invalid range
    }

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    if (g_user_config.velocity_filter_alpha != alpha) {
        g_user_config.velocity_filter_alpha = alpha;
        g_user_config_dirty = 1;
        return save_user_config_to_flash();
    }

    return HAL_OK;
}

/**
 * @brief Get acceleration filter alpha
 */
float flash_get_accel_alpha(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }
    return g_user_config.accel_filter_alpha;
}

/**
 * @brief Set acceleration filter alpha
 */
HAL_StatusTypeDef flash_set_accel_alpha(float alpha) {
    if (alpha < 0.0f || alpha > 1.0f) {
        return HAL_ERROR;  // Invalid range
    }

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    if (g_user_config.accel_filter_alpha != alpha) {
        g_user_config.accel_filter_alpha = alpha;
        g_user_config_dirty = 1;
        return save_user_config_to_flash();
    }

    return HAL_OK;
}

/**
 * @brief Get boot count
 */
uint32_t flash_get_boot_count(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }
    return g_user_config.boot_count;
}

/**
 * @brief Increment boot counter
 */
HAL_StatusTypeDef flash_increment_boot_count(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }

    g_user_config.boot_count++;
    g_user_config_dirty = 1;
    return save_user_config_to_flash();
}

/**
 * @brief Increment error counter
 */
HAL_StatusTypeDef flash_increment_error_count(uint8_t error_type) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }

    switch (error_type) {
        case 0:  // Hard fault
            g_critical_data.hard_fault_count++;
            break;
        case 1:  // Encoder error
            g_critical_data.encoder_error_count++;
            break;
        case 2:  // CAN error
            g_critical_data.can_error_count++;
            break;
        case 3:  // Flash error
            g_critical_data.flash_error_count++;
            break;
        default:
            return HAL_ERROR;  // Invalid error type
    }

    return save_critical_data_to_flash();
}

/**
 * @brief Get reset sticky flag
 */
uint32_t flash_get_reset_sticky_flag(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }
    return g_critical_data.reset_sticky_flag;
}

/**
 * @brief Set reset sticky flag
 */
HAL_StatusTypeDef flash_set_reset_sticky_flag(uint32_t flag) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }

    if (g_critical_data.reset_sticky_flag != flag) {
        g_critical_data.reset_sticky_flag = flag;
        return save_critical_data_to_flash();
    }

    return HAL_OK;
}

/**
 * @brief Clear reset sticky flag
 */
HAL_StatusTypeDef flash_clear_reset_sticky_flag(void) {
    return flash_set_reset_sticky_flag(0);
}

/**
 * @brief Factory reset user configuration
 */
HAL_StatusTypeDef flash_factory_reset(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }

    // Preserve boot count across factory reset
    uint32_t saved_boot_count = g_user_config.boot_count;

    // Reset to defaults
    init_user_config_defaults();

    // Restore boot count
    g_user_config.boot_count = saved_boot_count;

    g_user_config_dirty = 1;
    return save_user_config_to_flash();
}

/**
 * @brief Get firmware version
 */
uint32_t flash_get_firmware_version(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }
    return g_critical_data.firmware_version;
}

/**
 * @brief Get device serial number
 */
uint32_t flash_get_serial_number(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }
    return g_critical_data.device_serial_number;
}

/**
 * @brief Get user configuration pointer (read-only)
 */
const flash_user_config_t* flash_get_user_config_ptr(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }
    return &g_user_config;
}

/**
 * @brief Get critical data pointer (read-only)
 */
const flash_critical_data_t* flash_get_critical_data_ptr(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }
    return &g_critical_data;
}

/**
 * @brief Force save of user configuration
 */
HAL_StatusTypeDef flash_save_user_config(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }

    g_user_config_dirty = 1;
    return save_user_config_to_flash();
}

/**
 * @brief Validate flash data integrity
 */
HAL_StatusTypeDef flash_validate_integrity(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }

    // Validate user config magic and checksum
    if (g_user_config.magic != 0xCAFEBABE) {
        return HAL_ERROR;
    }

    uint32_t user_crc = flash_crc32((uint8_t*)&g_user_config,
                                   sizeof(g_user_config) - sizeof(uint32_t));
    if (user_crc != g_user_config.checksum) {
        return HAL_ERROR;
    }

    // Validate critical data magic and checksum
    if (g_critical_data.magic != 0xDEADBEEF) {
        return HAL_ERROR;
    }

    uint32_t critical_crc = flash_crc32((uint8_t*)&g_critical_data,
                                       sizeof(g_critical_data) - sizeof(uint32_t));
    if (critical_crc != g_critical_data.checksum) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Get flash usage statistics
 */
void flash_get_usage_stats(uint32_t* user_pages_used,
                          uint32_t* critical_pages_used,
                          uint32_t* total_pages_available) {
    if (user_pages_used) {
        *user_pages_used = USER_CONFIG_SIZE / FLASH_PAGE_SIZE;
    }

    if (critical_pages_used) {
        *critical_pages_used = CRITICAL_DATA_SIZE / FLASH_PAGE_SIZE;
    }

    if (total_pages_available) {
        *total_pages_available = (USER_CONFIG_SIZE + CRITICAL_DATA_SIZE) / FLASH_PAGE_SIZE;
    }
}
