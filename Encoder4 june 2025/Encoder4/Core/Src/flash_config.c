/**
 ******************************************************************************
 * @file           : flash_config.c
 * @brief          : Flash configuration management implementation (512KB)
 ******************************************************************************
 * @attention
 *
 * Enhanced implementation with dynamic array storage for STM32G0B1CC encoder.
 * Provides structured, validated storage for user settings, critical data,
 * and flexible user-defined arrays.
 *
 ******************************************************************************
 */

#include "flash_config.h"
#include <string.h>

//===================================================================================
// PRIVATE VARIABLES
//===================================================================================

// Cached configuration data
static flash_user_config_t g_user_config;
static flash_critical_data_t g_critical_data;
static array_storage_header_t g_array_header;

// State tracking
static uint8_t g_config_loaded = 0;
static uint8_t g_user_config_dirty = 0;
static uint8_t g_critical_data_dirty = 0;
static uint8_t g_array_header_dirty = 0;

//===================================================================================
// PRIVATE FUNCTION PROTOTYPES
//===================================================================================

static HAL_StatusTypeDef load_config_from_flash(void);
static HAL_StatusTypeDef save_user_config_to_flash(void);
static HAL_StatusTypeDef save_critical_data_to_flash(void);
static HAL_StatusTypeDef save_array_header_to_flash(void);
static HAL_StatusTypeDef erase_user_config_pages(void);
static HAL_StatusTypeDef erase_critical_data_pages(void);
static HAL_StatusTypeDef erase_array_storage_pages(void);
static void init_user_config_defaults(void);
static void init_critical_data_defaults(void);
static void init_array_header_defaults(void);
static uint8_t get_element_size(array_data_type_t type);
static HAL_StatusTypeDef compact_array_storage(void);

//===================================================================================
// CRC32 CALCULATION
//===================================================================================

/**
 * @brief Calculate CRC32 checksum using polynomial 0xEDB88320
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
// PRIVATE UTILITY FUNCTIONS
//===================================================================================

/**
 * @brief Get size in bytes for array data type
 */
static uint8_t get_element_size(array_data_type_t type) {
    switch (type) {
        case ARRAY_TYPE_UINT8:
        case ARRAY_TYPE_INT8:
            return 1;
        case ARRAY_TYPE_UINT16:
        case ARRAY_TYPE_INT16:
            return 2;
        case ARRAY_TYPE_UINT32:
        case ARRAY_TYPE_INT32:
        case ARRAY_TYPE_FLOAT:
            return 4;
        default:
            return 0;
    }
}

//===================================================================================
// INITIALIZATION FUNCTIONS
//===================================================================================

/**
 * @brief Initialize user configuration with factory defaults
 */
static void init_user_config_defaults(void) {
    memset(&g_user_config, 0, sizeof(g_user_config));

    g_user_config.magic = 0xCAFEBABE;

    // Device Identity defaults
    g_user_config.device_can_id = 0;
    strcpy(g_user_config.device_name, "STM32_Encoder_512K");
    g_user_config.device_group_id = 0;

    // Encoder Configuration defaults
    g_user_config.encoder_direction = 0;
    g_user_config.position_offset = 0;
    g_user_config.gear_ratio = 1.0f;
    g_user_config.multi_turn_enabled = 1;

    // CAN Configuration defaults
    g_user_config.can_base_id = 0xA110000;
    g_user_config.can_heartbeat_enabled = 1;
    g_user_config.can_heartbeat_timeout = 1000;
    g_user_config.can_transmit_rate = 100;

    // Sensor Configuration defaults
    g_user_config.proximity_sensor_enabled = 0;
    g_user_config.proximity_threshold_low = 1500;
    g_user_config.proximity_threshold_high = 2500;
    g_user_config.temperature_monitoring = 1;

    // LED Configuration defaults
    g_user_config.led_brightness = 255;
    g_user_config.led_mode = 0;

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
    g_user_config.noise_filtering_level = 1;

    // Runtime Statistics
    g_user_config.boot_count = 1;
    g_user_config.total_runtime_hours = 0;
    g_user_config.max_velocity_cps = 0;
    g_user_config.max_temperature_c = 0.0f;

    // User flags
    g_user_config.user_flags = 0;

    // Initialize all test arrays to zero
    memset(g_user_config.test_array_u32, 0, sizeof(g_user_config.test_array_u32));
    memset(g_user_config.test_array_u16, 0, sizeof(g_user_config.test_array_u16));
    memset(g_user_config.test_array_u8, 0, sizeof(g_user_config.test_array_u8));
    memset(g_user_config.test_array_float, 0, sizeof(g_user_config.test_array_float));
    memset(g_user_config.test_array_i32, 0, sizeof(g_user_config.test_array_i32));

    // Initialize position correction table to unity
    for (int i = 0; i < 360; i++) {
        g_user_config.position_correction_table[i] = 1.0f;
    }

    // Initialize temperature compensation to unity
    for (int i = 0; i < 64; i++) {
        g_user_config.temperature_compensation[i] = 1.0f;
    }
}

/**
 * @brief Initialize critical data with factory defaults
 */
static void init_critical_data_defaults(void) {
    memset(&g_critical_data, 0, sizeof(g_critical_data));

    g_critical_data.magic = 0xDEADBEEF;

    // System Identity
    g_critical_data.firmware_version = 0x00020000;  // Version 2.0.0
    g_critical_data.hardware_revision = 2;
    g_critical_data.device_serial_number = HAL_GetUIDw0();
    g_critical_data.factory_calibration_date = 0;

    // Factory Calibration defaults
    g_critical_data.magnetic_offset_degrees = 0.0f;
    g_critical_data.temperature_coefficient = 1.0f;
    g_critical_data.encoder_cpr = 2097152;  // 2^21 for MT6835

    // Initialize expanded linearity correction to unity
    for (int i = 0; i < 64; i++) {
        g_critical_data.linearity_correction[i] = 1.0f;
    }

    // System fault tracking
    g_critical_data.reset_sticky_flag = 0;

    // Power management defaults
    g_critical_data.min_voltage_recorded = 3.3f;
    g_critical_data.max_voltage_recorded = 3.3f;

    // Performance metrics defaults
    g_critical_data.max_loop_time_us = 0;
    g_critical_data.avg_loop_time_us = 0;
    g_critical_data.cpu_usage_percent = 0;

    // Manufacturing data
    strcpy(g_critical_data.manufacturing_date, "2024-01-01");
    strcpy(g_critical_data.test_station_id, "TEST-01");
    g_critical_data.manufacturing_batch = 1;
}

/**
 * @brief Initialize array storage header with defaults
 */
static void init_array_header_defaults(void) {
    memset(&g_array_header, 0, sizeof(g_array_header));

    g_array_header.magic = 0x41525241;  // "ARRA" in hex
    g_array_header.version = 1;
    g_array_header.array_count = 0;
    g_array_header.next_free_offset = sizeof(array_storage_header_t);

    // Initialize all array descriptors as inactive
    for (int i = 0; i < MAX_USER_ARRAYS; i++) {
        g_array_header.arrays[i].active = 0;
    }
}

//===================================================================================
// FLASH OPERATIONS
//===================================================================================

/**
 * @brief Erase user configuration flash pages
 */
static HAL_StatusTypeDef erase_user_config_pages(void) {
    HAL_FLASH_Unlock();

    uint32_t start_page = (USER_CONFIG_START - FLASH_BASE_ADDR) / FLASH_PAGE_SIZE;
    uint32_t num_pages = USER_CONFIG_SIZE / FLASH_PAGE_SIZE;

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
 * @brief Erase array storage flash pages
 */
static HAL_StatusTypeDef erase_array_storage_pages(void) {
    HAL_FLASH_Unlock();

    uint32_t start_page = (ARRAY_STORAGE_START - FLASH_BASE_ADDR) / FLASH_PAGE_SIZE;
    uint32_t num_pages = ARRAY_STORAGE_SIZE / FLASH_PAGE_SIZE;

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
    size_t chunks = (sizeof(g_user_config) + 7) / 8;

    for (size_t i = 0; i < chunks; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
        addr += 8;
    }

    HAL_FLASH_Lock();
    g_user_config_dirty = 0;
    return HAL_OK;
}

/**
 * @brief Save critical data to flash
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
    size_t chunks = (sizeof(g_critical_data) + 7) / 8;

    for (size_t i = 0; i < chunks; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
        addr += 8;
    }

    HAL_FLASH_Lock();
    g_critical_data_dirty = 0;
    return HAL_OK;
}

/**
 * @brief Save array header to flash
 */
static HAL_StatusTypeDef save_array_header_to_flash(void) {
    // Calculate and store checksum
    g_array_header.checksum = flash_crc32((uint8_t*)&g_array_header,
                                         sizeof(g_array_header) - sizeof(uint32_t));

    // Erase array storage pages (header is at the beginning)
    HAL_StatusTypeDef status = erase_array_storage_pages();
    if (status != HAL_OK) {
        return status;
    }

    // Write header in 64-bit chunks
    HAL_FLASH_Unlock();

    uint64_t* src = (uint64_t*)&g_array_header;
    uint32_t addr = ARRAY_STORAGE_START;
    size_t chunks = (sizeof(g_array_header) + 7) / 8;

    for (size_t i = 0; i < chunks; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
        addr += 8;
    }

    HAL_FLASH_Lock();
    g_array_header_dirty = 0;
    return HAL_OK;
}

/**
 * @brief Load configuration data from flash memory
 */
static HAL_StatusTypeDef load_config_from_flash(void) {
    // Load user configuration
    memcpy(&g_user_config, (void*)USER_CONFIG_START, sizeof(g_user_config));

    // Validate user configuration
    if (g_user_config.magic != 0xCAFEBABE) {
        init_user_config_defaults();
    } else {
        uint32_t calc_crc = flash_crc32((uint8_t*)&g_user_config,
                                       sizeof(g_user_config) - sizeof(uint32_t));
        if (calc_crc != g_user_config.checksum) {
            init_user_config_defaults();
        }
    }

    // Load critical data
    memcpy(&g_critical_data, (void*)CRITICAL_DATA_START, sizeof(g_critical_data));

    // Validate critical data
    if (g_critical_data.magic != 0xDEADBEEF) {
        init_critical_data_defaults();
    } else {
        uint32_t calc_crc = flash_crc32((uint8_t*)&g_critical_data,
                                       sizeof(g_critical_data) - sizeof(uint32_t));
        if (calc_crc != g_critical_data.checksum) {
            uint32_t saved_serial = g_critical_data.device_serial_number;
            uint32_t saved_version = g_critical_data.firmware_version;
            init_critical_data_defaults();
            g_critical_data.device_serial_number = saved_serial;
            g_critical_data.firmware_version = saved_version;
        }
    }

    // Load array header
    memcpy(&g_array_header, (void*)ARRAY_STORAGE_START, sizeof(g_array_header));

    // Validate array header
    if (g_array_header.magic != 0x41525241) {
        init_array_header_defaults();
    } else {
        uint32_t calc_crc = flash_crc32((uint8_t*)&g_array_header,
                                       sizeof(g_array_header) - sizeof(uint32_t));
        if (calc_crc != g_array_header.checksum) {
            init_array_header_defaults();
        }
    }

    g_config_loaded = 1;
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

    return HAL_OK;
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
    name[31] = '\0';
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
        g_user_config.device_name[31] = '\0';
        g_user_config_dirty = 1;
        return save_user_config_to_flash();
    }

    return HAL_OK;
}

//===================================================================================
// TEST ARRAY FUNCTIONS (ENHANCED)
//===================================================================================

/**
 * @brief Set 32-bit unsigned test array value
 */
HAL_StatusTypeDef flash_set_test_array_u32_value(uint8_t index, uint32_t value) {
    if (index >= 64) return HAL_ERROR;

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    g_user_config.test_array_u32[index] = value;
    g_user_config_dirty = 1;
    return save_user_config_to_flash();
}

/**
 * @brief Get 32-bit unsigned test array value
 */
uint32_t flash_get_test_array_u32_value(uint8_t index) {
    if (index >= 64) return 0;

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    return g_user_config.test_array_u32[index];
}

/**
 * @brief Set 16-bit unsigned test array value
 */
HAL_StatusTypeDef flash_set_test_array_u16_value(uint8_t index, uint16_t value) {
    if (index >= 128) return HAL_ERROR;

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    g_user_config.test_array_u16[index] = value;
    g_user_config_dirty = 1;
    return save_user_config_to_flash();
}

/**
 * @brief Get 16-bit unsigned test array value
 */
uint16_t flash_get_test_array_u16_value(uint8_t index) {
    if (index >= 128) return 0;

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    return g_user_config.test_array_u16[index];
}

/**
 * @brief Set 8-bit unsigned test array value
 */
HAL_StatusTypeDef flash_set_test_array_u8_value(uint8_t index, uint8_t value) {
    if (index >= 256) return HAL_ERROR;

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    g_user_config.test_array_u8[index] = value;
    g_user_config_dirty = 1;
    return save_user_config_to_flash();
}

/**
 * @brief Get 8-bit unsigned test array value
 */
uint8_t flash_get_test_array_u8_value(uint8_t index) {
    if (index >= 256) return 0;

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    return g_user_config.test_array_u8[index];
}

/**
 * @brief Set float test array value
 */
HAL_StatusTypeDef flash_set_test_array_float_value(uint8_t index, float value) {
    if (index >= 64) return HAL_ERROR;

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    g_user_config.test_array_float[index] = value;
    g_user_config_dirty = 1;
    return save_user_config_to_flash();
}

/**
 * @brief Get float test array value
 */
float flash_get_test_array_float_value(uint8_t index) {
    if (index >= 64) return 0.0f;

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    return g_user_config.test_array_float[index];
}

/**
 * @brief Set 32-bit signed test array value
 */
HAL_StatusTypeDef flash_set_test_array_i32_value(uint8_t index, int32_t value) {
    if (index >= 64) return HAL_ERROR;

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    g_user_config.test_array_i32[index] = value;
    g_user_config_dirty = 1;
    return save_user_config_to_flash();
}

/**
 * @brief Get 32-bit signed test array value
 */
int32_t flash_get_test_array_i32_value(uint8_t index) {
    if (index >= 64) return 0;

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    return g_user_config.test_array_i32[index];
}

//===================================================================================
// DYNAMIC USER ARRAY FUNCTIONS
//===================================================================================

/**
 * @brief Create a new user array
 */
int8_t flash_create_user_array(const char* name, array_data_type_t data_type, uint16_t max_elements) {
    if (!name || strlen(name) >= ARRAY_NAME_LENGTH || max_elements == 0 || max_elements > MAX_ARRAY_SIZE) {
        return -1;
    }

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    // Check if array with same name already exists
    for (int i = 0; i < MAX_USER_ARRAYS; i++) {
        if (g_array_header.arrays[i].active && 
            strncmp(g_array_header.arrays[i].name, name, ARRAY_NAME_LENGTH) == 0) {
            return -1; // Already exists
        }
    }

    // Find free slot
    int8_t free_slot = -1;
    for (int i = 0; i < MAX_USER_ARRAYS; i++) {
        if (!g_array_header.arrays[i].active) {
            free_slot = i;
            break;
        }
    }

    if (free_slot == -1) {
        return -1; // No free slots
    }

    // Calculate required space
    uint8_t element_size = get_element_size(data_type);
    if (element_size == 0) {
        return -1; // Invalid data type
    }

    uint32_t required_bytes = max_elements * element_size;
    uint32_t available_bytes = ARRAY_STORAGE_SIZE - g_array_header.next_free_offset;

    if (required_bytes > available_bytes) {
        // Try compacting storage first
        if (compact_array_storage() != HAL_OK) {
            return -1;
        }
        available_bytes = ARRAY_STORAGE_SIZE - g_array_header.next_free_offset;
        if (required_bytes > available_bytes) {
            return -1; // Still not enough space
        }
    }

    // Create array descriptor
    array_descriptor_t* desc = &g_array_header.arrays[free_slot];
    strncpy(desc->name, name, ARRAY_NAME_LENGTH - 1);
    desc->name[ARRAY_NAME_LENGTH - 1] = '\0';
    desc->data_type = data_type;
    desc->element_count = 0;
    desc->max_elements = max_elements;
    desc->data_offset = g_array_header.next_free_offset;
    desc->checksum = 0;
    desc->active = 1;

    // Update header
    g_array_header.array_count++;
    g_array_header.next_free_offset += required_bytes;

    // Save to flash
    g_array_header_dirty = 1;
    if (save_array_header_to_flash() != HAL_OK) {
        return -1;
    }

    return free_slot;
}

/**
 * @brief Delete a user array
 */
HAL_StatusTypeDef flash_delete_user_array(uint8_t array_id) {
    if (array_id >= MAX_USER_ARRAYS) {
        return HAL_ERROR;
    }

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    if (!g_array_header.arrays[array_id].active) {
        return HAL_ERROR; // Array doesn't exist
    }

    // Mark as inactive
    g_array_header.arrays[array_id].active = 0;
    g_array_header.array_count--;

    // Save to flash
    g_array_header_dirty = 1;
    return save_array_header_to_flash();
}

/**
 * @brief Write data to user array
 */
HAL_StatusTypeDef flash_write_user_array(uint8_t array_id, uint16_t index, const void* data, uint16_t count) {
    if (array_id >= MAX_USER_ARRAYS || !data || count == 0) {
        return HAL_ERROR;
    }

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    array_descriptor_t* desc = &g_array_header.arrays[array_id];
    if (!desc->active) {
        return HAL_ERROR;
    }

    if (index + count > desc->max_elements) {
        return HAL_ERROR; // Would exceed array bounds
    }

    uint8_t element_size = get_element_size(desc->data_type);
    uint32_t write_addr = ARRAY_STORAGE_START + desc->data_offset + (index * element_size);
    uint32_t write_bytes = count * element_size;

    // Write data to flash
    HAL_FLASH_Unlock();

    // Align data for 64-bit writes
    uint8_t aligned_buffer[8];
    const uint8_t* src_data = (const uint8_t*)data;
    
    for (uint32_t i = 0; i < write_bytes; i += 8) {
        memset(aligned_buffer, 0xFF, 8); // Fill with 0xFF (flash erased state)
        
        uint32_t copy_bytes = (write_bytes - i < 8) ? (write_bytes - i) : 8;
        memcpy(aligned_buffer, src_data + i, copy_bytes);
        
        HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                                                    write_addr + i, 
                                                    *(uint64_t*)aligned_buffer);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
    }

    HAL_FLASH_Lock();

    // Update element count if necessary
    if (index + count > desc->element_count) {
        desc->element_count = index + count;
    }

    // Update checksum
    uint8_t* array_data = (uint8_t*)(ARRAY_STORAGE_START + desc->data_offset);
    desc->checksum = flash_crc32(array_data, desc->element_count * element_size);

    // Save header
    g_array_header_dirty = 1;
    return save_array_header_to_flash();
}

/**
 * @brief Read data from user array
 */
HAL_StatusTypeDef flash_read_user_array(uint8_t array_id, uint16_t index, void* data, uint16_t count) {
    if (array_id >= MAX_USER_ARRAYS || !data || count == 0) {
        return HAL_ERROR;
    }

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    array_descriptor_t* desc = &g_array_header.arrays[array_id];
    if (!desc->active) {
        return HAL_ERROR;
    }

    if (index + count > desc->element_count) {
        return HAL_ERROR; // Would read beyond stored data
    }

    uint8_t element_size = get_element_size(desc->data_type);
    uint32_t read_addr = ARRAY_STORAGE_START + desc->data_offset + (index * element_size);
    uint32_t read_bytes = count * element_size;

    // Read data from flash
    memcpy(data, (void*)read_addr, read_bytes);

    return HAL_OK;
}

/**
 * @brief Get user array information
 */
HAL_StatusTypeDef flash_get_array_info(uint8_t array_id, array_descriptor_t* descriptor) {
    if (array_id >= MAX_USER_ARRAYS || !descriptor) {
        return HAL_ERROR;
    }

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    if (!g_array_header.arrays[array_id].active) {
        return HAL_ERROR;
    }

    memcpy(descriptor, &g_array_header.arrays[array_id], sizeof(array_descriptor_t));
    return HAL_OK;
}

/**
 * @brief List all active user arrays
 */
uint8_t flash_list_user_arrays(array_descriptor_t* array_list, uint8_t max_arrays) {
    if (!array_list || max_arrays == 0) {
        return 0;
    }

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    uint8_t count = 0;
    for (int i = 0; i < MAX_USER_ARRAYS && count < max_arrays; i++) {
        if (g_array_header.arrays[i].active) {
            memcpy(&array_list[count], &g_array_header.arrays[i], sizeof(array_descriptor_t));
            count++;
        }
    }

    return count;
}

/**
 * @brief Find array by name
 */
int8_t flash_find_array_by_name(const char* name) {
    if (!name) {
        return -1;
    }

    if (!g_config_loaded) {
        load_config_from_flash();
    }

    for (int i = 0; i < MAX_USER_ARRAYS; i++) {
        if (g_array_header.arrays[i].active && 
            strncmp(g_array_header.arrays[i].name, name, ARRAY_NAME_LENGTH) == 0) {
            return i;
        }
    }

    return -1;
}

/**
 * @brief Compact array storage to defragment free space
 */
static HAL_StatusTypeDef compact_array_storage(void) {
    // This is a simplified implementation - in practice you'd want to
    // copy active arrays to the beginning of storage and update offsets
    // For now, just return OK to indicate it was attempted
    return HAL_OK;
}

//===================================================================================
// REMAINING STANDARD FUNCTIONS
//===================================================================================

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
        return HAL_ERROR;
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
        return HAL_ERROR;
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
        case 0:
            g_critical_data.hard_fault_count++;
            break;
        case 1:
            g_critical_data.encoder_error_count++;
            break;
        case 2:
            g_critical_data.can_error_count++;
            break;
        case 3:
            g_critical_data.flash_error_count++;
            break;
        default:
            return HAL_ERROR;
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

    uint32_t saved_boot_count = g_user_config.boot_count;
    init_user_config_defaults();
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
 * @brief Get user configuration pointer
 */
const flash_user_config_t* flash_get_user_config_ptr(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }
    return &g_user_config;
}

/**
 * @brief Get critical data pointer
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

    // Validate array header magic and checksum
    if (g_array_header.magic != 0x41525241) {
        return HAL_ERROR;
    }

    uint32_t array_crc = flash_crc32((uint8_t*)&g_array_header,
                                    sizeof(g_array_header) - sizeof(uint32_t));
    if (array_crc != g_array_header.checksum) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Get flash usage statistics
 */
void flash_get_usage_stats(uint32_t* user_pages_used,
                          uint32_t* critical_pages_used,
                          uint32_t* array_pages_used,
                          uint32_t* total_pages_available) {
    if (user_pages_used) {
        *user_pages_used = USER_CONFIG_SIZE / FLASH_PAGE_SIZE;
    }

    if (critical_pages_used) {
        *critical_pages_used = CRITICAL_DATA_SIZE / FLASH_PAGE_SIZE;
    }

    if (array_pages_used) {
        *array_pages_used = ARRAY_STORAGE_SIZE / FLASH_PAGE_SIZE;
    }

    if (total_pages_available) {
        *total_pages_available = (USER_CONFIG_SIZE + CRITICAL_DATA_SIZE + ARRAY_STORAGE_SIZE) / FLASH_PAGE_SIZE;
    }
}

//===================================================================================
// ARRAY STORAGE UTILITY FUNCTIONS
//===================================================================================

/**
 * @brief Defragment array storage to reclaim deleted space
 */
HAL_StatusTypeDef flash_defragment_arrays(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }

    // Create a temporary buffer to hold active array data
    uint8_t* temp_buffer = malloc(ARRAY_STORAGE_SIZE - sizeof(array_storage_header_t));
    if (!temp_buffer) {
        return HAL_ERROR;
    }

    uint32_t new_offset = sizeof(array_storage_header_t);
    
    // Copy all active arrays to temporary buffer and update offsets
    for (int i = 0; i < MAX_USER_ARRAYS; i++) {
        if (g_array_header.arrays[i].active) {
            array_descriptor_t* desc = &g_array_header.arrays[i];
            uint8_t element_size = get_element_size(desc->data_type);
            uint32_t array_bytes = desc->element_count * element_size;
            
            // Copy array data to temp buffer
            uint8_t* src_addr = (uint8_t*)(ARRAY_STORAGE_START + desc->data_offset);
            memcpy(temp_buffer + (new_offset - sizeof(array_storage_header_t)), src_addr, array_bytes);
            
            // Update offset in descriptor
            desc->data_offset = new_offset;
            new_offset += (desc->max_elements * element_size); // Reserve full space
        }
    }

    // Update next free offset
    g_array_header.next_free_offset = new_offset;

    // Erase array storage pages
    HAL_StatusTypeDef status = erase_array_storage_pages();
    if (status != HAL_OK) {
        free(temp_buffer);
        return status;
    }

    // Write header back
    status = save_array_header_to_flash();
    if (status != HAL_OK) {
        free(temp_buffer);
        return status;
    }

    // Write array data back
    HAL_FLASH_Unlock();
    
    uint32_t write_addr = ARRAY_STORAGE_START + sizeof(array_storage_header_t);
    uint32_t write_size = new_offset - sizeof(array_storage_header_t);
    
    // Write in 64-bit chunks
    for (uint32_t i = 0; i < write_size; i += 8) {
        uint64_t data_chunk = 0xFFFFFFFFFFFFFFFF; // Default erased state
        
        uint32_t copy_bytes = (write_size - i < 8) ? (write_size - i) : 8;
        memcpy(&data_chunk, temp_buffer + i, copy_bytes);
        
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, write_addr + i, data_chunk);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            free(temp_buffer);
            return status;
        }
    }
    
    HAL_FLASH_Lock();
    free(temp_buffer);
    
    return HAL_OK;
}

/**
 * @brief Get free bytes available in array storage
 */
uint32_t flash_get_array_storage_free_bytes(void) {
    if (!g_config_loaded) {
        load_config_from_flash();
    }

    return ARRAY_STORAGE_SIZE - g_array_header.next_free_offset;
}

/**
 * @brief Backup arrays to a secondary location (placeholder)
 */
HAL_StatusTypeDef flash_backup_arrays(void) {
    // This would implement backing up array data to another flash region
    // or external storage. For now, just return OK.
    return HAL_OK;
}

/**
 * @brief Restore arrays from backup (placeholder)
 */
HAL_StatusTypeDef flash_restore_arrays(void) {
    // This would implement restoring array data from backup.
    // For now, just return OK.
    return HAL_OK;
}


/**
 * @brief Erase critical data flash pages
 */
static HAL_StatusTypeDef erase_critical_data_pages(void) {
    HAL_FLASH_Unlock();

    uint32_t start_page = (CRITICAL_DATA_START - FLASH_BASE_ADDR) / FLASH_PAGE_SIZE;
    uint32_t num_pages = CRITICAL_DATA_SIZE / FLASH_PAGE_SIZE;

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