/*
 * error_blink.h
 *
 *  Created on: Mar 1, 2025
 *      Author: kyleh
 */

#ifndef ERROR_BLINK_H
#define ERROR_BLINK_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Possible encoder error statuses
 */
//typedef enum {
//    ENCODER_STATUS_OK             = 0,
//    ENCODER_STATUS_CAN_TX_FIFO_FULL = 1,
//    ENCODER_STATUS_NO_CANBUS      = 2,
//    ENCODER_STATUS_LOOP_OVERRUN   = 3
//} ErrorCode;

/**
 * @brief Handles error logic and triggers Morse-code LED blinking if needed.
 *
 * This function internally calls:
 *  - blink_led_morse_init() when a new error arises
 *  - blink_led_morse_process() in subsequent calls to show the Morse code
 *
 * @param errorStatus The current error status to handle
 */
void handle_error_blink(const char *message, float hue2);

#ifdef __cplusplus
}
#endif

#endif // ERROR_BLINK_H
