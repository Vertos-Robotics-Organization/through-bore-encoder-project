/*
 * error_blink.c
 *
 * Created on: Mar 1, 2025
 * Author: kyleh
 */

#include "error_blink.h"
#include <stdbool.h>

/*
 * These two functions are assumed to be defined elsewhere in your codebase:
 * blink_led_morse_init(const char *code, float color)
 * blink_led_morse_process(void) -> int (returns nonzero if the sequence has just finished)
 */
extern void blink_led_morse_init(const char *code, float color);
extern int blink_led_morse_process(void);

/**
 * We will maintain two static variables to track our blinking state:
 * 1) s_currentError: The error code whose Morse pattern we are blinking.
 * 2) s_isBlinking: Whether or not we've started blinking an error.
 */
// static ErrorCode s_currentError = ENCODER_STATUS_OK;
static bool s_isBlinking = false;
static bool error_occoured = false;

void handle_error_blink(const char *message, float hue2) {
    error_occoured = true;

    if (!s_isBlinking) {
        blink_led_morse_init(message, hue2);
        s_isBlinking = true;
    }

    if (blink_led_morse_process()) {
        s_isBlinking = false;
    }
}
