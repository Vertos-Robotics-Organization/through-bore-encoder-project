#ifndef NON_BLOCKING_MORSE_H
#define NON_BLOCKING_MORSE_H

//#ifdef __cplusplus
//extern "C" {
//#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize (start) a Morse code blinking of the given message.
 *
 * Call this once with your desired message and hue. After that, call
 * blink_led_morse_process() frequently (e.g., in main loop) to advance
 * the blinking state machine.
 *
 * @param message The ASCII string to blink in Morse code.
 * @param hue     The desired hue for the LED while blinking (0.0 to 1.0).
 */
void blink_led_morse_init(const char *message, float hue);

/**
 * @brief Process the Morse code blinking state machine (non-blocking).
 *
 * Call this repeatedly (e.g., in the main loop) after calling blink_led_morse_init().
 * It will return quickly (does not block). Once the message is fully sent,
 * it will return true; otherwise false.
 *
 * @return bool  True if the message has finished blinking, false otherwise.
 */
bool blink_led_morse_process(void);

/**
 * @brief Get the Morse code representation of a character (A-Z, 0-9).
 *
 * @param c Character to convert to Morse.
 * @return const char* Pointer to a static Morse code string (".-", "-...", etc.)
 */
const char* get_morse_code(char c);

//#ifdef __cplusplus
//}
//#endif

#endif // NON_BLOCKING_MORSE_H
