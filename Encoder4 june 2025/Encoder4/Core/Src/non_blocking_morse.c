#include "stm32g0xx_hal.h"
#include <ctype.h>    // For toupper
#include <non_blocking_morse.h>

// You may need to include or declare the function that sets the LED hue/brightness
// For example:
extern void set_led_hue(float hue, float brightness);

// Define timing for Morse code (all in milliseconds)
#define DOT_DURATION    200
#define DASH_DURATION   600
#define SYMBOL_PAUSE    200
#define LETTER_PAUSE    600

// Define states for our state machine
typedef enum {
    MORSE_STATE_IDLE,             // Not started or just finished one character
    MORSE_STATE_NEXT_SYMBOL,      // Moving to next dot/dash
    MORSE_STATE_SYMBOL_ON,        // LED is on for dot/dash
    MORSE_STATE_SYMBOL_OFF,       // LED is off for inter-symbol pause
    MORSE_STATE_LETTER_PAUSE,     // Pause between letters
    MORSE_STATE_DONE              // Completed the entire message
} MorseBlinkState;

/**
 * @brief Holds the current Morse-blink context:
 */
typedef struct {
    const char      *message;   // Pointer to the original string we're blinking
    const char      *morse_ptr; // Current position in the Morse code for a letter
    uint32_t        timestamp;  // For non-blocking timing
    MorseBlinkState state;      // Current state in the state machine
    float           hue;        // The hue for the LED
} MorseBlinkContext;

// We keep a static context so that we can handle only one blinking instance.
// You can adapt this approach if you want multiple parallel messages.
static MorseBlinkContext ctx = {0};

/**
 * @brief Checks if the specified duration has passed since start_time.
 *
 * @param start_time  The tick value when the event started.
 * @param duration    Desired delay in milliseconds.
 * @return true       If the desired delay has elapsed.
 * @return false      Otherwise.
 */
static bool is_delay_elapsed(uint32_t start_time, uint32_t duration)
{
    uint32_t now = HAL_GetTick();
    return ( (now - start_time) >= duration );
}

void blink_led_morse_init(const char *message, float hue2)
{
    // Initialize the context with the provided message and hue
    ctx.message   = message;
    ctx.morse_ptr = NULL;
    ctx.timestamp = 0;
    ctx.state     = MORSE_STATE_IDLE;

    ctx.hue       = hue2;
    //ctx.hue       = 0.33;
}

bool blink_led_morse_process(void)
{
    // If we are already done, just return true immediately
    if (ctx.state == MORSE_STATE_DONE) {
        return true;
    }

    switch (ctx.state)
    {
        //----------------------------------------------------------------------
        // MORSE_STATE_IDLE:
        //   - Grab the next character in the message
        //   - If it's a space, do a letter pause
        //   - If end of string, go to DONE
        //   - Otherwise, get the Morse code for that character
        //----------------------------------------------------------------------
        case MORSE_STATE_IDLE:
        {
            char c = *ctx.message++;
            if (c == '\0') {
                // Reached the end of the message
                ctx.state = MORSE_STATE_DONE;
                break;
            }

            if (c == ' ') {
                // Space indicates new word (or separation)
                ctx.timestamp = HAL_GetTick();
                ctx.state     = MORSE_STATE_LETTER_PAUSE;
            }
            else {
                // Convert to uppercase (for A-Z range)
                c = (char)toupper((int)c);

                // Get the Morse code string for this character
                ctx.morse_ptr = get_morse_code(c);

                // Move to handle the first Morse symbol (dot or dash)
                ctx.state = MORSE_STATE_NEXT_SYMBOL;
            }
            break;
        }

        //----------------------------------------------------------------------
        // MORSE_STATE_NEXT_SYMBOL:
        //   - If we reached end of Morse string, letter pause
        //   - Otherwise, turn LED on for dot/dash
        //----------------------------------------------------------------------
        case MORSE_STATE_NEXT_SYMBOL:
            if (ctx.morse_ptr == NULL || *ctx.morse_ptr == '\0') {
                // No more symbols in this letter
                ctx.timestamp = HAL_GetTick();
                ctx.state     = MORSE_STATE_LETTER_PAUSE;
            } else {
                // Turn LED on (hue, full brightness)
                set_led_hue(ctx.hue, 1.0f);

                // Figure out how long to keep the LED on
                //uint32_t on_time = (*ctx.morse_ptr == '.') ? DOT_DURATION : DASH_DURATION;

                // Record time, move to SYMBOL_ON
                ctx.timestamp = HAL_GetTick();
                ctx.state     = MORSE_STATE_SYMBOL_ON;
            }
            break;

        //----------------------------------------------------------------------
        // MORSE_STATE_SYMBOL_ON:
        //   - Wait until dot/dash ON time is finished
        //   - Then turn LED off and do inter-symbol pause
        //----------------------------------------------------------------------
        case MORSE_STATE_SYMBOL_ON:
        {
            bool is_dot  = (*ctx.morse_ptr == '.');
            uint32_t needed = is_dot ? DOT_DURATION : DASH_DURATION;

            if (is_delay_elapsed(ctx.timestamp, needed)) {
                // Time's up for the current symbol. Turn off LED
                set_led_hue(0.0f, 0.0f);

                // Move to SYMBOL_OFF
                ctx.timestamp = HAL_GetTick();
                ctx.state     = MORSE_STATE_SYMBOL_OFF;
            }
            break;
        }

        //----------------------------------------------------------------------
        // MORSE_STATE_SYMBOL_OFF:
        //   - Inter-symbol pause
        //   - When done, move to next symbol (ctx.morse_ptr++)
        //----------------------------------------------------------------------
        case MORSE_STATE_SYMBOL_OFF:
            if (is_delay_elapsed(ctx.timestamp, SYMBOL_PAUSE)) {
                // Advance to next dot/dash in the same letter
                ctx.morse_ptr++;
                ctx.state = MORSE_STATE_NEXT_SYMBOL;
            }
            break;

        //----------------------------------------------------------------------
        // MORSE_STATE_LETTER_PAUSE:
        //   - Wait the letter pause
        //   - Then go back to IDLE for next character
        //----------------------------------------------------------------------
        case MORSE_STATE_LETTER_PAUSE:
            if (is_delay_elapsed(ctx.timestamp, LETTER_PAUSE)) {
                ctx.state = MORSE_STATE_IDLE;
            }
            break;

        //----------------------------------------------------------------------
        // MORSE_STATE_DONE:
        //   - We’re finished. We'll never leave this state unless re-initialized
        //----------------------------------------------------------------------
        case MORSE_STATE_DONE:
        default:
            // Do nothing
            break;
    }

    // Return true if we've finished blinking the entire message
    return (ctx.state == MORSE_STATE_DONE);
}

const char* get_morse_code(char c)
{
    // Morse code table for A-Z, 0-9
    static const char *morse_table[] = {
        ".-",    // A
        "-...",  // B
        "-.-.",  // C
        "-..",   // D
        ".",     // E
        "..-.",  // F
        "--.",   // G
        "....",  // H
        "..",    // I
        ".---",  // J
        "-.-",   // K
        ".-..",  // L
        "--",    // M
        "-.",    // N
        "---",   // O
        ".--.",  // P
        "--.-",  // Q
        ".-.",   // R
        "...",   // S
        "-",     // T
        "..-",   // U
        "...-",  // V
        ".--",   // W
        "-..-",  // X
        "-.--",  // Y
        "--..",  // Z

        "-----", // 0
        ".----", // 1
        "..---", // 2
        "...--", // 3
        "....-", // 4
        ".....", // 5
        "-....", // 6
        "--...", // 7
        "---..", // 8
        "----."  // 9
    };

    // If it's A-Z
    if (c >= 'A' && c <= 'Z') {
        return morse_table[c - 'A'];
    }
    // If it's 0-9
    if (c >= '0' && c <= '9') {
        return morse_table[(c - '0') + 26];
    }

    // Unknown character or punctuation -> no Morse output
    return "";
}
