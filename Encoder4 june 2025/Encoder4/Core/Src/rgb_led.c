/*
 * hue_to_rbg.c
 *
 * Created on: Nov 7, 2024
 * Author: kyleh
 */

#include "rgb_led.h"
// #include "main.h"

// Function to convert hue (0 to 1) to RGB values
void hue_to_rgb(float hue, int *r, int *g, int *b)
{
    if (hue < 0.0 || hue > 1.0) {
        // Clamp hue between 0 and 1
        hue = (hue < 0.0) ? 0.0 : 1.0;
    }

    float h = hue * 360.0; // Convert to 0-360 degrees
    float s = 1.0; // Full saturation
    float v = 1.0; // Full value

    int i = (int)(h / 60.0) % 6;
    float f = (h / 60.0) - i;
    float p = v * (1.0 - s);
    float q = v * (1.0 - f * s);
    float t = v * (1.0 - (1.0 - f) * s);

    float r_f, g_f, b_f;
    switch (i) {
        case 0:
            r_f = v;
            g_f = t;
            b_f = p;
            break;
        case 1:
            r_f = q;
            g_f = v;
            b_f = p;
            break;
        case 2:
            r_f = p;
            g_f = v;
            b_f = t;
            break;
        case 3:
            r_f = p;
            g_f = q;
            b_f = v;
            break;
        case 4:
            r_f = t;
            g_f = p;
            b_f = v;
            break;
        case 5:
            r_f = v;
            g_f = p;
            b_f = q;
            break;
        default:
            r_f = g_f = b_f = 0; // Should never reach here
            break;
    }

    // Convert to 0-255 range and assign to output
    *r = (int)(r_f * 255);
    *g = (int)(g_f * 255);
    *b = (int)(b_f * 255);

    return;
}

// Function to cycle through all hues once in 1 second
void cycle_hue_once(void)
{
    float hue = 0.0;
    const float hue_increment = 1.0 / 100.0; // Increment to cycle in 1 second (100 steps)

    for (int step = 0; step < 100; ++step) {
        int r, g, b;
        hue_to_rgb(hue, &r, &g, &b);

        // HAL_SPI_Transmit(&hspi1, 1, 6, 100);

        hue += hue_increment;
        HAL_Delay(10); // Delay 10 ms to achieve 1-second total duration
    }
}

void set_led_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    // Scale factors for color brightness adjustment
    float green_scale = 1.2; // Green is brighter, so scale it down more
    float blue_scale = 1.1; // Blue is slightly brighter, scale it down slightly

    // // Debugging outputs
    // int test1 = TIM2->CCR4; // Scaled green PWM value
    // int test2 = TIM3->CCR3; // Scaled blue PWM value
    // int test3 = TIM4->CCR4; // Scaled red PWM value

    // Compute scaled PWM values
    TIM2->CCR4 = (uint8_t)(255 - (green / green_scale)); // Green channel
    TIM3->CCR3 = (uint8_t)(255 - (blue / blue_scale)); // Blue channel
    TIM4->CCR4 = (uint8_t)(255 - red); // Red channel (no scaling)

    // test1 = TIM4->CCR4;
}

// Function to set the LED color using hue and brightness (both between 0 and 1)
void set_led_hue(float hue, float brightness)
{
    int r, g, b;

    // Convert hue to RGB values
    hue_to_rgb(hue, &r, &g, &b);

    // Apply brightness scaling
    r = (int)(r * brightness);
    g = (int)(g * brightness);
    b = (int)(b * brightness);

    // Set the RGB PWM outputs (active-low for sink configuration)
    set_led_rgb((uint8_t)r, (uint8_t)g, (uint8_t)b);
}
