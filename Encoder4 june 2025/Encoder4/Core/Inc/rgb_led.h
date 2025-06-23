/*
 * hue_to_rgb.h
 *
 *  Created on: Nov 7, 2024
 *      Author: kyleh
 */

#ifndef INC_RGB_LED_H_
#define INC_RGB_LED_H_

#include "stm32g0xx_hal.h"

// Function declaration for converting hue to RGB
void hue_to_rgb(float hue, int *r, int *g, int *b);

// Function to cycle through all hues once in 1 second
void cycle_hue_once(void);

void set_led_rgb(uint8_t red, uint8_t green, uint8_t blue);

void set_led_hue(float hue, float brightness);

#endif /* INC_RGB_LED_H_ */
