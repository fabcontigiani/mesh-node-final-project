/**
 * @file app_config.h
 * @brief Application configuration and constants
 */

#pragma once

/* Compatibility macro for IDF 5.x */
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

/* ESP32-CAM (AI-Thinker) Pin Definitions */
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1  // Software reset will be performed
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD    26  // SDA
#define CAM_PIN_SIOC    27  // SCL

#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0      5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22
