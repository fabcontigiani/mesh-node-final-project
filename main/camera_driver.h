/**
 * @file camera_driver.h
 * @brief ESP32-CAM camera driver interface
 */

#pragma once

#include "esp_err.h"
#include "esp_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the camera module
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_init(void);

/**
 * @brief Capture a photo and return the frame buffer
 * @return Pointer to camera frame buffer on success, NULL on failure
 * @note Caller must call camera_return_frame_buffer() to free the buffer
 */
camera_fb_t* camera_capture_photo(void);

/**
 * @brief Return the frame buffer to the camera driver
 * @param fb Frame buffer to return
 */
void camera_return_frame_buffer(camera_fb_t* fb);

/**
 * @brief Check if camera is supported on this platform
 * @return true if camera is supported, false otherwise
 */
bool camera_is_supported(void);

#ifdef __cplusplus
}
#endif
