// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mdf_common.h"
#include "mwifi.h"
#include "driver/uart.h"
#include "app_config.h"
#include "camera_driver.h"

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <errno.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/gpio.h"
#include "sd_test_io.h"
#include "esp_http_client.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#if SOC_SDMMC_IO_POWER_EXTERNAL
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#endif

#include "freertos/queue.h"
#include "driver/gpio.h"

#define PIR_SENSOR_PIN 12 // GPIO 12
#define DATA_BUFFER_SIZE 65536 // 64KB buffer for ESP-CAM image data

// Structure to hold received data for storage task
typedef struct {
    uint8_t src_addr[MWIFI_ADDR_LEN];
    uint8_t *data;
    size_t size;
    mwifi_data_type_t data_type;
} received_data_t;

// HTTP server configuration
#define HTTP_SERVER_URL "http://fabcontigiani.uno:8000/upload/"  // Change this to your server
#define HTTP_TIMEOUT_MS 10000

static const char *TAG = "router_example";

// Task handle for motion detection task
static TaskHandle_t motion_detection_task_handle = NULL;

// Queue and task handle for HTTP operations
static QueueHandle_t http_queue = NULL;
static TaskHandle_t http_task_handle = NULL;

#define HTTP_QUEUE_SIZE 10

// ISR handler for GPIO interrupt
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify the motion detection task
    if (motion_detection_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(motion_detection_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static bool is_sd_card_mounted(void)
{
    struct stat st;
    return (stat("/sdcard", &st) == 0);
}

static esp_err_t s_example_write_file(const char *path, uint8_t *data, size_t size)
{
    ESP_LOGI(TAG, "Writing binary file: %s (%zu bytes)", path, size);
    
    // Check if SD card is mounted
    if (!is_sd_card_mounted()) {
        ESP_LOGE(TAG, "SD card not mounted, cannot write file");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Validate input parameters
    if (path == NULL || data == NULL || size == 0) {
        ESP_LOGE(TAG, "Invalid parameters for file write");
        return ESP_ERR_INVALID_ARG;
    }
    
    FILE *f = fopen(path, "wb");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing: %s (errno: %d)", path, errno);
        return ESP_FAIL;
    }
    
    size_t written = fwrite(data, 1, size, f);
    int fclose_result = fclose(f);
    
    if (fclose_result != 0) {
        ESP_LOGE(TAG, "Failed to close file: %s (errno: %d)", path, errno);
    }

    if (written != size)
    {
        ESP_LOGE(TAG, "Failed to write complete data to file (wrote %zu of %zu bytes)", written, size);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Binary file written successfully: %zu bytes", written);
    return ESP_OK;
}

/**
 * @brief Task to handle motion detection notifications
 */
static void motion_detection_task(void *arg)
{
    ESP_LOGI(TAG, "Motion detection task started");

    for (;;)
    {
        // Wait for notification from ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Print motion detected message
        ESP_LOGI(TAG, "*** MOVEMENT DETECTED! *** PIR sensor triggered");

        // Optional: Add debouncing delay to prevent spam
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

/**
 * @brief Task to handle HTTP POST of received data
 */
static void http_post_task(void *arg)
{
    ESP_LOGI(TAG, "HTTP POST task started");
    received_data_t received_item;
    
    for (;;)
    {
        // Wait for data to be queued
        if (xQueueReceive(http_queue, &received_item, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGI(TAG, "Processing received data from: " MACSTR " (%d bytes)", 
                     MAC2STR(received_item.src_addr), received_item.size);
            
            // Validate received data
            if (received_item.data == NULL || received_item.size == 0) {
                ESP_LOGW(TAG, "Invalid received data, skipping HTTP POST");
                if (received_item.data != NULL) {
                    MDF_FREE(received_item.data);
                }
                continue;
            }
            
            // Generate filename for the upload
            char filename[64];
            snprintf(filename, sizeof(filename), 
                     "received_%02x%02x%02x%02x%02x%02x_%lu.jpg", 
                     received_item.src_addr[0], received_item.src_addr[1], 
                     received_item.src_addr[2], received_item.src_addr[3], 
                     received_item.src_addr[4], received_item.src_addr[5], 
                     (unsigned long)(esp_timer_get_time() / 1000));
            
            ESP_LOGI(TAG, "Sending received data via HTTP POST: %s (%d bytes)", filename, received_item.size);
            ESP_LOGI(TAG, "HTTP Server URL: %s", HTTP_SERVER_URL);
            
            // Try using esp_http_client_perform with pre-built data
            // Create the complete multipart body in memory first
            char boundary[] = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
            
            char form_start[512];
            char form_end[128];
            
            snprintf(form_start, sizeof(form_start),
                "--%s\r\n"
                "Content-Disposition: form-data; name=\"image\"; filename=\"%s\"\r\n"
                "Content-Type: image/jpeg\r\n\r\n", 
                boundary, filename);
            snprintf(form_end, sizeof(form_end), "\r\n--%s--\r\n", boundary);
            
            int start_len = strlen(form_start);
            int end_len = strlen(form_end);
            int total_body_len = start_len + received_item.size + end_len;
            
            // Allocate memory for complete body in PSRAM
            uint8_t *complete_body = heap_caps_calloc(1, total_body_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (complete_body == NULL) {
                ESP_LOGW(TAG, "Failed to allocate HTTP body in PSRAM, trying regular heap (%d bytes)", total_body_len);
                complete_body = MDF_MALLOC(total_body_len);
                if (complete_body == NULL) {
                    ESP_LOGE(TAG, "Failed to allocate memory for HTTP body (%d bytes)", total_body_len);
                    ESP_LOGE(TAG, "Free heap: %u bytes, largest free block: %u bytes", 
                             esp_get_free_heap_size(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
                    MDF_FREE(received_item.data);
                    continue;
                }
            }
            
            // Build complete multipart body
            memcpy(complete_body, form_start, start_len);
            memcpy(complete_body + start_len, received_item.data, received_item.size);
            memcpy(complete_body + start_len + received_item.size, form_end, end_len);
            
            ESP_LOGI(TAG, "Built complete multipart body: %d bytes", total_body_len);
            ESP_LOGI(TAG, "Form structure preview: %.100s", (char*)complete_body);
            
            // Configure HTTP client with complete body
            esp_http_client_config_t config = {
                .url = HTTP_SERVER_URL,
                .method = HTTP_METHOD_POST,
                .timeout_ms = HTTP_TIMEOUT_MS,
                .max_redirection_count = 5,
                .disable_auto_redirect = false,
            };
            
            esp_http_client_handle_t client = esp_http_client_init(&config);
            if (client == NULL) {
                ESP_LOGE(TAG, "Failed to initialize HTTP client");
                MDF_FREE(received_item.data);
                MDF_FREE(complete_body);
                continue;
            }
            
            // Set content type header
            char content_type[128];
            snprintf(content_type, sizeof(content_type), "multipart/form-data; boundary=%s", boundary);
            esp_http_client_set_header(client, "Content-Type", content_type);
            
            // Set the complete body
            esp_http_client_set_post_field(client, (char*)complete_body, total_body_len);
            
            // Perform the request
            esp_err_t err = esp_http_client_perform(client);
            
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "HTTP request completed successfully");
            } else {
                ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
            }
            
            // Get response details
            int status_code = esp_http_client_get_status_code(client);
            int content_length = esp_http_client_get_content_length(client);
            
            ESP_LOGI(TAG, "HTTP POST completed - Status: %d, Content-Length: %d", status_code, content_length);
            
            if (status_code >= 200 && status_code < 300) {
                ESP_LOGI(TAG, "Successfully uploaded image: %s", filename);
            } else if (status_code >= 300 && status_code < 400) {
                // Handle redirect responses
                ESP_LOGW(TAG, "HTTP redirect response: %d", status_code);
                
                // Get the Location header for debugging
                // char location_buffer[256];
                // int location_len = esp_http_client_get_header(client, "Location", location_buffer, sizeof(location_buffer) - 1);
                // if (location_len > 0) {
                //     location_buffer[location_len] = '\0';
                //     ESP_LOGW(TAG, "Redirect location: %s", location_buffer);
                // }
                
                ESP_LOGE(TAG, "HTTP upload failed due to redirect: %d", status_code);
            } else {
                ESP_LOGE(TAG, "HTTP upload failed with status: %d", status_code);
                
                // Read error response if available
                if (content_length > 0 && content_length < 1024) {
                    char response_buffer[1024];
                    int read_len = esp_http_client_read_response(client, response_buffer, sizeof(response_buffer) - 1);
                    if (read_len > 0) {
                        response_buffer[read_len] = '\0';
                        ESP_LOGE(TAG, "Server response: %s", response_buffer);
                    }
                }
            }
            
            // Cleanup
            esp_http_client_cleanup(client);
            MDF_FREE(complete_body);
            MDF_FREE(received_item.data);
        }
    }
    
    vTaskDelete(NULL);
}

// #define MEMORY_DEBUG

#define EXAMPLE_MAX_CHAR_SIZE 64

#define MOUNT_POINT "/sdcard"

#ifdef CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS
const char *names[] = {"CLK", "CMD", "D0", "D1", "D2", "D3"};
const int pins[] = {CONFIG_EXAMPLE_PIN_CLK,
                    CONFIG_EXAMPLE_PIN_CMD,
                    CONFIG_EXAMPLE_PIN_D0
#ifdef CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4
                    ,
                    CONFIG_EXAMPLE_PIN_D1,
                    CONFIG_EXAMPLE_PIN_D2,
                    CONFIG_EXAMPLE_PIN_D3
#endif
};

const int pin_count = sizeof(pins) / sizeof(pins[0]);

#if CONFIG_EXAMPLE_ENABLE_ADC_FEATURE
const int adc_channels[] = {CONFIG_EXAMPLE_ADC_PIN_CLK,
                            CONFIG_EXAMPLE_ADC_PIN_CMD,
                            CONFIG_EXAMPLE_ADC_PIN_D0
#ifdef CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4
                            ,
                            CONFIG_EXAMPLE_ADC_PIN_D1,
                            CONFIG_EXAMPLE_ADC_PIN_D2,
                            CONFIG_EXAMPLE_ADC_PIN_D3
#endif
};
#endif // CONFIG_EXAMPLE_ENABLE_ADC_FEATURE

pin_configuration_t config = {
    .names = names,
    .pins = pins,
#if CONFIG_EXAMPLE_ENABLE_ADC_FEATURE
    .adc_channels = adc_channels,
#endif
};
#endif // CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS


#if CONFIG_EXAMPLE_PIN_CARD_POWER_RESET
static esp_err_t s_example_reset_card_power(void)
{
    esp_err_t ret = ESP_FAIL;
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << CONFIG_EXAMPLE_PIN_CARD_POWER_RESET),
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to config GPIO");
        return ret;
    }

    ret = gpio_set_level(CONFIG_EXAMPLE_PIN_CARD_POWER_RESET, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set GPIO level");
        return ret;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);

    ret = gpio_set_level(CONFIG_EXAMPLE_PIN_CARD_POWER_RESET, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set GPIO level");
        return ret;
    }

    return ESP_OK;
}
#endif // CONFIG_EXAMPLE_PIN_CARD_POWER_RESET

#define BUF_SIZE (1024)

static esp_netif_t *netif_sta = NULL;

// Function prototypes
static esp_err_t capture_and_save_photo(int photo_counter);

// Global counter for photo naming
static int g_photo_counter = 0;

/**
 * @brief Task to capture and save photos periodically when mesh is ready
 */
static void capture_photo_task(void *arg)
{
    ESP_LOGI(TAG, "Capture photo task started - mesh is ready!");

    for (;;)
    {
        /* Wait for mesh connection before attempting capture */
        if (!mwifi_is_connected())
        {
            ESP_LOGW(TAG, "Waiting for mesh connection...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Capture and save photo */
        ESP_LOGI(TAG, "Capturing photo #%d...", g_photo_counter + 1);
        esp_err_t ret = capture_and_save_photo(++g_photo_counter);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Photo #%d captured and saved successfully!", g_photo_counter);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to capture/save photo #%d: %s", g_photo_counter, esp_err_to_name(ret));
        }

        /* Wait 10 seconds before next capture */
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

    /* This should never be reached, but good practice */
    vTaskDelete(NULL);
}

static mdf_err_t sd_init()
{
    /* Initialize SD card */
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.

    ESP_LOGI(TAG, "Using SDMMC peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 40MHz for SDMMC)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
#if CONFIG_EXAMPLE_SDMMC_SPEED_HS
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
#elif CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_SDR50
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_SDR50;
    host.flags &= ~SDMMC_HOST_FLAG_DDR;
#elif CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_DDR50
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_DDR50;
#elif CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_SDR104
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_SDR104;
    host.flags &= ~SDMMC_HOST_FLAG_DDR;
#endif

    // For SoCs where the SD power can be supplied both via an internal or external (e.g. on-board LDO) power supply.
    // When using specific IO pins (which can be used for ultra high-speed SDMMC) to connect to the SD card
    // and the internal LDO power supply, we need to initialize the power supply first.
#if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_IO_ID,
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;

    ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create a new on-chip LDO power control driver");
        return ret;
    }
    host.pwr_ctrl_handle = pwr_ctrl_handle;
#endif

#if CONFIG_EXAMPLE_PIN_CARD_POWER_RESET
    ESP_ERROR_CHECK(s_example_reset_card_power());
#endif

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
#if EXAMPLE_IS_UHS1
    slot_config.flags |= SDMMC_SLOT_FLAG_UHS1;
#endif

    // Set bus width to use:
#ifdef CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4
    slot_config.width = 4;
#else
    slot_config.width = 1;
#endif

    // On chips where the GPIOs used for SD card can be configured, set them in
    // the slot_config structure:
#ifdef CONFIG_SOC_SDMMC_USE_GPIO_MATRIX
    slot_config.clk = CONFIG_EXAMPLE_PIN_CLK;
    slot_config.cmd = CONFIG_EXAMPLE_PIN_CMD;
    slot_config.d0 = CONFIG_EXAMPLE_PIN_D0;
#ifdef CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4
    slot_config.d1 = CONFIG_EXAMPLE_PIN_D1;
    slot_config.d2 = CONFIG_EXAMPLE_PIN_D2;
    slot_config.d3 = CONFIG_EXAMPLE_PIN_D3;
#endif // CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4
#endif // CONFIG_SOC_SDMMC_USE_GPIO_MATRIX

    // Enable internal pullups on enabled pins. The internal pullups
    // are insufficient however, please make sure 10k external pullups are
    // connected on the bus. This is for debug / example purpose only.
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                          "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
#ifdef CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS
            check_sd_card_pins(&config, pin_count);
#endif
        }
        return ret;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    return ret;
}

static void root_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    // Allocate larger data buffer in PSRAM for ESP-CAM image data
    // Use 64KB buffer to handle larger image chunks or accumulated data
    uint8_t *data = heap_caps_calloc(1, DATA_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (data == NULL) {
        ESP_LOGW(TAG, "Failed to allocate 64KB data buffer in PSRAM, trying smaller size");
        data = heap_caps_calloc(1, MWIFI_PAYLOAD_LEN, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (data == NULL) {
            ESP_LOGW(TAG, "Failed to allocate in PSRAM, trying regular heap");
            data = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
            if (data == NULL) {
                ESP_LOGE(TAG, "Failed to allocate data buffer");
                ESP_LOGE(TAG, "Free heap: %u bytes, largest free block: %u bytes", 
                         esp_get_free_heap_size(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
                vTaskDelete(NULL);
                return;
            }
        }
    }
    size_t size = DATA_BUFFER_SIZE;

    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    mwifi_data_type_t data_type = {0};

    MDF_LOGI("Root task is running");

    // Wait until node is actually root before attempting to read
    while (!esp_mesh_is_root()) {
        ESP_LOGI(TAG, "Waiting to become root node...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        if (!mwifi_is_started()) {
            ESP_LOGW(TAG, "Mesh not started, waiting...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
    }
    
    ESP_LOGI(TAG, "Node is now root, starting to read data");

    for (;;)
    {
        if (!mwifi_is_started())
        {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        // Double-check we're still root before attempting read
        if (!esp_mesh_is_root()) {
            ESP_LOGW(TAG, "Node is no longer root, waiting to become root again...");
            while (!esp_mesh_is_root()) {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                if (!mwifi_is_started()) {
                    break;
                }
            }
            ESP_LOGI(TAG, "Node is root again, resuming data reading");
            continue;
        }

        size = DATA_BUFFER_SIZE;
        memset(data, 0, DATA_BUFFER_SIZE);
        ret = mwifi_root_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_root_read", mdf_err_to_name(ret));
        MDF_LOGI("Root receive, addr: " MACSTR ", size: %d", MAC2STR(src_addr), size);

        // Validate received data size
        if (size == 0) {
            ESP_LOGW(TAG, "Received empty data, skipping save");
            continue;
        }

        // Prepare data for storage task
        received_data_t received_item;
        memcpy(received_item.src_addr, src_addr, MWIFI_ADDR_LEN);
        received_item.data_type = data_type;
        received_item.size = size;
        
        // Allocate memory for the data copy in PSRAM (for large image data)
        received_item.data = heap_caps_calloc(1, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (received_item.data == NULL) {
            ESP_LOGW(TAG, "Failed to allocate in PSRAM, trying regular heap for %zu bytes", size);
            // Fallback to regular heap for smaller data
            received_item.data = MDF_MALLOC(size);
            if (received_item.data == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for received data (%zu bytes)", size);
                ESP_LOGE(TAG, "Free heap: %u bytes, largest free block: %u bytes", 
                         esp_get_free_heap_size(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
                continue;
            }
        }
        memcpy(received_item.data, data, size);
        
        // Queue the data for HTTP POST task
        if (http_queue != NULL) {
            if (xQueueSend(http_queue, &received_item, pdMS_TO_TICKS(1000)) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to queue received data for HTTP POST (queue full?)");
                MDF_FREE(received_item.data); // Free memory if queueing failed
            } else {
                ESP_LOGI(TAG, "Queued received data for HTTP POST");
            }
        } else {
            ESP_LOGE(TAG, "HTTP queue not initialized");
            MDF_FREE(received_item.data);
        }

        // size = sprintf(data, "(%d) Hello node!", i);
        // ret = mwifi_root_write(src_addr, 1, &data_type, data, size, true);
        // MDF_ERROR_CONTINUE(ret != MDF_OK, "mwifi_root_recv, ret: %x", ret);
        // MDF_LOGI("Root send, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);
    }

    MDF_LOGW("Root is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);
}

/**
 * @brief Capture and save a photo to SD card with counter
 * @param photo_counter Counter for unique photo naming
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t capture_and_save_photo(int photo_counter)
{
    if (!camera_is_supported())
    {
        ESP_LOGW(TAG, "Camera not supported on this platform");
        return ESP_ERR_NOT_SUPPORTED;
    }

    camera_fb_t *frame_buffer = camera_capture_photo();
    if (frame_buffer == NULL)
    {
        ESP_LOGE(TAG, "Failed to capture photo");
        return ESP_FAIL;
    }

    /* Create unique filename with counter */
    char photo_path[64];
    snprintf(photo_path, sizeof(photo_path), "/sdcard/pic_%d.jpg", photo_counter);

    ESP_LOGI(TAG, "Attempting to save photo to: %s", photo_path);

    /* Save photo to SD card */
    esp_err_t write_ret = s_example_write_file(photo_path, frame_buffer->buf, frame_buffer->len);
    if (write_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write photo to SD card: %s", esp_err_to_name(write_ret));
        camera_return_frame_buffer(frame_buffer);
        return write_ret;
    }

    ESP_LOGI(TAG, "Photo saved to: %s (size: %d bytes)", photo_path, frame_buffer->len);

    /* Send image over mesh (only if connected) */
    if (mwifi_is_connected())
    {
        ESP_LOGI(TAG, "Sending image over mesh, size: %d bytes", frame_buffer->len);
        mwifi_data_type_t data_type = {0x0};

        if (esp_mesh_is_root())
        {
            ESP_LOGI(TAG, "Node is root, skipping mesh transmission");
            camera_return_frame_buffer(frame_buffer);
            return ESP_OK;
        }

        mdf_err_t mesh_ret = mwifi_write(NULL, &data_type, frame_buffer->buf, frame_buffer->len, true);
        if (mesh_ret != MDF_OK)
        {
            ESP_LOGE(TAG, "Failed to send image over mesh: %s", mdf_err_to_name(mesh_ret));
        }
        else
        {
            ESP_LOGI(TAG, "Image sent over mesh successfully!");
        }
    }
    else
    {
        ESP_LOGW(TAG, "Mesh not connected, skipping mesh transmission");
    }

    /* Return the frame buffer */
    camera_return_frame_buffer(frame_buffer);

    return ESP_OK;
}

/**
 * @brief Timed printing system information
 */
static void print_system_info_timercb(TimerHandle_t timer)
{
    uint8_t primary = 0;
    wifi_second_chan_t second = 0;
    mesh_addr_t parent_bssid = {0};
    uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};
    wifi_sta_list_t wifi_sta_list = {0x0};

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);
    esp_mesh_get_parent_bssid(&parent_bssid);

    MDF_LOGI("System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
             ", parent rssi: %d, node num: %d, free heap: %" PRIu32,
             primary,
             esp_mesh_get_layer(), MAC2STR(sta_mac), MAC2STR(parent_bssid.addr),
             mwifi_get_parent_rssi(), esp_mesh_get_total_node_num(), esp_get_free_heap_size());

    for (int i = 0; i < wifi_sta_list.num; i++)
    {
        MDF_LOGI("Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
    }

#ifdef MEMORY_DEBUG

    if (!heap_caps_check_integrity_all(true))
    {
        MDF_LOGE("At least one heap is corrupt");
    }

    mdf_mem_print_heap();
    mdf_mem_print_record();
    mdf_mem_print_task();
#endif /**< MEMORY_DEBUG */
}

static mdf_err_t wifi_init()
{
    mdf_err_t ret = nvs_flash_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        MDF_ERROR_ASSERT(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    MDF_ERROR_ASSERT(ret);

    MDF_ERROR_ASSERT(esp_netif_init());
    MDF_ERROR_ASSERT(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));
    MDF_ERROR_ASSERT(esp_wifi_init(&cfg));
    MDF_ERROR_ASSERT(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    MDF_ERROR_ASSERT(esp_wifi_set_mode(WIFI_MODE_STA));
    MDF_ERROR_ASSERT(esp_wifi_set_ps(WIFI_PS_NONE));
    MDF_ERROR_ASSERT(esp_mesh_set_6m_rate(false));
    MDF_ERROR_ASSERT(esp_wifi_start());

    return MDF_OK;
}

/**
 * @brief All module events will be sent to this task in esp-mdf
 *
 * @Note:
 *     1. Do not block or lengthy operations in the callback function.
 *     2. Do not consume a lot of memory in the callback function.
 *        The task memory of the callback function is only 4KB.
 */
static mdf_err_t event_loop_cb(mdf_event_loop_t event, void *ctx)
{
    MDF_LOGI("event_loop_cb, event: %" PRIu32, event);

    switch (event)
    {
    case MDF_EVENT_MWIFI_STARTED:
        MDF_LOGI("MESH is started");
        break;

    case MDF_EVENT_MWIFI_PARENT_CONNECTED:
        MDF_LOGI("Parent is connected on station interface");

        if (esp_mesh_is_root())
        {
            esp_netif_dhcpc_start(netif_sta);
        }

        break;

    case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
        MDF_LOGI("Parent is disconnected on station interface");
        break;

    case MDF_EVENT_MWIFI_ROUTING_TABLE_ADD:
    case MDF_EVENT_MWIFI_ROUTING_TABLE_REMOVE:
        MDF_LOGI("total_num: %d", esp_mesh_get_total_node_num());
        break;

    case MDF_EVENT_MWIFI_ROOT_GOT_IP:
    {
        MDF_LOGI("Root obtains the IP address. It is posted by LwIP stack automatically");
        break;
    }

    default:
        break;
    }

    return MDF_OK;
}

void app_main()
{
    /* Initialize camera */
    if (camera_is_supported())
    {
        if (camera_init() != ESP_OK)
        {
            ESP_LOGE(TAG, "Camera initialization failed, exiting");
            return;
        }
    }
    else
    {
        ESP_LOGW(TAG, "Camera not supported, continuing with SD card only");
    }

    /* Initialize SD card for all nodes */
    ESP_LOGI(TAG, "Initializing SD card...");
    mdf_err_t sd_ret = sd_init();
    if (sd_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SD card initialization failed: %s", esp_err_to_name(sd_ret));
        ESP_LOGE(TAG, "Continuing without SD card functionality");
    }
    else
    {
        ESP_LOGI(TAG, "SD card initialized successfully");
    }

    mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
    mwifi_config_t config = {
        .router_ssid = CONFIG_ROUTER_SSID,
        .router_password = CONFIG_ROUTER_PASSWORD,
        .mesh_id = CONFIG_MESH_ID,
        .mesh_password = CONFIG_MESH_PASSWORD,
    };

    /**
     * @brief Set the log level for serial port printing.
     */
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    /**
     * @brief Initialize wifi mesh.
     */
    MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));
    MDF_ERROR_ASSERT(wifi_init());
    MDF_ERROR_ASSERT(mwifi_init(&cfg));
    MDF_ERROR_ASSERT(mwifi_set_config(&config));
    MDF_ERROR_ASSERT(mwifi_start());

    /**
     * @brief select/extend a group memebership here
     *      group id can be a custom address
     */
    const uint8_t group_id_list[2][6] = {{0x01, 0x00, 0x5e, 0xae, 0xae, 0xae},
                                         {0x01, 0x00, 0x5e, 0xae, 0xae, 0xaf}};

    MDF_ERROR_ASSERT(esp_mesh_set_group_id((mesh_addr_t *)group_id_list,
                                           sizeof(group_id_list) / sizeof(group_id_list[0])));

    ESP_LOGI(TAG, "Mesh initialization complete. Waiting for root to get IP...");

    TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_PERIOD_MS,
                                       true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);

    // Create motion detection task
    ESP_LOGI(TAG, "Creating motion detection task...");
    BaseType_t ret_task = xTaskCreate(motion_detection_task, "motion_detection", 2048, NULL, 5, &motion_detection_task_handle);
    if (ret_task != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create motion detection task");
        return;
    }

    // Create HTTP queue and task
    ESP_LOGI(TAG, "Creating HTTP queue and task...");
    http_queue = xQueueCreate(HTTP_QUEUE_SIZE, sizeof(received_data_t));
    if (http_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create HTTP queue");
        return;
    }
    
    ret_task = xTaskCreate(http_post_task, "http_post_task", 12288, NULL, 4, &http_task_handle);
    if (ret_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create HTTP POST task");
        vQueueDelete(http_queue);
        return;
    }
    ESP_LOGI(TAG, "HTTP queue and task created successfully");

    // Configure GPIO 12 as input with pull-down and interrupt on rising edge
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIR_SENSOR_PIN),
        .pull_down_en = 1,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    // Install GPIO ISR service and add handler
    // gpio_install_isr_service(0); // already installed
    gpio_isr_handler_add(PIR_SENSOR_PIN, gpio_isr_handler, NULL);


    MDF_LOGI("Creating root task...");
    xTaskCreate(root_task, "root_task", 4 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);

}
