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
#include "sdcard.h"

char *root = "/sdcard";
int clk = 14;
int cmd = 15;
int d0 = 2;

// #define MEMORY_DEBUG

#define BUF_SIZE (1024)

static const char *TAG = "router_example";
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
    
    for (;;) {
        /* Wait for mesh connection before attempting capture */
        if (!mwifi_is_connected()) {
            ESP_LOGW(TAG, "Waiting for mesh connection...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        
        /* Capture and save photo */
        ESP_LOGI(TAG, "Capturing photo #%d...", g_photo_counter + 1);
        esp_err_t ret = capture_and_save_photo(++g_photo_counter);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Photo #%d captured and saved successfully!", g_photo_counter);
        } else {
            ESP_LOGE(TAG, "Failed to capture/save photo #%d: %s", g_photo_counter, esp_err_to_name(ret));
        }
        
        /* Wait 10 seconds before next capture */
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
    
    /* This should never be reached, but good practice */
    vTaskDelete(NULL);
}

/**
 * @brief Capture and save a photo to SD card with counter
 * @param photo_counter Counter for unique photo naming
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t capture_and_save_photo(int photo_counter)
{
    if (!camera_is_supported()) {
        ESP_LOGW(TAG, "Camera not supported on this platform");
        return ESP_ERR_NOT_SUPPORTED;
    }

    camera_fb_t *frame_buffer = camera_capture_photo();
    if (frame_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to capture photo");
        return ESP_FAIL;
    }

    /* Create unique filename with counter */
    char photo_path[64];
    snprintf(photo_path, sizeof(photo_path), "/sdcard/pic_%d.jpg", photo_counter);
    
    ESP_LOGI(TAG, "Attempting to save photo to: %s", photo_path);
    
    /* Save photo to SD card */
    esp_err_t write_ret = sdcard.file.write(photo_path, frame_buffer->buf, &frame_buffer->len);
    if (write_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write photo to SD card: %s", esp_err_to_name(write_ret));
        camera_return_frame_buffer(frame_buffer);
        return write_ret;
    }
    
    ESP_LOGI(TAG, "Photo saved to: %s (size: %d bytes)", photo_path, frame_buffer->len);

    /* Send image over mesh (only if connected) */
    if (mwifi_is_connected()) {
        ESP_LOGI(TAG, "Sending image over mesh, size: %d bytes", frame_buffer->len);
        mwifi_data_type_t data_type = { 0x0 };

        mdf_err_t mesh_ret = mwifi_write(NULL, &data_type, frame_buffer->buf, frame_buffer->len, true);
        if (mesh_ret != MDF_OK) {
            ESP_LOGE(TAG, "Failed to send image over mesh: %s", mdf_err_to_name(mesh_ret));
        } else {
            ESP_LOGI(TAG, "Image sent over mesh successfully!");
        }
    } else {
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
        MDF_LOGI("Creating capture photo task...");
        xTaskCreate(capture_photo_task, "capture_photo_task", 4 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
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

    /* Initialize SD card */
    sdcard.debug = true;
    sdcard.version();
    esp_err_t err = sdcard.sdmmc.mount_data1(root, clk, cmd, d0);
    if (err)
    {
        ESP_LOGI(TAG, "sdcard.sdmmc.mount_data1 error (%d) %s\n", err, esp_err_to_name(err));
        return;
    }

    err = sdcard.info_print(root);
    if (err)
    {
        ESP_LOGI(TAG, "sdcard.info_print error (%d) %s\n", err, esp_err_to_name(err));
        return;
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
}
