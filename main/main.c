#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "driver/rmt.h"
#include "nvs_flash.h"

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event_loop.h"

#include "mqtt.h"

static const char *TAG = "salt_level_sensor";

#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define RMT_RX_CHANNEL RMT_CHANNEL_1
#define PING_TX_PIN 22
#define PING_RX_PIN 32

#define ECHO_TIMEOUT_US 18500 // Max echo time in μs

// 80MHz/80 == 1μs ticks
#define RMT_CLK_DIV 80 // 1μs precision

TimerHandle_t timer_handle = NULL;
mqtt_client* mqtt_client_handle = NULL;
RingbufHandle_t rx_ring_buffer_handle = NULL;

rmt_item32_t items[] = {
    // 10μs pulse
    {{{ 10, 1, 0, 0 }}}
};

void connected_cb(mqtt_client *client, mqtt_event_data_t *event_data);
void disconnected_cb(mqtt_client *client, mqtt_event_data_t *event_data);

mqtt_settings mqttsettings = {
    .host = CONFIG_MQTT_HOST,
    .username = CONFIG_MQTT_USERNAME,
    .password = CONFIG_MQTT_PASSWORD,
    .port = CONFIG_MQTT_PORT,
    .client_id = "esp32_sls",
    .lwt_topic = CONFIG_MQTT_LWT_TOPIC,
    .lwt_msg = "offline",
    .lwt_qos = 1,
    .lwt_retain = 1,
    .clean_session = 1,
    .keepalive = 60,
	.connected_cb = connected_cb,
	.disconnected_cb = disconnected_cb,
};

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_GOT_IP:
            mqtt_start(&mqttsettings);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            mqtt_stop();
            break;
        default:
            break;
    }
    return ESP_OK;
}

void init_wifi()
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );
}

void rmt_tx_init()
{
    rmt_config_t txConfig = {
        .rmt_mode = RMT_MODE_TX,
        .channel = RMT_TX_CHANNEL,
        .gpio_num = PING_TX_PIN,
        .mem_block_num = 1,
        .tx_config.loop_en = 0,
        .tx_config.carrier_en = 0,
        .tx_config.idle_output_en = 1,
        .tx_config.idle_level = 0,
        .clk_div = RMT_CLK_DIV
    };

    ESP_ERROR_CHECK(rmt_config(&txConfig));
    ESP_ERROR_CHECK(rmt_driver_install(txConfig.channel, 0, 0));
}

void rmt_rx_init()
{
    rmt_config_t rxConfig = {
        .rmt_mode = RMT_MODE_RX,
        .channel = RMT_RX_CHANNEL,
        .gpio_num = PING_RX_PIN,
        .clk_div = RMT_CLK_DIV,
        .mem_block_num = 1,
        .rx_config.idle_threshold = ECHO_TIMEOUT_US
    };

    ESP_ERROR_CHECK(rmt_config(&rxConfig));
    ESP_ERROR_CHECK(rmt_driver_install(rxConfig.channel, 128, 0));
    ESP_ERROR_CHECK(rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rx_ring_buffer_handle));
}

size_t rx_size;
char distance_buf[16];

void take_reading(void *pvParameter)
{
    // Write the 10us trigger pulse
    ESP_ERROR_CHECK(rmt_write_items(RMT_TX_CHANNEL, items, 1, true));

    ESP_ERROR_CHECK(rmt_rx_start(RMT_RX_CHANNEL, 1));

    rx_size = 0;
    rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rx_ring_buffer_handle, &rx_size, 1000);

    if (item) {
        // Got something
        float distance = item->duration0 / 148.0; // to inches
        int msg_size = sprintf(distance_buf, "%f", distance);

        ESP_LOGI(TAG, "%s in.\n", distance_buf);
        mqtt_publish(mqtt_client_handle, CONFIG_MQTT_SALT_LEVEL_TOPIC, distance_buf, msg_size, 0, 1);

        vRingbufferReturnItem(rx_ring_buffer_handle, (void*) item);
    }

    ESP_ERROR_CHECK(rmt_rx_stop(RMT_RX_CHANNEL));

    vTaskDelete(NULL);
}

void readingTimerCallback(TimerHandle_t xTimer)
{
    xTaskCreate(&take_reading, "Take Reading Task", 2048, NULL, 5, NULL);
}

void connected_cb(mqtt_client *client, mqtt_event_data_t *event_data)
{
    mqtt_client_handle = client;
    ESP_LOGI(TAG, "MQTT connected");

    if (!timer_handle)
    {
        timer_handle = xTimerCreate(
                "reading_timer",
                pdMS_TO_TICKS(CONFIG_SENSOR_READ_INTERVAL * 1000),
                true,
                (void *) 0,
                readingTimerCallback
        );

        if(timer_handle == NULL)
        {
            ESP_LOGE(TAG, "Failed to create timer");
        }
    }

    if(xTimerStart(timer_handle, 1) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to start timer");
    }

    mqtt_publish(mqtt_client_handle, CONFIG_MQTT_LWT_TOPIC, "online", 6, 1, 1);

    // Take a reading right away
    readingTimerCallback(NULL);
}

void disconnected_cb(mqtt_client *client, mqtt_event_data_t *event_data)
{
    mqtt_client_handle = NULL;
    ESP_LOGI(TAG, "MQTT disconnected");

    if(xTimerStop(timer_handle, 1) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to stop timer");
    }

    esp_wifi_connect();
}

void app_main()
{
    nvs_flash_init();
    init_wifi();

    rmt_tx_init();
    rmt_rx_init();
}
