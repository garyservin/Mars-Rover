#include "serial_controller.h"
#include "rover_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define MAX_REGISTRATED_CALLBACKS 2

#define SERIAL_DATA_NOTIFICATION 1

#define SERIAL_PACKET_LENGTH 12

static const char *TAG = "SERIAL_CONTROLLER";

static IRAM_ATTR void serial_availible_isr(void);

static void serial_state_checker(void *args);
static void serial_receive_task(void *args);

static SerialControllerStatusCb *status_callbacks[MAX_REGISTRATED_CALLBACKS];
static uint8_t num_callbacks;
static bool is_initialized = false;
static uint16_t channel_values[RC_NUM_CHANNELS] = {0};
static SerialControllerStatus status = SERIAL_CONTROLLER_DISCONNECTED;
static uint8_t receive_buf[SERIAL_PACKET_LENGTH];

static uint32_t last_serial_data_ticks = 0;
static uint32_t last_serial_data_checked_ticks = 0;

static TaskHandle_t task_handle;
static xSemaphoreHandle serial_sem;

void serial_controller_init()
{
    assert(!is_initialized);
    BaseType_t task_create_status;

    memset(channel_values, 0, sizeof(channel_values));
    num_callbacks = 0;
    serial_sem = xSemaphoreCreateBinary();
    assert(serial_sem != NULL);

    SERIAL_CONTROLLER_PORT.begin(SERIAL_CONTROLLER_PORT_SPEED);

    xSemaphoreGive(serial_sem);

    task_create_status = xTaskCreate(serial_state_checker, "serial_state_checker", 2048, NULL, tskIDLE_PRIORITY, NULL);
    assert(task_create_status == pdPASS);

    task_create_status = xTaskCreate(serial_receive_task, "serial_receive", 2048, NULL, 2, &task_handle);
    assert(task_create_status == pdPASS);

    is_initialized = true;

    ESP_LOGI(TAG, "Starting serial OK!");
}

void serial_controller_register_connection_callback(SerialControllerStatusCb *cb)
{
    assert(num_callbacks <= MAX_REGISTRATED_CALLBACKS);
    assert(is_initialized);
    status_callbacks[num_callbacks] = cb;
    num_callbacks++;
}

uint16_t serial_controller_get_val(uint8_t channel)
{
    uint16_t value;
    assert(status == SERIAL_CONTROLLER_CONNECTED);
    assert(channel < RC_NUM_CHANNELS);
    value = channel_values[channel];
    return value;
}

/*
void serial_controller_send(uint8_t *buf, uint16_t length)
{
    assert(xSemaphoreTake(serial_sem, pdMS_TO_TICKS(50)) == pdTRUE);
    SERIAL_CONTROLLER_PORT.write(buf, length);
    // SERIAL_CONTROLLER_PORT.endPacket();
    // SERIAL_CONTROLLER_PORT.receive();
    xSemaphoreGive(serial_sem);
}
    */

static void serial_receive_task(void *args)
{
    while (true)
    {
        uint32_t i = 0;

        assert(xSemaphoreTake(serial_sem, pdMS_TO_TICKS(50)) == pdTRUE);

        int bytes_available = SERIAL_CONTROLLER_PORT.available();
        if (bytes_available)
        {
            last_serial_data_ticks = xTaskGetTickCount();
            // printf("Serial Data available: %d bytes\n", bytes_available);
            for (int byte = 0; byte < bytes_available; byte++)
            {
                receive_buf[byte] = (uint8_t)SERIAL_CONTROLLER_PORT.read();
            }

            if (bytes_available != SERIAL_PACKET_LENGTH)
            {
                printf("ERROR: Unexpected data length: %d\n", bytes_available);
            }
            else
            {
                memcpy(channel_values, receive_buf, SERIAL_PACKET_LENGTH);
                // printf("Got: %d, %d \t %d, %d \t %d, %d\n", channel_values[0], channel_values[1], channel_values[2], channel_values[3], channel_values[4], channel_values[5]);
            }
        }
        xSemaphoreGive(serial_sem);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void serial_state_checker(void *args)
{
    while (true)
    {
        if (last_serial_data_ticks > last_serial_data_checked_ticks)
        {
            if (status == SERIAL_CONTROLLER_DISCONNECTED)
            {
                // printf("Setting status to connected\n");
                status = SERIAL_CONTROLLER_CONNECTED;
                for (uint8_t i = 0; i < num_callbacks; i++)
                {
                    status_callbacks[i](SERIAL_CONTROLLER_CONNECTED);
                }
            }
        }
        else
        {
            if (status == SERIAL_CONTROLLER_CONNECTED)
            {
                // printf("Setting status to disconnected\n");
                status = SERIAL_CONTROLLER_DISCONNECTED;
                for (uint8_t i = 0; i < num_callbacks; i++)
                {
                    status_callbacks[i](SERIAL_CONTROLLER_DISCONNECTED);
                }
            }
        }
        last_serial_data_checked_ticks = xTaskGetTickCount();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
