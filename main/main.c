#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <esp_task_wdt.h>
#include "io/voltage_reader.h"
#include "mtcp/mtcp_interface.h"



void app_main(void)
{
 voltage_reader_t vrt = {
        .width = ADC_WIDTH_BIT_12,
        .channel = ADC1_CHANNEL_6,
        .atten_level = ADC_ATTEN_DB_0,
        .adc_res = 4096,
        .div_r1 = 51700.0f,
        .div_r2 = 10000.0f
    };
    if(vrt_init(&vrt) == -1)
    {
        printf("Failed to initialize voltage reader!\n");
        return;
    }

    mtcp_interface_t iface;

    mtcp_if_cfg_t cfg = {
        .queue_size = 10,
        .rx_buffer_size = 1024 * 2,
        .tx_buffer_size = 1024 * 2,
        .rx_gpio_num = 16,
        .tx_gpio_num = 17,
        .uart_port = UART_NUM_2
    };

    if(mtcp_if_init(&iface, &cfg) != ESP_OK)
    {
        printf("failed to initialize MTCP\n");
        return;
    }
    // vrt_task_t task = {
    //     .reader = &vrt,
    // };

    // task.output_queue = xQueueCreate(10, sizeof(float));

    // TaskHandle_t vrt_task;
    // xTaskCreate(voltage_reader_task, "Voltage Reader Task", 1000, &task, 1, &vrt_task);


    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}
