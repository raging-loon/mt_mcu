#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
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

    mtcp_interface_t iface = {.uart_port = UART_NUM_2};

    mtcp_if_cfg_t cfg = {
        .queue_size = 10,
        .rx_buffer_size = 1024 * 2,
        .tx_buffer_size = 1024 * 2,
        .rx_gpio_num = 16,
        .tx_gpio_num = 17
    };

    if(mtcp_if_init(&iface, &cfg) != ESP_OK)
    {
        printf("failed to initialize MTCP\n");
        return;
    }

      char buffer[32];
    while (1) 
    {
        float vin = vrt_read(&vrt);

        memset(buffer, 0, sizeof(buffer));

        int length = snprintf(buffer, sizeof(buffer), "Voltage: %.2f\r\n", vin);
        if(iface.is_active)
            uart_write_bytes(iface.uart_port, buffer, length);
        


        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}
