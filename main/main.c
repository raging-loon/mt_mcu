#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include "io/voltage_reader.h"
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

      char buffer[32];

    while (1) 
    {
        float vin = vrt_read(&vrt);

        memset(buffer, 0, sizeof(buffer));

        int length = snprintf(buffer, sizeof(buffer), "Voltage: %.2f", vin);
        
        printf("%s\n",buffer);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}
