#include "voltage_reader.h"
#include <freertos/freertos.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
static float get_vlt_range_from_atten(int atten)
{
    switch(atten) 
    {
        case ADC_ATTEN_DB_0:
            return 1.1;
        case ADC_ATTEN_DB_2_5:
            return 1.5;
        case ADC_ATTEN_DB_6:
            return 2.2;
        case ADC_ATTEN_DB_12:
            return 3.3;
        default:
            return 0;
    }
}

int vrt_init(voltage_reader_t* reader)
{
    if(!reader)
        return -1;

    if(adc1_config_width(reader->width) != ESP_OK)
        return -1;

    if(adc1_config_channel_atten(reader->channel, reader->atten_level) != ESP_OK)
        return -1;

    reader->v_ref = get_vlt_range_from_atten(reader->atten_level);

    return 0;
}

int vrt_read_raw(const voltage_reader_t* reader)
{
    if(!reader)
        return -1;

    return adc1_get_raw(reader->channel);
}

float vrt_read(const voltage_reader_t* reader)
{
    if(!reader || reader->adc_res <= 0)
        return -1;

    int raw = adc1_get_raw(reader->channel);

    if(raw == -1)
        return -1;

    float vin_raw = ((float)raw * reader->v_ref) / (float)reader->adc_res;

    if(reader->div_r1 <= 0.0f || reader->div_r2 <= 0.0f)
        return vin_raw;

    return vin_raw * (reader->div_r1 + reader->div_r2) / reader->div_r2;
}

void voltage_reader_task(void* param)
{
    vrt_task_t* us = (vrt_task_t*)param;

    float vin = 0.0f;
    int iterations = 0;
    while (1)
    {
        
        vin = vrt_read(us->reader);
        vTaskDelay(50 / portTICK_PERIOD_MS);       
    }

}