#include "osc_scope.h"
#include <esp_log.h>
#include <esp_check.h>
#include <soc/soc_caps.h>
static const char* TAG = "OSC_SCOPE";


esp_err_t osc_scope_init_adc(adc_continuous_handle_t* handle_out, adc_continuous_callback_t cb)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = 256
    };

    ESP_RETURN_ON_ERROR(
        adc_continuous_new_handle(&adc_config, &handle),
        TAG, "Failed to create a new ADC handle"
    );

    adc_continuous_config_t cfg = {
        .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_LOW,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
        .pattern_num = 1
    };

    adc_digi_pattern_config_t adc_pattern = {
        .atten = ADC_ATTEN_DB_0,
        .bit_width = ADC_BITWIDTH_12,
        .unit = ADC_UNIT_1,
        .channel = ADC_CHANNEL_6 & 0x07,
    };

    cfg.adc_pattern = &adc_pattern;

    ESP_RETURN_ON_ERROR(
        adc_continuous_config(handle, &cfg),
        TAG, "Failed to set config"
    );
    
    adc_continuous_evt_cbs_t cbs = {.on_conv_done = cb};

    ESP_RETURN_ON_ERROR(
        adc_continuous_register_event_callbacks(handle, &cbs, NULL),
        TAG, "Failed to set callback"
    );

    *handle_out = handle;

    return ESP_OK;

    
}

esp_err_t osc_scope_start(adc_continuous_handle_t* handle)
{
    if(!handle)
        return ESP_ERR_INVALID_ARG;

    ESP_RETURN_ON_ERROR(
        adc_continuous_start(handle), 
        TAG, "Failed to start ADC"
    );

    return ESP_OK;
}