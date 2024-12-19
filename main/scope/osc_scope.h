#ifndef OSC_SCOPE_H_
#define OSC_SCOPE_H_

#include <esp_adc/adc_continuous.h>

esp_err_t osc_scope_init_adc(adc_continuous_handle_t* handle_out, adc_continuous_callback_t cb);

esp_err_t osc_scope_start(adc_continuous_handle_t* );

#endif // OSC_SCOPE_H_