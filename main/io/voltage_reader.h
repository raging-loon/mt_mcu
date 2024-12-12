#ifndef IO_VOLTAGE_READER_H_
#define IO_VOLTAGE_READER_H_

#include <stdint.h>

#include <driver/adc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
///
/// @brief
///     Reader for a ADC compat. GPIO pin
///
///     The information present controls which
///     pin we are getting information from, as well as
///     values for calculating the voltage:
///     
///     VinR = (ADC_Input / reader.adc_res) * reader.v_ref
///     Vin  = VinR * (reader.div_r1 + reader.div_r2) * reader.div_r2
///
typedef struct voltage_reader
{
    /// ADC resolution    
    adc_bits_width_t    width;

    /// Controls which GPIO pin to read from
    adc_channel_t       channel;

    /// Attentuation Level
    int                 atten_level;

    /// reference voltage
    float               v_ref;

    /// ADC resolution
    int                 adc_res;
#define VR_RESISTOR_VALUE_NONE      0.0f
    /// Value of resistor 1 in the divider (ohms)
    float               div_r1;
    /// Value of resistor 2 in the divider (ohms)
    float               div_r2;
} voltage_reader_t;


int vrt_init(voltage_reader_t* reader);

int vrt_read_raw(const voltage_reader_t* reader);

float vrt_read(const voltage_reader_t* reader);

/// Voltage Reader task
typedef struct 
{
    voltage_reader_t*    reader;
    QueueHandle_t       output_queue;  
} vrt_task_t;


void voltage_reader_task(void* param);


#endif // IO_VOLTAGE_READER_H_