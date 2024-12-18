#include "mtcp_interface.h"
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_check.h>
#include <esp_intr_alloc.h>
#include <string.h>
#include <utils.h>
#include <mtcp.h>

static const char* TAG = "MTCP_IF";

///
/// @brief 
///     Install a UART driver for mif->uart_port
///
/// @param[in] mif      MTCP Interface 
/// @param[in] ifcfg    MTCP Interface Configuration
///
/// @return ESP_OK or error code
///
static esp_err_t mtcp_if_init_uart(mtcp_interface_t* mif, const mtcp_if_cfg_t* ifcfg)
{
    uart_config_t cfg = {
        .baud_rate              = 115200,
        .data_bits              = UART_DATA_8_BITS,
        .parity                 = UART_PARITY_DISABLE,
        .stop_bits              = UART_STOP_BITS_1,
        .flow_ctrl              = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh    = 122
    };

    esp_err_t stat = uart_param_config(ifcfg->uart_port, &cfg);

    if(stat != ESP_OK)
        return stat;

    stat = uart_set_pin(ifcfg->uart_port, ifcfg->tx_gpio_num, ifcfg->rx_gpio_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    if(stat != ESP_OK)
        return stat;
    
    stat = uart_driver_install(
        ifcfg->uart_port, 
        ifcfg->rx_buffer_size, 
        ifcfg->tx_buffer_size, 
        ifcfg->queue_size, 
        &mif->uart_queue, 
        0
    );

    if(stat != ESP_OK)
        return stat;

    mif->queue_size             = ifcfg->queue_size;
    mif->tx_buffer_size         = ifcfg->tx_buffer_size;
    mif->rx_buffer_size         = ifcfg->rx_buffer_size;
    mif->uart_port              = ifcfg->uart_port;
    return ESP_OK;
}

static esp_err_t mtcp_if_pull_down_uart_pins(const mtcp_if_cfg_t* cfg)
{
    gpio_config_t gpcfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << cfg->rx_gpio_num) | (1ULL << cfg->tx_gpio_num),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };

    ESP_RETURN_ON_ERROR(gpio_config(&gpcfg), TAG, "Failed to configure RX/TX GPIO pin");
    return ESP_OK;
}


///
/// @brief
///     This is called when there is a major voltage change
///     on whatever GPIO pin the CTS pin on the USB module is connected to
/// 
/// @param[in] arg  A pointer to an mtcp_interface_t object
///
static void mtcp_if_usb_plug_isr_handler(void* arg)
{
    int cts_state = gpio_get_level(MTCP_GPIO_CTS_MON_PIN);
    mtcp_interface_t* mif = (mtcp_interface_t*)(arg);
    BaseType_t  xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;
    if(cts_state == 1 && !MT_HAS_FLAG(mif->flags, MTCP_IF_FLAG_CONNECTED))
    {

        mif->flags |= (MTCP_IF_FLAG_PLUGGED_IN | MTCP_IF_FLAG_NEW_CONNECTION);      
        xSemaphoreGiveFromISR(mif->smphr, &xHigherPriorityTaskWoken);
    }
    else
    {
        if(mif->flags != 0)
        {
            mif->flags |= MTCP_IF_FLAG_CONNECTION_LOST | MTCP_IF_FLAG_UNPLUGGED;        
            // clear pluggeed in flag
            mif->flags &= ~(MTCP_IF_FLAG_PLUGGED_IN | MTCP_IF_FLAG_NEW_CONNECTION);
            xSemaphoreGiveFromISR(mif->smphr, &xHigherPriorityTaskWoken);

        }
    }



    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

esp_err_t mtcp_if_install_isr_watcher(mtcp_interface_t* mif)
{
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LOWMED));
    gpio_config_t cfg = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << MTCP_GPIO_CTS_MON_PIN),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE
    };

    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "ISR Installation: GPIO CFG Failed");
    gpio_set_level(MTCP_GPIO_CTS_MON_PIN, 0);
    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_add(MTCP_GPIO_CTS_MON_PIN, mtcp_if_usb_plug_isr_handler, (void*)mif),
        TAG, "Failed to install ISR Handler"
    );
    ESP_LOGI(TAG, "Installed MTCP ISR Interrupt");
    gpio_intr_enable(MTCP_GPIO_CTS_MON_PIN);
    
    // // Call the ISR now, the PC may be connected at boot
    // mtcp_if_usb_plug_isr_handler((void*)mif);

    return ESP_OK;
}

esp_err_t mtcp_if_init(mtcp_interface_t* mif, const mtcp_if_cfg_t* ifcfg)
{
    if(!mif || !ifcfg)
        return ESP_ERR_INVALID_ARG;
    
    memset(mif, 0, sizeof(mtcp_interface_t));    

    mif->smphr = xSemaphoreCreateBinary();
    
    ESP_RETURN_ON_FALSE(
        mif->smphr != NULL, ESP_ERR_INVALID_RESPONSE, TAG, "Failed to create Semaphore!"
    );


    xTaskCreate((TaskFunction_t)(mtcp_if_handler_service), "MTCP Handler", 4096, mif, 3, NULL);

    ESP_RETURN_ON_ERROR(
        mtcp_if_install_isr_watcher(mif), TAG, "Failed to initialiaze ISR"
    );

    if(mtcp_if_pull_down_uart_pins(ifcfg) != ESP_OK)
        return ESP_ERR_INVALID_RESPONSE;

    // ESP_RETURN_ON_ERROR(
    //     mtcp_if_init_uart(mif, ifcfg), TAG, "Failed to initialiaze UART"
    // );



    return ESP_OK;
}


esp_err_t mtcp_if_destroy(mtcp_interface_t* m)
{
    if(!m)
        return ESP_ERR_INVALID_ARG;

    ESP_RETURN_ON_ERROR(uart_driver_delete(m->uart_port), TAG, "Failed to uninstall UART driver");

    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_remove(MTCP_GPIO_CTS_MON_PIN),
        TAG, "Failed to remove ISR Watcher for GPIO %d", MTCP_GPIO_CTS_MON_PIN);

    return ESP_OK;
}

static void mtcp_if_print_flags(uint8_t flags)
{
    char buffer[1024];
    memset(buffer, 0, sizeof(buffer));
    if(MT_HAS_FLAG(flags, MTCP_IF_FLAG_CONNECTED))
        strncat(buffer, "CONNECTED | ", 13);
    if(MT_HAS_FLAG(flags, MTCP_IF_FLAG_CONNECTION_LOST))
        strncat(buffer, "CONNECTION_LOST | ", 19);
    if(MT_HAS_FLAG(flags, MTCP_IF_FLAG_NEW_CONNECTION))
        strncat(buffer, "NEW_CONNECTION | ", 18);
    if(MT_HAS_FLAG(flags, MTCP_IF_FLAG_DISCONNECTED))
        strncat(buffer, "DISCONNECTED | ", 16);
    if(MT_HAS_FLAG(flags, MTCP_IF_FLAG_PLUGGED_IN))
        strncat(buffer, "PLUGGED IN | ", 14);
    if(MT_HAS_FLAG(flags, MTCP_IF_FLAG_UNPLUGGED))
        strncat(buffer, "UNPLUGGED | ", 13);
    
    ESP_LOGI(TAG, "Flags: %s", buffer);
}



void mtcp_if_handler_service(mtcp_interface_t* mif)
{
    ESP_LOGI(TAG, "Started MTCP Handler Service");
    TaskHandle_t h_handshake_task = NULL;

    while(1) 
    {
        if(xSemaphoreTake(mif->smphr, portMAX_DELAY) == pdPASS)
        {
            ESP_LOGI(TAG, "------------------------------------------");
            mtcp_if_print_flags(mif->flags);
            if(MT_HAS_FLAG(mif->flags, (MTCP_IF_FLAG_NEW_CONNECTION)))
            {
                MT_CLEAR_FLAG(mif->flags, (MTCP_IF_FLAG_NEW_CONNECTION | MTCP_IF_FLAG_DISCONNECTED | MTCP_IF_FLAG_CONNECTION_LOST));
                mif->flags |= MTCP_IF_FLAG_CONNECTED;
                ESP_LOGI(TAG, "Got a connection :O");
                xTaskCreate(mtcp_task_handshake, "HANDSHAKE", 1000, (void*)mif, 1, &h_handshake_task);

            }
            else if(MT_HAS_FLAG(mif->flags, MTCP_IF_FLAG_CONNECTION_LOST) & !MT_HAS_FLAG(mif->flags, MTCP_IF_FLAG_DISCONNECTED))
            {
                mif->flags |= MTCP_IF_FLAG_DISCONNECTED;
                 MT_CLEAR_FLAG(mif->flags, (MTCP_IF_FLAG_NEW_CONNECTION | MTCP_IF_FLAG_CONNECTED));
                ESP_LOGI(TAG, "Lost a connection : (");

            }
            mtcp_if_print_flags(mif->flags);
            ESP_LOGI(TAG, "------------------------------------------");

        }

    }
}


void mtcp_task_handshake(mtcp_interface_t* mif)
{
    mtcp_header_t header = {
        .type = MTCP_FT_GREETING,
        .data_size = 0
    };
    
    vTaskDelay(pdMS_TO_TICKS(1000));

    uart_write_bytes(mif->uart_port, &header, sizeof(header));
    uart_wait_tx_done(mif->uart_port, pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "Send Greeting");

    memset(&header, 0, sizeof(header));

    int read = uart_read_bytes(mif->uart_port, &header, sizeof(header), pdMS_TO_TICKS(10000));

    if(read == -1)
    {
        ESP_LOGI(TAG, "Did not receive greeting in time");
        MT_CLEAR_FLAG(mif->flags, MTCP_IF_FLAG_CONNECTED | MTCP_IF_FLAG_NEW_CONNECTION);
        goto end_task;
    }

    if(header.type == MTCP_FT_GREETING)
    {
        ESP_LOGI(TAG, "Got greeting back");
        mif->flags |= MTCP_IF_FLAG_CONNECTED;
        MT_CLEAR_FLAG(mif->flags, MTCP_IF_FLAG_NEW_CONNECTION);
    }

end_task:

    // NULL -> delete us
    vTaskDelete(NULL);

}