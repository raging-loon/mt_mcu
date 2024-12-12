#include "mtcp_interface.h"
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_check.h>
#include <esp_intr_alloc.h>
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

    if(cts_state == 0)
        mif->flags |= (MTCP_IF_FLAG_CONNECTED | MTCP_IF_FLAG_NEW_CONNECTION);        
    else
        mif->flags |= (MTCP_IF_FLAG_DISCONNECTED | MTCP_IF_FLAG_CONNECTION_LOST);        

}

///
/// @brief 
///     Install an interrupt handler that will active the MTCP interface
/// @details
///     When the MCU gets connected to a PC, the USB Module will send data via the RX
///     pin. When we receive that, 
/// @param mif 
/// @param ifcfg 
///
/// @return 
///
static esp_err_t mtcp_if_install_isr_watcher(mtcp_interface_t* mif, const mtcp_if_cfg_t* ifcfg)
{
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LOWMED));
    gpio_config_t cfg = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << MTCP_GPIO_CTS_MON_PIN),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "ISR Installation: GPIO CFG Failed");
    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_add(MTCP_GPIO_CTS_MON_PIN, mtcp_if_usb_plug_isr_handler, (void*)mif),
        TAG, "Failed to install ISR Handler"
    );
    ESP_LOGI(TAG, "Installed MTCP ISR Interrupt");
    gpio_intr_enable(MTCP_GPIO_CTS_MON_PIN);
    return ESP_OK;
}

esp_err_t mtcp_if_init(mtcp_interface_t* mif, const mtcp_if_cfg_t* ifcfg)
{
    if(!mif || !ifcfg)
        return ESP_ERR_INVALID_ARG;
    
    ESP_RETURN_ON_ERROR(
        mtcp_if_init_uart(mif, ifcfg), TAG, "Failed to initialiaze UART"
    );

    ESP_RETURN_ON_ERROR(
        mtcp_if_install_isr_watcher(mif, ifcfg), TAG, "Failed to initialiaze ISR"
    );

    return ESP_OK;
}


esp_err_t mtcp_destroy_interface(mtcp_interface_t* m)
{
    if(!m)
        return ESP_ERR_INVALID_ARG;

    ESP_RETURN_ON_ERROR(uart_driver_delete(m->uart_port), TAG, "Failed to uninstall UART driver");

    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_remove(MTCP_GPIO_CTS_MON_PIN),
        TAG, "Failed to remove ISR Watcher for GPIO %d", MTCP_GPIO_CTS_MON_PIN);

    return ESP_OK;
}
