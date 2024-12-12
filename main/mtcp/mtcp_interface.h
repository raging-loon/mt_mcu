#ifndef MTCP_MTCP_INTERFACE_H_
#define MTCP_MTCP_INTERFACE_H_

#include <mtcp.h>
#include <stdint.h>
#include <driver/uart.h>

#define MTCP_GPIO_CTS_MON_PIN           22

typedef struct {
    uart_port_t uart_port;
    int         tx_gpio_num;
    int         rx_gpio_num;
    int         tx_buffer_size;
    int         queue_size;
    int         rx_buffer_size;
} mtcp_if_cfg_t;

#define MTCP_IF_FLAG_NEW_CONNECTION     (1 << 1) /// Set as soon as a new device is plugged in
#define MTCP_IF_FLAG_CONNECTION_LOST    (1 << 2) /// Set as soon as a device is unplugged       
#define MTCP_IF_FLAG_CONNECTED          (1 << 3) 
#define MTCP_IF_FLAG_DISCONNECTED       (1 << 4)

typedef struct mtcp_interface
{
    uart_port_t         uart_port;
    QueueHandle_t       uart_queue;
    int                 tx_buffer_size;
    int                 rx_buffer_size;
    int                 queue_size;
    int                 is_active;
    uint8_t             flags;
} mtcp_interface_t;

///
/// @brief
///     Create a new MTCP Interface
/// @details
///     This will create a new UART driver and ISR watcher for
///     detecting when something gets plugged/unplugged
///
/// @param[in,out] mif      Interface object
/// @param[in]     ifcfg    Interface configuration
///
/// @returns ESP_OK or error code
///
esp_err_t mtcp_if_init(mtcp_interface_t* mif, const mtcp_if_cfg_t* ifcfg);

///
/// @brief 
///     Destroy an MTCP Interface
/// @details
///     Uninstalls UART driver and ISR watcher
///
/// @param[in] mif  Interface object to destroy 
///
/// @return ESP_OK, ESP_ERROR_INVALID_ARG if mif == NULL, or other error code
///
esp_err_t mtcp_if_destroy(mtcp_interface_t* mif);

void mtcp_if_handle_connection(mtcp_interface_t* mif);
#endif // MTCP_MTCP_INTERFACE_H_