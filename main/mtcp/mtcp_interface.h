#ifndef MTCP_MTCP_INTERFACE_H_
#define MTCP_MTCP_INTERFACE_H_

#include <mtcp.h>
#include <stdint.h>
#include <driver/uart.h>
#include <freertos/task.h>

#define MTCP_GPIO_CTS_MON_PIN           22


typedef struct {

} mtcp_conversation_t;



typedef struct {
    /// UART Port. e.g. UART_NUM_2
    uart_port_t uart_port;
    /// Transmission GPIO Pin
    int         tx_gpio_num;
    /// Receipt GPIO Pin
    int         rx_gpio_num;
    int         tx_buffer_size;
    int         rx_buffer_size;
    /// size of uart queue, used by QueueHandle_t
    int         queue_size;
} mtcp_if_cfg_t;

#define MTCP_IF_FLAG_NEW_CONNECTION     (1 << 1) /// Set as soon as a new device is plugged in
#define MTCP_IF_FLAG_CONNECTION_LOST    (1 << 2) /// Set as soon as a device is unplugged       
#define MTCP_IF_FLAG_CONNECTED          (1 << 3) /// Set when we have an active connection to the host application 
#define MTCP_IF_FLAG_DISCONNECTED       (1 << 4) /// We no longer have a connection, e.g. the user closed the desktop app
#define MTCP_IF_FLAG_PLUGGED_IN         (1 << 5) /// Set if a device is plugged in, but not necessarily sending data
#define MTCP_IF_FLAG_UNPLUGGED          (1 << 5) /// Set if there is no device plugged in

///
/// @brief 
///     Represents an MTCP Interface. 
///     There will probably only ever be one
///
typedef struct mtcp_interface
{
    /// E.g. UART_NUM_2
    uart_port_t         uart_port;
    QueueHandle_t       uart_queue;
    int                 queue_size;
    
    /// transmission buffer size
    int                 tx_buffer_size;
    /// reciept buffer size
    int                 rx_buffer_size;
    /// Flags: MTCP_IF_FLAG_*
    uint8_t             flags;
    /// freertos task handle
    TaskHandle_t        task_handle;
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



#endif // MTCP_MTCP_INTERFACE_H_