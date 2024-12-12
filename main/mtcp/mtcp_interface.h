#ifndef MTCP_MTCP_INTERFACE_H_
#define MTCP_MTCP_INTERFACE_H_

#include <mtcp.h>

#include <driver/uart.h>

#define MTCP_GPIO_CTS_MON_PIN           22

typedef struct {
    int         tx_gpio_num;
    int         rx_gpio_num;
    int         tx_buffer_size;
    int         queue_size;
    int         rx_buffer_size;
} mtcp_if_cfg_t;

typedef struct mtcp_interface
{
    uart_port_t         uart_port;
    QueueHandle_t       uart_queue;
    int                 tx_buffer_size;
    int                 rx_buffer_size;
    int                 queue_size;
    int                 is_active;
} mtcp_interface_t;

esp_err_t mtcp_if_init(mtcp_interface_t* mif, const mtcp_if_cfg_t* ifcfg);

esp_err_t mtcp_if_destroy(mtcp_interface_t* mif);

#endif // MTCP_MTCP_INTERFACE_H_