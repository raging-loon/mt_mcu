#include <mtcp.h>
#include <driver/uart.h>
#include "utils.h"
#include <string.h>
#include "mtcp_interface.h"
#include <esp_log.h>

static const char* TAG = "MTCP_TASK";

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