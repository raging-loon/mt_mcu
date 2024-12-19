#include "mtcp_interface.h"
#include <mtcp.h>
#include <esp_check.h>
#include <esp_log.h>
#include <freertos/task.h>
#include <driver/uart.h>
#include <string.h>
#include "utils.h"
static const char* TAG = "MTCP_TASK";

void mtcp_if_handler_service(volatile mtcp_interface_t* mif)
{
    ESP_LOGI(TAG, "Started MTCP Handler Service");
    TaskHandle_t h_handshake_task = NULL;

    while(1) 
    {
        if(xSemaphoreTake(mif->smphr, portMAX_DELAY) == pdPASS)
        {
            // ESP_LOGI(TAG, "------------------------------------------");
            // mtcp_if_print_flags(mif->flags);
            
            // If there is a NEW connection and we are not already fully connected
            // then figure out if we should start a conversation
            if(MT_HAS_FLAG(mif->flags, (MTCP_IF_FLAG_NEW_CONNECTION)) && !MT_HAS_FLAG(mif->flags, MTCP_IF_FLAG_CONNECTED) )
            {
                MT_CLEAR_FLAG(mif->flags, (MTCP_IF_FLAG_NEW_CONNECTION | MTCP_IF_FLAG_DISCONNECTED | MTCP_IF_FLAG_CONNECTION_LOST));
                ESP_LOGI(TAG, "Got a connection :O");

                // All values greater than eReady point to an invalid or ended task
                // If that is so, run the task
                if(!h_handshake_task || eTaskGetState(h_handshake_task) > eReady)
                    xTaskCreate((TaskHandle_t)mtcp_task_handshake, "HANDSHAKE", 4096, (void*)mif, 1, &h_handshake_task);
                else
                    ESP_LOGI(TAG, "Handshake task already runnning, skipping...");
            }
            else if(MT_HAS_FLAG(mif->flags, MTCP_IF_FLAG_CONNECTION_LOST) & !MT_HAS_FLAG(mif->flags, MTCP_IF_FLAG_DISCONNECTED))
            {
                mif->flags |= MTCP_IF_FLAG_DISCONNECTED;
                 MT_CLEAR_FLAG(mif->flags, (MTCP_IF_FLAG_NEW_CONNECTION | MTCP_IF_FLAG_CONNECTED));
                ESP_LOGI(TAG, "Lost a connection : (");

            }
            // mtcp_if_print_flags(mif->flags);
            // ESP_LOGI(TAG, "------------------------------------------");

        }

    }
}

static void mtcp_task_tx(mtcp_interface_t* mif)
{
    
}

static bool mtcp_task_handle_greeting(mtcp_interface_t* mif, mtcp_header_t* frame)
{
    if(frame->type != MTCP_FT_GREETING)
        return false;
    
    uart_write_bytes(mif->uart_port, frame, sizeof(mtcp_header_t));

    uart_wait_tx_done(mif->uart_port, pdMS_TO_TICKS(100));

    return true;

}

void mtcp_task_handshake(volatile mtcp_interface_t* mif)
{
    if(!uart_is_driver_installed(mif->uart_port))
        mtcp_if_init_uart(mif);
    
    // wait for user to finish plugging in device
    vTaskDelay(pdMS_TO_TICKS(1000));

    mtcp_header_t frame;    
    memset(&frame, 0, sizeof(frame));

    ESP_LOGI(TAG, "Waiting for client greeting...");

    while(true)
    {
        int readlen = uart_read_bytes(mif->uart_port, &frame, sizeof(frame), pdMS_TO_TICKS(1000));
        
        if(readlen > 0)
        {
            if(!mtcp_task_handle_greeting(mif, &frame))
                ESP_LOGI(TAG, "Connection failed. Data is either invalid or non-existant");
            else
            {
                ESP_LOGI(TAG, "Received greeting back");
                mif->flags |= MTCP_IF_FLAG_CONNECTED;
                break;
            }
        }

    }

    ESP_LOGI(TAG, "Starting MTCP RX Task...");

    // NULL -> delete us
    vTaskDelete(NULL);
}