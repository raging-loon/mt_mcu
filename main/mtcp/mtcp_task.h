#ifndef MTCP_MTCP_TASK_H_
#define MTCP_MTCP_TASK_H_

#include <freertos/task.h>
#include <mtcp.h>
#include "mtcp_interface.h"

typedef struct 
{
    mtcp_interface_t*   iface;
    
    TaskHandle_t        task_handle;

    

} mtcp_task_t;




#endif // MTCP_MTCP_TASK_H_