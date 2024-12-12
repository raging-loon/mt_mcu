
set(IO_SOURCES
    "io/voltage_reader.c"
)

set(MTCP_SOURCES
    "mtcp/mtcp_interface.c"
    "mtcp/mtcp_task.c"
)

idf_component_register(
    SRCS 
        "main.c" 
        ${IO_SOURCES}
        ${MTCP_SOURCES}

    INCLUDE_DIRS "." "../../mtcp/"
    
    )
