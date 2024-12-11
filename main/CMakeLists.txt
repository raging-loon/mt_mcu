
set(IO_SOURCES
    "io/voltage_reader.c"
)

idf_component_register(
    SRCS 
        "main.c" 
        ${IO_SOURCES}

    INCLUDE_DIRS "."
    
    )
