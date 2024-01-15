idf_component_register(SRCS "main.c" 
                       INCLUDE_DIRS "." "${CMAKE_SOURCE_DIR}/../include"
                       REQUIRES client camera lidar spi wifi serial)