idf_component_register(SRCS "host.c" 
                       INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/../include" 
                       "$ENV{LCM_INCLUDE_DIR}/mbot_lcm_serial/include/mbot_lcm_serial"
                       "$ENV{LCM_INCLUDE_DIR}/build/mbot_msgs/")