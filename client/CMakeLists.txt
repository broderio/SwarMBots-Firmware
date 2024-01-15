cmake_minimum_required(VERSION 3.16)

set(COMPONENT_DIRS "$ENV{IDF_PATH}/components" "${CMAKE_SOURCE_DIR}/../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(client_main)