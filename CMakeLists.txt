# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(nabto_esp32_modbus_gateway)

set(COMPONENT_ADD_INCLUDEDIRS "main/include")
set(COMPONENT_SRCDIRS "main")
set(MAIN_SRCS ${PROJECT_ROOT}/main/gateway_main.c)

set(COMPONENT_REQUIRES lwip nghttp freemodbus_master_slave)

register_component()
