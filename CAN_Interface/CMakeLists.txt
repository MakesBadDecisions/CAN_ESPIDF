cmake_minimum_required(VERSION 3.16.0)

# Include local and shared components
set(EXTRA_COMPONENT_DIRS 
    "${CMAKE_CURRENT_SOURCE_DIR}/components"
    "${CMAKE_CURRENT_SOURCE_DIR}/../shared"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(CAN_Interface)
