# CAN_ESPIDF Display Node - Top-level CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

# Include shared components from the shared directory
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/../shared"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(can_espidf_display)
