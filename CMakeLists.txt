# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32_IoT)

# Add SPIFFS image build
set(SPIFFS_IMAGE_SOURCE_DIR "${CMAKE_SOURCE_DIR}/data")
if(NOT IS_DIRECTORY "${SPIFFS_IMAGE_SOURCE_DIR}")
    message(FATAL_ERROR "SPIFFS source directory not found: ${SPIFFS_IMAGE_SOURCE_DIR}")
endif()

spiffs_create_partition_image(storage "${SPIFFS_IMAGE_SOURCE_DIR}" FLASH_IN_PROJECT)
