cmake_minimum_required(VERSION 3.16)

# Set the project name
project(wb-idf-core)

# Set IDF version (if necessary)
set(IDF_VERSION "5.0" CACHE STRING "")

# Register components
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Add additional components if necessary
set(EXTRA_COMPONENT_DIRS "wb-idf-i2c")