# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

add_compile_definitions(QUIRC_FLOAT_TYPE=float)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(EventHub_QR_Checkin)
