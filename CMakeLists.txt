cmake_minimum_required(VERSION 3.10)

project(jpeg_decoder)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -I/usr/local/include")

set(SOURCE_FILES main.cpp jpeg_decoder.cpp)
add_executable(jpeg_decoder ${SOURCE_FILES})
