cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)

option(BUILD_TESTS "Build test" ON)

file (GLOB 6502src
    "src/*.cpp"
    "src/*.c"
)
add_library(6502 ${6502src})
target_include_directories(6502 PRIVATE include/)

set (6502_LIBRARIES 6502)
set (6502_DIRECTORIES include/)