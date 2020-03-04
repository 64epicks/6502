cmake_minimum_required(VERSION 2.8)
project(6502)

set (CMAKE_CXX_STANDARD 11)

option(BUILD_TESTS "Build tests" ON)

file (GLOB 6502src
    "6502.cpp"
)
add_library(6502 ${6502src})
target_include_directories(6502 PRIVATE .)

set (6502_LIBRARIES 6502)
set (6502_DIRECTORIES .)

if (BUILD_TESTS)
    file (GLOB testSrc
        "tests/*.cpp"
        "tests/*.c"    
    )
    add_executable(tests ${testSrc})
    target_include_directories(tests PRIVATE ${6502_DIRECTORIES})
    target_link_libraries(tests ${6502_LIBRARIES})
endif ()