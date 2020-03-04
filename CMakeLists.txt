cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)

option(BUILD_TESTS "Build tests" ON)

file (GLOB 6502src
    "src/*.cpp"
    "src/*.c"
)
add_library(6502 ${6502src})
target_include_directories(6502 PRIVATE include/)

set (6502_LIBRARIES 6502)
set (6502_DIRECTORIES include/)

if (BUILD_TESTS)
    file (GLOB testSrc
        "src/tests/*.cpp"
        "src/tests/*.c"    
    )
    add_executable(tests ${testSrc})
    target_include_directories(tests PRIVATE ${6502_DIRECTORIES})
    target_link_libraries(tests ${6502_LIBRARIES})
endif ()