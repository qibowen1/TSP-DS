cmake_minimum_required(VERSION 3.16)
project(MTSPDS LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(PROJ_INC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

set(MTSPDS_SOURCES
    DroneSchedulerCombine.cpp
    main.cpp
    main_two_phase.cpp
    SetPartitioningPolisher.cpp
    TSPHeuristics.cpp
    TwoPhaseMatheuristic.cpp
)

add_executable(MTSPDS.out
    ${MTSPDS_SOURCES}
)

target_include_directories(MTSPDS.out PRIVATE
    ${PROJ_INC}
)

target_compile_definitions(MTSPDS.out PRIVATE
    NDEBUG
)

if(MSVC)
    target_compile_options(MTSPDS.out PRIVATE /O2)
else()
    target_compile_options(MTSPDS.out PRIVATE -O3)
endif()

target_link_libraries(MTSPDS.out PRIVATE
    OpenMP::OpenMP_CXX
    Threads::Threads
)

set_target_properties(MTSPDS.out PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

message(STATUS "Build target: MTSPDS.out")
