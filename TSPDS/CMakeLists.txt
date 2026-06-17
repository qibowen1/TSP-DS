cmake_minimum_required(VERSION 3.16)
project(TSPDS LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(PROJ_INC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/LKH_LIB
    ${CMAKE_CURRENT_SOURCE_DIR}/LKH_LIB/INCLUDE
)

find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

set(TSPDS_SOURCES
    DroneScheduler.cpp
    InitialSolutionGenerator.cpp
    LocalSearchOperators.cpp
    PerturbationOperators.cpp
    TSPDSSolver.cpp
    TSPDSUtils.cpp
    VNSLogger.cpp
)

file(GLOB LKH_C_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/LKH_LIB/*.c
)

# KSwapKick.c and KSwapKick_HUNT.c cannot be compiled together.
list(FILTER LKH_C_SOURCES EXCLUDE REGEX ".*/KSwapKick_HUNT\\.c$")

set(LKH_CPP_SOURCES
    LKH_LIB/LKH_LIB.cpp
    LKH_LIB/pch.cpp
)

add_executable(TSPDS.out
    TSPDS-main.cpp
    ${TSPDS_SOURCES}
    ${LKH_C_SOURCES}
    ${LKH_CPP_SOURCES}
)

target_include_directories(TSPDS.out PRIVATE
    ${PROJ_INC}
)

target_compile_definitions(TSPDS.out PRIVATE
    NDEBUG
)

target_compile_options(TSPDS.out PRIVATE
    -O3
)

target_link_libraries(TSPDS.out PRIVATE
    OpenMP::OpenMP_CXX
    Threads::Threads
    dl
    m
)

set_target_properties(TSPDS.out PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

message(STATUS "Build target: TSPDS.out")
message(STATUS "LKH source dir=${CMAKE_CURRENT_SOURCE_DIR}/LKH_LIB")
