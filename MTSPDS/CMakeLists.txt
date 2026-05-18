cmake_minimum_required(VERSION 3.16)
project(MTSPDS LANGUAGES CXX)

# -------------------------
# C++ standard
# -------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -------------------------
# Build type
# -------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# -------------------------
# Required CPLEX path
# CPLEX 是必需依赖，不提供关闭选项
# 使用方式：
#   cmake -S . -B build -DCPLEX_ROOT=/path/to/CPLEX_Studio201
# 或者：
#   export CPLEX_ROOT=/path/to/CPLEX_Studio201
#   cmake -S . -B build
# -------------------------
set(CPLEX_ROOT "" CACHE PATH "Root directory of IBM ILOG CPLEX Studio")

if(NOT CPLEX_ROOT AND DEFINED ENV{CPLEX_ROOT})
    set(CPLEX_ROOT $ENV{CPLEX_ROOT})
endif()

if(NOT CPLEX_ROOT)
    message(FATAL_ERROR
        "CPLEX is required, but CPLEX_ROOT is not set.\n"
        "Please configure with:\n"
        "  cmake -S . -B build -DCPLEX_ROOT=/path/to/CPLEX_Studio201\n"
        "or set environment variable:\n"
        "  export CPLEX_ROOT=/path/to/CPLEX_Studio201"
    )
endif()

if(NOT EXISTS "${CPLEX_ROOT}/cplex/include/ilcplex/ilocplex.h")
    message(FATAL_ERROR
        "Cannot find CPLEX header:\n"
        "  ${CPLEX_ROOT}/cplex/include/ilcplex/ilocplex.h\n"
        "Please check CPLEX_ROOT."
    )
endif()

if(NOT EXISTS "${CPLEX_ROOT}/concert/include/ilconcert/iloenv.h")
    message(FATAL_ERROR
        "Cannot find Concert header:\n"
        "  ${CPLEX_ROOT}/concert/include/ilconcert/iloenv.h\n"
        "Please check CPLEX_ROOT."
    )
endif()

# -------------------------
# CPLEX library directory
# 优先使用 static_pic，若不存在则尝试 static
# -------------------------
set(CPLEX_LIB_DIR "${CPLEX_ROOT}/cplex/lib/x86-64_linux/static_pic")
set(CONCERT_LIB_DIR "${CPLEX_ROOT}/concert/lib/x86-64_linux/static_pic")

if(NOT EXISTS "${CPLEX_LIB_DIR}")
    set(CPLEX_LIB_DIR "${CPLEX_ROOT}/cplex/lib/x86-64_linux/static")
endif()

if(NOT EXISTS "${CONCERT_LIB_DIR}")
    set(CONCERT_LIB_DIR "${CPLEX_ROOT}/concert/lib/x86-64_linux/static")
endif()

if(NOT EXISTS "${CPLEX_LIB_DIR}")
    message(FATAL_ERROR
        "Cannot find CPLEX library directory under:\n"
        "  ${CPLEX_ROOT}/cplex/lib/x86-64_linux/static_pic\n"
        "or\n"
        "  ${CPLEX_ROOT}/cplex/lib/x86-64_linux/static"
    )
endif()

if(NOT EXISTS "${CONCERT_LIB_DIR}")
    message(FATAL_ERROR
        "Cannot find Concert library directory under:\n"
        "  ${CPLEX_ROOT}/concert/lib/x86-64_linux/static_pic\n"
        "or\n"
        "  ${CPLEX_ROOT}/concert/lib/x86-64_linux/static"
    )
endif()

# -------------------------
# Project include directories
# -------------------------
set(PROJ_INC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# -------------------------
# Packages
# -------------------------
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

# -------------------------
# MTSPDS source files
# 根据 Visual Studio 成功编译日志整理
# -------------------------
set(MTSPDS_SOURCES
    DroneSchedulerCombine.cpp
    main.cpp
    main_two_phase.cpp
    SetPartitioningPolisher.cpp
    TSPHeuristics.cpp
    TwoPhaseMatheuristic.cpp
)

# -------------------------
# Executable
# -------------------------
add_executable(MTSPDS.out
    ${MTSPDS_SOURCES}
)

# -------------------------
# Include directories
# -------------------------
target_include_directories(MTSPDS.out PRIVATE
    ${PROJ_INC}
    ${CPLEX_ROOT}/cplex/include
    ${CPLEX_ROOT}/concert/include
)

# -------------------------
# Compile definitions
# -------------------------
target_compile_definitions(MTSPDS.out PRIVATE
    NDEBUG
    IL_STD
    ENABLE_CPLEX=1
)

# -------------------------
# Compile options
# Release 版本使用优化
# -------------------------
target_compile_options(MTSPDS.out PRIVATE
    -O3
)

# -------------------------
# Library directories
# -------------------------
target_link_directories(MTSPDS.out PRIVATE
    ${CPLEX_LIB_DIR}
    ${CONCERT_LIB_DIR}
)

# -------------------------
# Link libraries
# -------------------------
target_link_libraries(MTSPDS.out PRIVATE
    OpenMP::OpenMP_CXX
    Threads::Threads
    ilocplex
    concert
    cplex
    dl
    m
)

# -------------------------
# Output directory
# -------------------------
set_target_properties(MTSPDS.out PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

message(STATUS "Build target: MTSPDS.out")
message(STATUS "CPLEX_ROOT=${CPLEX_ROOT}")
message(STATUS "CPLEX_LIB_DIR=${CPLEX_LIB_DIR}")
message(STATUS "CONCERT_LIB_DIR=${CONCERT_LIB_DIR}")