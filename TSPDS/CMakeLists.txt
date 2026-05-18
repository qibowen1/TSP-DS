cmake_minimum_required(VERSION 3.16)
project(TSPDS LANGUAGES C CXX)

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
    ${CMAKE_CURRENT_SOURCE_DIR}/LKH_LIB
    ${CMAKE_CURRENT_SOURCE_DIR}/LKH_LIB/INCLUDE
)

# -------------------------
# Packages
# -------------------------
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

# -------------------------
# TSP-DS source files
# -------------------------
set(TSPDS_SOURCES
    CplexBendersSolver.cpp
    CplexF2Solver.cpp
    DroneScheduler.cpp
    InitialSolutionGenerator.cpp
    LocalSearchOperators.cpp
    PerturbationOperators.cpp
    TSPDS-main-anly.cpp
    TSPDSSolver.cpp
    TSPDSUtils.cpp
    VNSLogger.cpp
)

# -------------------------
# LKH source files
# LKH_LIB 使用项目内相对路径，不写死机器路径
# -------------------------
file(GLOB LKH_C_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/LKH_LIB/*.c
)

# KSwapKick.c 与 KSwapKick_HUNT.c 不能同时编译
list(FILTER LKH_C_SOURCES EXCLUDE REGEX ".*/KSwapKick_HUNT\\.c$")

set(LKH_CPP_SOURCES
    LKH_LIB/LKH_LIB.cpp
    LKH_LIB/pch.cpp
)

# -------------------------
# Executable
# -------------------------
add_executable(TSPDS.out
    TSPDS-main.cpp
    ${TSPDS_SOURCES}
    ${LKH_C_SOURCES}
    ${LKH_CPP_SOURCES}
)

# -------------------------
# Include directories
# -------------------------
target_include_directories(TSPDS.out PRIVATE
    ${PROJ_INC}
    ${CPLEX_ROOT}/cplex/include
    ${CPLEX_ROOT}/concert/include
)

# -------------------------
# Compile definitions
# CPLEX 是必需项，因此固定 ENABLE_CPLEX=1
# -------------------------
target_compile_definitions(TSPDS.out PRIVATE
    NDEBUG
    IL_STD
    ENABLE_CPLEX=1
)

# -------------------------
# Compile options
# -------------------------
target_compile_options(TSPDS.out PRIVATE
    -O3
)

# -------------------------
# Library directories
# -------------------------
target_link_directories(TSPDS.out PRIVATE
    ${CPLEX_LIB_DIR}
    ${CONCERT_LIB_DIR}
)

# -------------------------
# Link libraries
# -------------------------
target_link_libraries(TSPDS.out PRIVATE
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
set_target_properties(TSPDS.out PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

message(STATUS "Build target: TSPDS.out")
message(STATUS "CPLEX_ROOT=${CPLEX_ROOT}")
message(STATUS "CPLEX_LIB_DIR=${CPLEX_LIB_DIR}")
message(STATUS "CONCERT_LIB_DIR=${CONCERT_LIB_DIR}")
message(STATUS "LKH source dir=${CMAKE_CURRENT_SOURCE_DIR}/LKH_LIB")