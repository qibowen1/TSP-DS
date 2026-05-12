cmake_minimum_required(VERSION 3.16)
project(RSP_AMSN LANGUAGES CXX)

# -------------------------
# C++ standard
# -------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -------------------------
# Options
# -------------------------
option(BUILD_TSPDS "Build TSP-DS executable (TSPDS-main.cpp)" ON)
option(BUILD_RSP  "Build RSP executable (main.cpp)" ON)
option(ENABLE_LKH "Enable LKH-related code" ON)
option(ENABLE_CPLEX "Enable IBM ILOG CPLEX (requires CPLEX_ROOT)" OFF)

# -------------------------
# Common include dirs
# -------------------------
set(PROJ_INC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# -------------------------
# Threads (Linux often needs pthread)
# -------------------------
find_package(Threads REQUIRED)

# -------------------------
# Core sources (no main)
# 你项目是平铺结构：把“非入口 cpp”都放这里
# -------------------------
set(CORE_SOURCES
    benchmark_loader.cpp
    DroneScheduler.cpp
    InitialSolutionGenerator.cpp
    LocalSearchOperators.cpp
    PerturbationOperators.cpp
    TSPDSSolver.cpp
    TSPDSUtils.cpp
    VNSLogger.cpp
    tsplib_parser.cpp
    solver.cpp
)

# LKH (可选)
if(ENABLE_LKH)
    list(APPEND CORE_SOURCES
        lkh_solver.cpp
    )
    add_compile_definitions(ENABLE_LKH=1)
else()
    add_compile_definitions(ENABLE_LKH=0)
endif()

# CPLEX (可选：仅当你要用精确求解时开启)
if(ENABLE_CPLEX)
    if(NOT DEFINED CPLEX_ROOT)
        message(FATAL_ERROR "ENABLE_CPLEX=ON but CPLEX_ROOT is not set. Example: -DCPLEX_ROOT=/opt/ibm/ILOG/CPLEX_Studio2211")
    endif()

    list(APPEND CORE_SOURCES
        CplexF1Solver.cpp
        CplexF2Solver.cpp
    )
    add_compile_definitions(ENABLE_CPLEX=1)

else()
    add_compile_definitions(ENABLE_CPLEX=0)
endif()

# -------------------------
# Build a core object library
# (so we can reuse in multiple executables)
# -------------------------
add_library(core_obj OBJECT ${CORE_SOURCES})
target_include_directories(core_obj PRIVATE ${PROJ_INC})
target_link_libraries(core_obj PRIVATE Threads::Threads)

# MSVC / GCC/Clang warnings (optional, safe)
if(MSVC)
    target_compile_options(core_obj PRIVATE /W3 /permissive-)
else()
    target_compile_options(core_obj PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

# -------------------------
# CPLEX link settings (Linux/Windows)
# -------------------------
if(ENABLE_CPLEX)
    target_include_directories(core_obj PRIVATE
        ${CPLEX_ROOT}/cplex/include
        ${CPLEX_ROOT}/concert/include
    )

    if(WIN32)
        # Windows: 一般是 x64_windows_msvc14 或类似目录，按你安装版本调整
        target_link_directories(core_obj PRIVATE
            ${CPLEX_ROOT}/cplex/lib/x64_windows_msvc14
            ${CPLEX_ROOT}/concert/lib/x64_windows_msvc14
        )
        target_link_libraries(core_obj PRIVATE ilocplex concert cplex)
    else()
        # Linux: 常见路径是 x86-64_linux/static_pic 或 x86-64_linux/static
        # 你可以按实际安装目录改一次即可
        target_link_directories(core_obj PRIVATE
            ${CPLEX_ROOT}/cplex/lib/x86-64_linux/static_pic
            ${CPLEX_ROOT}/concert/lib/x86-64_linux/static_pic
        )
        target_link_libraries(core_obj PRIVATE
            ilocplex concert cplex
            m dl
        )
    endif()
endif()

# -------------------------
# Executables
# 你的工程里有两个入口：TSPDS-main.cpp 和 main.cpp
# -------------------------
if(BUILD_TSPDS)
    add_executable(tspds TSPDS-main.cpp $<TARGET_OBJECTS:core_obj>)
    target_include_directories(tspds PRIVATE ${PROJ_INC})
    target_link_libraries(tspds PRIVATE Threads::Threads)
endif()

if(BUILD_RSP)
    add_executable(rsp main.cpp $<TARGET_OBJECTS:core_obj>)
    target_include_directories(rsp PRIVATE ${PROJ_INC})
    target_link_libraries(rsp PRIVATE Threads::Threads)
endif()

# -------------------------
# Nice output dir (optional)
# -------------------------
set_target_properties(
    $<$<BOOL:${BUILD_TSPDS}>:tspds>
    $<$<BOOL:${BUILD_RSP}>:rsp>
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

message(STATUS "BUILD_TSPDS=${BUILD_TSPDS}, BUILD_RSP=${BUILD_RSP}, ENABLE_LKH=${ENABLE_LKH}, ENABLE_CPLEX=${ENABLE_CPLEX}")
