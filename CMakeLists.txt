cmake_minimum_required(VERSION 3.24)

# Auto-detect modern MinGW toolchain if default is ancient GCC on Windows
if(WIN32 AND (NOT DEFINED CMAKE_CXX_COMPILER OR CMAKE_CXX_COMPILER MATCHES "MinGW/bin"))
    file(GLOB WINLIBS_PATHS "C:/Users/*/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.*/mingw64/bin/g++.exe")
    if(WINLIBS_PATHS)
        list(GET WINLIBS_PATHS 0 MODERN_CXX)
        get_filename_component(MODERN_BIN_DIR "${MODERN_CXX}" DIRECTORY)
        set(CMAKE_CXX_COMPILER "${MODERN_BIN_DIR}/g++.exe" CACHE FILEPATH "C++ Compiler" FORCE)
        set(CMAKE_C_COMPILER "${MODERN_BIN_DIR}/gcc.exe" CACHE FILEPATH "C Compiler" FORCE)
    endif()
endif()

project(vlsi_soc_studio
    VERSION 1.0.0
    DESCRIPTION "Industrial Apple A/M-Series Class EDA Physical Design and Verification Platform"
    LANGUAGES CXX C
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Optimization and warning flags according to GEMINI.md
if(MSVC)
    add_compile_options(/O2 /W4 /permissive- /utf-8)
else()
    add_compile_options(-O3 -march=native -Wall -Wextra -Wpedantic)
    add_link_options(-static -static-libgcc -static-libstdc++)
endif()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Core Database Library
add_library(core_db STATIC
    src/core/db/chip_db.cpp
    src/core/db/chip_db.hpp
    src/core/db/arena_pool.hpp
    src/core/db/rtree.hpp
    src/core/db/observer.hpp
    src/core/db/id_types.hpp
)

target_include_directories(core_db PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core/db
)

# Core Parsers Library (GDSII, LEF, DEF)
add_library(core_parsers STATIC
    src/core/parsers/gdsii_stream.cpp
    src/core/parsers/gdsii_stream.hpp
    src/core/parsers/lef_def_parser.cpp
    src/core/parsers/lef_def_parser.hpp
)

target_link_libraries(core_parsers PUBLIC core_db)
target_include_directories(core_parsers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core/parsers
)

# Automated CTest Verification Suite
include(CTest)
enable_testing()

add_executable(test_chipdb_hierarchy
    tests/test_chipdb_hierarchy.cpp
)

target_link_libraries(test_chipdb_hierarchy PRIVATE
    core_parsers
    core_db
)

add_test(NAME ChipDBHierarchyTest COMMAND test_chipdb_hierarchy)
