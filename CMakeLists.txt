cmake_minimum_required(VERSION 3.24)

# Auto-detect modern MinGW toolchain if default is ancient GCC on Windows
if(WIN32)
    if(EXISTS "D:/Qt/Tools/mingw1310_64/bin/g++.exe")
        set(CMAKE_CXX_COMPILER "D:/Qt/Tools/mingw1310_64/bin/g++.exe" CACHE FILEPATH "C++ Compiler" FORCE)
        set(CMAKE_C_COMPILER "D:/Qt/Tools/mingw1310_64/bin/gcc.exe" CACHE FILEPATH "C Compiler" FORCE)
    elseif(NOT DEFINED CMAKE_CXX_COMPILER OR CMAKE_CXX_COMPILER MATCHES "MinGW/bin")
        file(GLOB WINLIBS_PATHS "C:/Users/*/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.*/mingw64/bin/g++.exe")
        if(WINLIBS_PATHS)
            list(GET WINLIBS_PATHS 0 MODERN_CXX)
            get_filename_component(MODERN_BIN_DIR "${MODERN_CXX}" DIRECTORY)
            set(CMAKE_CXX_COMPILER "${MODERN_BIN_DIR}/g++.exe" CACHE FILEPATH "C++ Compiler" FORCE)
            set(CMAKE_C_COMPILER "${MODERN_BIN_DIR}/gcc.exe" CACHE FILEPATH "C Compiler" FORCE)
        endif()
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

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Configure Qt 6 MinGW installation path
if(EXISTS "D:/Qt/6.6.3/mingw_64")
    list(PREPEND CMAKE_PREFIX_PATH "D:/Qt/6.6.3/mingw_64")
elseif(EXISTS "C:/Qt/6.6.3/mingw_64")
    list(PREPEND CMAKE_PREFIX_PATH "C:/Qt/6.6.3/mingw_64")
endif()

find_package(Qt6 COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets REQUIRED)

# Optimization and warning flags according to GEMINI.md
if(MSVC)
    add_compile_options(/O2 /W4 /permissive- /utf-8)
else()
    add_compile_options(-O3 -march=native -Wall -Wextra -Wpedantic)
    add_link_options(-static-libgcc -static-libstdc++)
endif()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 1. Core Database Library
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

# 2. Core Parsers Library (GDSII, LEF, DEF)
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

# 3. Core Scripting & Command Dispatcher Library
add_library(core_script STATIC
    src/core/script/command_dispatcher.cpp
    src/core/script/command_dispatcher.hpp
)

target_link_libraries(core_script PUBLIC core_parsers core_db)
target_include_directories(core_script PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core/script
)

# 4. GUI & Viewport Library
add_library(gui_core STATIC
    src/gui/gui_types.hpp
    src/gui/main_window.cpp
    src/gui/main_window.hpp
    src/gui/display_control_panel.cpp
    src/gui/display_control_panel.hpp
    src/gui/hierarchy_browser.cpp
    src/gui/hierarchy_browser.hpp
    src/gui/inspector_panel.cpp
    src/gui/inspector_panel.hpp
    src/gui/script_console.cpp
    src/gui/script_console.hpp
    src/gui/viewport/layout_viewport.cpp
    src/gui/viewport/layout_viewport.hpp
    src/gui/viewport/gl_shaders.hpp
)

target_link_libraries(gui_core PUBLIC
    core_script
    core_parsers
    core_db
    Qt6::Widgets
    Qt6::OpenGLWidgets
    opengl32
)

target_include_directories(gui_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/viewport
)

# 5. Desktop Application Executable
add_executable(vlsi_soc_studio_gui
    src/gui/main.cpp
)

target_link_libraries(vlsi_soc_studio_gui PRIVATE
    gui_core
)

# 6. Automated CTest Verification Suite
include(CTest)
enable_testing()

add_executable(test_chipdb_hierarchy
    tests/test_chipdb_hierarchy.cpp
)

target_link_libraries(test_chipdb_hierarchy PRIVATE
    core_parsers
    core_db
)

add_executable(test_gui_headless
    tests/test_gui_headless.cpp
)

target_link_libraries(test_gui_headless PRIVATE
    gui_core
)

add_test(NAME ChipDBHierarchyTest COMMAND test_chipdb_hierarchy)
add_test(NAME GUIHeadlessTest COMMAND test_gui_headless)

# Ensure Qt runtime DLLs are found during CTest runs
if(WIN32)
    set(QT_BIN_PATH "D:/Qt/6.6.3/mingw_64/bin;C:/Qt/6.6.3/mingw_64/bin;$ENV{PATH}")
    set_tests_properties(ChipDBHierarchyTest PROPERTIES ENVIRONMENT "PATH=${QT_BIN_PATH}")
    set_tests_properties(GUIHeadlessTest PROPERTIES ENVIRONMENT "PATH=${QT_BIN_PATH};QT_QPA_PLATFORM=offscreen")
endif()
