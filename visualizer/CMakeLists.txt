cmake_minimum_required(VERSION 3.20)

project(upper_monitor_visualizer C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(NOT DEFINED RAYLIB_ROOT)
    set(RAYLIB_ROOT "D:/biancheng/C_third_library/raylib" CACHE PATH "Path to raylib root")
endif()

set(RAYLIB_INCLUDE_DIR "${RAYLIB_ROOT}/include")
set(RAYLIB_LIBRARY "${RAYLIB_ROOT}/lib/libraylib.a")
set(RAYLIB_DLL "${RAYLIB_ROOT}/lib/raylib.dll")

foreach(required_path IN ITEMS "${RAYLIB_INCLUDE_DIR}/raylib.h" "${RAYLIB_LIBRARY}" "${RAYLIB_DLL}")
    if(NOT EXISTS "${required_path}")
        message(FATAL_ERROR "Missing raylib dependency: ${required_path}")
    endif()
endforeach()

add_executable(
    upper_monitor_visualizer
    src/main.c
    src/math3d.c
    src/scene_assets.c
    src/score_receiver.c
    src/ui_theme.c
    src/udp_receiver.c
)

target_include_directories(
    upper_monitor_visualizer
    PRIVATE
        "${RAYLIB_INCLUDE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(
    upper_monitor_visualizer
    PRIVATE
        "${RAYLIB_LIBRARY}"
        ws2_32
        winmm
        gdi32
        opengl32
)

add_custom_command(
    TARGET upper_monitor_visualizer
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${RAYLIB_DLL}"
            $<TARGET_FILE_DIR:upper_monitor_visualizer>
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/../resources"
            "$<TARGET_FILE_DIR:upper_monitor_visualizer>/resources"
)
