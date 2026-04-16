cmake_minimum_required(VERSION 3.20.0)

# Option to choose keyboard side (left or right)
option(KEYBOARD_RIGHT "Build for right keyboard half" OFF)

if(KEYBOARD_RIGHT)
    set(DTC_OVERLAY_FILE app_right.overlay)
else()
    set(DTC_OVERLAY_FILE app_left.overlay)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(leaf_fall_keyboard)

if(KEYBOARD_RIGHT)
    target_compile_definitions(app PRIVATE KEYBOARD_RIGHT)
else()
    target_compile_definitions(app PRIVATE KEYBOARD_LEFT)
endif()

target_sources(app PRIVATE src/main.c)
