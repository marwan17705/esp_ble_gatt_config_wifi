# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS ${app_sources})
# idf_component_register(SRCS "main.c"  "mw_wifi.c" "mw_ble_gatt.c" "mw_gpio.c"
#                     INCLUDE_DIRS "."
#                     )
