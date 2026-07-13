add_library(plugin_rgb INTERFACE)

target_sources(plugin_rgb INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/rgb.c
 ${CMAKE_CURRENT_LIST_DIR}/rgbv2.c
)

target_include_directories(plugin_rgb INTERFACE ${CMAKE_CURRENT_LIST_DIR})
