cmake_minimum_required(VERSION 3.16)
project(hdmap_core LANGUAGES CXX)

find_package(lanelet2_core REQUIRED)

add_library(hdmap_core INTERFACE)
add_library(hdmap::core ALIAS hdmap_core)
set_target_properties(hdmap_core PROPERTIES EXPORT_NAME core)
target_compile_features(hdmap_core INTERFACE cxx_std_17)
target_include_directories(hdmap_core INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${lanelet2_core_INCLUDE_DIRS})
target_link_libraries(hdmap_core INTERFACE ${lanelet2_core_LIBRARIES})

include(GNUInstallDirs)
install(TARGETS hdmap_core EXPORT hdmap_core_targets)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT hdmap_core_targets NAMESPACE hdmap::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hdmap_core)
