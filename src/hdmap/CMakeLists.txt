cmake_minimum_required(VERSION 3.18)
project(hdmap_core LANGUAGES CXX)

find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(Boost REQUIRED)

add_library(hdmap_core INTERFACE)
add_library(hdmap::core ALIAS hdmap_core)
set_target_properties(hdmap_core PROPERTIES EXPORT_NAME core)
target_compile_features(hdmap_core INTERFACE cxx_std_17)
target_include_directories(hdmap_core INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${lanelet2_core_INCLUDE_DIRS}
    ${lanelet2_io_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})
target_link_libraries(hdmap_core INTERFACE ${lanelet2_core_LIBRARIES} ${lanelet2_io_LIBRARIES})

include(GNUInstallDirs)
option(HDMAP_BUILD_PYTHON "Build the Python API using Lanelet2 Python types" ON)
if(HDMAP_BUILD_PYTHON)
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
    find_package(Boost REQUIRED COMPONENTS python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR})
    Python3_add_library(hdmap_python MODULE WITH_SOABI python/bindings.cpp)
    set_target_properties(hdmap_python PROPERTIES OUTPUT_NAME hdmap)
    target_link_libraries(hdmap_python PRIVATE hdmap::core
        Boost::python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR})
    install(TARGETS hdmap_python LIBRARY DESTINATION
        ${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages)
endif()

install(TARGETS hdmap_core EXPORT hdmap_core_targets)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT hdmap_core_targets NAMESPACE hdmap::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hdmap_core)

include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/hdmap_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/hdmap_coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hdmap_core)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/hdmap_coreConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hdmap_core)
