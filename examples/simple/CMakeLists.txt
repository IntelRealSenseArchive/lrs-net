# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesSimple)

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(rs-simple rs-simple.cpp ../example.hpp)
    set_property(TARGET rs-simple PROPERTY CXX_STANDARD 11)
    include_directories(rs-simple ../ ../../third-party/tclap/include/)
    target_link_libraries(rs-simple ${DEPENDENCIES} realsense2-net)
    set_target_properties (rs-simple PROPERTIES FOLDER Examples)
    install(TARGETS rs-simple RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
