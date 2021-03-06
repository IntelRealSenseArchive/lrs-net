# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
# ubuntu 16.04 LTS cmake version 3.5.1
cmake_minimum_required(VERSION 2.8.3)

project(RealsenseToolsRosbagInspector)

if(BUILD_GRAPHICAL_EXAMPLES)

    set(RS_ROSBAG_INSPECTOR_CPP
            rs-rosbag-inspector.cpp
            files_container.h
            print_helpers.h
            rosbag_content.h
            ${CMAKE_SOURCE_DIR}/common/os.cpp
            ${CMAKE_SOURCE_DIR}/third-party/glad/glad.c
            ${CMAKE_SOURCE_DIR}/third-party/imgui/imgui.cpp
            ${CMAKE_SOURCE_DIR}/third-party/imgui/imgui_draw.cpp
            ${CMAKE_SOURCE_DIR}/third-party/imgui/imgui_impl_glfw.cpp
            ${CMAKE_SOURCE_DIR}/third-party/imgui/imgui-fonts-karla.hpp
            ${CMAKE_SOURCE_DIR}/third-party/imgui/imgui-fonts-fontawesome.hpp
            ${CMAKE_SOURCE_DIR}/third-party/tinyfiledialogs/tinyfiledialogs.c
            ${CMAKE_SOURCE_DIR}/third-party/tinyfiledialogs/tinyfiledialogs.h
        )

    if(WIN32)
        add_executable(rs-rosbag-inspector WIN32 ${RS_ROSBAG_INSPECTOR_CPP})

        include_directories(rs-rosbag-inspector ${CMAKE_SOURCE_DIR}/third-party/imgui ${CMAKE_SOURCE_DIR}/common ${CMAKE_SOURCE_DIR}/third-party/glad 
                                             ${CMAKE_SOURCE_DIR}/third-party/tinyfiledialogs ${CMAKE_SOURCE_DIR}/third-party ${CMAKE_CURRENT_SOURCE_DIR}/res/)
    else()
        add_executable(rs-rosbag-inspector ${RS_ROSBAG_INSPECTOR_CPP})
        include_directories(rs-rosbag-inspector ${CMAKE_SOURCE_DIR}/third-party/imgui ${CMAKE_SOURCE_DIR}/common ${CMAKE_SOURCE_DIR}/third-party/glad 
                                             ${CMAKE_SOURCE_DIR}/third-party/tinyfiledialogs ${CMAKE_SOURCE_DIR}/third-party)
    endif()
    set_property(TARGET rs-rosbag-inspector PROPERTY CXX_STANDARD 11)

    target_include_directories(rs-rosbag-inspector PRIVATE
        ${ROSBAG_HEADER_DIRS}
        ${BOOST_INCLUDE_PATH}
        ${LZ4_INCLUDE_PATH}
        )

    target_link_libraries(rs-rosbag-inspector realsense-file ${DEPENDENCIES})

    set_target_properties (rs-rosbag-inspector PROPERTIES
        FOLDER Tools
    )

    install(
        TARGETS

        rs-rosbag-inspector

        RUNTIME DESTINATION
        ${CMAKE_INSTALL_BINDIR}
    )
endif()
