cmake_minimum_required(VERSION 3.20)

project(segmentation_pcl_cgal_qt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(PCL REQUIRED COMPONENTS
        common
        io
        filters
        features
        kdtree
        search
        segmentation
        sample_consensus
        visualization
)

find_package(CGAL REQUIRED)

add_executable(seg_gui
        main.cpp
        MainWindow.h
        MainWindow.cpp
        Primitive.h
        PointCloudPipeline.h
        PointCloudPipeline.cpp
)

target_include_directories(seg_gui PRIVATE
        ${PCL_INCLUDE_DIRS}
)

target_link_directories(seg_gui PRIVATE
        ${PCL_LIBRARY_DIRS}
)

target_compile_definitions(seg_gui PRIVATE
        ${PCL_DEFINITIONS}
)

target_link_libraries(seg_gui PRIVATE
        Qt5::Widgets
        ${PCL_LIBRARIES}
        CGAL::CGAL
)