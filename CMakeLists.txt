cmake_minimum_required(VERSION 3.20)

project(segmentation_pcl_cgal)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED COMPONENTS
        common
        io
        filters
        features
        kdtree
        search
        visualization
)

find_package(CGAL REQUIRED)

add_executable(seg main.cpp)

target_include_directories(seg PRIVATE ${PCL_INCLUDE_DIRS})
target_link_directories(seg PRIVATE ${PCL_LIBRARY_DIRS})
target_compile_definitions(seg PRIVATE ${PCL_DEFINITIONS})

target_link_libraries(seg PRIVATE
        ${PCL_LIBRARIES}
        CGAL::CGAL
)