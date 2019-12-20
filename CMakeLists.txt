CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

project(CopaManager)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package( PCL REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#set(project_link_libs RealSenseManager )
add_executable(visualizer visualizer.cpp)
target_link_libraries(visualizer ${OpenCV_LIBS} ${PCL_LIBRARIES})