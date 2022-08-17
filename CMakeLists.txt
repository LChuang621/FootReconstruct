cmake_minimum_required(VERSION 3.10)
project(FootReconstruct)

set(CMAKE_CXX_FLAGS "-march=native ${CMAKE_CXX_FLAGS}")

find_package(PCL 1.7 REQUIRED COMPONENTS common io)

message(STATUS "Using PCL ${PCL_VERSION}")
message(STATUS "Using PCL_INCLUDE_DIRS ${PCL_INCLUDE_DIRS}")
message(STATUS "Using PCL_LIBRARY_DIRS ${PCL_LIBRARY_DIRS}")
message(STATUS "Using PCL_LIBRARIES ${PCL_LIBRARIES}")


include_directories(${PCL_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/include" ${PCL_LIBRARY_DIRS}/../3rdParty/FLANN/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
ADD_EXECUTABLE(FootReconstruct ${PROJECT_SOURCE_DIR}/src/FilterGround.cpp)
target_link_libraries(FootReconstruct ${PCL_LIBRARIES} pcl_segmentationd.lib pcl_filtersd.lib pcl_visualizationd.lib)