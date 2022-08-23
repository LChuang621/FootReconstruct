cmake_minimum_required(VERSION 3.10)
project(FootReconstruct)

set(CMAKE_CXX_FLAGS "-march=native ${CMAKE_CXX_FLAGS}")

find_package(PCL 1.7 REQUIRED COMPONENTS common io)

set(OpenCV_DIR "C:/Program Files/PCL 1.12.0/3rdParty/opencv/build")
find_package(OpenCV REQUIRED)
set(OpenCV_LIBRARIES_DIRS "${OpenCV_DIR}/x64/vc15/lib")

message(STATUS "Using opencv ${OpenCV_VERSION}")
message(STATUS "Using OpenCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Using OpenCV_LIBRARIES_DIRS ${OpenCV_LIBRARIES_DIRS}")
message(STATUS "Using OpenCV_LIBS ${OpenCV_LIBS}")

message(STATUS "Using PCL ${PCL_VERSION}")
message(STATUS "Using PCL_INCLUDE_DIRS ${PCL_INCLUDE_DIRS}")
message(STATUS "Using PCL_LIBRARY_DIRS ${PCL_LIBRARY_DIRS}")
message(STATUS "Using PCL_LIBRARIES ${PCL_LIBRARIES}")


include_directories(${PCL_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/include" ${PCL_LIBRARY_DIRS}/../3rdParty/FLANN/include ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})
ADD_EXECUTABLE(FootReconstruct ${PROJECT_SOURCE_DIR}/src/FilterGround.cpp ${PROJECT_SOURCE_DIR}/src/Coordinate.cpp ${PROJECT_SOURCE_DIR}/src/Boundryextract.cpp ${PROJECT_SOURCE_DIR}/src/MeshProcess.cpp ${PROJECT_SOURCE_DIR}/src/HoleFix.cpp)
target_link_libraries(FootReconstruct ${PCL_LIBRARIES}
	pcl_segmentationd.lib
	pcl_filtersd.lib
	pcl_visualizationd.lib
	pcl_searchd.lib
	pcl_kdtreed.lib
	pcl_featuresd.lib
	pcl_surfaced.lib
	opencv_world453d.lib)