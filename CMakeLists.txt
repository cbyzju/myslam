cmake_minimum_required(VERSION 2.8)
project(myslam)

set(CMAKE_CXX_COMPILER "g++")
set(CAMKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3")

###### structure ######
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


###### dependencies ######
#Eigen
if(WIN32)	#Microsoft windows
message(STATUS "used in windows")
include_directories("E:/windows/eigen-3.3.4")
elseif(APPLE)  #apple
message(STATUS "used in MacOS")
##do nothing
elseif(UNIX AND NOT APPLE)  #linux or unix
message(STATUS "used in linux or unix")
include_directories("/usr/local/include/eigen3" "/usr/include/eigen3")
endif()

#opencv, for viz module, 3.x branch is required
find_package(OpenCV 3.1 REQUIRED)
if(G2O_FOUND)
message(STATUS "Find OpenCV in " ${OpenCV_INCLUDE_DIRS})
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})
endif()

#Sophus
find_package(Sophus REQUIRED)
if(Sophus_FOUND)
message(STATUS "Find Sophus in " ${Sophus_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})
link_directories(${Sophus_LIBRARY_DIRS})
endif()

#G2O
find_package(G2O REQUIRED)
if(G2O_FOUND)
message(STATUS "Find g2o in " ${G2O_INCLUDE_DIRS})
include_directories(${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})
endif()


set(THIRD_PARTY_LIBS ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES} g2o_core g2o_stuff g2o_types_sba)

####### source and test ######
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory( src )
add_subdirectory( test )