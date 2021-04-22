cmake_minimum_required(VERSION 2.8)

project(roadmarking)

SET(CMAKE_CXX_FLAGS "-O3 -std=c++11 -w -fopenmp -pthread ")
SET(CMAKE_C_FLAGS "-O3 -std=c++11 -fopenmp -pthread ")

#SET(CMAKE_BUILD_TYPE "Debug") 
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

####### Required libs #######
#eigen3, liblas, OpenCV (>=2.4), pcl (>=1.7)

#eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("DEP: EIGEN [OK] ")

#PCL1.8
FIND_PACKAGE(PCL 1.7 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LIST(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
message("DEP: PCL [OK] ")

#OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
message("DEP: OPENCV [OK] ")

#libLAS 
FIND_PACKAGE(libLAS REQUIRED)
IF(libLAS_FOUND) 
INCLUDE_DIRECTORIES(${libLAS_INCLUDE_DIR})
MESSAGE("Found libLAS in: " ${libLAS_INCLUDE_DIR}) 
ENDIF(libLAS_FOUND)
message("DEP: libLAS [OK] ")

####### Optional libs ####### 
#CGAL,Qhull, GDAL

# head
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/dxflib)
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src SRC_LIST)
#message("SRC_LIST is " ${SRC_LIST})

SET(DEP_LIBS ${DEP_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${libLAS_LIBRARIES})

add_executable(roadmarking ${PROJECT_SOURCE_DIR}/test/roadmarking_main.cpp ${SRC_LIST})
target_link_libraries(roadmarking ${DEP_LIBS})


