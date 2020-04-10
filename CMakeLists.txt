cmake_minimum_required (VERSION 3.1)
project (HexVolumeRenderer)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMake)
#MESSAGE(STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

if (UNIX AND NOT APPLE)
	# Dirty fix for bug in Ubuntu 18.04: https://bugs.launchpad.net/ubuntu/+source/liblemon/+bug/1827886
	list(APPEND CMAKE_PREFIX_PATH /usr/lib/cmake)
endif()


set (CMAKE_CXX_STANDARD 11)

file(GLOB_RECURSE SOURCES src/*.cpp src/*.c)
include_directories(src)

option(USE_EMBREE "Build with Embree as Ray-Mesh Intersection Library" OFF)
option(USE_CORK "Build with Cork as CSG Library" OFF)

if(USE_EMBREE)
	find_package(embree 3.0 REQUIRED)
	include_directories(${EMBREE_INCLUDE_DIRS})
	add_definitions(-DUSE_EMBREE)
else()
	list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/Renderers/Intersection/RayMeshIntersection_Embree.cpp)
endif()

#make VERBOSE=1

IF(WIN32)
	add_executable(HexVolumeRenderer WIN32 ${SOURCES})
ELSE()
	add_executable(HexVolumeRenderer ${SOURCES})
ENDIF()

if(USE_CORK)
	find_package(Cork REQUIRED)
	find_package(GMP REQUIRED)
	include_directories(${CORK_INCLUDE_DIR})
	add_definitions(-DUSE_CORK)
	add_definitions(-DUSE_CSG)
	target_link_libraries(HexVolumeRenderer ${CORK_LIBRARIES})
	target_link_libraries(HexVolumeRenderer ${GMP_LIBRARIES})
endif()

if(USE_EMBREE)
	target_link_libraries(HexVolumeRenderer ${EMBREE_LIBRARIES})
endif()

IF(WIN32)
	SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mwindows")
	target_link_libraries(HexVolumeRenderer mingw32 SDL2main SDL2)
ELSE()
	target_link_libraries(HexVolumeRenderer SDL2)
ENDIF()


cmake_policy(SET CMP0012 NEW)
find_package(OpenMP REQUIRED)
find_package(SGL REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(LEMON REQUIRED)
find_package(GLEW REQUIRED)
find_package(CURL REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
IF(WIN32)
	target_link_libraries(HexVolumeRenderer opengl32)
ELSE()
	target_link_libraries(HexVolumeRenderer GL)
ENDIF()
target_link_libraries(HexVolumeRenderer tinyxml2)
target_link_libraries(HexVolumeRenderer Eigen3::Eigen)
target_link_libraries(HexVolumeRenderer ${SGL_LIBRARIES} ${Boost_LIBRARIES} ${GLEW_LIBRARIES} ${NETCDF_LIBRARIES} ${CURL_LIBRARIES} ${JSONCPP_LIBRARIES}) # ${LEMON_LIBRARIES}

include_directories(${Boost_INCLUDES} ${LEMON_INCLUDES} ${GLEW_INCLUDES} ${NETCDF_INCLUDES} ${CURL_INCLUDES} ${JSONCPP_INCLUDES})

if(OPENMP_FOUND)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
