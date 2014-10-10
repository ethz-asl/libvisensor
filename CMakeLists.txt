#Thomas Schneider, 2/12/2013
#Pascal Gohl, 21/08/2014
cmake_minimum_required(VERSION 2.8.0)

############
# CATKIN STUFF
############
project(libvisensor)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)

if(NOT DEFINED DONT_USE_CATKIN)
find_package(catkin QUIET)
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS
  LIBRARIES visensor
)
endif()

############
# SETTINGS
############
set(PKGNAME visensor)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

###########
# BUILD
###########
FILE(
  GLOB SRCS 
  src/*.cpp 
  src/sensors/*.cpp
  src/networking/*.cpp
  src/synchronization/*.cpp
  src/serial_bridge/*.cpp
  src/visensor/*.cpp
  src/helpers/*.cpp
)
INCLUDE_DIRECTORIES("include")
include_directories(${EIGEN_INCLUDE_DIR})

#Release / debug build

ADD_DEFINITIONS (-DVISENSOR_EXPORT -fPIC -fmessage-length=0 -MMD -MP -Wall -pedantic -std=c++0x -fvisibility=hidden)
if(NOT DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
    ADD_DEFINITIONS (-march=native -O3)
endif(NOT DEFINED CMAKE_BUILD_TYPE)

#shared library
ADD_LIBRARY(${PKGNAME} SHARED ${SRCS})
SET_TARGET_PROPERTIES(${PKGNAME} PROPERTIES OUTPUT_NAME ${PKGNAME})
TARGET_LINK_LIBRARIES(${PKGNAME} boost_thread boost_system) 

#static library
ADD_LIBRARY(${PKGNAME}_static STATIC ${SRCS})
SET_TARGET_PROPERTIES(${PKGNAME}_static PROPERTIES OUTPUT_NAME ${PKGNAME})
TARGET_LINK_LIBRARIES(${PKGNAME}_static boost_thread boost_system) 

###########
# INSTALL
###########
#shared library
INSTALL(
  FILES lib/lib${PKGNAME}.so 
  DESTINATION lib 
  COMPONENT shared-lib
)

#static library + headers (-dev package)
INSTALL(
  FILES lib/lib${PKGNAME}.a 
  DESTINATION lib 
  COMPONENT static-lib
)

INSTALL(
  FILES include/visensor/visensor_api.hpp
        include/visensor/visensor_config.hpp
        include/visensor/visensor_constants.hpp
        include/visensor/visensor_datatypes.hpp
        include/visensor/visensor_exceptions.hpp
        include/visensor/visensor_version.hpp
        include/visensor/visensor.hpp
  DESTINATION include/visensor
  COMPONENT static-lib
)

INSTALL(
  FILES cmake/FindVISensor.cmake 
  DESTINATION lib/visensor
  COMPONENT static-lib
)

INSTALL(
  FILES pkgconfig/visensor.pc 
  DESTINATION share/pkgconfig 
  COMPONENT 
  static-lib
)
