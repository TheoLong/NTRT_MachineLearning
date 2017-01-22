# Install script for directory: /media/psf/NTRT_MachineLearning/env/build/jsoncpp_3515db/include

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/media/psf/NTRT_MachineLearning/env")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/json" TYPE FILE FILES
    "/media/psf/NTRT_MachineLearning/env/build/jsoncpp_3515db/include/json/assertions.h"
    "/media/psf/NTRT_MachineLearning/env/build/jsoncpp_3515db/include/json/autolink.h"
    "/media/psf/NTRT_MachineLearning/env/build/jsoncpp_3515db/include/json/config.h"
    "/media/psf/NTRT_MachineLearning/env/build/jsoncpp_3515db/include/json/features.h"
    "/media/psf/NTRT_MachineLearning/env/build/jsoncpp_3515db/include/json/forwards.h"
    "/media/psf/NTRT_MachineLearning/env/build/jsoncpp_3515db/include/json/json.h"
    "/media/psf/NTRT_MachineLearning/env/build/jsoncpp_3515db/include/json/reader.h"
    "/media/psf/NTRT_MachineLearning/env/build/jsoncpp_3515db/include/json/value.h"
    "/media/psf/NTRT_MachineLearning/env/build/jsoncpp_3515db/include/json/version.h"
    "/media/psf/NTRT_MachineLearning/env/build/jsoncpp_3515db/include/json/writer.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
