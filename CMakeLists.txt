CMAKE_MINIMUM_REQUIRED( VERSION 3.5.1 )

ADD_DEFINITIONS( -std=c++11 )

PROJECT( LIDAR_DRIVER )

FIND_PACKAGE( Threads )

SET( EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin" )

AUX_SOURCE_DIRECTORY( ${PROJECT_SOURCE_DIR}/src lidar_src )

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/sdk/include )

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/sdk/src )

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/sdk/src/hal )

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/sdk/src/arch/linux )

LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib )

ADD_EXECUTABLE( lidar_driver ${lidar_src} )

TARGET_LINK_LIBRARIES( lidar_driver librplidar_sdk.a ${CMAKE_THREAD_LIBS_INIT} )


