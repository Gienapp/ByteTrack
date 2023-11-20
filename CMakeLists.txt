# 
# To be defined header with copyright information
# 

# Set minimal required CMake version
cmake_minimum_required( VERSION 3.20 )

project(bytetrack
        VERSION   0.1
        LANGUAGES CXX 
)

# Define custom namespace
set( CUSTOM_PROJECT_NAMESPACE "Bytetrack" )

# Add the directory the custom cmake scripts 
# are located to the search path
list( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# Add dependencies (both required and optional)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.4 REQUIRED)

#add_definitions(-std=c++11)

# Globally set C++ standard and whether to insist on 
# the compiler being able to support this standard
# Can be overwritten on per-target basis using
# o set_target_properties( ... PROPERTIES CXX_STANDARD ... ) and
# o set_target_properties( ... PROPERTIES CXX_STANDARD_REQUIRED ... )
set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

# Compiler options interface pseudo target
include( CustomCompilerOptions  )
add_library( ${PROJECT_NAME}_compiler_options INTERFACE )
set_custom_compiler_options( ${PROJECT_NAME}_compiler_options )

# Compiler warning interface pseudo target
include( CustomCompilerWarnings )
add_library( ${PROJECT_NAME}_compiler_warnings INTERFACE )
set_custom_compiler_warnings( ${PROJECT_NAME}_compiler_warnings )

# > Library soures
add_subdirectory( src )



#include_directories(${PROJECT_SOURCE_DIR}/include)
#link_directories(${PROJECT_SOURCE_DIR}/include)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

# Installation

# [Lib]

# > Installation targets
#   sets {PROJECT_NAME}_exported_targets name that
#   can be referenced for writing a file with the
#   targets
 install(
     TARGETS
 		 bytetrack
         ${PROJECT_NAME}_compiler_options 
         ${PROJECT_NAME}_compiler_warnings
     EXPORT
         ${PROJECT_NAME}_exported_targets
     LIBRARY DESTINATION
         lib
     ARCHIVE DESTINATION
         lib
 )

# > Write targets into {PROJECT_NAME}Targets.cmake file
#   file will be located in <INSTALL_DIRECTORY>/<PROJECT_NAME>
 install(
     EXPORT
         ${PROJECT_NAME}_exported_targets
     FILE
         ${PROJECT_NAME}Targets.cmake
     NAMESPACE
         ${CUSTOM_PROJECT_NAMESPACE}::
     DESTINATION
         lib/cmake/${PROJECT_NAME}
)

# > Install header files
 install(
     DIRECTORY
         ${PROJECT_SOURCE_DIR}/include/
     DESTINATION
         include/${PROJECT_NAME}
 )

# > Write version package file
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
        "${PROJECT_BINARY_DIR}/cmake/${PROJECT_NAME}ConfigVersion.cmake"
        VERSION
        ${PROJECT_VERSION}
        COMPATIBILITY
        AnyNewerVersion
)


# > Install both config files
 install(
         FILES
         "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake"
         DESTINATION
         "lib/cmake/${PROJECT_NAME}"
 )



#file(GLOB My_Source_Files ${PROJECT_SOURCE_DIR}/src/*.cpp)

#add_definitions(-O2 -pthread)

