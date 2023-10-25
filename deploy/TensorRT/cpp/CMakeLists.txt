cmake_minimum_required(VERSION 2.6)

project(bytetrack)

add_definitions(-std=c++11)

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


include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(/usr/local/include/eigen3)
link_directories(${PROJECT_SOURCE_DIR}/include)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB My_Source_Files ${PROJECT_SOURCE_DIR}/src/*.cpp)
target_link_libraries(bytetrack nvinfer)
target_link_libraries(bytetrack cudart)
target_link_libraries(bytetrack ${OpenCV_LIBS})

add_definitions(-O2 -pthread)

