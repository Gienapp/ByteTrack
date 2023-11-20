# Custom CMake file to utilize 'bytetrack' library
# within the drives project
# @author Jan Phan
# @date   2023.11.01

add_library( 
    bytetrack 
    STATIC
        BYTETracker.cpp
        kalmanFilter.cpp
        lapjv.cpp
        STrack.cpp
        utils.cpp
)

set_property( TARGET bytetrack PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories( 
    bytetrack
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS}
)


# > Link libraries
target_link_libraries(
    bytetrack
    PRIVATE
    ${PROJECT_NAME}_compiler_options
    ${PROJECT_NAME}_compiler_warnings
    PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
)
