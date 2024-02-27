cmake_minimum_required (VERSION 3.1.0)

project(imu_datos_calibracion VERSION 1.0
        DESCRIPTION "Estimador de pose mediante un filtro complementario."
        LANGUAGES CXX)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Find RealSense2 lib
find_package(realsense2 REQUIRED)

# Complementary Filter Lib config
set(LIB_SOURCE_DIR complementary_filter/src)
set(LIB_INCLUDE_DIR complementary_filter/include)

set(LIB_SOURCE_FILES ${LIB_SOURCE_DIR}/complementary_filter.cpp)
set(LIB_HEADER_FILES ${LIB_INCLUDE_DIR}/complementary_filter.h)

# Adds directory to import headers
include_directories(${LIB_INCLUDE_DIR})

add_library(complementary_filter_LIBRARY STATIC ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES}) 

# Main file
set(MAIN_SRCS src/main.cpp)
# Add the aplication sources to the target
add_executable(${PROJECT_NAME} ${MAIN_SRCS})

# Link libraries to the target
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY} complementary_filter_LIBRARY) 
