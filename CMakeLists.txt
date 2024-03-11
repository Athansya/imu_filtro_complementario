cmake_minimum_required (VERSION 3.1.0)

project(imu_datos_calibracion VERSION 1.0
        DESCRIPTION "Estimador de pose mediante un filtro complementario."
        LANGUAGES CXX)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_FLAGS -pthread)

# Find RealSense2 lib
find_package(realsense2 REQUIRED)

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Complementary Filter Lib config
set(LIB_SOURCE_DIR complementary_filter/src)
set(LIB_INCLUDE_DIR complementary_filter/include)

set(LIB_SOURCE_FILES ${LIB_SOURCE_DIR}/complementary_filter.cpp)
set(LIB_HEADER_FILES ${LIB_INCLUDE_DIR}/complementary_filter.h)

# Adds directory to import headers
include_directories(${LIB_INCLUDE_DIR})

add_library(complementary_filter_LIBRARY STATIC ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES}) 

# Slamtec RPLIDAR SDK
# Por alguna razón debe ser el que está ahí
set(RPLIDAR_SDK_PATH "/home/nvidia/Documentos/Tesis/Rplidar_sample_code/sdk")


add_library(rplidar STATIC IMPORTED)

set_target_properties(rplidar PROPERTIES
        IMPORTED_LOCATION ${RPLIDAR_SDK_PATH}/output/Linux/Release/librplidar_sdk.a
)

target_include_directories(rplidar INTERFACE
        ${RPLIDAR_SDK_PATH}/sdk/include/
        ${RPLIDAR_SDK_PATH}/sdk/src/
)

# Main file
set(MAIN_SRCS src/main.cpp)
# Add the aplication sources to the target
add_executable(${PROJECT_NAME} ${MAIN_SRCS})

# Link libraries to the target
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY} complementary_filter_LIBRARY Eigen3::Eigen rplidar) 

# Warnings
# target_compile_options(${PROJECT_NAME} PRIVATE
#         -Wall
#         -Wextra
#         -Wpedantic
#         -Werror
#         -Wconversion
#         -Wsign-conversion
# )