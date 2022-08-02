# On Ubuntu 18.04, get the latest CMake from https://apt.kitware.com/.
cmake_minimum_required(VERSION 3.18)

project(RealityModelingPCD LANGUAGES C CXX)

# The options need to be the same as Open3D's default
# If Open3D is configured and built with custom options, you'll also need to
# specify the same custom options.
option(STATIC_WINDOWS_RUNTIME "Use static (MT/MTd) Windows runtime" ON)
if(STATIC_WINDOWS_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

# Find installed Open3D, which exports Open3D::Open3D
find_package(Open3D REQUIRED)
include_directories(${Open3D_INCLUDE_DIRS})

add_executable(ReadParam test/ReadParam.cpp)
add_executable(LiDARImageToPointcloud src/LiDARImageToPointcloud.cpp)
add_executable(PointcloudToLidarImage src/PointcloudToLidarImage.cpp)
add_executable(ImageIntegrate src/ImageIntegrate.cpp)

target_link_libraries(LiDARImageToPointcloud ${Open3D_LIBRARIES})
target_link_libraries(PointcloudToLidarImage ${Open3D_LIBRARIES})
target_link_libraries(ImageIntegrate ${Open3D_LIBRARIES})


