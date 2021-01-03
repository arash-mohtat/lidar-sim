cmake_minimum_required(VERSION 3.5)

project(LiDAR_cpp_minimal LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library (Helpers helpers.cpp)
add_library (Body body.cpp)
add_library (Lidar lidar.cpp)

add_executable(LiDAR_cpp_minimal main.cpp body.cpp helpers.cpp lidar.cpp)


