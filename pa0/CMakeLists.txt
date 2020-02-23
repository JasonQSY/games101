cmake_minimum_required (VERSION 2.8.11)
project (Transformation)

find_package(Eigen3 REQUIRED)
include_directories("/usr/local/Cellar/eigen/3.3.7/include")

add_executable (Transformation main.cpp)
