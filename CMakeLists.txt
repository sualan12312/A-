cmake_minimum_required(VERSION 3.10)
project(AStar)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)

add_executable(astar main.cpp)
target_link_libraries(astar ${OpenCV_LIBS})
