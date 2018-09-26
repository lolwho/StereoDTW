cmake_minimum_required(VERSION 3.10)

include_directories(
  "${CMAKE_CURRENT_SOURCE_DIR}"
)

add_subdirectory(json-cpp)

project(stereodtw)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(dtw dtw.h src/dtw.cpp)
target_link_libraries(dtw)

add_library(stereodtw src/stereo_matcher.cpp stereo_matcher.h)
target_link_libraries(stereodtw dtw jsoncpp ${OpenCV_LIBS})

add_executable(stereodtw_demo demo/main.cpp)
target_link_libraries(stereodtw_demo stereodtw ${OpenCV_LIBS})
