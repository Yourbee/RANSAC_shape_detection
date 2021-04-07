cmake_minimum_required(VERSION 3.0)
project(pc_ransac)

set(CMAKE_CXX_FLAGS "-fpermissive")

include_directories(./)
include_directories(./MiscLib/)
FILE(GLOB SRC "./*.cpp")
FILE(GLOB MISC_SRC "./MiscLib/*.cpp")

set(OpenCV_DIR "C:/dev/OpenCV/build")

add_library(libransac STATIC  ${SRC} ${MISC_SRC})

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

add_executable(main main.cpp)

target_link_libraries(main libransac ${PCL_LIBRARIES} ${OpenCV_LIBS})
