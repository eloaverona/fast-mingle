cmake_minimum_required(VERSION 3.5)
project(mingle)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.61.0 COMPONENTS math) 
find_package(Boost COMPONENTS program_options REQUIRED)
add_subdirectory(ann_1.1.2)

set(MINGLER_SOURCE_FILES main.cpp main.h Edge.cpp Edge.h GetTotalInkWhenMovedByFraction.cpp GetTotalInkWhenMovedByFraction.h Point.cpp Point.h Edge.cpp Edge.h Graph.h Graph.cpp InkBundleAndBundleOperationType.h EdgeAndBundleOperationType.h BundleOperation.h EdgeIdGenerator.cpp EdgeIdGenerator.h)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-register")
include_directories(${Boost_INCLUDE_DIRS}) 
add_executable(mingle ${MINGLER_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ann ${Boost_LIBRARIES})