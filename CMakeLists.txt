cmake_minimum_required(VERSION 3.5)
project(mingle)

set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

add_subdirectory(ann_1.1.2)

set(MINGLER_SOURCE_FILES main.cpp main.h Graph.cpp Graph.h Edge.cpp Edge.h Point.cpp Point.h PointOrStar.h PointOrStar.h)
set(RENDERER_SOURCE_FILES renderer_main.h renderer_main.cpp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-register")
add_executable(mingle ${MINGLER_SOURCE_FILES})
add_executable(render_graph ${RENDERER_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ann ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
target_link_libraries(render_graph ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})