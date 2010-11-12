cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/libfreenect/Modules/")
#add_subdirectory (libfreenect/lib)
#add_subdirectory (libfreenect/examples)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/../libfreenect/Modules/")

include_directories(${PROJECT_SOURCE_DIR}/../libfreenect/include)
link_directories(${PROJECT_SOURCE_DIR}/../libfreenect/lib)

#find_package(USB REQUIRED)
#include_directories(${USB_INCLUDE_DIRS})

#find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLU REQUIRED)
find_package(GLUT REQUIRED)


#rosbuild_add_library (freenect libfreenect/lib/cameras.c ${PROJECT_SOURCE_DIR}/libfreenect/lib/inits.c)
#target_link_libraries (freenect ${CMAKE_THREAD_LIBS_INIT} ${USB_LIBRARIES})

#find_library (PTHREAD pthread)

include_directories("/usr/include/libusb-1.0/")

rosbuild_add_executable(kinect_node src/kinect_node.cpp)
#target_link_libraries(kinect_node freenect GL GLU glut m)
target_link_libraries(kinect_node freenect usb-1.0 GL GLU glut)
rosbuild_link_boost(kinect_node thread)

rosbuild_add_executable(glview ../libfreenect/examples/glview.c)
target_link_libraries(glview freenect GL GLU glut m usb-1.0)
