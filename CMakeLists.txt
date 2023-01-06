cmake_minimum_required(VERSION 3.24)
project(LPOS_VFD)
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")
set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_CXX_STANDARD 23)
include_directories(
        /home/user/Documents/restbed/distribution/include
)

set(Boost_USE_STATIC OFF)
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED OFF)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS thread log log_setup system date_time chrono REQUIRED)
if (Boost_FOUND)
    message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
    message(STATUS "Boost_VERSION: ${Boost_VERSION}")
endif ()
add_executable(LPOS_VFD main.cpp SerialCommunicationHandler.cpp SerialCommunicationHandler.h)
set_target_properties(LPOS_VFD PROPERTIES OUTPUT_NAME "LPOS_VFD")
target_link_libraries(LPOS_VFD
        PRIVATE
        libboost_system.a
        libboost_thread.a
        libboost_log.a
        /home/user/Documents/restbed/distribution/library/librestbed.a
        )