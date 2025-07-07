cmake_minimum_required(VERSION 3.5...3.26)
# https://cmake.org/cmake/help/latest/variable/BUILD_SHARED_LIBS.html
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif()

project(dqrobotics-interface-webots LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

if (UNIX AND NOT APPLE)
    set(WEBOTS_HOME "/usr/local/webots")
    link_directories(${WEBOTS_HOME}/lib/controller)
endif()

if (APPLE)
    find_package(cppzmq REQUIRED)
    set(WEBOTS_HOME /Applications/Webots.app/Contents/)

    # The DQ Robotics library is installed in /usr/local/include/ on macOS platforms.
    # The following directives are required to find and link the DQ Robotics to the
    # dqrobotics-interface-coppeliasim-zmq. Otherwise, this project could not compile
    # from sources on macOS, as reported in
    # https://github.com/dqrobotics/cpp-interface-coppeliasim-zmq/issues/16
    INCLUDE_DIRECTORIES(/usr/local/include/)
    LINK_DIRECTORIES(/usr/local/lib/
                     ${WEBOTS_HOME}/lib/controller
                )
endif()
if (WIN32)
    # Instructions for windows here
endif ()

add_library(${PROJECT_NAME} 
    src/dqrobotics/interfaces/webots/DQ_WebotsInterface.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
        include
        ${WEBOTS_HOME}/include/controller/c
        ${WEBOTS_HOME}/include/controller/cpp
)

set(WEBOT_LIBRARIES m ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})


if(APPLE)
    target_link_libraries(${PROJECT_NAME}
            dqrobotics
            Eigen3::Eigen
            ${WEBOT_LIBRARIES}
    )
endif()
if(UNIX AND NOT APPLE)
    target_link_libraries(${PROJECT_NAME}
            dqrobotics
            Eigen3::Eigen
            ${WEBOT_LIBRARIES}
    )
endif()
if(WIN32)
    target_link_libraries(${PROJECT_NAME}
            cppzmq
            dqrobotics
            ${WEBOT_LIBRARIES}
    )
endif()


include_directories(include)
add_executable(test src/test.cpp)
target_link_libraries(test
dqrobotics
dqrobotics-interface-webots
)

