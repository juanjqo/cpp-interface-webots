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
    src/dqrobotics/interfaces/webots/DQ_WebotsRobot.cpp
    src/dqrobotics/interfaces/webots/robots/URXWebotsRobot.cpp
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
            dqrobotics
            Eigen3::Eigen
            ${WEBOT_LIBRARIES}
    )
endif()


SET_TARGET_PROPERTIES(${PROJECT_NAME}
    PROPERTIES PUBLIC_HEADER
    "include/dqrobotics/interfaces/webots/DQ_WebotsInterface.h"
    )

INSTALL(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dqrobotics/interfaces/webots"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

if(WIN32)
INSTALL(TARGETS ${PROJECT_NAME}
# https://stackoverflow.com/questions/21592361/cmake-install-is-not-installing-libraries-on-windows
    RUNTIME DESTINATION "bin"
    LIBRARY DESTINATION "lib"
    ARCHIVE DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dqrobotics"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
endif()


################################################################
# INSTALL HEADERS IN SUBFOLDERS
################################################################

# Other Headers
INSTALL(FILES
    include/dqrobotics/interfaces/webots/DQ_WebotsRobot.h
    DESTINATION "include/dqrobotics/interfaces/webots")

# Robots
INSTALL(FILES
    include/dqrobotics/interfaces/webots/robots/URXWebotsRobot.h
    DESTINATION "include/dqrobotics/interfaces/webots/crobots")

################################################################
# INSTALL SOURCE FILES (So that the debugger can find them)
################################################################

# base folder
INSTALL(FILES
    src/dqrobotics/interfaces/webots/DQ_WebotsInterface.cpp
    src/dqrobotics/interfaces/webots/DQ_WebotsRobot.cpp

    DESTINATION "src/dqrobotics/interfaces/webots")

# robots folder
INSTALL(FILES
        src/dqrobotics/interfaces/webots/robots/URXWebotsRobot.cpp
    DESTINATION "src/dqrobotics/interfaces/webots/robots")

