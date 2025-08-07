cmake_minimum_required(VERSION 3.22...3.26)
# https://cmake.org/cmake/help/latest/variable/BUILD_SHARED_LIBS.html
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif()

project(dqrobotics-interface-webots
        VERSION 1.0
        DESCRIPTION "An unofficial DQ Robotics interface for C++17 to connect with Webots."
        HOMEPAGE_URL "https://github.com/juanjqo/cpp-interface-webots"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# strongly encouraged to enable this globally to avoid conflicts between
# -Wpedantic being enabled and -std=c++20 and -std=gnu++20 for example
# when compiling with PCH enabled
set(CMAKE_CXX_EXTENSIONS OFF)



include(cmake/PreventInSourceBuilds.cmake)
myproject_assure_out_of_source_builds()

include(ProjectOptions.cmake)
myproject_enable_ipo()



find_package(Eigen3 REQUIRED)
find_library(MATH_LIBRARY m REQUIRED)

find_package(Doxygen QUIET)

if (DOXYGEN_FOUND)
    message(STATUS "Doxygen found!")
    set(DOXYGEN_INPUT_DIRS
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )

    set(DOXYGEN_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/docs")

    set(DOXYGEN_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")

    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in
        ${DOXYGEN_CONFIG_FILE}
        @ONLY
    )

    add_custom_target(doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_CONFIG_FILE}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
else()
    message(STATUS "Doxygen not found. Documentation target will not be available.")
endif()


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
        #$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        #$<BUILD_INTERFACE:${WEBOTS_HOME}/include/controller/c>
        #$<BUILD_INTERFACE:${WEBOTS_HOME}/include/controller/cpp>
)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    myproject_supports_sanitizers()
    include(cmake/Sanitizers.cmake)
    option(myproject_ENABLE_SANITIZER_ADDRESS "Enable address sanitizer" ${SUPPORTS_ASAN})
    option(myproject_ENABLE_SANITIZER_LEAK "Enable leak sanitizer" ON)
    option(myproject_ENABLE_SANITIZER_UNDEFINED "Enable undefined sanitizer" ${SUPPORTS_UBSAN})
    option(myproject_ENABLE_SANITIZER_THREAD "Enable thread sanitizer" OFF)
    option(myproject_ENABLE_SANITIZER_MEMORY "Enable memory sanitizer" OFF)

    myproject_enable_sanitizers(
      ${PROJECT_NAME}
      ${myproject_ENABLE_SANITIZER_ADDRESS}
      ${myproject_ENABLE_SANITIZER_LEAK}
      ${myproject_ENABLE_SANITIZER_UNDEFINED}
      ${myproject_ENABLE_SANITIZER_THREAD}
      ${myproject_ENABLE_SANITIZER_MEMORY})
endif()


set(WEBOT_LIBRARIES ${MATH_LIBRARY}
    ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})


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
    DESTINATION "include/dqrobotics/interfaces/webots/robots")

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



#For debug
#include_directories(        include
#    ${WEBOTS_HOME}/include/controller/c
#    ${WEBOTS_HOME}/include/controller/cpp)
#add_executable(test
#               examples/ur3/ur3.cpp)

#target_link_libraries(test
#    Eigen3::Eigen
#    dqrobotics
#    dqrobotics-interface-webots)
