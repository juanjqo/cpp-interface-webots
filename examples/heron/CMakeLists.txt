cmake_minimum_required(VERSION 3.5...3.26)



project(heron LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(Eigen3 REQUIRED)


if (UNIX AND NOT APPLE)
    set(WEBOTS_HOME "/usr/local/webots")
    link_directories(${WEBOTS_HOME}/lib/controller)
endif()


if (APPLE)
    set(WEBOTS_HOME /Applications/Webots.app/Contents/)
    INCLUDE_DIRECTORIES(
           /usr/local/include/
           /usr/local/include/eigen3
           # Most recent versions of brew install here
           /opt/homebrew/include/
           /opt/homebrew/include/eigen3
       )
   ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
   # The library is installed here when using the regular cmake ., make, sudo make install
   LINK_DIRECTORIES(
       /usr/local/lib/
       /opt/homebrew/lib
       ${WEBOTS_HOME}/lib/controller)
endif()



add_executable(${PROJECT_NAME}
               ${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    dqrobotics
    dqrobotics-interface-webots)


