find_package(Doxygen QUIET)
if (DOXYGEN_FOUND)
    message(STATUS "Doxygen found!")

    # Set input directories as a CMake list
    set(DOXYGEN_INPUT_DIRS
        "${CMAKE_CURRENT_SOURCE_DIR}/include/"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/"
    )

    # Convert to a space-separated string for Doxygen
    string(REPLACE ";" " " DOXYGEN_INPUT_DIRS_STR "${DOXYGEN_INPUT_DIRS}")

    set(DOXYGEN_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/docs")
    set(DOXYGEN_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")

    # Configure the Doxyfile with substituted variables
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in
        ${DOXYGEN_CONFIG_FILE}
        @ONLY
    )

    # Add custom doc target
    add_custom_target(doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_CONFIG_FILE}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
else()
    message(STATUS "Doxygen not found. Documentation target will not be available.")
endif()
