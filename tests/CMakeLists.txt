cmake_minimum_required(VERSION 3.16)


if (MSVC)
    # per default disable extended aligned storage for now on msvc
    add_definitions(-D_DISABLE_EXTENDED_ALIGNED_STORAGE -DHAVE_SNPRINTF)
endif()

# public header
set(TEST_SOURCES
        test_main.cpp
        cost_function/Test_count3DDistances.cpp
        cost_function/Test_calculate3DDistances.cpp
        )

add_executable(bundleadjustment_tests ${TEST_SOURCES})

target_link_libraries(bundleadjustment_tests PUBLIC
        ${CONAN_PACKAGE_NAME}
        )

target_include_directories(bundleadjustment_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_BINARY_DIR}
        )

install(TARGETS core_tests
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
