cmake_minimum_required(VERSION 3.16)

include(GenerateExportHeader)

option(WITH_TESTS "Build Tests" ON)
option(WITH_COVERAGE "" ON)

set(CMAKE_CXX_STANDARD 17)

# public header
set(LIB_HEADERS include/traact/ba/BundleAdjustment.cpp include/traact/ba/BundleAdjustment.h
        include/traact/ba/BACamera.cpp include/traact/ba/BACamera.h
        include/traact/ba/cost_function/CostFunctionUtils.h
        include/traact/ba/cost_function/CeresTargetNPointReprojectionError.h
        include/traact/ba/cost_function/CeresRefNPointReprojectionError.h)

# rest of source code
set(LIB_SOURCES
        include/traact/ba/BATarget.cpp include/traact/ba/BATarget.h include/traact/ba/BAProblemLoader.cpp include/traact/ba/BAProblemLoader.h include/traact/ba/BAData.cpp include/traact/ba/BAData.h include/traact/ba/cost_function/CostFunctionUtils.h include/traact/ba/cost_function/CeresRefTargetNPointLengthError.h include/traact/util/YAMLUtil.cpp include/traact/util/YAMLUtil.h)


# conan is required hard, library name, version and description are configured from conan
if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
    include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
else ()
    message(FATAL_ERROR "The file conanbuildinfo.cmake doesn't exist, you have to run conan install first")
endif ()

project(${CONAN_PACKAGE_NAME}_PROJECT VERSION ${CONAN_PACKAGE_VERSION})

conan_basic_setup(TARGETS)

if (WITH_COVERAGE)
    #add_compile_options(-g -O0 -Wall -W -Wshadow -Wunused-variable -Wunused-parameter -Wunused-function -Wunused -Wno-system-headers -Wno-deprecated -Woverloaded-virtual -Wwrite-strings -fprofile-arcs -ftest-coverage)
    add_link_options(-fprofile-arcs -ftest-coverage)
endif ()

if(CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DTBB_USE_DEBUG=1)
endif ()

add_definitions(-DTBB_PREVIEW_FLOW_GRAPH_FEATURES=1)
add_definitions(-DTBB_PREVIEW_FLOW_GRAPH_NODES=1)

# conan workspace and cmake don't work that well together, did not manage to get the build dir into the includes: 
# force generated file into source include dir 
# only one export file could be generated
# use files and defines instead
#set(CMAKE_CXX_VISIBILITY_PRESET hidden)
#set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)
#generate_export_header(${CONAN_PACKAGE_NAME} EXPORT_FILE_NAME "../../include/traact/${CONAN_PACKAGE_NAME}_export.h")

if(UNIX)
    if(APPLE)
        MESSAGE(STATUS "Building for Macos.")
        add_definitions(-DTRAACT_TARGET_APPLE=1)
    endif()
    MESSAGE(STATUS "Building for Unix.")
    add_definitions(-DTRAACT_TARGET_UNIX=1)
elseif(WIN32)
    MESSAGE(STATUS "Building for Windows.")
    add_definitions(-DTRAACT_TARGET_WINDOWS=1)
endif()

if (MSVC)
    # per default disable extended aligned storage for now on msvc
    add_definitions(-D_DISABLE_EXTENDED_ALIGNED_STORAGE -DHAVE_SNPRINTF)
endif()

add_library(${CONAN_PACKAGE_NAME} SHARED
        ${LIB_HEADERS}
        ${LIB_SOURCES}
        )


conan_target_link_libraries(${CONAN_PACKAGE_NAME})

set_target_properties(${CONAN_PACKAGE_NAME} PROPERTIES VERSION ${CONAN_PACKAGE_VERSION})
#set_target_properties(${CONAN_PACKAGE_NAME} PROPERTIES PUBLIC_HEADER "${LIB_HEADERS}")


target_include_directories(${CONAN_PACKAGE_NAME} PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${CONAN_INCLUDE_DIRS_NLOHMANN_JSON}
        )
target_include_directories(${CONAN_PACKAGE_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        )

install(TARGETS ${CONAN_PACKAGE_NAME}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})


install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/traact" # source directory
        DESTINATION "include" # target directory
        FILES_MATCHING # install only matched files
        PATTERN "*.h" # select header files
        )

#install(FILES "${PROJECT_BINARY_DIR}/traact_export.h" DESTINATION "include")

add_subdirectory(apps)

if (WITH_TESTS)
    add_subdirectory(tests)
endif ()

