cmake_minimum_required(VERSION 3.9)


if (MSVC)
    # per default disable extended aligned storage for now on msvc
    add_definitions(-D_DISABLE_EXTENDED_ALIGNED_STORAGE -DHAVE_SNPRINTF)
endif()

add_executable(artekmed_init_pose artekmed_init_pose.cpp)
target_link_libraries(artekmed_init_pose PUBLIC
        ${CONAN_PACKAGE_NAME}
        )
#conan_target_link_libraries(artekmed_init_pose)


install(TARGETS artekmed_init_pose
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows

#-----------------------------------------------------

add_executable(artekmed_target_tracking artekmed_target_tracking.cpp)

#conan_target_link_libraries(artekmed_target_tracking)

target_link_libraries(artekmed_target_tracking PUBLIC
        ${CONAN_PACKAGE_NAME}
        )

install(TARGETS artekmed_target_tracking
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows

#-----------------------------------------------------

add_executable(artekmed_bundleadjustment artekmed_bundleadjustment.cpp)

#conan_target_link_libraries(artekmed_target_tracking)

target_link_libraries(artekmed_bundleadjustment PUBLIC
        ${CONAN_PACKAGE_NAME}
        )

install(TARGETS artekmed_bundleadjustment
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows

#-----------------------------------------------------

add_executable(artekmed_kinect_registration artekmed_kinect_registration.cpp)

#conan_target_link_libraries(artekmed_target_tracking)

target_link_libraries(artekmed_kinect_registration PUBLIC
        ${CONAN_PACKAGE_NAME}
        )

install(TARGETS artekmed_kinect_registration
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows

