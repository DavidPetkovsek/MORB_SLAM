
######################################################################
# NEW KNEW 
######################################################################

set(TARGET_NAME MORB_VIEWER)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(realsense2 QUIET) # realsense only needed for example
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization regex)
find_package(Boost REQUIRED)
#find_package(ixwebsocket REQUIRED) # only needed for getting realsense data through a websocket

file(GLOB CPP_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc ${CMAKE_CURRENT_SOURCE_DIR}/src/*.ccp)
file(GLOB VIEWER_HEADERS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/include/MORB_SLAM/*.h ${CMAKE_CURRENT_SOURCE_DIR}/include/MORB_SLAM/*.hpp)

add_library(${TARGET_NAME} SHARED ${CPP_FILES})
add_library(${PROJECT_NAME}::${TARGET_NAME} ALIAS ${TARGET_NAME})
target_include_directories(${TARGET_NAME} PUBLIC
                       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # for headers when building
                       $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${TARGET_NAME}> # for client in install mode
                       )
target_link_libraries( ${TARGET_NAME} PUBLIC ${PROJECT_NAME}::DBoW2 )
target_link_libraries( ${TARGET_NAME} PUBLIC ${PROJECT_NAME}::g2o )
target_link_libraries( ${TARGET_NAME} PUBLIC ${PROJECT_NAME}::sophus )
target_link_libraries( ${TARGET_NAME} PUBLIC Threads::Threads )
target_link_libraries( ${TARGET_NAME} PUBLIC Eigen3::Eigen )
target_link_libraries( ${TARGET_NAME} PUBLIC Boost::serialization Boost::regex )
target_link_libraries( ${TARGET_NAME} PUBLIC ${OpenCV_LIBS} )
target_link_libraries( ${TARGET_NAME} PUBLIC OpenSSL::SSL)
target_link_libraries( ${TARGET_NAME} PUBLIC ${PROJECT_NAME}::${PROJECT_NAME} )
target_link_libraries( ${TARGET_NAME} PRIVATE ${Pangolin_LIBRARIES} )



# target_link_libraries( ${TARGET_NAME} PRIVATE OpenSSL::SSL )
#target_link_libraries( ${TARGET_NAME} PRIVATE ixwebsocket::ixwebsocket ) 


# https://cmake.org/cmake/help/latest/guide/importing-exporting/index.html#creating-a-package-version-file
# https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#creating-packages
# https://mail.kde.org/pipermail/kde-buildsystem/2008-April/004544.html
set_property(TARGET ${TARGET_NAME} PROPERTY VERSION ${MORB_VERSION}) # x.x.x used for symlinks
set_property(TARGET ${TARGET_NAME} PROPERTY SOVERSION ${MORB_VERSION_MAJOR}) # ABI/API compatibility at so level must be integer
set_property(TARGET ${TARGET_NAME} PROPERTY "INTERFACE_${TARGET_NAME}_MAJOR_VERSION" ${MORB_VERSION_MAJOR})
set_property(TARGET ${TARGET_NAME} APPEND PROPERTY COMPATIBLE_INTERFACE_STRING "${TARGET_NAME}_MAJOR_VERSION")



set(EXPORT_TARGET_NAME "${TARGET_NAME}Targets")
set(EXPORT_TARGET_FILE_NAME "${PROJECT_NAME}${EXPORT_TARGET_NAME}.cmake")


# install the target and create export-set
install(TARGETS ${TARGET_NAME}
        EXPORT ${EXPORT_TARGET_NAME}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# install header files
install(FILES ${VIEWER_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${TARGET_NAME}/${TARGET_NAME})



# generate and install export file
install(EXPORT ${EXPORT_TARGET_NAME}
        FILE ${EXPORT_TARGET_FILE_NAME}
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_PROJECT_CONFIG_INSTALL_DIR}
)
