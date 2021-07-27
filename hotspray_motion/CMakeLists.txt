cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
#find_package(ros_industrial_cmake_boilerplate REQUIRED)
#extract_package_metadata(pkg)
#project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

project(hotspray_motion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tesseract_monitoring
  tesseract_rosutils
  tesseract_planning_server
  pcl_conversions
)

find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(descartes_light REQUIRED)
# find_package(descartes_samplers REQUIRED)
find_package(ompl REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_process_managers REQUIRED)
find_package(tesseract_motion_planners REQUIRED)

find_package(trajopt REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(trajopt_sco REQUIRED)
find_package(trajopt_sqp REQUIRED)
find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(Boost REQUIRED)

catkin_package()
tesseract_variables()


#if(NOT TARGET console_bridge::console_bridge)
#  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
#  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${console_bridge_INCLUDE_DIRS})
#  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES ${console_bridge_LIBRARIES})
#else()
#  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
#  if (NOT ${CHECK_INCLUDE_DIRECTORIES})
#    set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${console_bridge_INCLUDE_DIRS})
#  endif()
#endif()

#link_directories(BEFORE ${OMPL_LIBRARY_DIRS})

#initialize_code_coverage()
#set(COVERAGE_EXCLUDE /usr/* /opt/* ${CMAKE_CURRENT_LIST_DIR}/test/* /*/gtest/* /*/include/descartes_light/* /*/include/opw_kinematics/* /*/include/trajopt/* /*/include/trajopt_sco/*)
#add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})

# Load variable for clang tidy args, compiler options and cxx version
#tesseract_variables()


# Descartes Planner


add_executable(${PROJECT_NAME}_node src/hotspray_motion_server.cpp src/hotspray_motion_node.cpp)


target_link_libraries(${PROJECT_NAME}_node Eigen3::Eigen  ${catkin_INCLUDE_DIRS} ${catkin_LIBRARIES} tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_command_language tesseract::tesseract_support tesseract::tesseract_process_managers   ${CMAKE_THREAD_LIBS_INIT})

target_compile_options(${PROJECT_NAME}_node PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_node PUBLIC VERSION 17)

target_include_directories(${PROJECT_NAME}_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include"
    "$<INSTALL_INTERFACE:include>"
 )
 target_include_directories(${PROJECT_NAME}_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
)

#target_compile_options(${PROJECT_NAME}_node PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE} ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
#target_compile_definitions(${PROJECT_NAME}_node PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
#target_clang_tidy(${PROJECT_NAME}_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
#target_cxx_version(${PROJECT_NAME}_node PRIVATE VERSION ${TESSERACT_CXX_VERSION})



#list(APPEND Examples ${PROJECT_NAME}_node)


# configure_package(NAMESPACE hotspray_motion
#                   TARGETS ${PROJECT_NAME}_node
#                   )

# Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}
#   DESTINATION include
#   PATTERN ".svn" EXCLUDE
# )

# add_subdirectory(examples)

# if (TESSERACT_ENABLE_TESTING)
#   enable_testing()
#   add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
#   add_subdirectory(test)
# endif()

install(
  DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

install(
  TARGETS
    ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)