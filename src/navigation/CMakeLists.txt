cmake_minimum_required(VERSION 3.0.2)
project(navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_compile_options(-fdiagnostics-color)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  std_msgs
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Catch2 2.13 REQUIRED)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navigation
  CATKIN_DEPENDS pcl_ros roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(DIR_SRC src)
set(DIR_TEST_SRC test/src)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  # /usr/include/eigen3
  ${DIR_SRC}
  ${DIR_TEST_SRC}
)
# include_directories(${EIGEN3_INCLUDE_DIR})

# Target names
set(TARGET_LIB common)
set(TARGET_BIN ${PROJECT_NAME})
set(TARGET_TEST catchTests)

# Libraries
set(LIBS
    Eigen3::Eigen
)

# Set appropriate source directories


# Application
file(GLOB SRC_BIN CONFIGURE_DEPENDS ${DIR_SRC}/*.cpp ${DIR_SRC}/*.h ${DIR_SRC}/*.hpp)

#static Library
set(SRC_LIB ${SRC_BIN})
list(FILTER SRC_LIB EXCLUDE REGEX ".*spacialDualLocalisation.cpp$")

if(SRC_LIB)
    add_library(${TARGET_LIB} ${SRC_LIB})
endif()


add_executable(spacialDualLocalisation src/spacialDualLocalisation.cpp)
target_link_libraries(spacialDualLocalisation ${catkin_LIBRARIES})
if(SRC_LIB)
  target_link_libraries(spacialDualLocalisation ${TARGET_LIB})
endif()
target_link_libraries(spacialDualLocalisation ${LIBS})
add_dependencies(spacialDualLocalisation navigation_generate_messages_cpp)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/navigation.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/navigation_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_navigation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
## set(SRC_DIR $(foreach x, $(src), $(wildcard $(addprefix $(x)/testing.cpp))))
# set(SRC_DIR src/testing.cpp)
# set(custom_depends src/testing.cpp src/Eig.cpp)
# file(GLOB SRC_TEST CONFIGURE_DEPENDS ${DIR_SRC}/*.cpp ${DIR_SRC}/*.h ${DIR_SRC}/*.hpp ${DIR_TEST_SRC}/*.cpp ${DIR_TEST_SRC}/*.h ${DIR_TEST_SRC}/*.hpp)

# set(TEST_SRC_LIB ${SRC_TEST})
# list(FILTER TEST_SRC_LIB EXCLUDE REGEX ".*spacialDualLocalisation.cpp$")
# # list(FILTER TEST_LIB EXCLUDE REGEX ".*spacialDualLocalisation.cpp$")
# set(TEST_LIB testing)
# if(TEST_SRC_LIB)
#     add_library(${TEST_LIB} ${TEST_SRC_LIB})
# endif()
# # set(custom_depends ${SRC_TEST})
# # if (CATKIN_ENABLE_TESTING)
#   catkin_add_gtest(navTest tests/navTest.cpp)
#   target_link_libraries(navTest ${TEST_LIB})
#   target_link_libraries(navTest ${LIBS})
#   target_link_libraries(navTest ${catkin_LIBRARIES})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)  `

file(GLOB SRC_TEST CONFIGURE_DEPENDS ${DIR_TEST_SRC}/*.cpp ${DIR_TEST_SRC}/*.h ${DIR_TEST_SRC}/*.hpp)

add_executable(${TARGET_TEST} ${SRC_TEST})
if(SRC_LIB)
    target_link_libraries(${TARGET_TEST} ${TARGET_LIB})
endif()
target_link_libraries(${TARGET_TEST} ${LIBS})
target_link_libraries(${TARGET_TEST} Catch2::Catch2)

add_custom_target(run_testing ALL
    COMMAND ${TARGET_TEST} --use-colour yes
    DEPENDS ${TARGET_TEST}
    DEPENDS spacialDualLocalisation
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Running tests"
)
