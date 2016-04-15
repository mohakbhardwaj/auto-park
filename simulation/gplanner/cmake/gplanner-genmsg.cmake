# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(FATAL_ERROR "Could not find messages which '/home/shivam/catkin_ws/src/gplanner/srv/OptimalSpotGenerator.srv' depends on. Did you forget to specify generate_messages(DEPENDENCIES ...)?
Cannot locate message [int]: unknown package [gplanner] on search path [{'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}]")
message(STATUS "gplanner: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(gplanner_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/shivam/catkin_ws/src/gplanner/srv/SpotsTreadCost.srv" NAME_WE)
add_custom_target(_gplanner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gplanner" "/home/shivam/catkin_ws/src/gplanner/srv/SpotsTreadCost.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(gplanner
  "/home/shivam/catkin_ws/src/gplanner/srv/SpotsTreadCost.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gplanner
)

### Generating Module File
_generate_module_cpp(gplanner
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gplanner
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(gplanner_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(gplanner_generate_messages gplanner_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/shivam/catkin_ws/src/gplanner/srv/SpotsTreadCost.srv" NAME_WE)
add_dependencies(gplanner_generate_messages_cpp _gplanner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gplanner_gencpp)
add_dependencies(gplanner_gencpp gplanner_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gplanner_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(gplanner
  "/home/shivam/catkin_ws/src/gplanner/srv/SpotsTreadCost.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gplanner
)

### Generating Module File
_generate_module_lisp(gplanner
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gplanner
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(gplanner_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(gplanner_generate_messages gplanner_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/shivam/catkin_ws/src/gplanner/srv/SpotsTreadCost.srv" NAME_WE)
add_dependencies(gplanner_generate_messages_lisp _gplanner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gplanner_genlisp)
add_dependencies(gplanner_genlisp gplanner_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gplanner_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(gplanner
  "/home/shivam/catkin_ws/src/gplanner/srv/SpotsTreadCost.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gplanner
)

### Generating Module File
_generate_module_py(gplanner
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gplanner
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(gplanner_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(gplanner_generate_messages gplanner_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/shivam/catkin_ws/src/gplanner/srv/SpotsTreadCost.srv" NAME_WE)
add_dependencies(gplanner_generate_messages_py _gplanner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gplanner_genpy)
add_dependencies(gplanner_genpy gplanner_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gplanner_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gplanner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gplanner
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(gplanner_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(gplanner_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gplanner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gplanner
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(gplanner_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(gplanner_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gplanner)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gplanner\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gplanner
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(gplanner_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(gplanner_generate_messages_py geometry_msgs_generate_messages_py)
