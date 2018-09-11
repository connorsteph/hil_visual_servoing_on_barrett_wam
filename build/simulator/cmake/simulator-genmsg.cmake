# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "simulator: 1 messages, 0 services")

set(MSG_I_FLAGS "-Isimulator:/home/cjs/ros_workspaces/wam_sim/src/simulator/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(simulator_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cjs/ros_workspaces/wam_sim/src/simulator/msg/Teleop.msg" NAME_WE)
add_custom_target(_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "simulator" "/home/cjs/ros_workspaces/wam_sim/src/simulator/msg/Teleop.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(simulator
  "/home/cjs/ros_workspaces/wam_sim/src/simulator/msg/Teleop.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simulator
)

### Generating Services

### Generating Module File
_generate_module_cpp(simulator
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simulator
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(simulator_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(simulator_generate_messages simulator_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cjs/ros_workspaces/wam_sim/src/simulator/msg/Teleop.msg" NAME_WE)
add_dependencies(simulator_generate_messages_cpp _simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(simulator_gencpp)
add_dependencies(simulator_gencpp simulator_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS simulator_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(simulator
  "/home/cjs/ros_workspaces/wam_sim/src/simulator/msg/Teleop.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/simulator
)

### Generating Services

### Generating Module File
_generate_module_eus(simulator
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/simulator
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(simulator_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(simulator_generate_messages simulator_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cjs/ros_workspaces/wam_sim/src/simulator/msg/Teleop.msg" NAME_WE)
add_dependencies(simulator_generate_messages_eus _simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(simulator_geneus)
add_dependencies(simulator_geneus simulator_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS simulator_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(simulator
  "/home/cjs/ros_workspaces/wam_sim/src/simulator/msg/Teleop.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/simulator
)

### Generating Services

### Generating Module File
_generate_module_lisp(simulator
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/simulator
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(simulator_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(simulator_generate_messages simulator_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cjs/ros_workspaces/wam_sim/src/simulator/msg/Teleop.msg" NAME_WE)
add_dependencies(simulator_generate_messages_lisp _simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(simulator_genlisp)
add_dependencies(simulator_genlisp simulator_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS simulator_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(simulator
  "/home/cjs/ros_workspaces/wam_sim/src/simulator/msg/Teleop.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/simulator
)

### Generating Services

### Generating Module File
_generate_module_nodejs(simulator
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/simulator
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(simulator_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(simulator_generate_messages simulator_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cjs/ros_workspaces/wam_sim/src/simulator/msg/Teleop.msg" NAME_WE)
add_dependencies(simulator_generate_messages_nodejs _simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(simulator_gennodejs)
add_dependencies(simulator_gennodejs simulator_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS simulator_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(simulator
  "/home/cjs/ros_workspaces/wam_sim/src/simulator/msg/Teleop.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simulator
)

### Generating Services

### Generating Module File
_generate_module_py(simulator
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simulator
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(simulator_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(simulator_generate_messages simulator_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cjs/ros_workspaces/wam_sim/src/simulator/msg/Teleop.msg" NAME_WE)
add_dependencies(simulator_generate_messages_py _simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(simulator_genpy)
add_dependencies(simulator_genpy simulator_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS simulator_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simulator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simulator
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/simulator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/simulator
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/simulator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/simulator
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/simulator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/simulator
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simulator)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simulator\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simulator
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
