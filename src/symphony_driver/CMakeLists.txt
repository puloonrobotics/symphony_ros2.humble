cmake_minimum_required(VERSION 3.10)
project(symphony_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(symphony_msgs REQUIRED)

# System Puloon RTPRI SDK (find_package); not an ament package.
# Custom install: add its prefix to CMAKE_PREFIX_PATH.
find_package(puloon_rtpri REQUIRED)

# 1.3.0+: RT client id, pause/resume, control authority (M).
set(_symphony_rtpri_has_1_3 FALSE)
if(DEFINED puloon_rtpri_VERSION AND puloon_rtpri_VERSION VERSION_GREATER_EQUAL "1.3.0")
  set(_symphony_rtpri_has_1_3 TRUE)
endif()
if(_symphony_rtpri_has_1_3)
  message(STATUS "puloon_rtpri ${puloon_rtpri_VERSION}: enabling 1.3 APIs (authority/pause/id)")
else()
  message(STATUS "puloon_rtpri ${puloon_rtpri_VERSION}: 1.2-compatible build (1.3 APIs disabled)")
endif()

add_library(symphony_driver SHARED
  src/hardware_interface.cpp
  src/motion_mode_switch.cpp
)
if(_symphony_rtpri_has_1_3)
  target_compile_definitions(symphony_driver PRIVATE SYMPHONY_RTPRI_HAS_1_3=1)
endif()

target_include_directories(symphony_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(symphony_driver
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  symphony_msgs
)

if(TARGET puloon_rtpri::rtpri)
  target_link_libraries(symphony_driver puloon_rtpri::rtpri)
  set(_rtpri_tgt puloon_rtpri::rtpri)
elseif(TARGET rtpri::rtpri)
  target_link_libraries(symphony_driver rtpri::rtpri)
  set(_rtpri_tgt rtpri::rtpri)
else()
  message(FATAL_ERROR
    "puloon_rtpri found, but neither puloon_rtpri::rtpri nor rtpri::rtpri target exists")
endif()

# Non-ament RTPRI lib dir → RPATH so pluginlib can dlopen librtpri.
set(_rtpri_loc "")
get_target_property(_rtpri_loc ${_rtpri_tgt} IMPORTED_LOCATION)
if(NOT _rtpri_loc)
  get_target_property(_rtpri_loc ${_rtpri_tgt} IMPORTED_LOCATION_RELEASE)
endif()
if(NOT _rtpri_loc)
  get_target_property(_rtpri_cfgs ${_rtpri_tgt} IMPORTED_CONFIGURATIONS)
  if(_rtpri_cfgs)
    list(GET _rtpri_cfgs 0 _rtpri_cfg)
    get_target_property(_rtpri_loc ${_rtpri_tgt} IMPORTED_LOCATION_${_rtpri_cfg})
  endif()
endif()
if(_rtpri_loc)
  get_filename_component(_rtpri_libdir "${_rtpri_loc}" DIRECTORY)
  set_target_properties(symphony_driver PROPERTIES
    BUILD_RPATH "${_rtpri_libdir}"
    INSTALL_RPATH "${_rtpri_libdir}"
  )
  set(RTPRI_LIBDIR "${_rtpri_libdir}")
  set(_rtpri_hook_in [=[
if [ -n "${LD_LIBRARY_PATH}" ]; then
  case ":${LD_LIBRARY_PATH}:" in
    *":@RTPRI_LIBDIR@:"*) ;;
    *) export LD_LIBRARY_PATH="@RTPRI_LIBDIR@:${LD_LIBRARY_PATH}" ;;
  esac
else
  export LD_LIBRARY_PATH="@RTPRI_LIBDIR@"
fi
]=])
  string(CONFIGURE "${_rtpri_hook_in}" _rtpri_hook_out @ONLY)
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/rtpri_library_path.sh" "${_rtpri_hook_out}")
  ament_environment_hooks(${CMAKE_CURRENT_BINARY_DIR}/rtpri_library_path.sh)
endif()

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/gz_ip.sh" [=[
if [ -z "${GZ_IP-}" ]; then
  export GZ_IP=127.0.0.1
fi
]=])
ament_environment_hooks(${CMAKE_CURRENT_BINARY_DIR}/gz_ip.sh)

pluginlib_export_plugin_description_file(hardware_interface symphony_driver.xml)

install(TARGETS symphony_driver
  EXPORT export_symphony_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Examples need +x; Windows copies are often 644 and launch_ros skips them.
set(_symphony_driver_examples
  examples/movep_example.py
  examples/movel_example.py
  examples/servoq_example.py
  examples/servox_example.py
  examples/servo_relative_example.py
  examples/servoq_abs_rel_example.py
  examples/servox_3dmouse_example.py
  examples/jtc_example.py
  examples/native_jtc_example.py
  examples/speedq_example.py
  examples/tool_modbus_2fg7_example.py
)
foreach(_ex ${_symphony_driver_examples})
  file(CHMOD "${CMAKE_CURRENT_SOURCE_DIR}/${_ex}"
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
endforeach()

install(PROGRAMS
  ${_symphony_driver_examples}
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  examples/example_common.py
  DESTINATION lib/${PROJECT_NAME}
)

install(CODE "
  file(GLOB _installed_ex \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/*.py\")
  foreach(_f \${_installed_ex})
    if(EXISTS \"\${_f}\" AND NOT \"\${_f}\" MATCHES \"example_common\\\\.py$\")
      file(CHMOD \"\${_f}\"
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                    GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
    endif()
  endforeach()
")

install(FILES symphony_driver.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_symphony_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  symphony_msgs
)

ament_package()
