#
# Drone headers
#
# argos3/plugins/robots/drone/control_interface
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_DRONE_CONTROLINTERFACE
  control_interface/ci_drone_cameras_system_sensor.h
  control_interface/ci_drone_flight_system_actuator.h
  control_interface/ci_drone_flight_system_sensor.h
  control_interface/ci_drone_leds_actuator.h
  control_interface/ci_drone_rangefinders_sensor.h
  control_interface/ci_drone_system_sensor.h)
# argos3/plugins/robots/drone/hardware
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_DRONE_HARDWARE
  hardware/drone.h
  hardware/drone_cameras_system_default_sensor.h
  hardware/drone_flight_system_default_actuator.h
  hardware/drone_flight_system_default_sensor.h
#  hardware/drone_leds_default_actuator.h
  hardware/drone_rangefinders_default_sensor.h
  hardware/drone_system_default_sensor.h)

#
# Drone sources
#
# argos3/plugins/robots/drone/control_interface
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_DRONE
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_DRONE_CONTROLINTERFACE}
  control_interface/ci_drone_cameras_system_sensor.cpp
  control_interface/ci_drone_flight_system_actuator.cpp
  control_interface/ci_drone_flight_system_sensor.cpp
  control_interface/ci_drone_leds_actuator.cpp
  control_interface/ci_drone_rangefinders_sensor.cpp
  control_interface/ci_drone_system_sensor.cpp)
# argos3/plugins/robots/drone/hardware
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_DRONE
  ${ARGOS3_SOURCES_PLUGINS_ROBOTS_DRONE}
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_DRONE_HARDWARE}
  hardware/drone.cpp
  hardware/drone_cameras_system_default_sensor.cpp
  hardware/drone_flight_system_default_actuator.cpp
  hardware/drone_flight_system_default_sensor.cpp
  hardware/drone_leds_default_actuator.cpp
  hardware/drone_rangefinders_default_sensor.cpp
  hardware/drone_system_default_sensor.cpp
  hardware/main.cpp)

# Create an executable for the robot
add_executable(argos3 ${ARGOS3_SOURCES_PLUGINS_ROBOTS_DRONE})
target_link_libraries(argos3 tbb
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
  argos3plugin_${ARGOS_BUILD_FOR}_srocs_genericrobot
  ${IIO_LIBRARY}
  ${APRILTAG_LIBRARY}
  ${TURBOJPEG_LIBRARY})
install(TARGETS argos3
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)

install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_DRONE_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/drone/control_interface)
install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_DRONE_HARDWARE} DESTINATION include/argos3/plugins/robots/drone/hardware)
