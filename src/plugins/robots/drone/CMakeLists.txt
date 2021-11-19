#
# Drone headers
#
# argos3/plugins/robots/drone/hardware
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_DRONE_HARDWARE
  hardware/actuator.h
  hardware/drone_cameras_system_default_sensor.h
  hardware/drone_flight_system_default_actuator.h
  hardware/drone_flight_system_default_sensor.h
  hardware/drone_leds_default_actuator.h
  hardware/drone_rangefinders_default_sensor.h
  hardware/drone_system_default_sensor.h
  hardware/pixhawk.h
  hardware/radios_default_actuator.h
  hardware/radios_default_sensor.h
  hardware/robot.h
  hardware/sensor.h)

#
# Drone sources
#
# argos3/plugins/robots/drone/hardware
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_DRONE_HARDWARE
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_DRONE_HARDWARE}
  hardware/drone_cameras_system_default_sensor.cpp
  hardware/drone_flight_system_default_actuator.cpp
  hardware/drone_flight_system_default_sensor.cpp
  hardware/drone_leds_default_actuator.cpp
  hardware/drone_rangefinders_default_sensor.cpp
  hardware/drone_system_default_sensor.cpp
  hardware/main.cpp
  hardware/pixhawk.cpp
  hardware/radios_default_actuator.cpp
  hardware/radios_default_sensor.cpp  
  hardware/robot.cpp)

# Create an executable for the robot
add_executable(argos3 ${ARGOS3_SOURCES_PLUGINS_ROBOTS_DRONE_HARDWARE})
target_link_libraries(argos3 tbb
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
  argos3plugin_${ARGOS_BUILD_FOR}_drone
  ${IIO_LIBRARY}
  ${APRILTAG_LIBRARY}
  ${TURBOJPEG_LIBRARY})

# Install executable and header files
install(TARGETS argos3
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_DRONE_HARDWARE} DESTINATION include/argos3/plugins/robots/drone/hardware)
