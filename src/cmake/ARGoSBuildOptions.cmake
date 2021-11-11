#
# What is ARGoS being built for?
# Accepted values: "simulator" or a robot name (lowercase)
#
set(ARGOS_BUILD_FOR "drone")
set(ARGOS_BUILD_FOR_SIMULATOR FALSE)

#
# Optimize code for current platform?
#
if(NOT DEFINED ARGOS_BUILD_NATIVE)
  option(ARGOS_BUILD_NATIVE "ON -> compile with platform-specific optimizations, OFF -> compile to portable binary" OFF)
endif(NOT DEFINED ARGOS_BUILD_NATIVE)
