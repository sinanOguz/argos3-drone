#
# Find ARGoS
#
find_package(ARGoS REQUIRED)
include_directories(${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})

#
# Find the AprilTag library
#
find_package(AprilTag)
if(NOT APRILTAG_FOUND)
  message(FATAL_ERROR "Required library AprilTag not found.")
endif(NOT APRILTAG_FOUND)
include_directories(${APRILTAG_INCLUDE_DIR})

#
# Find Analog Device's IIO library
#
find_package(IIO)
if(NOT IIO_FOUND)
  message(FATAL_ERROR "Required library IIO not found.")
endif(NOT IIO_FOUND)
include_directories(${IIO_INCLUDE_DIR})

#
# Find the Turbo JPEG library
#
find_package(TurboJPEG)
if(NOT TURBOJPEG_FOUND)
  message(FATAL_ERROR "Required Turbo JPEG library not found.")
endif(NOT TURBOJPEG_FOUND)
include_directories(${TURBOJPEG_INCLUDE_DIR})  
