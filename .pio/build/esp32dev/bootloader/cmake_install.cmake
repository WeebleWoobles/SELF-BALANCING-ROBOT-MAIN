# Install script for directory: C:/Users/arien/.platformio/packages/framework-espidf/components/bootloader/subproject

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/bootloader")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
<<<<<<< HEAD
  include("C:/Users/arien/OneDrive/Documents/GitHub/ecen361/Self-Balancing_Robot/SELF-BALANCING-ROBOT-MAIN/.pio/build/esp32dev/bootloader/esp-idf/cmake_install.cmake")
=======
  include("C:/Users/hiram/Git/SELF-BALANCING-ROBOT-MAIN/.pio/build/esp32dev/bootloader/esp-idf/cmake_install.cmake")
>>>>>>> c645504f9131a63544b09a08289dc44eb6c51ed1
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
<<<<<<< HEAD
file(WRITE "C:/Users/arien/OneDrive/Documents/GitHub/ecen361/Self-Balancing_Robot/SELF-BALANCING-ROBOT-MAIN/.pio/build/esp32dev/bootloader/${CMAKE_INSTALL_MANIFEST}"
=======
file(WRITE "C:/Users/hiram/Git/SELF-BALANCING-ROBOT-MAIN/.pio/build/esp32dev/bootloader/${CMAKE_INSTALL_MANIFEST}"
>>>>>>> c645504f9131a63544b09a08289dc44eb6c51ed1
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
