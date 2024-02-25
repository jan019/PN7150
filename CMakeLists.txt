# v3.21 implemented semantic changes regarding $<TARGET_OBJECTS:...>
# See https://cmake.org/cmake/help/v3.21/command/target_link_libraries.html#linking-object-libraries-via-target-objects
cmake_minimum_required(VERSION 3.21)

add_library(PN7150 INTERFACE)
add_library(PN7150_usage INTERFACE)

target_include_directories(PN7150_usage INTERFACE
  src
)


target_link_libraries(PN7150_usage INTERFACE
  base_config
  Wire
)

target_link_libraries(PN7150 INTERFACE PN7150_usage)



add_library(PN7150_bin OBJECT EXCLUDE_FROM_ALL
  src/Mode.cpp
  src/ndef_helper.cpp
  src/NdefMessage.cpp
  src/NdefRecord.cpp
  src/P2P_NDEF.cpp
  src/PN7150.cpp
  src/RemoteDevice.cpp
  src/RW_NDEF.cpp
  src/RW_NDEF_MIFARE.cpp
  src/RW_NDEF_T1T.cpp
  src/RW_NDEF_T2T.cpp
  src/RW_NDEF_T3T.cpp
  src/RW_NDEF_T4T.cpp
  src/T4T_NDEF_emu.cpp
  src/tool.cpp
)
target_link_libraries(PN7150_bin PUBLIC PN7150_usage)

target_link_libraries(PN7150 INTERFACE
  PN7150_bin
  $<TARGET_OBJECTS:PN7150_bin>
)

