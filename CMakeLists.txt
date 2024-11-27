# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(hobot_audio)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(audio_msg REQUIRED)
find_package(std_msgs REQUIRED)

set(PREFIX_PATH x3)

if(PLATFORM_X3)
  message("build platform X3")
  add_definitions(-DPLATFORM_X3)
  set(PREFIX_PATH x3)
elseif(PLATFORM_X5)
  message("build platform X5")
  add_definitions(-DPLATFORM_X5)
  set(PREFIX_PATH x5)
  set(HRSC_PATH "${PROJECT_SOURCE_DIR}/include/${PREFIX_PATH}/horizon_speech_sdk/")
  set(HRSC_PAT_2 "${PROJECT_SOURCE_DIR}/include/${PREFIX_PATH}/horizon_speech_sdk/hrsc")
  set(HRSC_FILE "${HRSC_PATH}/hrsc.tar.gz")
  if(NOT EXISTS ${HRSC_PAT_2})
    execute_process(
      COMMAND wget "https://archive.d-robotics.cc/models/X5_speech/hrsc.tar.gz" -O ${HRSC_FILE}
      RESULT_VARIABLE WGET_RESULT
    )
    if(WGET_RESULT EQUAL 0)
      execute_process(
        COMMAND tar xzvf ${HRSC_FILE} -C ${HRSC_PATH}
        RESULT_VARIABLE WGET_RESULT
      )
      execute_process(
        COMMAND rm -f ${HRSC_FILE}
        RESULT_VARIABLE WGET_RESULT
      )
    else()
        message(FATAL_ERROR "Failed to download file with wget.")
    endif()
  endif()
else()
  message("invalid platform, build platform X3 default")
  add_definitions(-DPLATFORM_X3)
  set(PREFIX_PATH x3)
endif()

message("PREFIX_PATH: ${PREFIX_PATH}")

include_directories(include
${PROJECT_SOURCE_DIR}
)

set(SYS_ROOT ${CMAKE_SYSROOT})
message("SYS_ROOT is " ${SYS_ROOT})
include_directories(
  ${SYS_ROOT}/usr/include/
  ${PROJECT_SOURCE_DIR}/include/${PREFIX_PATH}/horizon_speech_sdk/include
)

link_directories(
  ${PROJECT_SOURCE_DIR}/include/${PREFIX_PATH}/horizon_speech_sdk/lib
  ${SYS_ROOT}/usr/lib/
  ${SYS_ROOT}/usr/lib/hbmedia/
  ${SYS_ROOT}/usr/lib/hbbpu
  ${SYS_ROOT}/usr/lib/hobot/
  ${SYS_ROOT}/usr/hobot/lib/
  ${SYS_ROOT}/usr/lib/hobot/hobotlog
)

add_executable(hobot_audio
  src/utils/alsa_device.cpp
  src/audio_capture/hb_audio_capture.cpp
  src/audio_engine/audioengine.cpp
  src/audio_example.cpp
)

set(BASE_LIBRARIES pthread dl rt m)
if(PLATFORM_X3)
  set(X3_HAPI_LIB hbmedia cam isp vio multimedia iar)
  set(X3_AUDIO_LIB asound cnn_intf ion hrsc lvcsr modelvad)
elseif(PLATFORM_X5)
  set(X3_HAPI_LIB alog hbmem)
  set(X3_AUDIO_LIB asound cnn_intf dnn hrsc lvcsr fst)
else()
  set(X3_HAPI_LIB hbmedia cam isp vio multimedia iar)
  set(X3_AUDIO_LIB asound cnn_intf ion hrsc lvcsr modelvad)
endif()

target_link_libraries(${PROJECT_NAME}
   ${X3_AUDIO_LIB} ${X3_HAPI_LIB} ${BASE_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
"rclcpp"
"geometry_msgs"
"audio_msg")

ament_target_dependencies(${PROJECT_NAME} audio_msg)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Install libraries
install(DIRECTORY include/${PREFIX_PATH}/horizon_speech_sdk/lib/
DESTINATION lib/)

# Install hrsc
install(DIRECTORY include/${PREFIX_PATH}/horizon_speech_sdk/hrsc
DESTINATION lib/hobot_audio/)

# Install libraries
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/hobot_audio/)

# Install include directories
install(
  DIRECTORY config
  DESTINATION lib/hobot_audio/
)

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "RCLCPP_BUILDING_LIBRARY")

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

ament_package()
