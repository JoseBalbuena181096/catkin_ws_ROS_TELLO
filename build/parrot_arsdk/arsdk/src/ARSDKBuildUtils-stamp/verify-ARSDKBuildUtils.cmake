# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

if("/home/jose/catkin_ws/src/parrot_arsdk/sdk/arsdk_3_14_0_p1_stripped.tgz" STREQUAL "")
  message(FATAL_ERROR "LOCAL can't be empty")
endif()

if(NOT EXISTS "/home/jose/catkin_ws/src/parrot_arsdk/sdk/arsdk_3_14_0_p1_stripped.tgz")
  message(FATAL_ERROR "File not found: /home/jose/catkin_ws/src/parrot_arsdk/sdk/arsdk_3_14_0_p1_stripped.tgz")
endif()

if("MD5" STREQUAL "")
  message(WARNING "File will not be verified since no URL_HASH specified")
  return()
endif()

if("f73099c420dab7c1ad29a01b1df3ca3c" STREQUAL "")
  message(FATAL_ERROR "EXPECT_VALUE can't be empty")
endif()

message(STATUS "verifying file...
     file='/home/jose/catkin_ws/src/parrot_arsdk/sdk/arsdk_3_14_0_p1_stripped.tgz'")

file("MD5" "/home/jose/catkin_ws/src/parrot_arsdk/sdk/arsdk_3_14_0_p1_stripped.tgz" actual_value)

if(NOT "${actual_value}" STREQUAL "f73099c420dab7c1ad29a01b1df3ca3c")
  message(FATAL_ERROR "error: MD5 hash of
  /home/jose/catkin_ws/src/parrot_arsdk/sdk/arsdk_3_14_0_p1_stripped.tgz
does not match expected value
  expected: 'f73099c420dab7c1ad29a01b1df3ca3c'
    actual: '${actual_value}'
")
endif()

message(STATUS "verifying file... done")
