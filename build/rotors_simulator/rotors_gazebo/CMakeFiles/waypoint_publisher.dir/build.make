# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jose/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jose/catkin_ws/build

# Include any dependencies generated for this target.
include rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/depend.make

# Include the progress variables for this target.
include rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/progress.make

# Include the compile flags for this target's objects.
include rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/flags.make

rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/src/waypoint_publisher.cpp.o: rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/flags.make
rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/src/waypoint_publisher.cpp.o: /home/jose/catkin_ws/src/rotors_simulator/rotors_gazebo/src/waypoint_publisher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jose/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/src/waypoint_publisher.cpp.o"
	cd /home/jose/catkin_ws/build/rotors_simulator/rotors_gazebo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/waypoint_publisher.dir/src/waypoint_publisher.cpp.o -c /home/jose/catkin_ws/src/rotors_simulator/rotors_gazebo/src/waypoint_publisher.cpp

rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/src/waypoint_publisher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/waypoint_publisher.dir/src/waypoint_publisher.cpp.i"
	cd /home/jose/catkin_ws/build/rotors_simulator/rotors_gazebo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jose/catkin_ws/src/rotors_simulator/rotors_gazebo/src/waypoint_publisher.cpp > CMakeFiles/waypoint_publisher.dir/src/waypoint_publisher.cpp.i

rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/src/waypoint_publisher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/waypoint_publisher.dir/src/waypoint_publisher.cpp.s"
	cd /home/jose/catkin_ws/build/rotors_simulator/rotors_gazebo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jose/catkin_ws/src/rotors_simulator/rotors_gazebo/src/waypoint_publisher.cpp -o CMakeFiles/waypoint_publisher.dir/src/waypoint_publisher.cpp.s

# Object files for target waypoint_publisher
waypoint_publisher_OBJECTS = \
"CMakeFiles/waypoint_publisher.dir/src/waypoint_publisher.cpp.o"

# External object files for target waypoint_publisher
waypoint_publisher_EXTERNAL_OBJECTS =

/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/src/waypoint_publisher.cpp.o
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/build.make
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /opt/ros/noetic/lib/libroscpp.so
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /opt/ros/noetic/lib/librosconsole.so
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /opt/ros/noetic/lib/libeigen_conversions.so
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /usr/lib/liborocos-kdl.so
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /opt/ros/noetic/lib/librostime.so
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /opt/ros/noetic/lib/libcpp_common.so
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher: rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jose/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher"
	cd /home/jose/catkin_ws/build/rotors_simulator/rotors_gazebo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/waypoint_publisher.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/build: /home/jose/catkin_ws/devel/lib/rotors_gazebo/waypoint_publisher

.PHONY : rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/build

rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/clean:
	cd /home/jose/catkin_ws/build/rotors_simulator/rotors_gazebo && $(CMAKE_COMMAND) -P CMakeFiles/waypoint_publisher.dir/cmake_clean.cmake
.PHONY : rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/clean

rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/depend:
	cd /home/jose/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jose/catkin_ws/src /home/jose/catkin_ws/src/rotors_simulator/rotors_gazebo /home/jose/catkin_ws/build /home/jose/catkin_ws/build/rotors_simulator/rotors_gazebo /home/jose/catkin_ws/build/rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : rotors_simulator/rotors_gazebo/CMakeFiles/waypoint_publisher.dir/depend

