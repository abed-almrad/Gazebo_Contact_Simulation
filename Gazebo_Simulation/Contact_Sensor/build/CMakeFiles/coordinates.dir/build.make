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
CMAKE_SOURCE_DIR = /home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/Gazebo_Simulation/Contact_Sensor

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/Gazebo_Simulation/Contact_Sensor/build

# Include any dependencies generated for this target.
include CMakeFiles/coordinates.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/coordinates.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/coordinates.dir/flags.make

CMakeFiles/coordinates.dir/CoordinatesPlugin.cc.o: CMakeFiles/coordinates.dir/flags.make
CMakeFiles/coordinates.dir/CoordinatesPlugin.cc.o: ../CoordinatesPlugin.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/Gazebo_Simulation/Contact_Sensor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/coordinates.dir/CoordinatesPlugin.cc.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/coordinates.dir/CoordinatesPlugin.cc.o -c /home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/Gazebo_Simulation/Contact_Sensor/CoordinatesPlugin.cc

CMakeFiles/coordinates.dir/CoordinatesPlugin.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/coordinates.dir/CoordinatesPlugin.cc.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/Gazebo_Simulation/Contact_Sensor/CoordinatesPlugin.cc > CMakeFiles/coordinates.dir/CoordinatesPlugin.cc.i

CMakeFiles/coordinates.dir/CoordinatesPlugin.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/coordinates.dir/CoordinatesPlugin.cc.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/Gazebo_Simulation/Contact_Sensor/CoordinatesPlugin.cc -o CMakeFiles/coordinates.dir/CoordinatesPlugin.cc.s

# Object files for target coordinates
coordinates_OBJECTS = \
"CMakeFiles/coordinates.dir/CoordinatesPlugin.cc.o"

# External object files for target coordinates
coordinates_EXTERNAL_OBJECTS =

libcoordinates.so: CMakeFiles/coordinates.dir/CoordinatesPlugin.cc.o
libcoordinates.so: CMakeFiles/coordinates.dir/build.make
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so.3.6
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libdart.so.6.9.2
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libsdformat9.so.9.7.0
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libignition-common3-graphics.so.3.14.0
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so.3.6
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so.3.6
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libblas.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/liblapack.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libblas.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/liblapack.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libdart-external-odelcpsolver.so.6.9.2
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libccd.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libfcl.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libassimp.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/liboctomap.so.1.9.3
libcoordinates.so: /usr/lib/x86_64-linux-gnu/liboctomath.so.1.9.3
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libignition-transport8.so.8.2.1
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libignition-fuel_tools4.so.4.4.0
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libignition-msgs5.so.5.8.1
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libignition-math6.so.6.9.2
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libignition-common3.so.3.14.0
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libcoordinates.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libcoordinates.so: CMakeFiles/coordinates.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/Gazebo_Simulation/Contact_Sensor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libcoordinates.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/coordinates.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/coordinates.dir/build: libcoordinates.so

.PHONY : CMakeFiles/coordinates.dir/build

CMakeFiles/coordinates.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/coordinates.dir/cmake_clean.cmake
.PHONY : CMakeFiles/coordinates.dir/clean

CMakeFiles/coordinates.dir/depend:
	cd /home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/Gazebo_Simulation/Contact_Sensor/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/Gazebo_Simulation/Contact_Sensor /home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/Gazebo_Simulation/Contact_Sensor /home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/Gazebo_Simulation/Contact_Sensor/build /home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/Gazebo_Simulation/Contact_Sensor/build /home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/Gazebo_Simulation/Contact_Sensor/build/CMakeFiles/coordinates.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/coordinates.dir/depend

