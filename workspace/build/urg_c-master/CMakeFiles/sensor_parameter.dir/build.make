# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/robot/Desktop/workspace/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robot/Desktop/workspace/build

# Include any dependencies generated for this target.
include urg_c-master/CMakeFiles/sensor_parameter.dir/depend.make

# Include the progress variables for this target.
include urg_c-master/CMakeFiles/sensor_parameter.dir/progress.make

# Include the compile flags for this target's objects.
include urg_c-master/CMakeFiles/sensor_parameter.dir/flags.make

urg_c-master/CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.o: urg_c-master/CMakeFiles/sensor_parameter.dir/flags.make
urg_c-master/CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.o: /home/robot/Desktop/workspace/src/urg_c-master/current/samples/sensor_parameter.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot/Desktop/workspace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object urg_c-master/CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.o"
	cd /home/robot/Desktop/workspace/build/urg_c-master && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.o   -c /home/robot/Desktop/workspace/src/urg_c-master/current/samples/sensor_parameter.c

urg_c-master/CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.i"
	cd /home/robot/Desktop/workspace/build/urg_c-master && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/robot/Desktop/workspace/src/urg_c-master/current/samples/sensor_parameter.c > CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.i

urg_c-master/CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.s"
	cd /home/robot/Desktop/workspace/build/urg_c-master && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/robot/Desktop/workspace/src/urg_c-master/current/samples/sensor_parameter.c -o CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.s

urg_c-master/CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.o.requires:

.PHONY : urg_c-master/CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.o.requires

urg_c-master/CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.o.provides: urg_c-master/CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.o.requires
	$(MAKE) -f urg_c-master/CMakeFiles/sensor_parameter.dir/build.make urg_c-master/CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.o.provides.build
.PHONY : urg_c-master/CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.o.provides

urg_c-master/CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.o.provides.build: urg_c-master/CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.o


# Object files for target sensor_parameter
sensor_parameter_OBJECTS = \
"CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.o"

# External object files for target sensor_parameter
sensor_parameter_EXTERNAL_OBJECTS =

/home/robot/Desktop/workspace/devel/lib/urg_c/sensor_parameter: urg_c-master/CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.o
/home/robot/Desktop/workspace/devel/lib/urg_c/sensor_parameter: urg_c-master/CMakeFiles/sensor_parameter.dir/build.make
/home/robot/Desktop/workspace/devel/lib/urg_c/sensor_parameter: /home/robot/Desktop/workspace/devel/lib/libopen_urg_sensor.so
/home/robot/Desktop/workspace/devel/lib/urg_c/sensor_parameter: /home/robot/Desktop/workspace/devel/lib/libliburg_c.so
/home/robot/Desktop/workspace/devel/lib/urg_c/sensor_parameter: urg_c-master/CMakeFiles/sensor_parameter.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robot/Desktop/workspace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable /home/robot/Desktop/workspace/devel/lib/urg_c/sensor_parameter"
	cd /home/robot/Desktop/workspace/build/urg_c-master && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sensor_parameter.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
urg_c-master/CMakeFiles/sensor_parameter.dir/build: /home/robot/Desktop/workspace/devel/lib/urg_c/sensor_parameter

.PHONY : urg_c-master/CMakeFiles/sensor_parameter.dir/build

urg_c-master/CMakeFiles/sensor_parameter.dir/requires: urg_c-master/CMakeFiles/sensor_parameter.dir/current/samples/sensor_parameter.c.o.requires

.PHONY : urg_c-master/CMakeFiles/sensor_parameter.dir/requires

urg_c-master/CMakeFiles/sensor_parameter.dir/clean:
	cd /home/robot/Desktop/workspace/build/urg_c-master && $(CMAKE_COMMAND) -P CMakeFiles/sensor_parameter.dir/cmake_clean.cmake
.PHONY : urg_c-master/CMakeFiles/sensor_parameter.dir/clean

urg_c-master/CMakeFiles/sensor_parameter.dir/depend:
	cd /home/robot/Desktop/workspace/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/Desktop/workspace/src /home/robot/Desktop/workspace/src/urg_c-master /home/robot/Desktop/workspace/build /home/robot/Desktop/workspace/build/urg_c-master /home/robot/Desktop/workspace/build/urg_c-master/CMakeFiles/sensor_parameter.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : urg_c-master/CMakeFiles/sensor_parameter.dir/depend

