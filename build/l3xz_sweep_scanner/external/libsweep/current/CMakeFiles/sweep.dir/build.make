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
CMAKE_SOURCE_DIR = /home/zega/Desktop/ros_dev/src/l3xz_sweep_scanner

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner

# Include any dependencies generated for this target.
include external/libsweep/current/CMakeFiles/sweep.dir/depend.make

# Include the progress variables for this target.
include external/libsweep/current/CMakeFiles/sweep.dir/progress.make

# Include the compile flags for this target's objects.
include external/libsweep/current/CMakeFiles/sweep.dir/flags.make

external/libsweep/current/CMakeFiles/sweep.dir/src/unix/serial.cc.o: external/libsweep/current/CMakeFiles/sweep.dir/flags.make
external/libsweep/current/CMakeFiles/sweep.dir/src/unix/serial.cc.o: /home/zega/Desktop/ros_dev/src/l3xz_sweep_scanner/external/libsweep/current/src/unix/serial.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object external/libsweep/current/CMakeFiles/sweep.dir/src/unix/serial.cc.o"
	cd /home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/external/libsweep/current && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sweep.dir/src/unix/serial.cc.o -c /home/zega/Desktop/ros_dev/src/l3xz_sweep_scanner/external/libsweep/current/src/unix/serial.cc

external/libsweep/current/CMakeFiles/sweep.dir/src/unix/serial.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sweep.dir/src/unix/serial.cc.i"
	cd /home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/external/libsweep/current && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zega/Desktop/ros_dev/src/l3xz_sweep_scanner/external/libsweep/current/src/unix/serial.cc > CMakeFiles/sweep.dir/src/unix/serial.cc.i

external/libsweep/current/CMakeFiles/sweep.dir/src/unix/serial.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sweep.dir/src/unix/serial.cc.s"
	cd /home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/external/libsweep/current && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zega/Desktop/ros_dev/src/l3xz_sweep_scanner/external/libsweep/current/src/unix/serial.cc -o CMakeFiles/sweep.dir/src/unix/serial.cc.s

external/libsweep/current/CMakeFiles/sweep.dir/src/protocol.cc.o: external/libsweep/current/CMakeFiles/sweep.dir/flags.make
external/libsweep/current/CMakeFiles/sweep.dir/src/protocol.cc.o: /home/zega/Desktop/ros_dev/src/l3xz_sweep_scanner/external/libsweep/current/src/protocol.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object external/libsweep/current/CMakeFiles/sweep.dir/src/protocol.cc.o"
	cd /home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/external/libsweep/current && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sweep.dir/src/protocol.cc.o -c /home/zega/Desktop/ros_dev/src/l3xz_sweep_scanner/external/libsweep/current/src/protocol.cc

external/libsweep/current/CMakeFiles/sweep.dir/src/protocol.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sweep.dir/src/protocol.cc.i"
	cd /home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/external/libsweep/current && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zega/Desktop/ros_dev/src/l3xz_sweep_scanner/external/libsweep/current/src/protocol.cc > CMakeFiles/sweep.dir/src/protocol.cc.i

external/libsweep/current/CMakeFiles/sweep.dir/src/protocol.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sweep.dir/src/protocol.cc.s"
	cd /home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/external/libsweep/current && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zega/Desktop/ros_dev/src/l3xz_sweep_scanner/external/libsweep/current/src/protocol.cc -o CMakeFiles/sweep.dir/src/protocol.cc.s

external/libsweep/current/CMakeFiles/sweep.dir/src/sweep.cc.o: external/libsweep/current/CMakeFiles/sweep.dir/flags.make
external/libsweep/current/CMakeFiles/sweep.dir/src/sweep.cc.o: /home/zega/Desktop/ros_dev/src/l3xz_sweep_scanner/external/libsweep/current/src/sweep.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object external/libsweep/current/CMakeFiles/sweep.dir/src/sweep.cc.o"
	cd /home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/external/libsweep/current && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sweep.dir/src/sweep.cc.o -c /home/zega/Desktop/ros_dev/src/l3xz_sweep_scanner/external/libsweep/current/src/sweep.cc

external/libsweep/current/CMakeFiles/sweep.dir/src/sweep.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sweep.dir/src/sweep.cc.i"
	cd /home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/external/libsweep/current && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zega/Desktop/ros_dev/src/l3xz_sweep_scanner/external/libsweep/current/src/sweep.cc > CMakeFiles/sweep.dir/src/sweep.cc.i

external/libsweep/current/CMakeFiles/sweep.dir/src/sweep.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sweep.dir/src/sweep.cc.s"
	cd /home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/external/libsweep/current && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zega/Desktop/ros_dev/src/l3xz_sweep_scanner/external/libsweep/current/src/sweep.cc -o CMakeFiles/sweep.dir/src/sweep.cc.s

# Object files for target sweep
sweep_OBJECTS = \
"CMakeFiles/sweep.dir/src/unix/serial.cc.o" \
"CMakeFiles/sweep.dir/src/protocol.cc.o" \
"CMakeFiles/sweep.dir/src/sweep.cc.o"

# External object files for target sweep
sweep_EXTERNAL_OBJECTS =

external/libsweep/current/libsweep.a: external/libsweep/current/CMakeFiles/sweep.dir/src/unix/serial.cc.o
external/libsweep/current/libsweep.a: external/libsweep/current/CMakeFiles/sweep.dir/src/protocol.cc.o
external/libsweep/current/libsweep.a: external/libsweep/current/CMakeFiles/sweep.dir/src/sweep.cc.o
external/libsweep/current/libsweep.a: external/libsweep/current/CMakeFiles/sweep.dir/build.make
external/libsweep/current/libsweep.a: external/libsweep/current/CMakeFiles/sweep.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libsweep.a"
	cd /home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/external/libsweep/current && $(CMAKE_COMMAND) -P CMakeFiles/sweep.dir/cmake_clean_target.cmake
	cd /home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/external/libsweep/current && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sweep.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
external/libsweep/current/CMakeFiles/sweep.dir/build: external/libsweep/current/libsweep.a

.PHONY : external/libsweep/current/CMakeFiles/sweep.dir/build

external/libsweep/current/CMakeFiles/sweep.dir/clean:
	cd /home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/external/libsweep/current && $(CMAKE_COMMAND) -P CMakeFiles/sweep.dir/cmake_clean.cmake
.PHONY : external/libsweep/current/CMakeFiles/sweep.dir/clean

external/libsweep/current/CMakeFiles/sweep.dir/depend:
	cd /home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zega/Desktop/ros_dev/src/l3xz_sweep_scanner /home/zega/Desktop/ros_dev/src/l3xz_sweep_scanner/external/libsweep/current /home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner /home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/external/libsweep/current /home/zega/Desktop/ros_dev/src/build/l3xz_sweep_scanner/external/libsweep/current/CMakeFiles/sweep.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : external/libsweep/current/CMakeFiles/sweep.dir/depend

