# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/MyProject/UWB_g2o

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/MyProject/UWB_g2o/cmake-build-debug-remote-host

# Include any dependencies generated for this target.
include CMakeFiles/UWB_g2o.dir/depend.make
# Include the progress variables for this target.
include CMakeFiles/UWB_g2o.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/UWB_g2o.dir/flags.make

CMakeFiles/UWB_g2o.dir/main.cpp.o: CMakeFiles/UWB_g2o.dir/flags.make
CMakeFiles/UWB_g2o.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/MyProject/UWB_g2o/cmake-build-debug-remote-host/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/UWB_g2o.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/UWB_g2o.dir/main.cpp.o -c /home/MyProject/UWB_g2o/main.cpp

CMakeFiles/UWB_g2o.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/UWB_g2o.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/MyProject/UWB_g2o/main.cpp > CMakeFiles/UWB_g2o.dir/main.cpp.i

CMakeFiles/UWB_g2o.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/UWB_g2o.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/MyProject/UWB_g2o/main.cpp -o CMakeFiles/UWB_g2o.dir/main.cpp.s

CMakeFiles/UWB_g2o.dir/modules/include/sd.cpp.o: CMakeFiles/UWB_g2o.dir/flags.make
CMakeFiles/UWB_g2o.dir/modules/include/sd.cpp.o: ../modules/include/sd.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/MyProject/UWB_g2o/cmake-build-debug-remote-host/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/UWB_g2o.dir/modules/include/sd.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/UWB_g2o.dir/modules/include/sd.cpp.o -c /home/MyProject/UWB_g2o/modules/include/sd.cpp

CMakeFiles/UWB_g2o.dir/modules/include/sd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/UWB_g2o.dir/modules/include/sd.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/MyProject/UWB_g2o/modules/include/sd.cpp > CMakeFiles/UWB_g2o.dir/modules/include/sd.cpp.i

CMakeFiles/UWB_g2o.dir/modules/include/sd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/UWB_g2o.dir/modules/include/sd.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/MyProject/UWB_g2o/modules/include/sd.cpp -o CMakeFiles/UWB_g2o.dir/modules/include/sd.cpp.s

CMakeFiles/UWB_g2o.dir/modules/LocVertex.cpp.o: CMakeFiles/UWB_g2o.dir/flags.make
CMakeFiles/UWB_g2o.dir/modules/LocVertex.cpp.o: ../modules/LocVertex.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/MyProject/UWB_g2o/cmake-build-debug-remote-host/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/UWB_g2o.dir/modules/LocVertex.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/UWB_g2o.dir/modules/LocVertex.cpp.o -c /home/MyProject/UWB_g2o/modules/LocVertex.cpp

CMakeFiles/UWB_g2o.dir/modules/LocVertex.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/UWB_g2o.dir/modules/LocVertex.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/MyProject/UWB_g2o/modules/LocVertex.cpp > CMakeFiles/UWB_g2o.dir/modules/LocVertex.cpp.i

CMakeFiles/UWB_g2o.dir/modules/LocVertex.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/UWB_g2o.dir/modules/LocVertex.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/MyProject/UWB_g2o/modules/LocVertex.cpp -o CMakeFiles/UWB_g2o.dir/modules/LocVertex.cpp.s

CMakeFiles/UWB_g2o.dir/modules/MpcVertex.cpp.o: CMakeFiles/UWB_g2o.dir/flags.make
CMakeFiles/UWB_g2o.dir/modules/MpcVertex.cpp.o: ../modules/MpcVertex.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/MyProject/UWB_g2o/cmake-build-debug-remote-host/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/UWB_g2o.dir/modules/MpcVertex.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/UWB_g2o.dir/modules/MpcVertex.cpp.o -c /home/MyProject/UWB_g2o/modules/MpcVertex.cpp

CMakeFiles/UWB_g2o.dir/modules/MpcVertex.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/UWB_g2o.dir/modules/MpcVertex.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/MyProject/UWB_g2o/modules/MpcVertex.cpp > CMakeFiles/UWB_g2o.dir/modules/MpcVertex.cpp.i

CMakeFiles/UWB_g2o.dir/modules/MpcVertex.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/UWB_g2o.dir/modules/MpcVertex.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/MyProject/UWB_g2o/modules/MpcVertex.cpp -o CMakeFiles/UWB_g2o.dir/modules/MpcVertex.cpp.s

CMakeFiles/UWB_g2o.dir/modules/LocUnaryEdge.cpp.o: CMakeFiles/UWB_g2o.dir/flags.make
CMakeFiles/UWB_g2o.dir/modules/LocUnaryEdge.cpp.o: ../modules/LocUnaryEdge.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/MyProject/UWB_g2o/cmake-build-debug-remote-host/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/UWB_g2o.dir/modules/LocUnaryEdge.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/UWB_g2o.dir/modules/LocUnaryEdge.cpp.o -c /home/MyProject/UWB_g2o/modules/LocUnaryEdge.cpp

CMakeFiles/UWB_g2o.dir/modules/LocUnaryEdge.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/UWB_g2o.dir/modules/LocUnaryEdge.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/MyProject/UWB_g2o/modules/LocUnaryEdge.cpp > CMakeFiles/UWB_g2o.dir/modules/LocUnaryEdge.cpp.i

CMakeFiles/UWB_g2o.dir/modules/LocUnaryEdge.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/UWB_g2o.dir/modules/LocUnaryEdge.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/MyProject/UWB_g2o/modules/LocUnaryEdge.cpp -o CMakeFiles/UWB_g2o.dir/modules/LocUnaryEdge.cpp.s

CMakeFiles/UWB_g2o.dir/modules/MpcBinaryEdge.cpp.o: CMakeFiles/UWB_g2o.dir/flags.make
CMakeFiles/UWB_g2o.dir/modules/MpcBinaryEdge.cpp.o: ../modules/MpcBinaryEdge.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/MyProject/UWB_g2o/cmake-build-debug-remote-host/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/UWB_g2o.dir/modules/MpcBinaryEdge.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/UWB_g2o.dir/modules/MpcBinaryEdge.cpp.o -c /home/MyProject/UWB_g2o/modules/MpcBinaryEdge.cpp

CMakeFiles/UWB_g2o.dir/modules/MpcBinaryEdge.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/UWB_g2o.dir/modules/MpcBinaryEdge.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/MyProject/UWB_g2o/modules/MpcBinaryEdge.cpp > CMakeFiles/UWB_g2o.dir/modules/MpcBinaryEdge.cpp.i

CMakeFiles/UWB_g2o.dir/modules/MpcBinaryEdge.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/UWB_g2o.dir/modules/MpcBinaryEdge.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/MyProject/UWB_g2o/modules/MpcBinaryEdge.cpp -o CMakeFiles/UWB_g2o.dir/modules/MpcBinaryEdge.cpp.s

CMakeFiles/UWB_g2o.dir/tools/UWBtools.cpp.o: CMakeFiles/UWB_g2o.dir/flags.make
CMakeFiles/UWB_g2o.dir/tools/UWBtools.cpp.o: ../tools/UWBtools.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/MyProject/UWB_g2o/cmake-build-debug-remote-host/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/UWB_g2o.dir/tools/UWBtools.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/UWB_g2o.dir/tools/UWBtools.cpp.o -c /home/MyProject/UWB_g2o/tools/UWBtools.cpp

CMakeFiles/UWB_g2o.dir/tools/UWBtools.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/UWB_g2o.dir/tools/UWBtools.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/MyProject/UWB_g2o/tools/UWBtools.cpp > CMakeFiles/UWB_g2o.dir/tools/UWBtools.cpp.i

CMakeFiles/UWB_g2o.dir/tools/UWBtools.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/UWB_g2o.dir/tools/UWBtools.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/MyProject/UWB_g2o/tools/UWBtools.cpp -o CMakeFiles/UWB_g2o.dir/tools/UWBtools.cpp.s

CMakeFiles/UWB_g2o.dir/modules/LocMoveBinaryEdge.cpp.o: CMakeFiles/UWB_g2o.dir/flags.make
CMakeFiles/UWB_g2o.dir/modules/LocMoveBinaryEdge.cpp.o: ../modules/LocMoveBinaryEdge.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/MyProject/UWB_g2o/cmake-build-debug-remote-host/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/UWB_g2o.dir/modules/LocMoveBinaryEdge.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/UWB_g2o.dir/modules/LocMoveBinaryEdge.cpp.o -c /home/MyProject/UWB_g2o/modules/LocMoveBinaryEdge.cpp

CMakeFiles/UWB_g2o.dir/modules/LocMoveBinaryEdge.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/UWB_g2o.dir/modules/LocMoveBinaryEdge.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/MyProject/UWB_g2o/modules/LocMoveBinaryEdge.cpp > CMakeFiles/UWB_g2o.dir/modules/LocMoveBinaryEdge.cpp.i

CMakeFiles/UWB_g2o.dir/modules/LocMoveBinaryEdge.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/UWB_g2o.dir/modules/LocMoveBinaryEdge.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/MyProject/UWB_g2o/modules/LocMoveBinaryEdge.cpp -o CMakeFiles/UWB_g2o.dir/modules/LocMoveBinaryEdge.cpp.s

# Object files for target UWB_g2o
UWB_g2o_OBJECTS = \
"CMakeFiles/UWB_g2o.dir/main.cpp.o" \
"CMakeFiles/UWB_g2o.dir/modules/include/sd.cpp.o" \
"CMakeFiles/UWB_g2o.dir/modules/LocVertex.cpp.o" \
"CMakeFiles/UWB_g2o.dir/modules/MpcVertex.cpp.o" \
"CMakeFiles/UWB_g2o.dir/modules/LocUnaryEdge.cpp.o" \
"CMakeFiles/UWB_g2o.dir/modules/MpcBinaryEdge.cpp.o" \
"CMakeFiles/UWB_g2o.dir/tools/UWBtools.cpp.o" \
"CMakeFiles/UWB_g2o.dir/modules/LocMoveBinaryEdge.cpp.o"

# External object files for target UWB_g2o
UWB_g2o_EXTERNAL_OBJECTS =

UWB_g2o: CMakeFiles/UWB_g2o.dir/main.cpp.o
UWB_g2o: CMakeFiles/UWB_g2o.dir/modules/include/sd.cpp.o
UWB_g2o: CMakeFiles/UWB_g2o.dir/modules/LocVertex.cpp.o
UWB_g2o: CMakeFiles/UWB_g2o.dir/modules/MpcVertex.cpp.o
UWB_g2o: CMakeFiles/UWB_g2o.dir/modules/LocUnaryEdge.cpp.o
UWB_g2o: CMakeFiles/UWB_g2o.dir/modules/MpcBinaryEdge.cpp.o
UWB_g2o: CMakeFiles/UWB_g2o.dir/tools/UWBtools.cpp.o
UWB_g2o: CMakeFiles/UWB_g2o.dir/modules/LocMoveBinaryEdge.cpp.o
UWB_g2o: CMakeFiles/UWB_g2o.dir/build.make
UWB_g2o: /usr/local/lib/libg2o_stuff.so
UWB_g2o: /usr/local/lib/libg2o_core.so
UWB_g2o: CMakeFiles/UWB_g2o.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/MyProject/UWB_g2o/cmake-build-debug-remote-host/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX executable UWB_g2o"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/UWB_g2o.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/UWB_g2o.dir/build: UWB_g2o
.PHONY : CMakeFiles/UWB_g2o.dir/build

CMakeFiles/UWB_g2o.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/UWB_g2o.dir/cmake_clean.cmake
.PHONY : CMakeFiles/UWB_g2o.dir/clean

CMakeFiles/UWB_g2o.dir/depend:
	cd /home/MyProject/UWB_g2o/cmake-build-debug-remote-host && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/MyProject/UWB_g2o /home/MyProject/UWB_g2o /home/MyProject/UWB_g2o/cmake-build-debug-remote-host /home/MyProject/UWB_g2o/cmake-build-debug-remote-host /home/MyProject/UWB_g2o/cmake-build-debug-remote-host/CMakeFiles/UWB_g2o.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/UWB_g2o.dir/depend

