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
CMAKE_SOURCE_DIR = /home/cameron/NUMarine_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cameron/NUMarine_ws/build

# Include any dependencies generated for this target.
include navigation/CMakeFiles/catchTests.dir/depend.make

# Include the progress variables for this target.
include navigation/CMakeFiles/catchTests.dir/progress.make

# Include the compile flags for this target's objects.
include navigation/CMakeFiles/catchTests.dir/flags.make

navigation/CMakeFiles/catchTests.dir/test/src/GPS.cpp.o: navigation/CMakeFiles/catchTests.dir/flags.make
navigation/CMakeFiles/catchTests.dir/test/src/GPS.cpp.o: /home/cameron/NUMarine_ws/src/navigation/test/src/GPS.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object navigation/CMakeFiles/catchTests.dir/test/src/GPS.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/catchTests.dir/test/src/GPS.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/test/src/GPS.cpp

navigation/CMakeFiles/catchTests.dir/test/src/GPS.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/catchTests.dir/test/src/GPS.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/test/src/GPS.cpp > CMakeFiles/catchTests.dir/test/src/GPS.cpp.i

navigation/CMakeFiles/catchTests.dir/test/src/GPS.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/catchTests.dir/test/src/GPS.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/test/src/GPS.cpp -o CMakeFiles/catchTests.dir/test/src/GPS.cpp.s

navigation/CMakeFiles/catchTests.dir/test/src/eigenAtan2.cpp.o: navigation/CMakeFiles/catchTests.dir/flags.make
navigation/CMakeFiles/catchTests.dir/test/src/eigenAtan2.cpp.o: /home/cameron/NUMarine_ws/src/navigation/test/src/eigenAtan2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object navigation/CMakeFiles/catchTests.dir/test/src/eigenAtan2.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/catchTests.dir/test/src/eigenAtan2.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/test/src/eigenAtan2.cpp

navigation/CMakeFiles/catchTests.dir/test/src/eigenAtan2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/catchTests.dir/test/src/eigenAtan2.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/test/src/eigenAtan2.cpp > CMakeFiles/catchTests.dir/test/src/eigenAtan2.cpp.i

navigation/CMakeFiles/catchTests.dir/test/src/eigenAtan2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/catchTests.dir/test/src/eigenAtan2.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/test/src/eigenAtan2.cpp -o CMakeFiles/catchTests.dir/test/src/eigenAtan2.cpp.s

navigation/CMakeFiles/catchTests.dir/test/src/helpingFunctions.cpp.o: navigation/CMakeFiles/catchTests.dir/flags.make
navigation/CMakeFiles/catchTests.dir/test/src/helpingFunctions.cpp.o: /home/cameron/NUMarine_ws/src/navigation/test/src/helpingFunctions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object navigation/CMakeFiles/catchTests.dir/test/src/helpingFunctions.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/catchTests.dir/test/src/helpingFunctions.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/test/src/helpingFunctions.cpp

navigation/CMakeFiles/catchTests.dir/test/src/helpingFunctions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/catchTests.dir/test/src/helpingFunctions.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/test/src/helpingFunctions.cpp > CMakeFiles/catchTests.dir/test/src/helpingFunctions.cpp.i

navigation/CMakeFiles/catchTests.dir/test/src/helpingFunctions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/catchTests.dir/test/src/helpingFunctions.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/test/src/helpingFunctions.cpp -o CMakeFiles/catchTests.dir/test/src/helpingFunctions.cpp.s

navigation/CMakeFiles/catchTests.dir/test/src/lidar.cpp.o: navigation/CMakeFiles/catchTests.dir/flags.make
navigation/CMakeFiles/catchTests.dir/test/src/lidar.cpp.o: /home/cameron/NUMarine_ws/src/navigation/test/src/lidar.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object navigation/CMakeFiles/catchTests.dir/test/src/lidar.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/catchTests.dir/test/src/lidar.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/test/src/lidar.cpp

navigation/CMakeFiles/catchTests.dir/test/src/lidar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/catchTests.dir/test/src/lidar.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/test/src/lidar.cpp > CMakeFiles/catchTests.dir/test/src/lidar.cpp.i

navigation/CMakeFiles/catchTests.dir/test/src/lidar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/catchTests.dir/test/src/lidar.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/test/src/lidar.cpp -o CMakeFiles/catchTests.dir/test/src/lidar.cpp.s

navigation/CMakeFiles/catchTests.dir/test/src/main.cpp.o: navigation/CMakeFiles/catchTests.dir/flags.make
navigation/CMakeFiles/catchTests.dir/test/src/main.cpp.o: /home/cameron/NUMarine_ws/src/navigation/test/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object navigation/CMakeFiles/catchTests.dir/test/src/main.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/catchTests.dir/test/src/main.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/test/src/main.cpp

navigation/CMakeFiles/catchTests.dir/test/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/catchTests.dir/test/src/main.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/test/src/main.cpp > CMakeFiles/catchTests.dir/test/src/main.cpp.i

navigation/CMakeFiles/catchTests.dir/test/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/catchTests.dir/test/src/main.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/test/src/main.cpp -o CMakeFiles/catchTests.dir/test/src/main.cpp.s

navigation/CMakeFiles/catchTests.dir/test/src/quaternionConversions.cpp.o: navigation/CMakeFiles/catchTests.dir/flags.make
navigation/CMakeFiles/catchTests.dir/test/src/quaternionConversions.cpp.o: /home/cameron/NUMarine_ws/src/navigation/test/src/quaternionConversions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object navigation/CMakeFiles/catchTests.dir/test/src/quaternionConversions.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/catchTests.dir/test/src/quaternionConversions.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/test/src/quaternionConversions.cpp

navigation/CMakeFiles/catchTests.dir/test/src/quaternionConversions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/catchTests.dir/test/src/quaternionConversions.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/test/src/quaternionConversions.cpp > CMakeFiles/catchTests.dir/test/src/quaternionConversions.cpp.i

navigation/CMakeFiles/catchTests.dir/test/src/quaternionConversions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/catchTests.dir/test/src/quaternionConversions.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/test/src/quaternionConversions.cpp -o CMakeFiles/catchTests.dir/test/src/quaternionConversions.cpp.s

navigation/CMakeFiles/catchTests.dir/test/src/wamv.cpp.o: navigation/CMakeFiles/catchTests.dir/flags.make
navigation/CMakeFiles/catchTests.dir/test/src/wamv.cpp.o: /home/cameron/NUMarine_ws/src/navigation/test/src/wamv.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object navigation/CMakeFiles/catchTests.dir/test/src/wamv.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/catchTests.dir/test/src/wamv.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/test/src/wamv.cpp

navigation/CMakeFiles/catchTests.dir/test/src/wamv.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/catchTests.dir/test/src/wamv.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/test/src/wamv.cpp > CMakeFiles/catchTests.dir/test/src/wamv.cpp.i

navigation/CMakeFiles/catchTests.dir/test/src/wamv.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/catchTests.dir/test/src/wamv.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/test/src/wamv.cpp -o CMakeFiles/catchTests.dir/test/src/wamv.cpp.s

# Object files for target catchTests
catchTests_OBJECTS = \
"CMakeFiles/catchTests.dir/test/src/GPS.cpp.o" \
"CMakeFiles/catchTests.dir/test/src/eigenAtan2.cpp.o" \
"CMakeFiles/catchTests.dir/test/src/helpingFunctions.cpp.o" \
"CMakeFiles/catchTests.dir/test/src/lidar.cpp.o" \
"CMakeFiles/catchTests.dir/test/src/main.cpp.o" \
"CMakeFiles/catchTests.dir/test/src/quaternionConversions.cpp.o" \
"CMakeFiles/catchTests.dir/test/src/wamv.cpp.o"

# External object files for target catchTests
catchTests_EXTERNAL_OBJECTS =

/home/cameron/NUMarine_ws/devel/lib/navigation/catchTests: navigation/CMakeFiles/catchTests.dir/test/src/GPS.cpp.o
/home/cameron/NUMarine_ws/devel/lib/navigation/catchTests: navigation/CMakeFiles/catchTests.dir/test/src/eigenAtan2.cpp.o
/home/cameron/NUMarine_ws/devel/lib/navigation/catchTests: navigation/CMakeFiles/catchTests.dir/test/src/helpingFunctions.cpp.o
/home/cameron/NUMarine_ws/devel/lib/navigation/catchTests: navigation/CMakeFiles/catchTests.dir/test/src/lidar.cpp.o
/home/cameron/NUMarine_ws/devel/lib/navigation/catchTests: navigation/CMakeFiles/catchTests.dir/test/src/main.cpp.o
/home/cameron/NUMarine_ws/devel/lib/navigation/catchTests: navigation/CMakeFiles/catchTests.dir/test/src/quaternionConversions.cpp.o
/home/cameron/NUMarine_ws/devel/lib/navigation/catchTests: navigation/CMakeFiles/catchTests.dir/test/src/wamv.cpp.o
/home/cameron/NUMarine_ws/devel/lib/navigation/catchTests: navigation/CMakeFiles/catchTests.dir/build.make
/home/cameron/NUMarine_ws/devel/lib/navigation/catchTests: /home/cameron/NUMarine_ws/devel/lib/libcommon.so
/home/cameron/NUMarine_ws/devel/lib/navigation/catchTests: /usr/local/lib/libGeographic.so.19.2.0
/home/cameron/NUMarine_ws/devel/lib/navigation/catchTests: navigation/CMakeFiles/catchTests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX executable /home/cameron/NUMarine_ws/devel/lib/navigation/catchTests"
	cd /home/cameron/NUMarine_ws/build/navigation && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/catchTests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
navigation/CMakeFiles/catchTests.dir/build: /home/cameron/NUMarine_ws/devel/lib/navigation/catchTests

.PHONY : navigation/CMakeFiles/catchTests.dir/build

navigation/CMakeFiles/catchTests.dir/clean:
	cd /home/cameron/NUMarine_ws/build/navigation && $(CMAKE_COMMAND) -P CMakeFiles/catchTests.dir/cmake_clean.cmake
.PHONY : navigation/CMakeFiles/catchTests.dir/clean

navigation/CMakeFiles/catchTests.dir/depend:
	cd /home/cameron/NUMarine_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cameron/NUMarine_ws/src /home/cameron/NUMarine_ws/src/navigation /home/cameron/NUMarine_ws/build /home/cameron/NUMarine_ws/build/navigation /home/cameron/NUMarine_ws/build/navigation/CMakeFiles/catchTests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : navigation/CMakeFiles/catchTests.dir/depend

