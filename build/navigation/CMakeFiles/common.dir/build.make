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
include navigation/CMakeFiles/common.dir/depend.make

# Include the progress variables for this target.
include navigation/CMakeFiles/common.dir/progress.make

# Include the compile flags for this target's objects.
include navigation/CMakeFiles/common.dir/flags.make

navigation/CMakeFiles/common.dir/src/Eig.cpp.o: navigation/CMakeFiles/common.dir/flags.make
navigation/CMakeFiles/common.dir/src/Eig.cpp.o: /home/cameron/NUMarine_ws/src/navigation/src/Eig.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object navigation/CMakeFiles/common.dir/src/Eig.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/common.dir/src/Eig.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/src/Eig.cpp

navigation/CMakeFiles/common.dir/src/Eig.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/common.dir/src/Eig.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/src/Eig.cpp > CMakeFiles/common.dir/src/Eig.cpp.i

navigation/CMakeFiles/common.dir/src/Eig.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/common.dir/src/Eig.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/src/Eig.cpp -o CMakeFiles/common.dir/src/Eig.cpp.s

navigation/CMakeFiles/common.dir/src/helpers.cpp.o: navigation/CMakeFiles/common.dir/flags.make
navigation/CMakeFiles/common.dir/src/helpers.cpp.o: /home/cameron/NUMarine_ws/src/navigation/src/helpers.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object navigation/CMakeFiles/common.dir/src/helpers.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/common.dir/src/helpers.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/src/helpers.cpp

navigation/CMakeFiles/common.dir/src/helpers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/common.dir/src/helpers.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/src/helpers.cpp > CMakeFiles/common.dir/src/helpers.cpp.i

navigation/CMakeFiles/common.dir/src/helpers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/common.dir/src/helpers.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/src/helpers.cpp -o CMakeFiles/common.dir/src/helpers.cpp.s

navigation/CMakeFiles/common.dir/src/lidar.cpp.o: navigation/CMakeFiles/common.dir/flags.make
navigation/CMakeFiles/common.dir/src/lidar.cpp.o: /home/cameron/NUMarine_ws/src/navigation/src/lidar.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object navigation/CMakeFiles/common.dir/src/lidar.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/common.dir/src/lidar.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/src/lidar.cpp

navigation/CMakeFiles/common.dir/src/lidar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/common.dir/src/lidar.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/src/lidar.cpp > CMakeFiles/common.dir/src/lidar.cpp.i

navigation/CMakeFiles/common.dir/src/lidar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/common.dir/src/lidar.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/src/lidar.cpp -o CMakeFiles/common.dir/src/lidar.cpp.s

navigation/CMakeFiles/common.dir/src/map.cpp.o: navigation/CMakeFiles/common.dir/flags.make
navigation/CMakeFiles/common.dir/src/map.cpp.o: /home/cameron/NUMarine_ws/src/navigation/src/map.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object navigation/CMakeFiles/common.dir/src/map.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/common.dir/src/map.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/src/map.cpp

navigation/CMakeFiles/common.dir/src/map.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/common.dir/src/map.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/src/map.cpp > CMakeFiles/common.dir/src/map.cpp.i

navigation/CMakeFiles/common.dir/src/map.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/common.dir/src/map.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/src/map.cpp -o CMakeFiles/common.dir/src/map.cpp.s

navigation/CMakeFiles/common.dir/src/particleFilter.cpp.o: navigation/CMakeFiles/common.dir/flags.make
navigation/CMakeFiles/common.dir/src/particleFilter.cpp.o: /home/cameron/NUMarine_ws/src/navigation/src/particleFilter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object navigation/CMakeFiles/common.dir/src/particleFilter.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/common.dir/src/particleFilter.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/src/particleFilter.cpp

navigation/CMakeFiles/common.dir/src/particleFilter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/common.dir/src/particleFilter.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/src/particleFilter.cpp > CMakeFiles/common.dir/src/particleFilter.cpp.i

navigation/CMakeFiles/common.dir/src/particleFilter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/common.dir/src/particleFilter.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/src/particleFilter.cpp -o CMakeFiles/common.dir/src/particleFilter.cpp.s

navigation/CMakeFiles/common.dir/src/spacialDual.cpp.o: navigation/CMakeFiles/common.dir/flags.make
navigation/CMakeFiles/common.dir/src/spacialDual.cpp.o: /home/cameron/NUMarine_ws/src/navigation/src/spacialDual.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object navigation/CMakeFiles/common.dir/src/spacialDual.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/common.dir/src/spacialDual.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/src/spacialDual.cpp

navigation/CMakeFiles/common.dir/src/spacialDual.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/common.dir/src/spacialDual.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/src/spacialDual.cpp > CMakeFiles/common.dir/src/spacialDual.cpp.i

navigation/CMakeFiles/common.dir/src/spacialDual.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/common.dir/src/spacialDual.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/src/spacialDual.cpp -o CMakeFiles/common.dir/src/spacialDual.cpp.s

navigation/CMakeFiles/common.dir/src/testing.cpp.o: navigation/CMakeFiles/common.dir/flags.make
navigation/CMakeFiles/common.dir/src/testing.cpp.o: /home/cameron/NUMarine_ws/src/navigation/src/testing.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object navigation/CMakeFiles/common.dir/src/testing.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/common.dir/src/testing.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/src/testing.cpp

navigation/CMakeFiles/common.dir/src/testing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/common.dir/src/testing.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/src/testing.cpp > CMakeFiles/common.dir/src/testing.cpp.i

navigation/CMakeFiles/common.dir/src/testing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/common.dir/src/testing.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/src/testing.cpp -o CMakeFiles/common.dir/src/testing.cpp.s

navigation/CMakeFiles/common.dir/src/wamv.cpp.o: navigation/CMakeFiles/common.dir/flags.make
navigation/CMakeFiles/common.dir/src/wamv.cpp.o: /home/cameron/NUMarine_ws/src/navigation/src/wamv.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object navigation/CMakeFiles/common.dir/src/wamv.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/common.dir/src/wamv.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/src/wamv.cpp

navigation/CMakeFiles/common.dir/src/wamv.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/common.dir/src/wamv.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/src/wamv.cpp > CMakeFiles/common.dir/src/wamv.cpp.i

navigation/CMakeFiles/common.dir/src/wamv.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/common.dir/src/wamv.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/src/wamv.cpp -o CMakeFiles/common.dir/src/wamv.cpp.s

# Object files for target common
common_OBJECTS = \
"CMakeFiles/common.dir/src/Eig.cpp.o" \
"CMakeFiles/common.dir/src/helpers.cpp.o" \
"CMakeFiles/common.dir/src/lidar.cpp.o" \
"CMakeFiles/common.dir/src/map.cpp.o" \
"CMakeFiles/common.dir/src/particleFilter.cpp.o" \
"CMakeFiles/common.dir/src/spacialDual.cpp.o" \
"CMakeFiles/common.dir/src/testing.cpp.o" \
"CMakeFiles/common.dir/src/wamv.cpp.o"

# External object files for target common
common_EXTERNAL_OBJECTS =

/home/cameron/NUMarine_ws/devel/lib/libcommon.so: navigation/CMakeFiles/common.dir/src/Eig.cpp.o
/home/cameron/NUMarine_ws/devel/lib/libcommon.so: navigation/CMakeFiles/common.dir/src/helpers.cpp.o
/home/cameron/NUMarine_ws/devel/lib/libcommon.so: navigation/CMakeFiles/common.dir/src/lidar.cpp.o
/home/cameron/NUMarine_ws/devel/lib/libcommon.so: navigation/CMakeFiles/common.dir/src/map.cpp.o
/home/cameron/NUMarine_ws/devel/lib/libcommon.so: navigation/CMakeFiles/common.dir/src/particleFilter.cpp.o
/home/cameron/NUMarine_ws/devel/lib/libcommon.so: navigation/CMakeFiles/common.dir/src/spacialDual.cpp.o
/home/cameron/NUMarine_ws/devel/lib/libcommon.so: navigation/CMakeFiles/common.dir/src/testing.cpp.o
/home/cameron/NUMarine_ws/devel/lib/libcommon.so: navigation/CMakeFiles/common.dir/src/wamv.cpp.o
/home/cameron/NUMarine_ws/devel/lib/libcommon.so: navigation/CMakeFiles/common.dir/build.make
/home/cameron/NUMarine_ws/devel/lib/libcommon.so: navigation/CMakeFiles/common.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX shared library /home/cameron/NUMarine_ws/devel/lib/libcommon.so"
	cd /home/cameron/NUMarine_ws/build/navigation && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/common.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
navigation/CMakeFiles/common.dir/build: /home/cameron/NUMarine_ws/devel/lib/libcommon.so

.PHONY : navigation/CMakeFiles/common.dir/build

navigation/CMakeFiles/common.dir/clean:
	cd /home/cameron/NUMarine_ws/build/navigation && $(CMAKE_COMMAND) -P CMakeFiles/common.dir/cmake_clean.cmake
.PHONY : navigation/CMakeFiles/common.dir/clean

navigation/CMakeFiles/common.dir/depend:
	cd /home/cameron/NUMarine_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cameron/NUMarine_ws/src /home/cameron/NUMarine_ws/src/navigation /home/cameron/NUMarine_ws/build /home/cameron/NUMarine_ws/build/navigation /home/cameron/NUMarine_ws/build/navigation/CMakeFiles/common.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : navigation/CMakeFiles/common.dir/depend

