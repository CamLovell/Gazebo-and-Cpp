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
include navigation/CMakeFiles/testing.dir/depend.make

# Include the progress variables for this target.
include navigation/CMakeFiles/testing.dir/progress.make

# Include the compile flags for this target's objects.
include navigation/CMakeFiles/testing.dir/flags.make

navigation/CMakeFiles/testing.dir/src/Eig.cpp.o: navigation/CMakeFiles/testing.dir/flags.make
navigation/CMakeFiles/testing.dir/src/Eig.cpp.o: /home/cameron/NUMarine_ws/src/navigation/src/Eig.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object navigation/CMakeFiles/testing.dir/src/Eig.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testing.dir/src/Eig.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/src/Eig.cpp

navigation/CMakeFiles/testing.dir/src/Eig.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testing.dir/src/Eig.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/src/Eig.cpp > CMakeFiles/testing.dir/src/Eig.cpp.i

navigation/CMakeFiles/testing.dir/src/Eig.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testing.dir/src/Eig.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/src/Eig.cpp -o CMakeFiles/testing.dir/src/Eig.cpp.s

navigation/CMakeFiles/testing.dir/src/helpers.cpp.o: navigation/CMakeFiles/testing.dir/flags.make
navigation/CMakeFiles/testing.dir/src/helpers.cpp.o: /home/cameron/NUMarine_ws/src/navigation/src/helpers.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object navigation/CMakeFiles/testing.dir/src/helpers.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testing.dir/src/helpers.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/src/helpers.cpp

navigation/CMakeFiles/testing.dir/src/helpers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testing.dir/src/helpers.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/src/helpers.cpp > CMakeFiles/testing.dir/src/helpers.cpp.i

navigation/CMakeFiles/testing.dir/src/helpers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testing.dir/src/helpers.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/src/helpers.cpp -o CMakeFiles/testing.dir/src/helpers.cpp.s

navigation/CMakeFiles/testing.dir/src/spacialDual.cpp.o: navigation/CMakeFiles/testing.dir/flags.make
navigation/CMakeFiles/testing.dir/src/spacialDual.cpp.o: /home/cameron/NUMarine_ws/src/navigation/src/spacialDual.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object navigation/CMakeFiles/testing.dir/src/spacialDual.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testing.dir/src/spacialDual.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/src/spacialDual.cpp

navigation/CMakeFiles/testing.dir/src/spacialDual.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testing.dir/src/spacialDual.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/src/spacialDual.cpp > CMakeFiles/testing.dir/src/spacialDual.cpp.i

navigation/CMakeFiles/testing.dir/src/spacialDual.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testing.dir/src/spacialDual.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/src/spacialDual.cpp -o CMakeFiles/testing.dir/src/spacialDual.cpp.s

navigation/CMakeFiles/testing.dir/src/testing.cpp.o: navigation/CMakeFiles/testing.dir/flags.make
navigation/CMakeFiles/testing.dir/src/testing.cpp.o: /home/cameron/NUMarine_ws/src/navigation/src/testing.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object navigation/CMakeFiles/testing.dir/src/testing.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testing.dir/src/testing.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/src/testing.cpp

navigation/CMakeFiles/testing.dir/src/testing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testing.dir/src/testing.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/src/testing.cpp > CMakeFiles/testing.dir/src/testing.cpp.i

navigation/CMakeFiles/testing.dir/src/testing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testing.dir/src/testing.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/src/testing.cpp -o CMakeFiles/testing.dir/src/testing.cpp.s

navigation/CMakeFiles/testing.dir/tests/navTest.cpp.o: navigation/CMakeFiles/testing.dir/flags.make
navigation/CMakeFiles/testing.dir/tests/navTest.cpp.o: /home/cameron/NUMarine_ws/src/navigation/tests/navTest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object navigation/CMakeFiles/testing.dir/tests/navTest.cpp.o"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testing.dir/tests/navTest.cpp.o -c /home/cameron/NUMarine_ws/src/navigation/tests/navTest.cpp

navigation/CMakeFiles/testing.dir/tests/navTest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testing.dir/tests/navTest.cpp.i"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/NUMarine_ws/src/navigation/tests/navTest.cpp > CMakeFiles/testing.dir/tests/navTest.cpp.i

navigation/CMakeFiles/testing.dir/tests/navTest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testing.dir/tests/navTest.cpp.s"
	cd /home/cameron/NUMarine_ws/build/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/NUMarine_ws/src/navigation/tests/navTest.cpp -o CMakeFiles/testing.dir/tests/navTest.cpp.s

# Object files for target testing
testing_OBJECTS = \
"CMakeFiles/testing.dir/src/Eig.cpp.o" \
"CMakeFiles/testing.dir/src/helpers.cpp.o" \
"CMakeFiles/testing.dir/src/spacialDual.cpp.o" \
"CMakeFiles/testing.dir/src/testing.cpp.o" \
"CMakeFiles/testing.dir/tests/navTest.cpp.o"

# External object files for target testing
testing_EXTERNAL_OBJECTS =

/home/cameron/NUMarine_ws/devel/lib/libtesting.so: navigation/CMakeFiles/testing.dir/src/Eig.cpp.o
/home/cameron/NUMarine_ws/devel/lib/libtesting.so: navigation/CMakeFiles/testing.dir/src/helpers.cpp.o
/home/cameron/NUMarine_ws/devel/lib/libtesting.so: navigation/CMakeFiles/testing.dir/src/spacialDual.cpp.o
/home/cameron/NUMarine_ws/devel/lib/libtesting.so: navigation/CMakeFiles/testing.dir/src/testing.cpp.o
/home/cameron/NUMarine_ws/devel/lib/libtesting.so: navigation/CMakeFiles/testing.dir/tests/navTest.cpp.o
/home/cameron/NUMarine_ws/devel/lib/libtesting.so: navigation/CMakeFiles/testing.dir/build.make
/home/cameron/NUMarine_ws/devel/lib/libtesting.so: navigation/CMakeFiles/testing.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cameron/NUMarine_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX shared library /home/cameron/NUMarine_ws/devel/lib/libtesting.so"
	cd /home/cameron/NUMarine_ws/build/navigation && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testing.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
navigation/CMakeFiles/testing.dir/build: /home/cameron/NUMarine_ws/devel/lib/libtesting.so

.PHONY : navigation/CMakeFiles/testing.dir/build

navigation/CMakeFiles/testing.dir/clean:
	cd /home/cameron/NUMarine_ws/build/navigation && $(CMAKE_COMMAND) -P CMakeFiles/testing.dir/cmake_clean.cmake
.PHONY : navigation/CMakeFiles/testing.dir/clean

navigation/CMakeFiles/testing.dir/depend:
	cd /home/cameron/NUMarine_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cameron/NUMarine_ws/src /home/cameron/NUMarine_ws/src/navigation /home/cameron/NUMarine_ws/build /home/cameron/NUMarine_ws/build/navigation /home/cameron/NUMarine_ws/build/navigation/CMakeFiles/testing.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : navigation/CMakeFiles/testing.dir/depend
