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
CMAKE_SOURCE_DIR = /home/robotflow/IUK/rbdl_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robotflow/IUK/rbdl_test/build

# Include any dependencies generated for this target.
include CMakeFiles/rbdltest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rbdltest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rbdltest.dir/flags.make

CMakeFiles/rbdltest.dir/MAIN.cpp.o: CMakeFiles/rbdltest.dir/flags.make
CMakeFiles/rbdltest.dir/MAIN.cpp.o: ../MAIN.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robotflow/IUK/rbdl_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rbdltest.dir/MAIN.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rbdltest.dir/MAIN.cpp.o -c /home/robotflow/IUK/rbdl_test/MAIN.cpp

CMakeFiles/rbdltest.dir/MAIN.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rbdltest.dir/MAIN.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robotflow/IUK/rbdl_test/MAIN.cpp > CMakeFiles/rbdltest.dir/MAIN.cpp.i

CMakeFiles/rbdltest.dir/MAIN.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rbdltest.dir/MAIN.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robotflow/IUK/rbdl_test/MAIN.cpp -o CMakeFiles/rbdltest.dir/MAIN.cpp.s

# Object files for target rbdltest
rbdltest_OBJECTS = \
"CMakeFiles/rbdltest.dir/MAIN.cpp.o"

# External object files for target rbdltest
rbdltest_EXTERNAL_OBJECTS =

rbdltest: CMakeFiles/rbdltest.dir/MAIN.cpp.o
rbdltest: CMakeFiles/rbdltest.dir/build.make
rbdltest: libRobotModel.a
rbdltest: libRBDL.a
rbdltest: libURDF.a
rbdltest: libCOMMON.a
rbdltest: CMakeFiles/rbdltest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robotflow/IUK/rbdl_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable rbdltest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rbdltest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rbdltest.dir/build: rbdltest

.PHONY : CMakeFiles/rbdltest.dir/build

CMakeFiles/rbdltest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rbdltest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rbdltest.dir/clean

CMakeFiles/rbdltest.dir/depend:
	cd /home/robotflow/IUK/rbdl_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robotflow/IUK/rbdl_test /home/robotflow/IUK/rbdl_test /home/robotflow/IUK/rbdl_test/build /home/robotflow/IUK/rbdl_test/build /home/robotflow/IUK/rbdl_test/build/CMakeFiles/rbdltest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rbdltest.dir/depend

