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
CMAKE_SOURCE_DIR = /home/robotflow/IUK/rbdl_using

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robotflow/IUK/rbdl_using/build

# Include any dependencies generated for this target.
include CMakeFiles/COMMON.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/COMMON.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/COMMON.dir/flags.make

CMakeFiles/COMMON.dir/common_utils/common.cpp.o: CMakeFiles/COMMON.dir/flags.make
CMakeFiles/COMMON.dir/common_utils/common.cpp.o: ../common_utils/common.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robotflow/IUK/rbdl_using/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/COMMON.dir/common_utils/common.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/COMMON.dir/common_utils/common.cpp.o -c /home/robotflow/IUK/rbdl_using/common_utils/common.cpp

CMakeFiles/COMMON.dir/common_utils/common.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/COMMON.dir/common_utils/common.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robotflow/IUK/rbdl_using/common_utils/common.cpp > CMakeFiles/COMMON.dir/common_utils/common.cpp.i

CMakeFiles/COMMON.dir/common_utils/common.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/COMMON.dir/common_utils/common.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robotflow/IUK/rbdl_using/common_utils/common.cpp -o CMakeFiles/COMMON.dir/common_utils/common.cpp.s

# Object files for target COMMON
COMMON_OBJECTS = \
"CMakeFiles/COMMON.dir/common_utils/common.cpp.o"

# External object files for target COMMON
COMMON_EXTERNAL_OBJECTS =

libCOMMON.a: CMakeFiles/COMMON.dir/common_utils/common.cpp.o
libCOMMON.a: CMakeFiles/COMMON.dir/build.make
libCOMMON.a: CMakeFiles/COMMON.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robotflow/IUK/rbdl_using/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libCOMMON.a"
	$(CMAKE_COMMAND) -P CMakeFiles/COMMON.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/COMMON.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/COMMON.dir/build: libCOMMON.a

.PHONY : CMakeFiles/COMMON.dir/build

CMakeFiles/COMMON.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/COMMON.dir/cmake_clean.cmake
.PHONY : CMakeFiles/COMMON.dir/clean

CMakeFiles/COMMON.dir/depend:
	cd /home/robotflow/IUK/rbdl_using/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robotflow/IUK/rbdl_using /home/robotflow/IUK/rbdl_using /home/robotflow/IUK/rbdl_using/build /home/robotflow/IUK/rbdl_using/build /home/robotflow/IUK/rbdl_using/build/CMakeFiles/COMMON.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/COMMON.dir/depend
