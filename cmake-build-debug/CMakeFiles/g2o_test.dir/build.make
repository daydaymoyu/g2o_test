# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_COMMAND = /home/xiangqian/Download/clion-2017.1.1/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/xiangqian/Download/clion-2017.1.1/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/xiangqian/C++projects/g2o_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xiangqian/C++projects/g2o_test/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/g2o_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/g2o_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/g2o_test.dir/flags.make

CMakeFiles/g2o_test.dir/main.cpp.o: CMakeFiles/g2o_test.dir/flags.make
CMakeFiles/g2o_test.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xiangqian/C++projects/g2o_test/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/g2o_test.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/g2o_test.dir/main.cpp.o -c /home/xiangqian/C++projects/g2o_test/main.cpp

CMakeFiles/g2o_test.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/g2o_test.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xiangqian/C++projects/g2o_test/main.cpp > CMakeFiles/g2o_test.dir/main.cpp.i

CMakeFiles/g2o_test.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/g2o_test.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xiangqian/C++projects/g2o_test/main.cpp -o CMakeFiles/g2o_test.dir/main.cpp.s

CMakeFiles/g2o_test.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/g2o_test.dir/main.cpp.o.requires

CMakeFiles/g2o_test.dir/main.cpp.o.provides: CMakeFiles/g2o_test.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/g2o_test.dir/build.make CMakeFiles/g2o_test.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/g2o_test.dir/main.cpp.o.provides

CMakeFiles/g2o_test.dir/main.cpp.o.provides.build: CMakeFiles/g2o_test.dir/main.cpp.o


# Object files for target g2o_test
g2o_test_OBJECTS = \
"CMakeFiles/g2o_test.dir/main.cpp.o"

# External object files for target g2o_test
g2o_test_EXTERNAL_OBJECTS =

../bin/g2o_test: CMakeFiles/g2o_test.dir/main.cpp.o
../bin/g2o_test: CMakeFiles/g2o_test.dir/build.make
../bin/g2o_test: CMakeFiles/g2o_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xiangqian/C++projects/g2o_test/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/g2o_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/g2o_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/g2o_test.dir/build: ../bin/g2o_test

.PHONY : CMakeFiles/g2o_test.dir/build

CMakeFiles/g2o_test.dir/requires: CMakeFiles/g2o_test.dir/main.cpp.o.requires

.PHONY : CMakeFiles/g2o_test.dir/requires

CMakeFiles/g2o_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/g2o_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/g2o_test.dir/clean

CMakeFiles/g2o_test.dir/depend:
	cd /home/xiangqian/C++projects/g2o_test/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xiangqian/C++projects/g2o_test /home/xiangqian/C++projects/g2o_test /home/xiangqian/C++projects/g2o_test/cmake-build-debug /home/xiangqian/C++projects/g2o_test/cmake-build-debug /home/xiangqian/C++projects/g2o_test/cmake-build-debug/CMakeFiles/g2o_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/g2o_test.dir/depend

