# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/juan/experiments-2021/runtime/libomp_oss

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/juan/experiments-2021/runtime/libomp_oss/build

# Utility rule file for libomp-lib.

# Include the progress variables for this target.
include src/CMakeFiles/libomp-lib.dir/progress.make

src/CMakeFiles/libomp-lib: src/libiomp5.so


libomp-lib: src/CMakeFiles/libomp-lib
libomp-lib: src/CMakeFiles/libomp-lib.dir/build.make

.PHONY : libomp-lib

# Rule to build all files generated by this target.
src/CMakeFiles/libomp-lib.dir/build: libomp-lib

.PHONY : src/CMakeFiles/libomp-lib.dir/build

src/CMakeFiles/libomp-lib.dir/clean:
	cd /home/juan/experiments-2021/runtime/libomp_oss/build/src && $(CMAKE_COMMAND) -P CMakeFiles/libomp-lib.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/libomp-lib.dir/clean

src/CMakeFiles/libomp-lib.dir/depend:
	cd /home/juan/experiments-2021/runtime/libomp_oss/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/juan/experiments-2021/runtime/libomp_oss /home/juan/experiments-2021/runtime/libomp_oss/src /home/juan/experiments-2021/runtime/libomp_oss/build /home/juan/experiments-2021/runtime/libomp_oss/build/src /home/juan/experiments-2021/runtime/libomp_oss/build/src/CMakeFiles/libomp-lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/libomp-lib.dir/depend
