# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.3

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.3.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.3.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23

# Utility rule file for genprebuilt.

# Include the progress variables for this target.
include CMakeFiles/genprebuilt.dir/progress.make

CMakeFiles/genprebuilt:
	/usr/local/Cellar/cmake/3.3.2/bin/cmake -DOUTPUT=scripts/pnglibconf.h.prebuilt -P /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23/scripts/gensrc.cmake

genprebuilt: CMakeFiles/genprebuilt
genprebuilt: CMakeFiles/genprebuilt.dir/build.make

.PHONY : genprebuilt

# Rule to build all files generated by this target.
CMakeFiles/genprebuilt.dir/build: genprebuilt

.PHONY : CMakeFiles/genprebuilt.dir/build

CMakeFiles/genprebuilt.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/genprebuilt.dir/cmake_clean.cmake
.PHONY : CMakeFiles/genprebuilt.dir/clean

CMakeFiles/genprebuilt.dir/depend:
	cd /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23 /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23 /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23 /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23 /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23/CMakeFiles/genprebuilt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/genprebuilt.dir/depend

