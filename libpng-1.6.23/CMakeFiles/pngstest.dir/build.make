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

# Include any dependencies generated for this target.
include CMakeFiles/pngstest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pngstest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pngstest.dir/flags.make

CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.o: CMakeFiles/pngstest.dir/flags.make
CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.o: contrib/libtests/pngstest.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Jaco/Documents/Research/MingleC/libpng-1.6.23/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.o   -c /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23/contrib/libtests/pngstest.c

CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23/contrib/libtests/pngstest.c > CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.i

CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23/contrib/libtests/pngstest.c -o CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.s

CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.o.requires:

.PHONY : CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.o.requires

CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.o.provides: CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.o.requires
	$(MAKE) -f CMakeFiles/pngstest.dir/build.make CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.o.provides.build
.PHONY : CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.o.provides

CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.o.provides.build: CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.o


# Object files for target pngstest
pngstest_OBJECTS = \
"CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.o"

# External object files for target pngstest
pngstest_EXTERNAL_OBJECTS =

pngstest: CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.o
pngstest: CMakeFiles/pngstest.dir/build.make
pngstest: libpng16.16.23.0.dylib
pngstest: /usr/lib/libz.dylib
pngstest: /usr/lib/libm.dylib
pngstest: CMakeFiles/pngstest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/Jaco/Documents/Research/MingleC/libpng-1.6.23/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable pngstest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pngstest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pngstest.dir/build: pngstest

.PHONY : CMakeFiles/pngstest.dir/build

CMakeFiles/pngstest.dir/requires: CMakeFiles/pngstest.dir/contrib/libtests/pngstest.c.o.requires

.PHONY : CMakeFiles/pngstest.dir/requires

CMakeFiles/pngstest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pngstest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pngstest.dir/clean

CMakeFiles/pngstest.dir/depend:
	cd /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23 /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23 /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23 /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23 /Users/Jaco/Documents/Research/MingleC/libpng-1.6.23/CMakeFiles/pngstest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pngstest.dir/depend

