# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/book/SLAM_Learning/ch2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/book/SLAM_Learning/ch2/build

# Include any dependencies generated for this target.
include CMakeFiles/lib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/lib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lib.dir/flags.make

CMakeFiles/lib.dir/libtest.cpp.o: CMakeFiles/lib.dir/flags.make
CMakeFiles/lib.dir/libtest.cpp.o: ../libtest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/book/SLAM_Learning/ch2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/lib.dir/libtest.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib.dir/libtest.cpp.o -c /home/book/SLAM_Learning/ch2/libtest.cpp

CMakeFiles/lib.dir/libtest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib.dir/libtest.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/book/SLAM_Learning/ch2/libtest.cpp > CMakeFiles/lib.dir/libtest.cpp.i

CMakeFiles/lib.dir/libtest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib.dir/libtest.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/book/SLAM_Learning/ch2/libtest.cpp -o CMakeFiles/lib.dir/libtest.cpp.s

# Object files for target lib
lib_OBJECTS = \
"CMakeFiles/lib.dir/libtest.cpp.o"

# External object files for target lib
lib_EXTERNAL_OBJECTS =

liblib.so: CMakeFiles/lib.dir/libtest.cpp.o
liblib.so: CMakeFiles/lib.dir/build.make
liblib.so: CMakeFiles/lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/book/SLAM_Learning/ch2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library liblib.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lib.dir/build: liblib.so

.PHONY : CMakeFiles/lib.dir/build

CMakeFiles/lib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lib.dir/clean

CMakeFiles/lib.dir/depend:
	cd /home/book/SLAM_Learning/ch2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/book/SLAM_Learning/ch2 /home/book/SLAM_Learning/ch2 /home/book/SLAM_Learning/ch2/build /home/book/SLAM_Learning/ch2/build /home/book/SLAM_Learning/ch2/build/CMakeFiles/lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lib.dir/depend
