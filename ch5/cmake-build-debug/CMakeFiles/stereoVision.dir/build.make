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
CMAKE_COMMAND = /home/book/clion-2020.2.5/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/book/clion-2020.2.5/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/book/SLAM_Learning/ch5

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/book/SLAM_Learning/ch5/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/stereoVision.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stereoVision.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stereoVision.dir/flags.make

CMakeFiles/stereoVision.dir/stereoVision/stereoVision.cpp.o: CMakeFiles/stereoVision.dir/flags.make
CMakeFiles/stereoVision.dir/stereoVision/stereoVision.cpp.o: ../stereoVision/stereoVision.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/book/SLAM_Learning/ch5/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stereoVision.dir/stereoVision/stereoVision.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereoVision.dir/stereoVision/stereoVision.cpp.o -c /home/book/SLAM_Learning/ch5/stereoVision/stereoVision.cpp

CMakeFiles/stereoVision.dir/stereoVision/stereoVision.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereoVision.dir/stereoVision/stereoVision.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/book/SLAM_Learning/ch5/stereoVision/stereoVision.cpp > CMakeFiles/stereoVision.dir/stereoVision/stereoVision.cpp.i

CMakeFiles/stereoVision.dir/stereoVision/stereoVision.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereoVision.dir/stereoVision/stereoVision.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/book/SLAM_Learning/ch5/stereoVision/stereoVision.cpp -o CMakeFiles/stereoVision.dir/stereoVision/stereoVision.cpp.s

# Object files for target stereoVision
stereoVision_OBJECTS = \
"CMakeFiles/stereoVision.dir/stereoVision/stereoVision.cpp.o"

# External object files for target stereoVision
stereoVision_EXTERNAL_OBJECTS =

stereoVision: CMakeFiles/stereoVision.dir/stereoVision/stereoVision.cpp.o
stereoVision: CMakeFiles/stereoVision.dir/build.make
stereoVision: /usr/local/lib/libopencv_dnn.so.3.4.16
stereoVision: /usr/local/lib/libopencv_highgui.so.3.4.16
stereoVision: /usr/local/lib/libopencv_ml.so.3.4.16
stereoVision: /usr/local/lib/libopencv_objdetect.so.3.4.16
stereoVision: /usr/local/lib/libopencv_shape.so.3.4.16
stereoVision: /usr/local/lib/libopencv_stitching.so.3.4.16
stereoVision: /usr/local/lib/libopencv_superres.so.3.4.16
stereoVision: /usr/local/lib/libopencv_videostab.so.3.4.16
stereoVision: /usr/local/lib/libopencv_viz.so.3.4.16
stereoVision: /usr/local/lib/libpangolin.so
stereoVision: /usr/local/lib/libopencv_calib3d.so.3.4.16
stereoVision: /usr/local/lib/libopencv_features2d.so.3.4.16
stereoVision: /usr/local/lib/libopencv_flann.so.3.4.16
stereoVision: /usr/local/lib/libopencv_photo.so.3.4.16
stereoVision: /usr/local/lib/libopencv_video.so.3.4.16
stereoVision: /usr/local/lib/libopencv_videoio.so.3.4.16
stereoVision: /usr/local/lib/libopencv_imgcodecs.so.3.4.16
stereoVision: /usr/local/lib/libopencv_imgproc.so.3.4.16
stereoVision: /usr/local/lib/libopencv_core.so.3.4.16
stereoVision: /usr/lib/x86_64-linux-gnu/libGL.so
stereoVision: /usr/lib/x86_64-linux-gnu/libGLU.so
stereoVision: /usr/lib/x86_64-linux-gnu/libGLEW.so
stereoVision: /usr/lib/x86_64-linux-gnu/libX11.so
stereoVision: /usr/lib/x86_64-linux-gnu/libXext.so
stereoVision: /usr/lib/x86_64-linux-gnu/libpng.so
stereoVision: /usr/lib/x86_64-linux-gnu/libz.so
stereoVision: CMakeFiles/stereoVision.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/book/SLAM_Learning/ch5/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable stereoVision"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stereoVision.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stereoVision.dir/build: stereoVision

.PHONY : CMakeFiles/stereoVision.dir/build

CMakeFiles/stereoVision.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stereoVision.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stereoVision.dir/clean

CMakeFiles/stereoVision.dir/depend:
	cd /home/book/SLAM_Learning/ch5/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/book/SLAM_Learning/ch5 /home/book/SLAM_Learning/ch5 /home/book/SLAM_Learning/ch5/cmake-build-debug /home/book/SLAM_Learning/ch5/cmake-build-debug /home/book/SLAM_Learning/ch5/cmake-build-debug/CMakeFiles/stereoVision.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stereoVision.dir/depend

