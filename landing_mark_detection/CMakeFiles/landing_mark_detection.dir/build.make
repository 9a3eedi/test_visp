# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/mohammed/opencv_ws/testing_cpp/landing_mark_detection

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mohammed/opencv_ws/testing_cpp/landing_mark_detection

# Include any dependencies generated for this target.
include CMakeFiles/landing_mark_detection.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/landing_mark_detection.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/landing_mark_detection.dir/flags.make

CMakeFiles/landing_mark_detection.dir/main.cpp.o: CMakeFiles/landing_mark_detection.dir/flags.make
CMakeFiles/landing_mark_detection.dir/main.cpp.o: main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/mohammed/opencv_ws/testing_cpp/landing_mark_detection/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/landing_mark_detection.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/landing_mark_detection.dir/main.cpp.o -c /home/mohammed/opencv_ws/testing_cpp/landing_mark_detection/main.cpp

CMakeFiles/landing_mark_detection.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/landing_mark_detection.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/mohammed/opencv_ws/testing_cpp/landing_mark_detection/main.cpp > CMakeFiles/landing_mark_detection.dir/main.cpp.i

CMakeFiles/landing_mark_detection.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/landing_mark_detection.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/mohammed/opencv_ws/testing_cpp/landing_mark_detection/main.cpp -o CMakeFiles/landing_mark_detection.dir/main.cpp.s

CMakeFiles/landing_mark_detection.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/landing_mark_detection.dir/main.cpp.o.requires

CMakeFiles/landing_mark_detection.dir/main.cpp.o.provides: CMakeFiles/landing_mark_detection.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/landing_mark_detection.dir/build.make CMakeFiles/landing_mark_detection.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/landing_mark_detection.dir/main.cpp.o.provides

CMakeFiles/landing_mark_detection.dir/main.cpp.o.provides.build: CMakeFiles/landing_mark_detection.dir/main.cpp.o

# Object files for target landing_mark_detection
landing_mark_detection_OBJECTS = \
"CMakeFiles/landing_mark_detection.dir/main.cpp.o"

# External object files for target landing_mark_detection
landing_mark_detection_EXTERNAL_OBJECTS =

landing_mark_detection: CMakeFiles/landing_mark_detection.dir/main.cpp.o
landing_mark_detection: CMakeFiles/landing_mark_detection.dir/build.make
landing_mark_detection: /usr/local/lib/libopencv_calib3d.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_core.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_features2d.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_flann.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_highgui.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_imgproc.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_ml.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_objdetect.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_photo.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_shape.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_stitching.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_superres.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_video.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_videoio.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_videostab.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_viz.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_objdetect.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_calib3d.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_features2d.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_flann.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_highgui.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_ml.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_photo.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_video.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_videoio.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_imgproc.so.3.1.0
landing_mark_detection: /usr/local/lib/libopencv_core.so.3.1.0
landing_mark_detection: CMakeFiles/landing_mark_detection.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable landing_mark_detection"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/landing_mark_detection.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/landing_mark_detection.dir/build: landing_mark_detection
.PHONY : CMakeFiles/landing_mark_detection.dir/build

CMakeFiles/landing_mark_detection.dir/requires: CMakeFiles/landing_mark_detection.dir/main.cpp.o.requires
.PHONY : CMakeFiles/landing_mark_detection.dir/requires

CMakeFiles/landing_mark_detection.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/landing_mark_detection.dir/cmake_clean.cmake
.PHONY : CMakeFiles/landing_mark_detection.dir/clean

CMakeFiles/landing_mark_detection.dir/depend:
	cd /home/mohammed/opencv_ws/testing_cpp/landing_mark_detection && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mohammed/opencv_ws/testing_cpp/landing_mark_detection /home/mohammed/opencv_ws/testing_cpp/landing_mark_detection /home/mohammed/opencv_ws/testing_cpp/landing_mark_detection /home/mohammed/opencv_ws/testing_cpp/landing_mark_detection /home/mohammed/opencv_ws/testing_cpp/landing_mark_detection/CMakeFiles/landing_mark_detection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/landing_mark_detection.dir/depend

