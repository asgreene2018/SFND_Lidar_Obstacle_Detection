# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

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

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.19.1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.19.1/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/quiz/cluster

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build

# Include any dependencies generated for this target.
include CMakeFiles/quizCluster.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/quizCluster.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/quizCluster.dir/flags.make

CMakeFiles/quizCluster.dir/cluster.cpp.o: CMakeFiles/quizCluster.dir/flags.make
CMakeFiles/quizCluster.dir/cluster.cpp.o: ../cluster.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/quizCluster.dir/cluster.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/quizCluster.dir/cluster.cpp.o -c /Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/cluster.cpp

CMakeFiles/quizCluster.dir/cluster.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quizCluster.dir/cluster.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/cluster.cpp > CMakeFiles/quizCluster.dir/cluster.cpp.i

CMakeFiles/quizCluster.dir/cluster.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quizCluster.dir/cluster.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/cluster.cpp -o CMakeFiles/quizCluster.dir/cluster.cpp.s

CMakeFiles/quizCluster.dir/Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.o: CMakeFiles/quizCluster.dir/flags.make
CMakeFiles/quizCluster.dir/Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.o: /Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/render/render.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/quizCluster.dir/Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/quizCluster.dir/Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.o -c /Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/render/render.cpp

CMakeFiles/quizCluster.dir/Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quizCluster.dir/Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/render/render.cpp > CMakeFiles/quizCluster.dir/Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.i

CMakeFiles/quizCluster.dir/Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quizCluster.dir/Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/render/render.cpp -o CMakeFiles/quizCluster.dir/Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.s

# Object files for target quizCluster
quizCluster_OBJECTS = \
"CMakeFiles/quizCluster.dir/cluster.cpp.o" \
"CMakeFiles/quizCluster.dir/Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.o"

# External object files for target quizCluster
quizCluster_EXTERNAL_OBJECTS =

quizCluster: CMakeFiles/quizCluster.dir/cluster.cpp.o
quizCluster: CMakeFiles/quizCluster.dir/Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.o
quizCluster: CMakeFiles/quizCluster.dir/build.make
quizCluster: /usr/local/lib/libpcl_apps.dylib
quizCluster: /usr/local/lib/libpcl_outofcore.dylib
quizCluster: /usr/local/lib/libpcl_people.dylib
quizCluster: /usr/local/lib/libpcl_simulation.dylib
quizCluster: /usr/local/lib/libboost_system-mt.dylib
quizCluster: /usr/local/lib/libboost_filesystem-mt.dylib
quizCluster: /usr/local/lib/libboost_date_time-mt.dylib
quizCluster: /usr/local/lib/libboost_iostreams-mt.dylib
quizCluster: /usr/local/lib/libboost_regex-mt.dylib
quizCluster: /usr/local/lib/libqhull_p.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkChartsCore-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkInfovisCore-8.2.1.dylib
quizCluster: /Library/Developer/CommandLineTools/SDKs/MacOSX10.14.sdk/usr/lib/libz.tbd
quizCluster: /usr/local/lib/libjpeg.dylib
quizCluster: /usr/local/lib/libpng.dylib
quizCluster: /usr/local/lib/libtiff.dylib
quizCluster: /Library/Developer/CommandLineTools/SDKs/MacOSX10.14.sdk/usr/lib/libexpat.tbd
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkIOGeometry-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkIOLegacy-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkIOPLY-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkRenderingLOD-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkViewsContext2D-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkViewsCore-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkRenderingContextOpenGL2-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkRenderingOpenGL2-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkglew-8.2.1.dylib
quizCluster: /usr/local/lib/libflann_cpp.dylib
quizCluster: /usr/local/lib/libpcl_keypoints.dylib
quizCluster: /usr/local/lib/libpcl_tracking.dylib
quizCluster: /usr/local/lib/libpcl_recognition.dylib
quizCluster: /usr/local/lib/libpcl_registration.dylib
quizCluster: /usr/local/lib/libpcl_stereo.dylib
quizCluster: /usr/local/lib/libpcl_segmentation.dylib
quizCluster: /usr/local/lib/libpcl_ml.dylib
quizCluster: /usr/local/lib/libpcl_features.dylib
quizCluster: /usr/local/lib/libpcl_filters.dylib
quizCluster: /usr/local/lib/libpcl_sample_consensus.dylib
quizCluster: /usr/local/lib/libpcl_visualization.dylib
quizCluster: /usr/local/lib/libpcl_io.dylib
quizCluster: /usr/local/lib/libpcl_surface.dylib
quizCluster: /usr/local/lib/libpcl_search.dylib
quizCluster: /usr/local/lib/libpcl_kdtree.dylib
quizCluster: /usr/local/lib/libpcl_octree.dylib
quizCluster: /usr/local/lib/libpcl_common.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkInteractionWidgets-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkFiltersModeling-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkInteractionStyle-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkFiltersExtraction-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkFiltersStatistics-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkImagingFourier-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkFiltersHybrid-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkImagingGeneral-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkImagingSources-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkImagingHybrid-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkIOImage-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkDICOMParser-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkmetaio-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkRenderingAnnotation-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkImagingColor-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkRenderingVolume-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkIOXML-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkIOXMLParser-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkIOCore-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkdoubleconversion-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtklz4-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtklzma-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkImagingCore-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkRenderingContext2D-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkRenderingFreeType-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkfreetype-8.2.1.dylib
quizCluster: /Library/Developer/CommandLineTools/SDKs/MacOSX10.14.sdk/usr/lib/libz.tbd
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkRenderingCore-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonColor-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkFiltersGeometry-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkFiltersSources-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkFiltersGeneral-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonComputationalGeometry-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkFiltersCore-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonExecutionModel-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonDataModel-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonTransforms-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonMisc-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonMath-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonSystem-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonCore-8.2.1.dylib
quizCluster: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtksys-8.2.1.dylib
quizCluster: CMakeFiles/quizCluster.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable quizCluster"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/quizCluster.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/quizCluster.dir/build: quizCluster

.PHONY : CMakeFiles/quizCluster.dir/build

CMakeFiles/quizCluster.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/quizCluster.dir/cmake_clean.cmake
.PHONY : CMakeFiles/quizCluster.dir/clean

CMakeFiles/quizCluster.dir/depend:
	cd /Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/quiz/cluster /Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/quiz/cluster /Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build /Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build /Users/alexgreene/Documents/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build/CMakeFiles/quizCluster.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/quizCluster.dir/depend

