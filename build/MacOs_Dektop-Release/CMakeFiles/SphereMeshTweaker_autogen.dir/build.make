# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

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
CMAKE_COMMAND = /Applications/CMake.app/Contents/bin/cmake

# The command to remove a file.
RM = /Applications/CMake.app/Contents/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/davidepaollilo/Workspaces/C++/SphereMeshTweaker

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/davidepaollilo/Workspaces/C++/SphereMeshTweaker/build/MacOs_Dektop-Release

# Utility rule file for SphereMeshTweaker_autogen.

# Include any custom commands dependencies for this target.
include CMakeFiles/SphereMeshTweaker_autogen.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/SphereMeshTweaker_autogen.dir/progress.make

CMakeFiles/SphereMeshTweaker_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/davidepaollilo/Workspaces/C++/SphereMeshTweaker/build/MacOs_Dektop-Release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC and UIC for target SphereMeshTweaker"
	/Applications/CMake.app/Contents/bin/cmake -E cmake_autogen /Users/davidepaollilo/Workspaces/C++/SphereMeshTweaker/build/MacOs_Dektop-Release/CMakeFiles/SphereMeshTweaker_autogen.dir/AutogenInfo.json Release

SphereMeshTweaker_autogen: CMakeFiles/SphereMeshTweaker_autogen
SphereMeshTweaker_autogen: CMakeFiles/SphereMeshTweaker_autogen.dir/build.make
.PHONY : SphereMeshTweaker_autogen

# Rule to build all files generated by this target.
CMakeFiles/SphereMeshTweaker_autogen.dir/build: SphereMeshTweaker_autogen
.PHONY : CMakeFiles/SphereMeshTweaker_autogen.dir/build

CMakeFiles/SphereMeshTweaker_autogen.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SphereMeshTweaker_autogen.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SphereMeshTweaker_autogen.dir/clean

CMakeFiles/SphereMeshTweaker_autogen.dir/depend:
	cd /Users/davidepaollilo/Workspaces/C++/SphereMeshTweaker/build/MacOs_Dektop-Release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/davidepaollilo/Workspaces/C++/SphereMeshTweaker /Users/davidepaollilo/Workspaces/C++/SphereMeshTweaker /Users/davidepaollilo/Workspaces/C++/SphereMeshTweaker/build/MacOs_Dektop-Release /Users/davidepaollilo/Workspaces/C++/SphereMeshTweaker/build/MacOs_Dektop-Release /Users/davidepaollilo/Workspaces/C++/SphereMeshTweaker/build/MacOs_Dektop-Release/CMakeFiles/SphereMeshTweaker_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SphereMeshTweaker_autogen.dir/depend

