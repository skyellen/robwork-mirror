# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.6

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lpe/workspace/RobWork

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lpe/workspace/RobWork/build/release

# Include any dependencies generated for this target.
include ext/lua/CMakeFiles/lua.dir/depend.make

# Include the progress variables for this target.
include ext/lua/CMakeFiles/lua.dir/progress.make

# Include the compile flags for this target's objects.
include ext/lua/CMakeFiles/lua.dir/flags.make

ext/lua/CMakeFiles/lua.dir/src/lua.o: ext/lua/CMakeFiles/lua.dir/flags.make
ext/lua/CMakeFiles/lua.dir/src/lua.o: ../../ext/lua/src/lua.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lpe/workspace/RobWork/build/release/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object ext/lua/CMakeFiles/lua.dir/src/lua.o"
	cd /home/lpe/workspace/RobWork/build/release/ext/lua && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/lua.dir/src/lua.o -c /home/lpe/workspace/RobWork/ext/lua/src/lua.cpp

ext/lua/CMakeFiles/lua.dir/src/lua.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lua.dir/src/lua.i"
	cd /home/lpe/workspace/RobWork/build/release/ext/lua && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lpe/workspace/RobWork/ext/lua/src/lua.cpp > CMakeFiles/lua.dir/src/lua.i

ext/lua/CMakeFiles/lua.dir/src/lua.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lua.dir/src/lua.s"
	cd /home/lpe/workspace/RobWork/build/release/ext/lua && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lpe/workspace/RobWork/ext/lua/src/lua.cpp -o CMakeFiles/lua.dir/src/lua.s

ext/lua/CMakeFiles/lua.dir/src/lua.o.requires:
.PHONY : ext/lua/CMakeFiles/lua.dir/src/lua.o.requires

ext/lua/CMakeFiles/lua.dir/src/lua.o.provides: ext/lua/CMakeFiles/lua.dir/src/lua.o.requires
	$(MAKE) -f ext/lua/CMakeFiles/lua.dir/build.make ext/lua/CMakeFiles/lua.dir/src/lua.o.provides.build
.PHONY : ext/lua/CMakeFiles/lua.dir/src/lua.o.provides

ext/lua/CMakeFiles/lua.dir/src/lua.o.provides.build: ext/lua/CMakeFiles/lua.dir/src/lua.o
.PHONY : ext/lua/CMakeFiles/lua.dir/src/lua.o.provides.build

# Object files for target lua
lua_OBJECTS = \
"CMakeFiles/lua.dir/src/lua.o"

# External object files for target lua
lua_EXTERNAL_OBJECTS =

../../bin/Release/lua: ext/lua/CMakeFiles/lua.dir/src/lua.o
../../bin/Release/lua: ../../libs/Release/liblualib.a
../../bin/Release/lua: ext/lua/CMakeFiles/lua.dir/build.make
../../bin/Release/lua: ext/lua/CMakeFiles/lua.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../../../bin/Release/lua"
	cd /home/lpe/workspace/RobWork/build/release/ext/lua && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lua.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ext/lua/CMakeFiles/lua.dir/build: ../../bin/Release/lua
.PHONY : ext/lua/CMakeFiles/lua.dir/build

ext/lua/CMakeFiles/lua.dir/requires: ext/lua/CMakeFiles/lua.dir/src/lua.o.requires
.PHONY : ext/lua/CMakeFiles/lua.dir/requires

ext/lua/CMakeFiles/lua.dir/clean:
	cd /home/lpe/workspace/RobWork/build/release/ext/lua && $(CMAKE_COMMAND) -P CMakeFiles/lua.dir/cmake_clean.cmake
.PHONY : ext/lua/CMakeFiles/lua.dir/clean

ext/lua/CMakeFiles/lua.dir/depend:
	cd /home/lpe/workspace/RobWork/build/release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lpe/workspace/RobWork /home/lpe/workspace/RobWork/ext/lua /home/lpe/workspace/RobWork/build/release /home/lpe/workspace/RobWork/build/release/ext/lua /home/lpe/workspace/RobWork/build/release/ext/lua/CMakeFiles/lua.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ext/lua/CMakeFiles/lua.dir/depend

