# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /home/jungwon/clion-2018.2.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/jungwon/clion-2018.2.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jungwon/catkin_ws/src/mavswarm_client

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jungwon/catkin_ws/src/mavswarm_client/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/mavswarm_client.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mavswarm_client.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mavswarm_client.dir/flags.make

CMakeFiles/mavswarm_client.dir/src/client.cpp.o: CMakeFiles/mavswarm_client.dir/flags.make
CMakeFiles/mavswarm_client.dir/src/client.cpp.o: ../src/client.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jungwon/catkin_ws/src/mavswarm_client/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mavswarm_client.dir/src/client.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mavswarm_client.dir/src/client.cpp.o -c /home/jungwon/catkin_ws/src/mavswarm_client/src/client.cpp

CMakeFiles/mavswarm_client.dir/src/client.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mavswarm_client.dir/src/client.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jungwon/catkin_ws/src/mavswarm_client/src/client.cpp > CMakeFiles/mavswarm_client.dir/src/client.cpp.i

CMakeFiles/mavswarm_client.dir/src/client.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mavswarm_client.dir/src/client.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jungwon/catkin_ws/src/mavswarm_client/src/client.cpp -o CMakeFiles/mavswarm_client.dir/src/client.cpp.s

CMakeFiles/mavswarm_client.dir/src/USBDevice.cpp.o: CMakeFiles/mavswarm_client.dir/flags.make
CMakeFiles/mavswarm_client.dir/src/USBDevice.cpp.o: ../src/USBDevice.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jungwon/catkin_ws/src/mavswarm_client/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/mavswarm_client.dir/src/USBDevice.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mavswarm_client.dir/src/USBDevice.cpp.o -c /home/jungwon/catkin_ws/src/mavswarm_client/src/USBDevice.cpp

CMakeFiles/mavswarm_client.dir/src/USBDevice.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mavswarm_client.dir/src/USBDevice.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jungwon/catkin_ws/src/mavswarm_client/src/USBDevice.cpp > CMakeFiles/mavswarm_client.dir/src/USBDevice.cpp.i

CMakeFiles/mavswarm_client.dir/src/USBDevice.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mavswarm_client.dir/src/USBDevice.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jungwon/catkin_ws/src/mavswarm_client/src/USBDevice.cpp -o CMakeFiles/mavswarm_client.dir/src/USBDevice.cpp.s

CMakeFiles/mavswarm_client.dir/src/Crazyradio.cpp.o: CMakeFiles/mavswarm_client.dir/flags.make
CMakeFiles/mavswarm_client.dir/src/Crazyradio.cpp.o: ../src/Crazyradio.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jungwon/catkin_ws/src/mavswarm_client/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/mavswarm_client.dir/src/Crazyradio.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mavswarm_client.dir/src/Crazyradio.cpp.o -c /home/jungwon/catkin_ws/src/mavswarm_client/src/Crazyradio.cpp

CMakeFiles/mavswarm_client.dir/src/Crazyradio.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mavswarm_client.dir/src/Crazyradio.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jungwon/catkin_ws/src/mavswarm_client/src/Crazyradio.cpp > CMakeFiles/mavswarm_client.dir/src/Crazyradio.cpp.i

CMakeFiles/mavswarm_client.dir/src/Crazyradio.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mavswarm_client.dir/src/Crazyradio.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jungwon/catkin_ws/src/mavswarm_client/src/Crazyradio.cpp -o CMakeFiles/mavswarm_client.dir/src/Crazyradio.cpp.s

CMakeFiles/mavswarm_client.dir/src/CrazyflieUSB.cpp.o: CMakeFiles/mavswarm_client.dir/flags.make
CMakeFiles/mavswarm_client.dir/src/CrazyflieUSB.cpp.o: ../src/CrazyflieUSB.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jungwon/catkin_ws/src/mavswarm_client/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/mavswarm_client.dir/src/CrazyflieUSB.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mavswarm_client.dir/src/CrazyflieUSB.cpp.o -c /home/jungwon/catkin_ws/src/mavswarm_client/src/CrazyflieUSB.cpp

CMakeFiles/mavswarm_client.dir/src/CrazyflieUSB.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mavswarm_client.dir/src/CrazyflieUSB.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jungwon/catkin_ws/src/mavswarm_client/src/CrazyflieUSB.cpp > CMakeFiles/mavswarm_client.dir/src/CrazyflieUSB.cpp.i

CMakeFiles/mavswarm_client.dir/src/CrazyflieUSB.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mavswarm_client.dir/src/CrazyflieUSB.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jungwon/catkin_ws/src/mavswarm_client/src/CrazyflieUSB.cpp -o CMakeFiles/mavswarm_client.dir/src/CrazyflieUSB.cpp.s

CMakeFiles/mavswarm_client.dir/src/Crazyflie.cpp.o: CMakeFiles/mavswarm_client.dir/flags.make
CMakeFiles/mavswarm_client.dir/src/Crazyflie.cpp.o: ../src/Crazyflie.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jungwon/catkin_ws/src/mavswarm_client/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/mavswarm_client.dir/src/Crazyflie.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mavswarm_client.dir/src/Crazyflie.cpp.o -c /home/jungwon/catkin_ws/src/mavswarm_client/src/Crazyflie.cpp

CMakeFiles/mavswarm_client.dir/src/Crazyflie.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mavswarm_client.dir/src/Crazyflie.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jungwon/catkin_ws/src/mavswarm_client/src/Crazyflie.cpp > CMakeFiles/mavswarm_client.dir/src/Crazyflie.cpp.i

CMakeFiles/mavswarm_client.dir/src/Crazyflie.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mavswarm_client.dir/src/Crazyflie.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jungwon/catkin_ws/src/mavswarm_client/src/Crazyflie.cpp -o CMakeFiles/mavswarm_client.dir/src/Crazyflie.cpp.s

CMakeFiles/mavswarm_client.dir/src/crtp.cpp.o: CMakeFiles/mavswarm_client.dir/flags.make
CMakeFiles/mavswarm_client.dir/src/crtp.cpp.o: ../src/crtp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jungwon/catkin_ws/src/mavswarm_client/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/mavswarm_client.dir/src/crtp.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mavswarm_client.dir/src/crtp.cpp.o -c /home/jungwon/catkin_ws/src/mavswarm_client/src/crtp.cpp

CMakeFiles/mavswarm_client.dir/src/crtp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mavswarm_client.dir/src/crtp.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jungwon/catkin_ws/src/mavswarm_client/src/crtp.cpp > CMakeFiles/mavswarm_client.dir/src/crtp.cpp.i

CMakeFiles/mavswarm_client.dir/src/crtp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mavswarm_client.dir/src/crtp.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jungwon/catkin_ws/src/mavswarm_client/src/crtp.cpp -o CMakeFiles/mavswarm_client.dir/src/crtp.cpp.s

CMakeFiles/mavswarm_client.dir/src/ITransport.cpp.o: CMakeFiles/mavswarm_client.dir/flags.make
CMakeFiles/mavswarm_client.dir/src/ITransport.cpp.o: ../src/ITransport.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jungwon/catkin_ws/src/mavswarm_client/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/mavswarm_client.dir/src/ITransport.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mavswarm_client.dir/src/ITransport.cpp.o -c /home/jungwon/catkin_ws/src/mavswarm_client/src/ITransport.cpp

CMakeFiles/mavswarm_client.dir/src/ITransport.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mavswarm_client.dir/src/ITransport.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jungwon/catkin_ws/src/mavswarm_client/src/ITransport.cpp > CMakeFiles/mavswarm_client.dir/src/ITransport.cpp.i

CMakeFiles/mavswarm_client.dir/src/ITransport.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mavswarm_client.dir/src/ITransport.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jungwon/catkin_ws/src/mavswarm_client/src/ITransport.cpp -o CMakeFiles/mavswarm_client.dir/src/ITransport.cpp.s

# Object files for target mavswarm_client
mavswarm_client_OBJECTS = \
"CMakeFiles/mavswarm_client.dir/src/client.cpp.o" \
"CMakeFiles/mavswarm_client.dir/src/USBDevice.cpp.o" \
"CMakeFiles/mavswarm_client.dir/src/Crazyradio.cpp.o" \
"CMakeFiles/mavswarm_client.dir/src/CrazyflieUSB.cpp.o" \
"CMakeFiles/mavswarm_client.dir/src/Crazyflie.cpp.o" \
"CMakeFiles/mavswarm_client.dir/src/crtp.cpp.o" \
"CMakeFiles/mavswarm_client.dir/src/ITransport.cpp.o"

# External object files for target mavswarm_client
mavswarm_client_EXTERNAL_OBJECTS =

devel/lib/libmavswarm_client.so: CMakeFiles/mavswarm_client.dir/src/client.cpp.o
devel/lib/libmavswarm_client.so: CMakeFiles/mavswarm_client.dir/src/USBDevice.cpp.o
devel/lib/libmavswarm_client.so: CMakeFiles/mavswarm_client.dir/src/Crazyradio.cpp.o
devel/lib/libmavswarm_client.so: CMakeFiles/mavswarm_client.dir/src/CrazyflieUSB.cpp.o
devel/lib/libmavswarm_client.so: CMakeFiles/mavswarm_client.dir/src/Crazyflie.cpp.o
devel/lib/libmavswarm_client.so: CMakeFiles/mavswarm_client.dir/src/crtp.cpp.o
devel/lib/libmavswarm_client.so: CMakeFiles/mavswarm_client.dir/src/ITransport.cpp.o
devel/lib/libmavswarm_client.so: CMakeFiles/mavswarm_client.dir/build.make
devel/lib/libmavswarm_client.so: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
devel/lib/libmavswarm_client.so: CMakeFiles/mavswarm_client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jungwon/catkin_ws/src/mavswarm_client/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX shared library devel/lib/libmavswarm_client.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mavswarm_client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mavswarm_client.dir/build: devel/lib/libmavswarm_client.so

.PHONY : CMakeFiles/mavswarm_client.dir/build

CMakeFiles/mavswarm_client.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mavswarm_client.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mavswarm_client.dir/clean

CMakeFiles/mavswarm_client.dir/depend:
	cd /home/jungwon/catkin_ws/src/mavswarm_client/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jungwon/catkin_ws/src/mavswarm_client /home/jungwon/catkin_ws/src/mavswarm_client /home/jungwon/catkin_ws/src/mavswarm_client/cmake-build-debug /home/jungwon/catkin_ws/src/mavswarm_client/cmake-build-debug /home/jungwon/catkin_ws/src/mavswarm_client/cmake-build-debug/CMakeFiles/mavswarm_client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mavswarm_client.dir/depend

