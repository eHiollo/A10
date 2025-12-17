cmake_minimum_required(VERSION 3.12)
option(MYIR "myir compilation environment" OFF)
if(MYIR)
	set(CMAKE_C_COMPILER "/opt/poky/3.1.20/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-gcc")
	set(CMAKE_CXX_COMPILER "/opt/poky/3.1.20/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-g++")
endif(MYIR)

project(kaanhbin VERSION  2.5.2.240220 DESCRIPTION "")


# version
configure_file(${PROJECT_SOURCE_DIR}/config/kaanhbin_version.h.in kaanhbin_version.h)

# set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# set compile output path
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)

# find shared library
if(UNIX)
	if(MYIR)
		find_package(kaanhbot REQUIRED PATHS /opt/poky/3.1.20/sysroots/aarch64-poky-linux/usr/kaanhbot)
		message(STATUS "MYIR : ${MYIR}")
	else()
		find_package(kaanhbot REQUIRED PATHS /usr/kaanhbot)
	endif(MYIR)
elseif(WIN32)
	add_definitions(-DWIN32)
	add_definitions(-DNOMINMAX)
	add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/source-charset:utf-8>")
	add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/execution-charset:utf-8>")
	find_package(kaanhbot REQUIRED PATHS C:/kaanhbot)
endif()

# find Torch
# find_package(Torch REQUIRED PATHS /home/kaanh/libtorch/share/cmake/Torch)

get_filename_component(ARIS_LIBDIR "${aris_DIR}/../../../lib" ABSOLUTE)
message("aris lib: ${ARIS_LIBDIR}")
get_filename_component(KAANH_LIBDIR "${kaanh_DIR}/../../../lib" ABSOLUTE)
message("kaanhbot lib: ${KAANH_LIBDIR}")
get_filename_component(KAANHBOT_LIBDIR "${kaanhbot_DIR}/../../../lib" ABSOLUTE)
message("kaanhbot lib: ${KAANHBOT_LIBDIR}")
message("${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")


# build kaanh executable
add_executable(kaanhbin ${PROJECT_SOURCE_DIR}/main.cpp ${PROJECT_SOURCE_DIR}/robot.cpp 
${PROJECT_SOURCE_DIR}/robot.hpp ${PROJECT_SOURCE_DIR}/gravcomp.cpp ${PROJECT_SOURCE_DIR}/gravcomp.hpp 
${PROJECT_SOURCE_DIR}/curve.cpp ${PROJECT_SOURCE_DIR}/curve.hpp ${PROJECT_SOURCE_DIR}/assemcomd.cpp 
${PROJECT_SOURCE_DIR}/assemcomd.hpp 
${PROJECT_SOURCE_DIR}/datacomd.cpp ${PROJECT_SOURCE_DIR}/datacomd.hpp ${PROJECT_SOURCE_DIR}/rivetcomd.cpp 
${PROJECT_SOURCE_DIR}/rivetcomd.hpp 
${PROJECT_SOURCE_DIR}/searchDir.cpp ${PROJECT_SOURCE_DIR}/searchDir.hpp
${PROJECT_SOURCE_DIR}/test.cpp ${PROJECT_SOURCE_DIR}/test.hpp
${PROJECT_SOURCE_DIR}/movel.cpp ${PROJECT_SOURCE_DIR}/movel.hpp
${PROJECT_SOURCE_DIR}/a10_tcp_server.cpp ${PROJECT_SOURCE_DIR}/a10_tcp_server.hpp
${PROJECT_SOURCE_DIR}/master_slave.cpp ${PROJECT_SOURCE_DIR}/master_slave.hpp
${PROJECT_SOURCE_DIR}/gripper.cpp ${PROJECT_SOURCE_DIR}/gripper.hpp
)

target_include_directories(kaanhbin PUBLIC ${PROJECT_BINARY_DIR})
# target_include_directories(kaanhbin PUBLIC ${TORCH_INCLUDE_DIR})
target_link_libraries(kaanhbin PUBLIC ${kaanhbot_LIBRARIES})
# target_link_libraries(kaanhbin PUBLIC ${TORCH_LIBRARIES})

