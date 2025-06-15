cmake_minimum_required(VERSION 3.0)
project(TrackingCamera)

# Assure either debug or release is set
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

# Set build flags for debug and release
set(SYMBOLS_FLAGS "-g -rdynamic -fno-omit-frame-pointer -ffunction-sections")
# Broken on piCore 14 and 15, ld can't find libasan_preinit.o but it's in /usr/local/lib
#set(SAN "-fsanitize=address")
set(SAN "")
set(LINK_FLAGS "-rdynamic")
set(LINK_FLAGS_DEBUG "-fno-omit-frame-pointer ${SAN}")
set(LINK_FLAGS_RELEASE "")
set(CMAKE_C_FLAGS "${SYMBOLS_FLAGS} -Werror -Wall -std=gnu17")
set(CMAKE_C_FLAGS_DEBUG "-O1 -fno-omit-frame-pointer ${SAN}")
set(CMAKE_C_FLAGS_RELEASE "-O3")
string(CONCAT CMAKE_CXX_FLAGS "${SYMBOLS_FLAGS} -std=c++17"
	" -Wall -Wno-sign-compare -Wno-pointer-arith -Wno-format-overflow"
	" -Wno-unused-label -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function")
set(CMAKE_CXX_FLAGS_DEBUG "-O1 -fno-omit-frame-pointer ${SAN}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Make sure all libraries are present
find_library(LIB_BCMH NAMES bcm_host)
find_library(LIB_VCOS NAMES vcos)
find_library(LIB_VCSM NAMES vcsm PATHS ~/TrackingCamera)

# Set program resources
set(INCLUDES
	/usr/local/include
	dependencies
	source
	../shared
	../libvcsm
)
set(SOURCES
	source/camera/gcs_v4l2.cpp
	source/qpu/mailbox.c
	source/qpu/qpu_base.c
	source/qpu/qpu_program.c
	source/qpu/qpu_info.c
	source/util/fbUtil.c
	source/comm/comm.cpp
	source/comm/server.cpp
	source/comm/uart.cpp
	source/comm/timesync.cpp
	source/comm/wireless.cpp
	source/blob/detection.cpp
	source/main.cpp
	../shared/util/image.cpp
	../shared/comm/protocol_stream.cpp
	../shared/blob/refinement.cpp
	../shared/blob/blob.cpp
)
set(LIBRARIES
	m dl pthread atomic
	${LIB_BCMH} ${LIB_VCOS} ${LIB_VCSM}
	turbojpeg
)

# Setup build for Zero 1
add_executable(TrackingCamera_armv6l ${SOURCES})
target_link_libraries(TrackingCamera_armv6l ${LIBRARIES})
target_include_directories(TrackingCamera_armv6l PRIVATE ${INCLUDES})
target_compile_options(TrackingCamera_armv6l PUBLIC -mcpu=arm1176jzf-s)
target_compile_definitions(TrackingCamera_armv6l PUBLIC ARMV6)
target_link_options(TrackingCamera_armv6l PUBLIC ${LINK_FLAGS})

# Setup build for Zero 2
add_executable(TrackingCamera_armv7l ${SOURCES})
target_link_libraries(TrackingCamera_armv7l ${LIBRARIES})
target_include_directories(TrackingCamera_armv7l PRIVATE ${INCLUDES})
target_compile_options(TrackingCamera_armv7l PUBLIC -mcpu=cortex-a53)
target_compile_definitions(TrackingCamera_armv7l PUBLIC ARMV7)
target_link_options(TrackingCamera_armv7l PUBLIC ${LINK_FLAGS})

# All ASM source files to build
set(QPU_PROGRAM_SOURCES
	qpu_programs/qpu_blob_tiled_min.asm
)

# Setup VC4ASM build target to compile all ASM source files
add_custom_target(qpu_programs)
foreach (file ${QPU_PROGRAM_SOURCES})
    get_filename_component(file_name ${file} NAME_WE)
    add_custom_command(
		OUTPUT ${file_name}.bin
		DEPENDS ${file}
	    COMMAND vc4asm -V -o ${CMAKE_CURRENT_BINARY_DIR}/${file_name}.bin ${CMAKE_SOURCE_DIR}/${file}
	)
    add_custom_target(${file_name}
		DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${file_name}.bin
	)
	add_dependencies(qpu_programs ${file_name})
endforeach()

# Make sure they are build by default
add_dependencies(TrackingCamera_armv6l qpu_programs)
add_dependencies(TrackingCamera_armv7l qpu_programs)
