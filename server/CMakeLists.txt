cmake_minimum_required(VERSION 3.0)
project(AsterTrack)

# Assure either debug or release is set
if (NOT CMAKE_BUILD_TYPE)
	message(Defaulting to Release build...)
	set(CMAKE_BUILD_TYPE Release)
else()
	string(TOLOWER "${CMAKE_BUILD_TYPE}" buildType)
	if(buildType STREQUAL release)
		set(CMAKE_BUILD_TYPE Release)
	else()
		set(CMAKE_BUILD_TYPE Debug)
	endif()
endif()

# Accumulate sources
set(SERVER
	app.cpp server.cpp config.cpp simulation.cpp
	comm/usb.cpp comm/protocol_packet.cpp
	comm/streaming.cpp comm/timesync.cpp
	device/tracking_device.cpp device/parsing.cpp
	io/vrpn.cpp
	pipeline/pipeline.cpp pipeline/pipeline_tracking.cpp
	pipeline/pipeline_calibration_point.cpp pipeline/pipeline_calibration_target.cpp
	calib/obs_data.cpp calib/optimisation.cpp
	calib_point/reconstruction.cpp calib_point/calibration_room.cpp
	calib_target/reconstruction.cpp calib_target/aquisition.cpp calib_target/assembly.cpp
	point/sequence_data.cpp
	point/sequences2D.cpp point/triangulation.cpp point/tracking3D.cpp
	target/target.cpp target/detection2D.cpp target/detection3D.cpp
	target/tracking2D.cpp target/tracking3D.cpp
)
set(SERVER_SHARED
	util/image.cpp
)
set(INTERFACE
	ui/ui.cpp ui/menu.cpp
	ui/pipeline.cpp ui/control.cpp
	ui/pipeline_calibration_point.cpp ui/pipeline_calibration_target.cpp
	ui/insights.cpp ui/logging.cpp
	ui/devices.cpp ui/wireless.cpp ui/cameraSettings.cpp
	ui/parameters/sequence2D.cpp ui/parameters/calibration_point.cpp ui/parameters/calibration_target.cpp ui/parameters/tracking.cpp
	ui/targets.cpp
	ui/tools/lens_selection.cpp
	ui/cameraPanels.cpp ui/cameraView.cpp ui/view3D.cpp ui/visSettings.cpp
	ui/system/state.cpp ui/system/visGeneral.cpp ui/system/visTarget.cpp
	ui/gl/visualisation.cpp ui/gl/sharedGL.cpp
	ui/gl/mesh.cpp ui/gl/shader.cpp
	ui/imgui/imgui_custom.cpp ui/imgui/imgui_onDemand.cpp
	emulation/detection.cpp
)
set(INTERFACE_SHARED
	blob/refinement.cpp blob/blob.cpp
)
set(DEPENDENCIES
	imgui/imgui.cpp imgui/imgui_draw.cpp
	imgui/imgui_widgets.cpp imgui/imgui_tables.cpp
	imgui/backends/imgui_impl_glfw.cpp imgui/backends/imgui_impl_opengl3.cpp
	imgui/misc/cpp/imgui_stdlib.cpp
	imgui/imgui_demo.cpp
	imguizmo/ImSequencer.cpp
	implot/implot.cpp implot/implot_items.cpp implot/implot_demo.cpp
	recursive_shared_mutex/recursive_shared_mutex.cpp
	poselib/p3p_ding.cpp
	dbscan/dbscan.cpp
	GL/glew.c
	stb/stb_image.c
)
set(SOURCES_FILES "")
foreach(it ${SERVER})
	list(APPEND SOURCES_FILES "${PROJECT_SOURCE_DIR}/source/${it}")
endforeach()
foreach(it ${SERVER_SHARED})
	list(APPEND SOURCES_FILES "${PROJECT_SOURCE_DIR}/../shared/${it}")
endforeach()
foreach(it ${INTERFACE})
	list(APPEND SOURCES_FILES "${PROJECT_SOURCE_DIR}/source/${it}")
endforeach()
foreach(it ${INTERFACE_SHARED})
	list(APPEND SOURCES_FILES "${PROJECT_SOURCE_DIR}/../shared/${it}")
endforeach()
foreach(it ${DEPENDENCIES})
	list(APPEND SOURCES_FILES "${PROJECT_SOURCE_DIR}/dependencies/sources/${it}")
endforeach()

# Register application (with GUI entry point on windows)
add_executable(astertrack WIN32 ${SOURCES_FILES})

# Add include directories
target_include_directories(astertrack PUBLIC "${PROJECT_SOURCE_DIR}/source")
target_include_directories(astertrack PUBLIC "${PROJECT_SOURCE_DIR}/../shared")
target_include_directories(astertrack PRIVATE "${PROJECT_SOURCE_DIR}/dependencies/include")
target_include_directories(astertrack PRIVATE "${PROJECT_SOURCE_DIR}/dependencies/sources")
target_include_directories(astertrack PRIVATE "${PROJECT_SOURCE_DIR}/dependencies/sources/imgui")

# Specify language features used
target_compile_features(astertrack PRIVATE cxx_generalized_initializers cxx_static_assert cxx_std_20)

# Specify definitions
target_compile_definitions(astertrack PUBLIC GLEW_STATIC _UNICODE
	EIGEN_MPL2_ONLY EIGEN_NO_AUTOMATIC_RESIZING EIGEN_INITIALIZE_MATRICES_BY_NAN BLOB_EMULATION)
target_compile_definitions(astertrack PUBLIC $<$<CONFIG:Debug>:_DEBUG>)
target_compile_definitions(astertrack PUBLIC $<$<CONFIG:Release>:NDEBUG EIGEN_NO_DEBUG>)

# Disable linking of default system libraries, do it explicitly later
set(CMAKE_C_STANDARD_LIBRARIES_INIT "")
set(CMAKE_CXX_STANDARD_LIBRARIES_INIT "")
set(CMAKE_CXX_STANDARD_LIBRARIES "")

if(MSVC)
	set(PLT_LIB_DIR "${PROJECT_SOURCE_DIR}/dependencies/lib/win")

	# MSVC-specific compiler setup
	target_compile_definitions(astertrack PUBLIC _CRT_SECURE_NO_WARNINGS _USE_MATH_DEFINES _ENABLE_EXTENDED_ALIGNED_STORAGE INTERFACE_LINKED)
	target_compile_options(astertrack PUBLIC -nologo -bigobj -EHsc -W1)
	target_link_options(astertrack PUBLIC -subsystem:windows -ENTRY:mainCRTStartup) # re-route GUI entry point to main
#	target_compile_options(astertrack PUBLIC "-Wall -wd4477")
	target_compile_options(astertrack PUBLIC "/wd4348")

	# And because libusb defaults to /MD and msbuild does not allow you to override it, libusb is built with wrong runtime libs
	# Need to force linker to use this now
	# Update: Forcefully overrode setting with environment variable for libusb build, so this workaround is not needed anymore
	#target_link_options(astertrack PUBLIC /NODEFAULTLIB:msvcrt.lib /NODEFAULTLIB:msvcprt.lib)

	# Configuration-specific setup
	# Gave up on windows dual Release/Debug builds, support for that is lackluster in CMake at best
	if(CMAKE_BUILD_TYPE STREQUAL Debug)
		set(LIB_DIR ${PLT_LIB_DIR}/debug)
		target_compile_options(astertrack PUBLIC -MTd -Z7 -O2)
		set(MSVC_RUNTIME_LIBRARY MultiThreadedDebug)
		STRING (REGEX REPLACE "/RTC(su|[1su])" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
		STRING (REGEX REPLACE "/RTC(su|[1su])" "" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
	else()
		set(LIB_DIR ${PLT_LIB_DIR}/release)
		target_compile_options(astertrack PUBLIC -MT -O2)
		set(MSVC_RUNTIME_LIBRARY MultiThreaded)
	endif()

	# Find local libraries
	file(GLOB LIB_LUSB ${LIB_DIR}/libusb/libusb*.lib)
	file(GLOB LIB_VRPN ${LIB_DIR}/vrpn/*.lib)
	file(GLOB LIB_GLFW ${LIB_DIR}/glfw3.lib)
	if(LIB_LUSB STREQUAL "" OR LIB_VRPN STREQUAL "" OR LIB_GLFW STREQUAL "")
		message(STATUS "LIB_DIR: '${LIB_DIR}'")
		message(STATUS "LIB_LUSB: '${LIB_LUSB}'")
		message(STATUS "LIB_VRPN: '${LIB_VRPN}'")
		message(STATUS "LIB_GLFW: '${LIB_GLFW}'")
		message(FATAL_ERROR "Failed to find libraries! You may have to build the dependencies for the specific mode (release/debug)")
	endif()
	find_library(LIB_TJPEG NAMES turbojpeg-static PATHS ${PLT_LIB_DIR} "C:/libjpeg-turbo64/lib") # Built with -MT by default
	target_include_directories(astertrack PRIVATE "C:/libjpeg-turbo64/include")

	# Force CRT library to be statically linked (allows static linking with prebuilt turbojpeg which as been built with -MT instead of -MD)
	set_property(TARGET astertrack APPEND_STRING PROPERTY LINK_FLAGS " -NODEFAULTLIB:msvcrt -NODEFAULTLIB:msvcrtd -NODEFAULTLIB:msvcprt -NODEFAULTLIB:msvcprtd")
	set_property(TARGET astertrack APPEND_STRING PROPERTY LINK_FLAGS " -NODEFAULTLIB:ucrt -NODEFAULTLIB:ucrtd")
	set_property(TARGET astertrack APPEND_STRING PROPERTY LINK_FLAGS " -NODEFAULTLIB:cmt")

	# Link libraries
	target_link_libraries(astertrack ${LIB_VRPN} ${LIB_TJPEG} ${LIB_GLFW} ${LIB_LUSB})
	target_link_libraries(astertrack opengl32 gdi32.lib shell32.lib)

	# Add windows-specific resource file
	target_sources(astertrack PRIVATE "${PROJECT_SOURCE_DIR}/resources/conf.rs")

	# Prevent MSVC from putting it in /Debug or /Release subdirectories
	set_target_properties(astertrack PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR})
	set_target_properties(astertrack PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR})

	# Copy dlls post-build
	add_custom_command(TARGET astertrack POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different
		"${LIB_DIR}/libusb/libusb-1.0.dll" $<TARGET_FILE_DIR:astertrack>)

else()
	set(LIB_DIR "${PROJECT_SOURCE_DIR}/dependencies/lib/linux")

	# gcc-specific compiler setup for Debug and Release
	target_compile_definitions(astertrack PUBLIC GLEW_EGL _FILE_OFFSET_BITS=64)
	target_compile_options(astertrack PUBLIC "-msse4")
	target_compile_options(astertrack PUBLIC "$<$<CONFIG:Debug>:-g>")
	target_compile_options(astertrack PUBLIC "$<$<CONFIG:Release>:-O3>")
	target_compile_options(astertrack PUBLIC "-Wall")

	# Link specific libraries
	target_link_libraries(astertrack ${LIB_DIR}/libusb/libusb-1.0.so)
	target_link_libraries(astertrack ${LIB_DIR}/vrpn/libvrpn.a)
	target_link_libraries(astertrack ${LIB_DIR}/vrpn/libquat.a)
	target_link_libraries(astertrack ${LIB_DIR}/libglfw3.a)

	set_target_properties(astertrack PROPERTIES LINK_OPTIONS "-Wl,-gc-sections;-Wl,-rpath='$ORIGIN'")
	set_target_properties(astertrack PROPERTIES LINK_OPTIONS_DEBUG "")
	set_target_properties(astertrack PROPERTIES LINK_OPTIONS_RELEASE "")

	# Link system libraries
	target_link_libraries(astertrack "-lrt -lGL -lturbojpeg -lgomp")

	if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	else() # GCC
	endif()
endif()