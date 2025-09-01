# To be used with GNU Make


# Read implicit compiler defaults if not explicitly specified
ifneq ($(COMPILER), clang-libc)
ifneq ($(COMPILER), clang-gnu)
ifneq ($(COMPILER), gcc)
ifneq '' '$(findstring clang,$(CC))'
COMPILER=clang-gnu
else
COMPILER=gcc
endif
endif
endif
endif

# Base architecture to target
# v2 has better compatibility with older processors
# v3 is much better for linear algebra heavy applications such as ours
# But it will not run on very old processors or very low end Intel Core processors
# v4 is even better but too recent so we're compiling sensitive functions for x86-64-v4 with multi-target
march = -march=x86-64-v3

# Set compiler and options used
ifeq ($(COMPILER), clang-libc)
# clang compiler
CXX=clang++
CC=clang
LD=ld.lld
# clang libraries (libc++ + libomp)
cstd = -stdlib=libc++ -fopenmp=libomp -Wno-gcc-compat
lstd = -lc++ -fopenmp=libomp
lto = -flto=thin
ldstd =
else ifeq ($(COMPILER), clang-gnu)
# clang compiler
CXX=clang++
CC=clang
LD=ld.lld
# gnu libraries (libstdc++ + libgomp)
cstd = -stdlib=libstdc++ -fopenmp=libgomp -Wno-gcc-compat
lstd = -lstdc++ -fopenmp=libgomp
lto = -flto=thin
ldstd =
else ifeq ($(COMPILER), gcc)
# gcc compiler
CXX=g++
CC=gcc
LD=gcc
# gnu libraries (libstdc++ + libgomp)
cstd += -fopenmp -finline-limit=1000000
lstd += -lgomp
lto = -flto=auto
ldstd = -lstdc++exp
endif


# Set mode-specific options
ifeq ($(BUILD_TYPE), release)
# Very fast, but slow to iterate on, used for final deployment
# lto makes it faster to compile from scratch, but slower to iterate, as linking will do all optimisations
cmode = -O3 -DNDEBUG -DEIGEN_NO_DEBUG $(lto)
lmode = $(lto)
mode = release

else ifeq ($(BUILD_TYPE), release-native)
# Very fast, but slow to iterate on, only used to check for any modern optimisation we missed to take advantage of
# lto makes it faster to compile from scratch, but slower to iterate, as linking will do all optimisations
# Much slower to compile than non-native
cmode = -O3 -DNDEBUG -DEIGEN_NO_DEBUG $(lto)
lmode = $(lto)
march = -march=native -mtune=native
mode = release-native

else ifeq ($(BUILD_TYPE), release-verify)
# Fast, can verify memory is handled correctly, useful for testing code while not sacrificing much performance
cmode = -O3 -g -fno-omit-frame-pointer -fsanitize=address -DLOG_MAX_LEVEL_DEFAULT=LTrace
ifneq '' '$(findstring clang,$(CC))' # clang
	cmode += -DLIBCXX_ENABLE_ASSERTIONS=ON -D_LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS
else # gcc - may technically already break ABI, but doesn't matter for this project
	cmode += -D_GLIBCXX_DEBUG
endif
lmode = -fsanitize=address
mode = release-verify

else ifeq ($(BUILD_TYPE), sanitize-thread)
# Super slow (?), can verify no data races exist
cmode = -O3 -g -DNDEBUG -DEIGEN_NO_DEBUG -fno-omit-frame-pointer -fsanitize=thread
lmode = -fsanitize=thread
mode = sanitize-thread

else ifeq ($(BUILD_TYPE), sanitize-memory)
# Super slow (?), can verify no memory is used uninitialised
cmode = -O1 -g -DNDEBUG -DEIGEN_NO_DEBUG -fno-omit-frame-pointer -fsanitize=memory
lmode = -fsanitize=memory
mode = sanitize-memory

else ifeq ($(BUILD_TYPE), sanitize-address)
# Slow, can verify memory is handled correctly, useful for newly written code
cmode = -O1 -g -fno-omit-frame-pointer -fsanitize=address
ifneq '' '$(findstring clang,$(CC))' # clang
	cmode += -DLIBCXX_ENABLE_ASSERTIONS=ON -D_LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS
else # gcc - may technically already break ABI, but doesn't matter for this project
	cmode += -D_GLIBCXX_DEBUG
endif
lmode = -fsanitize=address
mode = sanitize-address

else ifeq ($(BUILD_TYPE), debug)
# Slow, for step-through debug
cmode = -Og -g -fno-omit-frame-pointer -fsanitize=address -DEIGEN_NO_DEBUG
lmode = -fsanitize=address
mode = debug

else ifeq ($(BUILD_TYPE), super-debug)
# Super slow (?), for step-through debug with extra verification
# Advanced debug mode of the standard library, may break ABI so might need libraries to be built with it, too (for clang at least)
cmode = -O0 -g -fno-omit-frame-pointer -fsanitize=address -fsanitize=undefined -fno-sanitize=alignment -DLOG_MAX_LEVEL_DEFAULT=LTrace
lmode = -fsanitize=address -fsanitize=undefined -fno-sanitize=alignment
ifneq '' '$(findstring clang,$(CC))' # clang
# requires all libraries to be built with it to work and forces BlockedQueue to be less performant (atomics need to be non-trivially-copyable like on windows)
	cmode += -DLIBCXX_ENABLE_ASSERTIONS=ON -D_LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS -D_LIBCPP_ENABLE_DEBUG_MODE
else # gcc - may technically already break ABI, but doesn't matter for this project
	cmode += -D_GLIBCXX_DEBUG -D_GLIBCXX_DEBUG_BACKTRACE
	lmode += -lstdc++exp
endif
mode = super-debug

else ifeq ($(BUILD_TYPE), super-verify)
# Slow, verify as much as possible, but still try to be useably fast
# Advanced debug mode of the standard library, may break ABI so might need libraries to be built with it, too (for clang at least)
cmode = -O3 -g -fno-omit-frame-pointer -fsanitize=address -fsanitize=undefined -fno-sanitize=alignment -DLOG_MAX_LEVEL_DEFAULT=LTrace
lmode = -fsanitize=address -fsanitize=undefined -fno-sanitize=alignment
ifneq '' '$(findstring clang,$(CC))' # clang
# requires all libraries to be built with it to work and forces BlockedQueue to be less performant (atomics need to be non-trivially-copyable like on windows)
	cmode += -DLIBCXX_ENABLE_ASSERTIONS=ON -D_LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS -D_LIBCPP_ENABLE_DEBUG_MODE
else # gcc - may technically already break ABI, but doesn't matter for this project
	cmode += -D_GLIBCXX_DEBUG -D_GLIBCXX_DEBUG_BACKTRACE
	lmode += -lstdc++exp
endif
mode = super-verify

endif


# Setup compiler environment
src = source
sh = ../shared
dep = dependencies/sources
b = build
o = $(b)/$(COMPILER)-$(mode)
cflags = -Idependencies/include -Idependencies/sources \
	-DGLEW_STATIC $(cmode) $(march) -ffunction-sections -fpic -mshstk \
	$(shell pkg-config --cflags dbus-1)	
cxxflags = $(cstd) $(cflags) -I$(src) -I$(sh) -I$(dep)/imgui -DEIGEN_MPL2_ONLY -D_FILE_OFFSET_BITS=64 \
	-Wno-deprecated-enum-enum-conversion \
	-D_UNICODE -DEIGEN_NO_AUTOMATIC_RESIZING -DEIGEN_INITIALIZE_MATRICES_BY_NAN \
	-DBLOB_EMULATION \
	-std=c++20 -ffp-contract=fast
lflags = -Wl,-rpath='$$ORIGIN' $(lstd) $(lmode) $(march) \
	$(shell pkg-config --libs dbus-1)


# Define all source files to be compiled
SERVER_CPP = \
	app.cpp server.cpp config.cpp simulation.cpp recording.cpp \
	comm/usb.cpp comm/protocol_packet.cpp \
	comm/streaming.cpp comm/timesync.cpp \
	device/tracking_controller.cpp device/tracking_camera.cpp \
	device/parsing.cpp device/camera_firmware.cpp device/controller_firmware.cpp \
	io/vrpn.cpp io/vrpn_imu.cpp \
	imu/trackerAsterTrack.cpp imu/trackerSlimeVR.cpp imu/remoteIMU.cpp \
	pipeline/pipeline.cpp pipeline/pipeline_tracking.cpp \
	pipeline/pipeline_calibration_point.cpp pipeline/pipeline_calibration_target.cpp \
	calib/obs_data.cpp calib/optimisation.cpp \
	calib_point/reconstruction.cpp calib_point/calibration_room.cpp \
	calib_target/reconstruction.cpp calib_target/aquisition.cpp calib_target/assembly.cpp \
	point/sequence_data.cpp point/sequences2D.cpp point/triangulation.cpp \
	target/target.cpp target/detection2D.cpp target/detection3D.cpp target/tracking2D.cpp \
	tracking/cluster.cpp tracking/tracking.cpp
#comm/server.cpp

SERVER_SHARED_CPP = \
	util/image.cpp \
	hash/crc32.cpp \
	hash/sha256.cpp
#comm/protocol_stream.cpp
SERVER_SHARED_C =

SERVER_DEP_CPP = \
	recursive_shared_mutex/recursive_shared_mutex.cpp \
	poselib/p3p_ding.cpp \
	dbscan/dbscan.cpp \
	nativefiledialog-extended/nfd_portal.cpp

SERVER_DEP_C = hidapi/linux/hid.c


INTERFACE_CPP = \
	ui/ui.cpp ui/menu.cpp \
	ui/pipeline.cpp ui/control.cpp \
	ui/pipeline_calibration_point.cpp ui/pipeline_calibration_target.cpp \
	ui/insights.cpp ui/logging.cpp \
	ui/devices.cpp ui/wireless.cpp ui/cameraSettings.cpp \
	ui/parameters/sequence2D.cpp ui/parameters/calibration_point.cpp ui/parameters/calibration_target.cpp ui/parameters/tracking.cpp \
	ui/targets.cpp \
	ui/tools/lens_selection.cpp \
	ui/cameraPanels.cpp ui/cameraView.cpp ui/view3D.cpp ui/visSettings.cpp \
	ui/system/state.cpp ui/system/visGeneral.cpp ui/system/visTarget.cpp \
	ui/gl/visualisation.cpp ui/gl/sharedGL.cpp \
	ui/gl/mesh.cpp ui/gl/shader.cpp \
	ui/imgui/imgui_custom.cpp ui/imgui/imgui_onDemand.cpp \
	emulation/detection.cpp

INTERFACE_SHARED_CPP = blob/blob.cpp blob/refinement.cpp
INTERFACE_SHARED_C =

INTERFACE_DEP_CPP = \
	imgui/imgui.cpp imgui/imgui_draw.cpp \
	imgui/imgui_widgets.cpp imgui/imgui_tables.cpp \
	imgui/backends/imgui_impl_glfw.cpp imgui/backends/imgui_impl_opengl3.cpp \
	imgui/misc/cpp/imgui_stdlib.cpp \
	imgui/imgui_demo.cpp \
	imguizmo/ImSequencer.cpp \
	implot/implot.cpp implot/implot_items.cpp implot/implot_demo.cpp

INTERFACE_DEP_C = GL/glew.c stb/stb_image.c

OBJECTS_SERVER = $(addprefix $(o)/,$(SERVER_CPP:.cpp=.obj)) \
				$(addprefix $(o)/shared/,$(SERVER_SHARED_CPP:.cpp=.obj)) \
				$(addprefix $(o)/shared/,$(SERVER_SHARED_C:.c=.obj)) \
				$(addprefix $(o)/dependencies/,$(SERVER_DEP_CPP:.cpp=.obj)) \
				$(addprefix $(o)/dependencies/,$(SERVER_DEP_C:.c=.obj))
OBJECTS_INTERFACE = $(addprefix $(o)/,$(INTERFACE_CPP:.cpp=.obj)) \
					$(addprefix $(o)/shared/,$(INTERFACE_SHARED_CPP:.cpp=.obj)) \
					$(addprefix $(o)/shared/,$(INTERFACE_SHARED_C:.c=.obj)) \
					$(addprefix $(o)/dependencies/,$(INTERFACE_DEP_CPP:.cpp=.obj)) \
					$(addprefix $(o)/dependencies/,$(INTERFACE_DEP_C:.c=.obj))
SOURCES_SERVER = $(addprefix $(src)/,$(SERVER_CPP)) $(addprefix $(dep)/,$(SERVER_DEP_CPP)) $(addprefix $(dep)/,$(SERVER_DEP_C))
SOURCES_INTERFACE = $(addprefix $(src)/,$(INTERFACE_CPP)) $(addprefix $(dep)/,$(INTERFACE_DEP_CPP)) $(addprefix $(dep)/,$(INTERFACE_DEP_C))

# Find libraries
libdir = dependencies/lib/linux
usblibs = $(libdir)/libusb/libusb-1.0.so.0
vrpnlibs = $(libdir)/vrpn/libvrpn.a $(libdir)/vrpn/libquat.a
glfwlibs = $(libdir)/libglfw3.a
trackerlibs = -ludev
server_libs = $(usblibs) $(vrpnlibs) $(trackerlibs) -lrt -lGL -lturbojpeg -latomic
interface_libs = $(glfwlibs)

# Run prechecks before starting compilation
checkLibs := $(shell ldconfig -p | grep libturbojpeg > /dev/null 2>&1; echo $$?)
ifneq ($(checkLibs),0)
  $(error Turbojpeg library not found, make sure to install it systemwide)
endif
checkLibs := $(shell test -d "dependencies/include/Eigen"; echo $$?)
ifneq ($(checkLibs),0)
  $(error Eigen include files are missing, make sure to build dependencies)
endif
checkLibs := $(shell test -d "dependencies/include/GLFW"; echo $$?)
ifneq ($(checkLibs),0)
  $(error GLFW include files are missing, make sure to build dependencies)
endif
checkLibs := $(shell test -d "dependencies/include/libusb"; echo $$?)
ifneq ($(checkLibs),0)
  $(error libusb include files are missing, make sure to build dependencies)
endif
checkLibs := $(shell test -d "dependencies/include/vrpn"; echo $$?)
ifneq ($(checkLibs),0)
  $(error vrpn include files are missing, make sure to build dependencies)
endif
checkLibs := $(shell test -f "$(libdir)/libglfw3.a"; echo $$?)
ifneq ($(checkLibs),0)
  $(error GLFW library is missing ($(libdir)/libglfw3.a), make sure to build dependencies)
endif
checkLibs := $(shell test -d "$(libdir)/libusb"; echo $$?)
ifneq ($(checkLibs),0)
  $(error libusb library is missing ($(libdir)/libusb), make sure to build dependencies)
endif
checkLibs := $(shell test -d "$(libdir)/vrpn"; echo $$?)
ifneq ($(checkLibs),0)
  $(error vrpn library is missing ($(libdir)/vrpn), make sure to build dependencies)
endif

all: $(o)/astertrack-interface.so $(o)/astertrack-server $(o)/astertrack-static copybinaries

.PHONY: copybinaries
copybinaries: $(o)/astertrack-interface.so $(o)/astertrack-server $(o)/astertrack-static
	@cp -u $(usblibs) "$(b)/"
	@rm $(b)/astertrack-static $(b)/astertrack-server $(b)/astertrack-interface.so
	@cp $(o)/astertrack-static $(b)/astertrack-static
	@cp $(o)/astertrack-server $(b)/astertrack-server
	@cp $(o)/astertrack-interface.so $(b)/astertrack-interface.so


# May prefix this before each target to measure compile times
# time --format="%E | $@"

# Generic compile targets
$(o)/%.obj: $(src)/%.cpp
	$(CXX) -MMD -MP $(cxxflags) -c $< -o $@
$(o)/shared/%.obj: $(sh)/%.cpp
	$(CXX) -MMD -MP $(cxxflags) -c $< -o $@
$(o)/shared/%.obj: $(sh)/%.c
	$(CC) $(cflags) -c $< -o $@
$(o)/dependencies/%.obj: $(dep)/%.cpp
	$(CXX) -MMD -MP $(cxxflags) -c $< -o $@
$(o)/dependencies/%.obj: $(dep)/%.c
	$(CC) $(cflags) -c $< -o $@

# Tempoarary Debug Mode files
#$(o)/target/tracking2D.obj: $(src)/target/tracking2D.cpp
#	$(CC) -O0 $(cxxflags) -O0 -c $< -o $@

# Include header dependencies as generated by -MMD -MP
-include $(OBJECTS_SERVER:.obj=.d)
-include $(OBJECTS_INTERFACE:.obj=.d)
$(shell mkdir -p $(dir $(OBJECTS_SERVER)) >/dev/null)
$(shell mkdir -p $(dir $(OBJECTS_INTERFACE)) >/dev/null)

# AsterTrack Static
# Combined server and interface just to have compile-time validation of interface
OBJECTS_STATIC = $(subst $(o)/app.obj,$(o)/app-static.obj,$(OBJECTS_SERVER)) $(OBJECTS_INTERFACE)
$(o)/app-static.obj: $(src)/app.cpp
	$(CXX) -MMD -MP $(cxxflags) -DINTERFACE_LINKED -c $< -o $@
$(o)/astertrack-static: $(OBJECTS_STATIC)
	$(CXX) -rdynamic -o $@ $(OBJECTS_STATIC) $(server_libs) $(interface_libs) $(lflags)

# AsterTrack Server
# Main binary, may load astertrack-interface.so
$(o)/astertrack-server: $(OBJECTS_SERVER)
	$(CXX) -rdynamic -o $@ $(OBJECTS_SERVER) $(server_libs) $(lflags)

# AsterTrack Interface
# UI loaded as a shared library, mostly opaque to server, but heavily interacts with server itself
$(o)/astertrack-interface.so: $(OBJECTS_INTERFACE)
	$(CXX) -shared -o $@ $(OBJECTS_INTERFACE) $(interface_libs) $(lflags)

# Always force re-link at least, since different modes share the same target
.PHONY: .FORCE
.FORCE:
