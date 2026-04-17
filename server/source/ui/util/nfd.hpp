/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef NFD_WRAPPER_H
#define NFD_WRAPPER_H

#include "signals.hpp"

#include "nativefiledialog-extended/nfd.h"
#include "nativefiledialog-extended/nfd_glfw3.h"

// NFD is always used with threadPool for async file picking
#include "ctpl/ctpl.hpp"
extern ctpl::thread_pool threadPool;

#include <mutex>

extern std::mutex nfd_lock;

struct NFD_Context
{
	bool initialised = false;

	NFD_Context()
	{
		// NFD is always used in thread-pools, and NFD_Init needs be called per-thread (TODO: verify again?)
		// But NFD_Quit will also mess up other threads currently using NFD such that they may block forever
		// This causes all kinds of issues like clogging threadPool and such must be avoided at all costs
		// So we lock here to ensure only one UI element is using NFD at once, in absence of other checks
		nfd_lock.lock();
		initialised = NFD_Init();
		if (!initialised)
		{ // Thread-specific init
			SignalErrorToUser(asprintf_s("Failed to initialise File Picker: %s", NFD_GetError()));
		}
	}

	~NFD_Context()
	{
		if (initialised)
			NFD_Quit();
		nfd_lock.unlock();
	}

	operator bool()
	{
		return initialised;
	}
};

bool ConvertGLFWHandleToNFD(GLFWwindow* glfwWindow, nfdwindowhandle_t* nativeWindow);

#endif // NFD_WRAPPER_H