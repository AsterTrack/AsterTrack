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

#include "threading.hpp"

#ifdef _WIN32       // Windows
#include <windows.h>
#else               // Linux
#include <pthread.h>
#include <unistd.h>
#include <sys/syscall.h>
#endif

bool SetThreadPriority(std::thread &thread, int priority)
{
#ifdef _WIN32
	return SetThreadPriority(thread.native_handle(), priority);
#else
	sched_param sch;
	int policy; 
	pthread_getschedparam(thread.native_handle(), &policy, &sch);
	sch.sched_priority = 20;
	return !pthread_setschedparam(thread.native_handle(), SCHED_FIFO, &sch);
#endif
}

bool SetCurrentThreadPriority(int priority)
{
#ifdef _WIN32
	return SetThreadPriority(GetCurrentThread(), priority);
#elif defined(SYS_gettid)
	pid_t pid = (pid_t)syscall(SYS_gettid);
	sched_param sch;
	int policy; 
	pthread_getschedparam(pid, &policy, &sch);
	sch.sched_priority = 20;
	return !pthread_setschedparam(pid, SCHED_FIFO, &sch);
#endif
}