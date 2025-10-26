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

#ifndef THREADING_H
#define THREADING_H

#include <thread>
#include <cassert>

/**
 * Custom thread wrapper
 * Combines automatic joining of std::jthread
 * With a way to signal the thread exited like std::async
 * While still leaving everything else bare and in control of the user
 * 
 * NOTE: Automatically set finished when exiting thread with:
 * const auto exitNotifier = sg::make_scope_guard([&]() noexcept { control.finished = true; });
 * NOTE: Pass control.stop_source.get_token() to thread
 * 
 * Not designed to need explicit synchronisation IF thread itself is handled on one main thread
 * Designed for threads that, throughout execution:
 * - have one explicit place to update state
 * - need to be interruptible
 */
struct ThreadControl
{
	std::thread *thread = nullptr;
	std::stop_source stop_source;
	std::atomic<bool> finished;

	void init()
	{
		assert(thread == nullptr);
		stop_source = {};
		finished = false;
	}

	void stop()
	{
		stop_source.request_stop();
		if (thread && thread->joinable())
		{
			if (thread->get_id() == std::this_thread::get_id())
				thread->detach(); // Will clean up ourself and exit
			else
				thread->join(); // Wait for it to end
		}
		delete thread;
		thread = nullptr;
		finished = false;
	}

	bool running() const
	{
		return thread != nullptr;
	}

	bool stopping() const
	{
		return thread != nullptr && stop_source.stop_requested();
	}

    ThreadControl()
    {
        init();
    }
    ~ThreadControl()
    {
        stop();
    }
};

#endif // THREADING_H