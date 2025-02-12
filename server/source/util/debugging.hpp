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

#ifndef DEBUGGING_H
#define DEBUGGING_H

#include <thread>
#include <chrono>
#include <atomic>

/**
 * Simple algorithm debugger for visual debugging (UI remains interactive if called correctly in a processing thread)
 * Trigger with EnterDebugging, then call Breakpoint on interesting states in an algorithm with visual output
 * Make sure to release all mutexes before calling breakpoint, else the UI might not be able to render
 * Use 'C' to jump to next breakpoint, 'B' to exit debugging (until the next trigger), and 'Z' to zoom in on target position in all camera views
 */

extern std::atomic<bool> dbg_isBreaking;
extern std::atomic<int> dbg_debugging;

static inline void EnterDebugging(int level = 1)
{
	dbg_debugging = level;
}

static inline void LeaveDebugging()
{
	dbg_debugging = 0;
}

static inline bool IsDebugging(int level = 1)
{
    return dbg_debugging.load() >= level;
}

static inline void Breakpoint(int level = 1)
{
	if (dbg_debugging.load() < level) return;
    dbg_isBreaking = true;
    dbg_isBreaking.wait(true);
}

#endif // DEBUGGING_H