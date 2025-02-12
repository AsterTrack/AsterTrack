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

#ifndef SYNCHRONISED_H
#define SYNCHRONISED_H

#include "folly/synchronized.hpp"
#include "recursive_shared_mutex/recursive_shared_mutex.hpp"

#include <memory> // shared_ptr
#include <mutex>

// No-op mutexes testing / semantics without synchronisation
struct NoMutex
{
	inline void lock() {}
	inline void unlock() {}
};
struct NoMutexShared
{
	inline void lock() {}
	inline void unlock() {}
	inline void lock_shared() {}
	inline void unlock_shared() {}
};

template <typename T>
using SynchronisedRS = folly::Synchronized<T, recursive_shared_mutex>;
template <typename T>
using SynchronisedS = folly::Synchronized<T, std::shared_mutex>;
// TODO: Switch many SynchronisedS to Synchronised to avoid shared_mutex overhead
// To not loose read/write semantics, switch to contextualRLock/contextualLock functions
// This also allows switching back to SynchronisedS easily in the future
template <typename T>
using Synchronised = folly::Synchronized<T, std::mutex>;

// Non-locking "Synchronised" wrapper for single-threaded data
// Allows developing with the locking paradigm even when not protected, without runtime costs
// Allows to quickly protect the data if the data will ever need to be accessed from another thread
template <typename T>
using NonSynchronisedS = folly::Synchronized<T, NoMutexShared>;
template <typename T>
using NonSynchronised = folly::Synchronized<T, NoMutex>;

#endif // SYNCHRONISED_H