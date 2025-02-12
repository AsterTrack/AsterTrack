#ifndef _RECURSIVE_SHARED_MUTEX_H
#define _RECURSIVE_SHARED_MUTEX_H

#include <chrono>
#include <thread>
#include <mutex>
#include <shared_mutex> // Not needed here, but for std::shared_lock in user code
#include <map>
#include <condition_variable>
#include <stdexcept>

/**
 * From https://stackoverflow.com/a/60046372
 * Simplifies code greatly by combining std::recursive_mutex with std::shared_mutex
 * - Allows recursive access
 * - Can switch between shared and exclisive access
 * -> No need to think about potential call hierarchy
 * Adds a bit of overhead, but that's worth it right now
 * 
 * WARNING: Having two threads that upgrade a shared_lock to a exclusive lock can and WILL result in a deadlock, because trying to lock does NOT give up the shared_lock first.
 * Alternate implementation that changes that might be preferred (e.g. https://github.com/KonanM/shared_recursive_mutex)
 */

struct recursive_shared_mutex
{
public:

	recursive_shared_mutex() :
		m_mtx{}, m_exclusive_thread_id{}, m_exclusive_count{ 0 }, m_shared_locks{}
	{}

	void lock();
	bool try_lock();
	void unlock();
	
	template<class Clock, class Duration>
	bool try_lock_until(const std::chrono::time_point<Clock, Duration> &timeout_time)
	{
		do
		{
			if (try_lock())
				return true;
		}
		while (Clock::now() < timeout_time);
		return false;
	}
	template<class Rep, class Period>
	bool try_lock_for(const std::chrono::duration<Rep, Period> &timeout_duration)
	{
		return try_lock_until(std::chrono::steady_clock::now() + timeout_duration);
	}

	void lock_shared() const;
	bool try_lock_shared() const;
	void unlock_shared() const;

	template<class Clock, class Duration>
	bool try_lock_shared_until(const std::chrono::time_point<Clock, Duration> &timeout_time) const
	{
		do
		{
			if (try_lock_shared())
				return true;
		}
		while (Clock::now() < timeout_time);
		return false;
	}
	template<class Rep, class Period>
	bool try_lock_shared_for(const std::chrono::duration<Rep, Period> &timeout_duration) const
	{
		return try_lock_shared_until(std::chrono::steady_clock::now() + timeout_duration);
	}

	recursive_shared_mutex(const recursive_shared_mutex&) = delete;
	recursive_shared_mutex& operator=(const recursive_shared_mutex&) = delete;

private:

	inline bool is_exclusive_locked() const
	{
		return m_exclusive_count > 0;
	}

	inline bool is_shared_locked()
	{
		return m_shared_locks.size() > 0;
	}

	inline bool can_exclusively_lock()
	{
		return can_start_exclusive_lock() || can_increment_exclusive_lock();
	}

	inline bool can_start_exclusive_lock()
	{
		return !is_exclusive_locked() && (!is_shared_locked() || is_shared_locked_only_on_this_thread());
	}

	inline bool can_increment_exclusive_lock()
	{
		return is_exclusive_locked_on_this_thread();
	}

	inline bool can_lock_shared() const
	{
		return !is_exclusive_locked() || is_exclusive_locked_on_this_thread();
	}

	inline bool is_shared_locked_only_on_this_thread() const
	{
		return is_shared_locked_only_on_thread(std::this_thread::get_id());
	}

	inline bool is_shared_locked_only_on_thread(std::thread::id id) const
	{
		return m_shared_locks.size() == 1 && m_shared_locks.find(id) != m_shared_locks.end();
	}

	inline bool is_exclusive_locked_on_this_thread() const
	{
		return is_exclusive_locked_on_thread(std::this_thread::get_id());
	}

	inline bool is_exclusive_locked_on_thread(std::thread::id id) const
	{
		return m_exclusive_count > 0 && m_exclusive_thread_id == id;
	}

	inline void start_exclusive_lock()
	{
		m_exclusive_thread_id = std::this_thread::get_id();
		m_exclusive_count++;
	}

	inline void increment_exclusive_lock()
	{
		m_exclusive_count++;
	}

	inline void decrement_exclusive_lock()
	{
		if (m_exclusive_count == 0)
		{
			throw std::logic_error("Not exclusively locked, cannot exclusively unlock");
		}
		if (m_exclusive_thread_id == std::this_thread::get_id())
		{
			m_exclusive_count--;
		}
		else
		{
			throw std::logic_error("Calling exclusively unlock from the wrong thread");
		}
	}

	inline void increment_shared_lock() const
	{
		increment_shared_lock(std::this_thread::get_id());
	}

	inline void increment_shared_lock(std::thread::id id) const
	{
		if (m_shared_locks.find(id) == m_shared_locks.end())
		{
			m_shared_locks[id] = 1;
		}
		else
		{
			m_shared_locks[id] += 1;
		}
	}

	inline void decrement_shared_lock() const
	{
		decrement_shared_lock(std::this_thread::get_id());
	}

	inline void decrement_shared_lock(std::thread::id id) const
	{
		if (m_shared_locks.size() == 0)
		{
			throw std::logic_error("Not shared locked, cannot shared unlock");
		}
		if (m_shared_locks.find(id) == m_shared_locks.end())
		{
			throw std::logic_error("Calling shared unlock from the wrong thread");
		}
		else
		{
			if (m_shared_locks[id] == 1)
			{
				m_shared_locks.erase(id);
			}
			else
			{
				m_shared_locks[id] -= 1;
			}
		}
	}

	// Allow shared access on const objects with mutable, but not exclusive access
	mutable std::mutex m_mtx;
	mutable std::thread::id m_exclusive_thread_id;
	mutable size_t m_exclusive_count;
	mutable std::map<std::thread::id, size_t> m_shared_locks;
	mutable std::condition_variable m_cond_var;
};

#endif
