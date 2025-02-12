#include "recursive_shared_mutex.hpp"
#include <chrono>

void recursive_shared_mutex::lock()
{
	std::unique_lock sync_lock(m_mtx);
	m_cond_var.wait(sync_lock, [this] { return can_exclusively_lock(); });
	if (is_exclusive_locked_on_this_thread())
	{
		increment_exclusive_lock();
	}
	else
	{
		start_exclusive_lock();
	}
}

bool recursive_shared_mutex::try_lock()
{
	std::unique_lock sync_lock(m_mtx);
	if (can_increment_exclusive_lock())
	{
		increment_exclusive_lock();
		return true;
	}
	if (can_start_exclusive_lock())
	{
		start_exclusive_lock();
		return true;
	}
	return false;
}

void recursive_shared_mutex::unlock()
{
	{
		std::unique_lock sync_lock(m_mtx);
		decrement_exclusive_lock();
	}
	m_cond_var.notify_all();
}

void recursive_shared_mutex::lock_shared() const
{
	std::unique_lock sync_lock(m_mtx);
	m_cond_var.wait(sync_lock, [this] { return can_lock_shared(); });
	increment_shared_lock();
}

bool recursive_shared_mutex::try_lock_shared() const
{
	std::unique_lock sync_lock(m_mtx);
	if (can_lock_shared())
	{
		increment_shared_lock();
		return true;
	}
	return false;
}

void recursive_shared_mutex::unlock_shared() const
{
	{
		std::unique_lock sync_lock(m_mtx);
		decrement_shared_lock();
	}
	m_cond_var.notify_all();
}