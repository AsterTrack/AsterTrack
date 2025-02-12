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

#ifndef BLOCKED_VECTOR_H
#define BLOCKED_VECTOR_H

#include <cassert>
#include <iterator>
#include <vector>
#include <array>
#include <list>
#include <mutex>

// For safe culled block deletion
#include <thread>
#include <memory> // shared_ptr
#include <queue>

/**
 * Two blocked datastructures for optimising many data entries
 * BlockedVector: Blocked vector with erase operations only modifying one block, leaving "holes" - iterator has to skip holes
 * ---missing---: Blocked vector without support for erase operations, allowing for faster random-access iteration
 * BlockedQueue: Queue-like container providing fast thread-safe push_back and iterators (mostly just using atomics) as well as somewhat-thread-safe popping (cull+delete)
 */

/**
 * Constant-time random access, Constant-time(N) push_back, Constant-time(N) remove/erase
 * No reallocation of data structures (only the access structures) - but data can be moved when removing/erasing
 * No data continuity (blocks of size N are continuous)
 * 
 * NON STANDARD INTERFACE, INDICES ARE DANGEROUS!
 * 
 * In contrast to std::deque, deleting doesn't reorder the whole array, only the block deleted from. 
 * - Indices will become invalid when an item in the same block is removed
 * - Some indices will cease to point to ANY element even though there exist smaller and/or bigger indices that do
 * - Essentially, holes in the array are created (which can be filled up with push)
 * - Results in a + (b-a) != b if there have been deletions in-between
 * 
 * std::deque should be a much better option if you optimise removing with std::remove_if, BUT:
 *  - On windows (MSVC), std::deque is horribly inefficient (block size 16 bytes), and until that changes, don't use it
 *  - Alternatively use boosts deque
 */
template<typename T, std::size_t N = 1024>
class BlockedVector : private std::vector<std::vector<T>>
{
private:

	typedef std::vector<T> BLOCK;
	typedef std::vector<std::vector<T>> BASE;

	std::size_t s, e; // size, end

	BLOCK &ensure_block(std::size_t b)
	{
		if (BASE::size() > b)
			return BASE::operator[](b);
		int p = BASE::size();
		BASE::resize(b+1);
		for (int i = p; i < b; i++)
			BASE::operator[](i).reserve(N); // Default initialise all new blocks
		return BASE::back();
	}

public:
	BlockedVector() : s(0), e(0) {}

	template<bool Const = false>
	class iterator_t {
		using BlockedVectorBase = typename std::conditional_t<Const, const BlockedVector, BlockedVector>;
		BlockedVectorBase *base;
	public:
		// iterator traits
		using value_type = typename std::conditional_t<Const, const T, T>;
		using difference_type = long;
		using pointer = value_type*;
		using reference = value_type&;
		using iterator_category = std::bidirectional_iterator_tag;

		std::size_t b, i;
		iterator_t(BlockedVectorBase &vec, std::size_t n) : base(&vec), b(n/N), i(n%N) {}
		iterator_t(BlockedVectorBase &vec, std::size_t block, std::size_t index) : base(&vec), b(block), i(index) {}
		iterator_t& operator++()
		{
			if (++i >= base->BASE::operator[](b).size())
			{ // If next block doesn't exist, that's fine, end is marked like that
				do b++;
				while (b < base->BASE::size() && base->BASE::operator[](b).size() == 0);
				i = 0;
			}
			return *this;
		}
		iterator_t operator++(int) { iterator_t retval = *this; ++(*this); return retval; }
		iterator_t& operator+=(long A)
		{
			if (b >= base->BASE::size())
				b = base->BASE::size();
			long index = (long)i + A;
			if (index >= 0)
			{
				while (b < base->BASE::size() && index >= base->BASE::operator[](b).size())
				{
					index -= base->BASE::operator[](b).size();
					b++;
				}
				if (b >= base->BASE::size()) index = 0;
			}
			else
			{
				while (b > 0 && index < 0)
				{
					b--;
					index += base->BASE::operator[](b).size();
				}
				if (index < 0) index = 0;
			}
			i = index;
			return *this;
		}
		iterator_t operator+(long A) const { iterator_t retval = *this; retval += A; return retval; }
		iterator_t& operator--()
		{
			if (i == 0)
			{
				do b--;
				while (b > 0 && base->BASE::operator[](b).size() == 0);
				i = base->BASE::operator[](b).size()-1;
			}
			else
				i--;
			return *this;
		}
		iterator_t operator--(int) { iterator_t retval = *this; --(*this); return retval; }
		iterator_t& operator-=(long A) { *this += -A; return *this; }
		iterator_t operator-(long A) const { iterator_t retval = *this; retval += -A; return retval; }
		long operator-(const iterator_t &other) const { return index() - other.index(); }
		bool operator<(const iterator_t &other) const { return index() < other.index(); }
		bool operator>(const iterator_t &other) const { return index() > other.index(); }
		bool operator<=(const iterator_t &other) const { return !(*this > other); }
		bool operator>=(const iterator_t &other) const { return !(*this < other); }
		bool operator==(const iterator_t &other) const { return (b == other.b && i == other.i) || (!other.valid() && !valid()); }
		bool operator!=(const iterator_t &other) const { return !(*this == other); }
		bool valid() const { return b >= 0 && b < base->BASE::size() && i < base->BASE::operator[](b).size(); }
		reference operator*() const { return base->BASE::operator[](b)[i]; }
		pointer operator->() const { return &base->BASE::operator[](b)[i]; }
		std::size_t index() const { return b*N+i; }
	};
	using iterator = iterator_t<false>;
	using const_iterator = iterator_t<true>;

	iterator begin()
	{ 
		for (int b = 0; b < BASE::size(); b++) 
			if (!BASE::operator[](b).empty()) 
				return iterator(*this, b, 0); 
		return iterator(*this, 0, 0);
	}
	iterator end() { return iterator(*this, BASE::size(), 0); }
	iterator pos(std::size_t n) { return iterator(*this, n/N, n%N); }
	T &front() { return *begin(); }
	T &back() { return *(--end()); }
	const_iterator begin() const { for (int b = 0; b < BASE::size(); b++) if (!BASE::operator[](b).empty()) return const_iterator(*this, b, 0); return const_iterator(*this, 0, 0); }
	const_iterator end() const { return const_iterator(*this, BASE::size(), 0); }
	const_iterator pos(std::size_t n) const { return const_iterator(*this, n/N, n%N); }
	const T &front() const { return *begin(); }
	const T &back() const { return *(--end()); }
	bool empty() const { return s == 0 || BASE::size() == 0; }

	template<typename U = T>
	std::size_t push(U&& x)
	{
		if (e != s)
		{ // Insert in the middle
			s++;
			for (int b = 0; b < BASE::size(); b++)
			{
				auto &block = BASE::operator[](b);
				if (block.size() < N)
				{
					block.push_back(std::forward<U>(x));
					return b*N+block.size()-1;
				}
			}
			// Should not happen, but worst case, fall back to push_back (or throw error?)
		}
		// Add to the end
		std::size_t index = e;
		push_back<U>(std::forward<U>(x));
		return index;
	}

	template<typename U = T>
	std::size_t push_back(U&& x)
	{
		std::size_t b = e/N;
		std::size_t i = e%N;
		auto &block = ensure_block(b);
		block.resize(i+1);
		block[i] = std::forward<U>(x);
		s++;
		return e++;
	}

	void remove(std::size_t n)
	{
		std::size_t b = n/N;
		std::size_t i = n%N;
		if (b < BASE::size())
		{
			auto &block = BASE::operator[](b);
			if (block.size() > i)
				erase(iterator(*this, b, i));
		}
	}

	iterator erase(const iterator &it)
	{
		if (it.b >= BASE::size())
			return it;
		iterator itt = it;
		itt++;
		auto &block = BASE::operator[](it.b);
		block.erase(block.begin()+it.i);
		s--;
		if (it.b+1 == BASE::size())
		{ // Last block
			if (block.empty())
			{ // Now empty, remove
				do { BASE::erase(std::prev(BASE::end())); }
				while (!BASE::empty() && BASE::back().empty());
				if (BASE::empty())
					e = BASE::size()*N;
				else
					e = BASE::size()*N + BASE::back().size();
				return end();
			}
			else
				e--;
		}
		if (itt != begin() && itt.b != BASE::size())
			itt--; // Neither beginning nor end
		return itt;
	}

	void resize(std::size_t n)
	{
		std::size_t b = n/N;
		std::size_t i = n%N;
		BASE::resize(b+1);
		for (int bb = 0; bb < b; bb++)
			BASE::operator[](bb).resize(N);
		BASE::operator[](b).resize(i);
		// Deallocate blocks over b?
		s = n;
		e = n;
	}

	void clear()
	{
		s = 0;
		e = 0;
		BASE::clear();
	}

	std::size_t size() const
	{
		return s;
	}

	std::size_t endIndex() const
	{
		return e;
	}

	std::size_t capacity() const
	{
		return BASE::size()*N;
	}

	T& operator[](std::size_t n) noexcept
	{
		return BASE::operator[](n/N)[n%N];
	}

	const T& operator[](std::size_t n) const noexcept
	{
		return BASE::operator[](n/N)[n%N];
	}

	T& ensureAt(std::size_t n) noexcept
	{
		int i = n%N;
		auto &block = ensure_block(n/N);
		if (block.size() <= i)
		{
			s += i+1-block.size();
			block.resize(i+1);
		}
		if (e <= n)
			e = n+1;
		return block[i];
	}

	void swap(BlockedVector<T> &other) noexcept
	{
		BASE::swap(other);
		std::swap(s, other.s);
		std::swap(e, other.e);
	}

	template<typename _T, std::size_t _N>
	constexpr inline void swap(BlockedVector<_T, _N> &a, BlockedVector<_T, _N> &b) noexcept
	{
		a.swap(b);
	}
};

//#define RESILIENT_BLOCKED_QUEUE // Try to fix errors

/**
 * Thread-Safe blocked queue-like data structure designed for many entries and fast read/write access
 * Allows thread-safe push_back/insert and Views for thread-safe, concurrent read-access of the blocked queue
 * Allows thread-safe removing blocks of elements from the beginning:
 * - cull_front/cull_all/cull_clear: Mark blocks as culled (with clear also resetting indexing to 0)
 * - delete_culled: Delete all culled blocks that are not referenced by any View anymore
 * - clear: Clear blocked_queue in a blocking manner
 * Views provide a thread-safe snapshot of the blocked-queue for concurrent read-access
 * - Write is also allowed if View is non-const (template parameter), but thread-safety depends on the elements themselves
 * - The view mostly behaves like a normal container, except that the index might be offset due to culled blocks
 * - e.g. beginIndex() > 0 - iterators have index() methods to get the index
 * - the Views pos() / operator[] methods use that shifted index, so pos(0) will NOT necessarily yield begin()
 * 
 * Internally uses a mutex but takes care to not hold it for very long. The following operations aquire the mutex:
 * - push_back/insert: short state-copy and block-check, as well as allocation for any new blocks required while locked
 * - getView: very short O(1) state-copy and increasing of the reference counter of the current block range
 * - cull_front/cull_all/cull_clear: very short O(1) access to switch state pointers and swap the current reference counter
 * - delete_culled: short O(#(cull_* calls)) access to swap block pointers for every culled range
 *   - Deconstruction and freeing of memory happens after lock is released, which may take a significant time
 * - clear: Unbounded blocking with mutex locked, waiting for all existing Views to be destroyed
 *   - WARNING: Potential for dead-lock, use cull_clear+delete_culled to reduce the risk.
 * NOTE: This is not necessarily the full runtime of each function, but only the period in which the mutex is held
 */
template<typename T, std::size_t N = 1024>
class BlockedQueue : private std::list<std::array<T, N>>
{
	typedef std::array<T, N> BLOCK;
	typedef std::list<std::array<T, N>> BASE;
	template<bool Const>
	using BlockIt = typename std::conditional_t<Const, typename BASE::const_iterator, typename BASE::iterator>;

private:
	struct STATE
	{
		// Block state
		std::size_t start, count;
		// Can't store begin/end as std::list::end() may point to the same end node
		// And we rely on it staying the same as blocks are added, as count is not updated if STATE is a snapshot
		BlockIt<false> begin, back;
		BlockIt<true> const_begin, const_back;
		// End index / size of whole queue including culled blocks
		std::size_t index;
	};

	STATE m_state; // Protected by m_state_access
	mutable std::mutex m_mutex; // shared_mutex?

	inline STATE getState() const
	{
		std::unique_lock lock(m_mutex); // shared_lock
		return m_state;
	}

	struct BlockAccess
	{
		// Can't store begin/end as std::list::end() always points to the same end node
		// And we dont wan't the block range to change as blocks are added, so front/back it is
		BlockIt<false> front, back;
	};

	std::shared_ptr<BlockAccess> m_blockLifetime;
	std::queue<std::shared_ptr<BlockAccess>> m_culledBlocks;

	inline void detachBlocks(BlockIt<false> begin, BlockIt<false> end)
	{
		// Exchange lifetime to allow controlled deletion of culled blocks
		std::shared_ptr<BlockAccess> blockLifetime = std::make_shared<BlockAccess>();
		m_blockLifetime.swap(blockLifetime);
		if (begin != end)
		{ // Have culled blocks to delete in a controlled manner later
			blockLifetime->front = begin;
			blockLifetime->back = std::prev(end);
			m_culledBlocks.push(std::move(blockLifetime));
		}
	}

	template<bool Const = false>
	static inline BlockIt<Const> getBlock(const STATE &state, std::size_t block)
	{
		BlockIt<Const> it;
		assert(block >= state.start && block <= state.start+state.count);
		if (block-state.start < state.count/2)
		{ // Start from begin
			if constexpr (Const) it = state.const_begin;
			else it = state.begin;
			std::advance(it, block-state.start);
		}
		else
		{ // Start from end
			if constexpr (Const) it = state.const_back;
			else it = state.back;
			std::advance(it, block-state.start-state.count+1);
		}
		return it;
	}

	bool ensure_block(std::size_t b)
	{
		// m_state_access should be locked!!
		if (b < m_state.start+m_state.count)
		{ // Block exists or existed
			return b >= m_state.start;
		}
		// Resize while accounting for culled blocks
		std::size_t added = b - m_state.start - m_state.count + 1;
		m_state.count = b - m_state.start + 1;
		BASE::resize(BASE::size()+added);
		if (m_state.begin == BASE::end())
		{ // Just added first non-culled block, initialise begin
			m_state.begin = std::prev(BASE::end(), m_state.count);
			m_state.const_begin = std::prev(std::add_const_t<BASE>::end(), m_state.count);
		}
		m_state.back = std::prev(BASE::end());
		m_state.const_back = std::prev(std::add_const_t<BASE>::end());
		return true;
	}

public:

	template<bool Const = false>
	class iterator_t {
	private:
		STATE S;
		BlockIt<Const> B;
		std::size_t b, i;
	public:
		// iterator traits
		using value_type = typename std::conditional_t<Const, const T, T>;
		using difference_type = long;
		using pointer = value_type*;
		using reference = value_type&;
		using iterator_category = std::random_access_iterator_tag;

		iterator_t()
		{ // Invalid iterator
			S.start = S.count = S.index = 0;
			b = i = 0;
		}
		iterator_t(STATE state) : S(state)
		{
			if constexpr (Const) B = S.const_begin;
			else B = S.begin;
			b = S.start;
			i = 0;
		}
		iterator_t(STATE state, std::size_t block, int index) : S(state), b(block), i(index)
		{
			assert((b >= S.start && b < S.start+S.count)
				|| (b == S.start+S.count && i == 0));
			assert(i >= 0 && i < N);
			assert(b*N+i <= S.index);
			B = getBlock<Const>(S, b);
		}
		iterator_t& operator++()
		{
			if (++i >= N)
			{ // If next pos or even block doesn't exist, that's fine, end is marked like that
				assert(b < S.start+S.count);
				B++;
				b++;
				i = 0;
			}
			return *this;
		}
		iterator_t operator++(int) { iterator_t retval = *this; ++(*this); return retval; }
		iterator_t& operator+=(long A)
		{
			long index = (long)i + A;
			std::size_t oldB = b;
			if (index < 0)
			{
				long db = index/(long)N-1;
				assert(b >= S.start-db);
			#ifdef RESILIENT_BLOCKED_QUEUE
				if (b < S.start-db)
				{
					b = S.start;
					i = 0;
				}
				else
			#endif
				{
					b += db;
					i = index%N;
				}
			}
			else
			{
				b += index/N;
			 	i = index%N;
				assert(b*N+i <= S.index);
			#ifdef RESILIENT_BLOCKED_QUEUE
				if (b*N+i > S.index)
				{
					b = S.index / N;
					i = S.index % N;
				}
			#endif
			}
			std::advance(B, b-oldB);
			return *this;
		}
		iterator_t operator+(long A) const { iterator_t retval = *this; retval += A; return retval; }
		iterator_t& operator--()
		{
			if (i == 0)
			{
				if (b <= S.start)
				{ // Stay with begin()
					if constexpr (Const)
						B = S.const_begin;
					else
						B = S.begin;
					b = S.start;
					i = 0;
					return *this;
				}
				B--;
				b--;
				i = N-1;
			}
			else
				i--;
			return *this;
		}
		iterator_t operator--(int) { iterator_t retval = *this; --(*this); return retval; }
		iterator_t& operator-=(long A) { *this += -A; return *this; }
		iterator_t operator-(long A) const { iterator_t retval = *this; retval += -A; return retval; }
		long operator-(const iterator_t &other) const { return index() - other.index(); }
		bool operator<(const iterator_t &other) const { return index() < other.index(); }
		bool operator>(const iterator_t &other) const { return index() > other.index(); }
		bool operator<=(const iterator_t &other) const { return !(*this > other); }
		bool operator>=(const iterator_t &other) const { return !(*this < other); }
		bool operator==(const iterator_t &other) const { return (b == other.b && i == other.i) || (!other.accessible() && !accessible()); }
		bool operator!=(const iterator_t &other) const { return !(*this == other); }
		bool valid() const { return b >= S.start && b <= S.start+S.count && b*N+i <= S.index; }
		bool accessible() const { return b >= S.start && b < S.start+S.count && b*N+i < S.index; }
		reference operator*() const { assert(accessible()); return B->operator[](i); }
		pointer operator->() const { assert(accessible()); return &B->operator[](i); }
		std::size_t index() const { return b*N+i; }
	};
	using iterator = iterator_t<false>;
	using const_iterator = iterator_t<true>;

	template<bool Const>
	struct View
	{
		using value_type = typename std::conditional_t<Const, const T, T>;

	private:
		STATE m_state;
		std::shared_ptr<BlockAccess> m_blockLock;

	public:

		View()
		{ // Invalid view
			m_state = {};
			m_blockLock = nullptr;
		}

		View(const BlockedQueue<T, N> &queue)
		{
			std::unique_lock lock(queue.m_mutex);
			m_blockLock = queue.m_blockLifetime;
			m_state = queue.m_state;
		}

		/**
		* Get the number of blocks that are not culled, that is std::distance(begin(), end())
		*/
		std::size_t unculled_blocks()
		{
			return m_state.count;
		}

		/**
		* Get the number of blocks before begin() that were culled but not yet deleted
		*/
		std::size_t culled_blocks()
		{
			return m_state.start;
		}

		/**
		* First index that is not culled
		*/
		std::size_t beginIndex() const
		{
			return m_state.start*N;
		}

		/**
		* Index of end (not a valid index)
		*/
		std::size_t endIndex() const
		{
			return m_state.index;
		}

		/**
		* Size of the unculled range
		*/
		std::size_t size() const
		{
			return endIndex()-beginIndex();
		}

		/**
		* Index into non-culled blocks
		*/
		value_type& operator[](std::size_t n) const
		{ return at(n); }

		/**
		* Index into non-culled blocks
		*/
		value_type& at(std::size_t n) const
		{
			assert(n >= beginIndex() && n < endIndex());
			return getBlock<Const>(m_state, n/N)->operator[](n%N);
		}

		iterator_t<Const> pos(std::size_t n) const { return iterator_t<Const>(m_state, n/N, n%N); }
		iterator_t<Const> begin() const { return iterator_t<Const>(m_state); }
		iterator_t<Const> end() const { return pos(endIndex()); }
		value_type &front() const { return *begin(); }
		value_type &back() const { return *(--end()); }

		bool empty() const { return size() == 0; }
	};

	template<bool Const = true>
	inline View<true> getView() const
	{
		return View<true>(*this);
	}

	template<bool Const = true>
	inline View<Const> getView()
	{
		return View<Const>(*this);
	}

	BlockedQueue() : m_mutex{}
	{
		std::unique_lock lock(m_mutex);
		m_blockLifetime = std::make_shared<BlockAccess>();
		m_state.start = 0;
		m_state.count = 0;
		m_state.index = 0;
		m_state.begin = BASE::end();
		m_state.back = BASE::end();
		m_state.const_begin = std::add_const_t<BASE>::end();
		m_state.const_back = std::add_const_t<BASE>::end();
	}

	template<typename U = T>
	void push_back(U&& x)
	{
		std::size_t index, b, i;
		BlockIt<false> block;
		{
			std::unique_lock lock(m_mutex);
			index = m_state.index++;
			ensure_block(index/N);
			block = m_state.back;
		}
		(*block)[index%N] = std::forward<U>(x);
	}

	template<typename U = T>
	void insert(std::size_t index, U&& x)
	{
		std::size_t b = index/N;
		STATE state;
		{
			std::unique_lock lock(m_mutex);
			if (!ensure_block(b)) return; // In culled block - not an error per-se
			m_state.index = std::max(m_state.index, index+1);
			state = m_state;
		}
		BlockIt<false> block = getBlock(state, b);
		(*block)[index%N] = std::forward<U>(x);
	}

	/**
	 * Blocks until all Views of this blocked queue cease to exist while preventing new Views from being created.
	 * Then clears the blocked queue completely, resetting the index and size to 0
	 * Use cull_clear() + delete_culled() for non-blocking clear
	 * Use cull_all() + delete_culled() for non-blocking cleanup without resetting the index/size
	 */
	void clear()
	{
		std::unique_lock lock(m_mutex);
		while (!m_culledBlocks.empty())
		{ // Past culled blocks might still being referenced by Views
			while (m_culledBlocks.front().use_count() > 1)
				std::this_thread::sleep_for(std::chrono::nanoseconds(100));
			BASE::erase(m_culledBlocks.front()->front, std::next(m_culledBlocks.front()->back));
			m_culledBlocks.pop();
		}
		// Wait for all current views to disappear (if on this thread, deadlock)
		while (m_blockLifetime.use_count() > 1)
			std::this_thread::sleep_for(std::chrono::nanoseconds(100));
		// Now with no more views active (and hopefully no iterators), we can clear
		BASE::clear();
		m_state.start = 0;
		m_state.count = 0;
		m_state.index = 0;
		m_state.begin = BASE::end();
		m_state.back = BASE::end();
		m_state.const_begin = std::add_const_t<BASE>::end();
		m_state.const_back = std::add_const_t<BASE>::end();
	}

	/**
	 * Cull all elements in the queue and resets indexing to 0 (size is 0).
	 *
	 * After this operation:
	 * - Any existing views as well as indices and iterators into them will continue to work as before
	 * - Any new views will treat the culled blocks as deleted:
	 *   - begin() and beginIndex() will refer to the first non-culled element
	 * - After delete_culled is called, all indices and iterators in culled blocks will cease to work
	 */
	void cull_clear()
	{
		std::unique_lock lock(m_mutex);
		auto front = m_state.begin;
		// Clear state, accounting for culled blocks
		m_state.start = 0;
		m_state.count = 0;
		m_state.index = 0;
		m_state.begin = BASE::end();
		m_state.back = BASE::end();
		m_state.const_begin = std::add_const_t<BASE>::end();
		m_state.const_back = std::add_const_t<BASE>::end();
		detachBlocks(front, m_state.begin);
	}
	
	/**
	 * Cull all elements in the queue.
	 *
	 * After this operation:
	 * - Any existing views as well as indices and iterators into them will continue to work as before
	 * - Any new views will treat the culled blocks as deleted:
	 *   - begin() and beginIndex() will refer to the first non-culled element
	 * - After delete_culled is called, all indices and iterators in culled blocks will cease to work
 	*/
	void cull_all()
	{
		std::unique_lock lock(m_mutex);
		auto front = m_state.begin;
		// Clear state, accounting for culled blocks
		m_state.start += m_state.count;
		m_state.count = 0;
		m_state.index = m_state.start*N;
		m_state.begin = BASE::end();
		m_state.back = BASE::end();
		m_state.const_begin = std::add_const_t<BASE>::end();
		m_state.const_back = std::add_const_t<BASE>::end();
		detachBlocks(front, m_state.begin);
	}

	/**
	 * Cull any number of blocks of size N from the front of the queue.
	 * Must not cull the last, currently writing block, so abs(num) < blocks
	 * if num < 0, count from back, leave -num blocks
	 * if num > 0, count from front, remove num blocks
	 *
	 * After this operation:
	 * - Any existing views as well as indices and iterators into them will continue to work as before
	 * - Any new views will treat the culled blocks as deleted:
	 *   - begin() and beginIndex() will refer to the first non-culled element
	 * - After delete_culled is called, all indices and iterators in culled blocks will cease to work
	 */
	void cull_front(int num = -1)
	{
		std::unique_lock lock(m_mutex);
		if (m_state.count <= std::abs(num))
			return; // Not allowed to cull last block - use cull_all instead
		auto front = m_state.begin;
		if (num < 0)
		{
			m_state.start += m_state.count+num;
			m_state.count = -num;
			m_state.index = m_state.start*N;
			m_state.begin = std::next(BASE::end(), num);
			m_state.const_begin = std::next(std::add_const_t<BASE>::end(), num);
			m_state.back = BASE::end();
			m_state.const_back = std::add_const_t<BASE>::end();
		}
		else
		{
			m_state.start += num;
			m_state.count -= num;
			m_state.begin = std::next(BASE::begin(), num);
			m_state.const_begin = std::next(std::add_const_t<BASE>::begin(), num);
			m_state.back = BASE::end();
			m_state.const_back = std::add_const_t<BASE>::end();
		}
		detachBlocks(front, m_state.begin);
	}

	/**
	 * Returns true if culled blocks exist that have not yet been deleted.
	 * e.g. cull_all or cull_front have been called successfully, but delete_culled has not yet been called after.
	 */
	bool has_culled() const
	{
		std::unique_lock lock(m_mutex);
		return !m_culledBlocks.empty();
	}

	/**
	 * Delete any unreferenced culled blocks from the front of the queue.
	 * Will not delete culled blocks still referenced by a View.
	 */
	void delete_culled()
	{
		std::list<std::array<T,N>> removedBlocks;
		std::unique_lock lock(m_mutex);
		while (!m_culledBlocks.empty())
		{ // Past culled blocks might still being referenced by Views
			if (m_culledBlocks.front().use_count() > 1)
				return;
			removedBlocks.splice(removedBlocks.end(), *this, m_culledBlocks.front()->front, std::next(m_culledBlocks.front()->back));
			m_culledBlocks.pop();
		}
		assert(BASE::begin() == m_state.begin);
		// Let destructor of removedBlocks delete the blocks (done this way to be able to do it without holding mutex)
		// Order of destruction should already put this after mutex is released, but be explicit anyway
		lock.unlock();
	}

	void swap(BlockedQueue<T, N> &other) noexcept
	{
		std::scoped_lock lock(m_mutex, other.m_mutex);
		BASE::swap(other);
		std::swap(m_state, other.m_state);
		std::swap(m_blockLifetime, other.m_blockLifetime);
		std::swap(m_culledBlocks, other.m_culledBlocks);
	}

	template<typename _T, std::size_t _N>
	constexpr inline void swap(BlockedQueue<_T, _N> &a, BlockedQueue<_T, _N> &b) noexcept
	{
		a.swap(b);
	}
};

#endif // BLOCKED_VECTOR_H