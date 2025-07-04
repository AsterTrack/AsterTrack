/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <chrono>
#include <mutex>
#include <shared_mutex>
#include <system_error>
#include <type_traits>

/**
 * Modified by Seneral to be standalone
 *
 * NOTE: Dropped unsafe_for_async_usage_if (tagging structs with folly_is_unsafe_for_async_usage)
 * That custom tagging system was used for their SharedMutex implementation
 * Since it is not used in this standalone version, it is dropped
*/

// https://en.cppreference.com/w/cpp/experimental/is_detected
namespace folly
{
namespace detail
{

template<typename ...Ts> struct make_void
{
    using type = void;
};
template<typename ...Ts> using void_t = typename make_void<Ts...>::type;

template<class Default,
         class AlwaysVoid,
         template<class...> class Op,
         class... Args>
struct detector
{
    using value_t = std::false_type;
    using type = Default;
};

template<class Default, template<class...> class Op, class... Args>
struct detector<Default, void_t<Op<Args...>>, Op, Args...>
{
    using value_t = std::true_type;
    using type = Op<Args...>;
};

template<class Default, template<class...> class Op, class... Args>
using detected_or = detector<Default, void, Op, Args...>;

template<class Default, template<class...> class Op, class... Args>
using detected_or_t = typename detected_or<Default, Op, Args...>::type;

}  // namespace detail
}  // namespace folly





namespace folly {

namespace access {

#define FOLLY_CREATE_MEMBER_INVOKER_SUITE(membername)                 \
  struct membername##_fn {                                                 \
    template <typename O, typename... Args>                                \
    inline constexpr auto operator()(                                      \
        O&& o, Args&&... args) const                                       \
        noexcept(noexcept(                                                 \
            static_cast<O&&>(o).membername(static_cast<Args&&>(args)...))) \
            -> decltype(static_cast<O&&>(o).membername(                    \
                static_cast<Args&&>(args)...)) {                           \
      return static_cast<O&&>(o).membername(static_cast<Args&&>(args)...); \
    }                                                                      \
  };                                                                       \
  inline constexpr membername##_fn membername {}

//  locks and unlocks
FOLLY_CREATE_MEMBER_INVOKER_SUITE(lock);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_lock);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_lock_for);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_lock_until);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(unlock);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(lock_shared);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_lock_shared);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_lock_shared_for);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_lock_shared_until);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(unlock_shared);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(lock_upgrade);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_lock_upgrade);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_lock_upgrade_for);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_lock_upgrade_until);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(unlock_upgrade);

//  transitions
FOLLY_CREATE_MEMBER_INVOKER_SUITE(unlock_and_lock_shared);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(unlock_and_lock_upgrade);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_unlock_shared_and_lock);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_unlock_shared_and_lock_for);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_unlock_shared_and_lock_until);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_unlock_shared_and_lock_upgrade);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_unlock_shared_and_lock_upgrade_for);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_unlock_shared_and_lock_upgrade_until);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(unlock_upgrade_and_lock);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_unlock_upgrade_and_lock);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_unlock_upgrade_and_lock_for);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(try_unlock_upgrade_and_lock_until);
FOLLY_CREATE_MEMBER_INVOKER_SUITE(unlock_upgrade_and_lock_shared);

} // namespace access

struct adopt_lock_state_t {};
inline constexpr adopt_lock_state_t adopt_lock_state{};

namespace detail {

//  A lock base class with a mostly-complete implementation suitable for either
//  unique, shared, or upgrade lock base classes. However, each particular base
//  class specific to each lock category must still be its own class to avoid
//  overly permissive overloads of member and free swap.
template <typename Mutex, typename Policy>
class lock_base {
 public:
  using mutex_type = Mutex;
  using state_type = std::invoke_result_t<typename Policy::lock_fn, mutex_type&>;

  static_assert(
      std::is_same<state_type, std::decay_t<state_type>>::value,
      "state_type, if not void, must be a value type");
  static_assert(
      std::is_void<state_type>::value ||
          (std::is_nothrow_default_constructible<state_type>::value &&
           std::is_nothrow_copy_constructible<state_type>::value &&
           std::is_nothrow_copy_assignable<state_type>::value &&
           std::is_nothrow_destructible<state_type>::value),
      "state_type, if not void, must be noexcept-semiregular");
  static_assert(
      std::is_void<state_type>::value ||
          std::is_constructible<bool, state_type>::value,
      "state_type, if not void, must explicitly convert to bool");

 private:
  static constexpr bool has_state_ = !std::is_void<state_type>::value;
  using owner_type = std::conditional_t<has_state_, state_type, bool>;
  template <bool C, typename V = int>
  using if_ = std::enable_if_t<C, V>;

  mutex_type* mutex_{};
  owner_type state_{};

 public:
  lock_base() = default;
  lock_base(lock_base&& that) noexcept
      : mutex_{std::exchange(that.mutex_, nullptr)},
        state_{std::exchange(that.state_, owner_type{})} {}
  template <typename M = mutex_type, if_<!has_state_, M>* = nullptr>
  lock_base(M& mutex, std::adopt_lock_t)
      : mutex_{std::addressof(mutex)}, state_{owner_type{}} {}
  template <typename M = mutex_type, if_<has_state_, M>* = nullptr>
  lock_base(M& mutex, std::adopt_lock_t, owner_type const& state)
      : mutex_{std::addressof(mutex)}, state_{state} {
    state_ || (check_fail_<true>(), 0);
  }
  template <typename M = mutex_type, if_<has_state_, M>* = nullptr>
  lock_base(M& mutex, adopt_lock_state_t, owner_type const& state)
      : lock_base{mutex, std::adopt_lock, state} {}
  explicit lock_base(mutex_type& mutex)
      : mutex_{std::addressof(mutex)} {
    lock();
  }
  lock_base(mutex_type& mutex, std::defer_lock_t) noexcept
      : mutex_{std::addressof(mutex)} {}
  lock_base(mutex_type& mutex, std::try_to_lock_t)
      : mutex_{std::addressof(mutex)} {
    try_lock();
  }
  template <typename Rep, typename Period>
  lock_base(
      mutex_type& mutex, std::chrono::duration<Rep, Period> const& timeout)
      : mutex_{std::addressof(mutex)} {
    try_lock_for(timeout);
  }
  template <typename Clock, typename Duration>
  lock_base(
      mutex_type& mutex,
      std::chrono::time_point<Clock, Duration> const& deadline)
      : mutex_{std::addressof(mutex)} {
    try_lock_until(deadline);
  }

  ~lock_base() {
    if (owns_lock()) {
      unlock();
    }
  }

  lock_base& operator=(lock_base&& that) noexcept {
    if (owns_lock()) {
      unlock();
    }
    mutex_ = std::exchange(that.mutex_, nullptr);
    state_ = std::exchange(that.state_, owner_type{});
    return *this;
  }

  void lock() {
    check<false>();
    if constexpr (has_state_) {
      state_ = typename Policy::lock_fn{}(*mutex_);
    } else {
      typename Policy::lock_fn{}(*mutex_);
      state_ = true;
    }
  }

  bool try_lock() {
    check<false>();
    state_ = typename Policy::try_lock_fn{}(*mutex_);
    return !!state_;
  }

  template <typename Rep, typename Period>
  bool try_lock_for(std::chrono::duration<Rep, Period> const& timeout) {
    check<false>();
    state_ = typename Policy::try_lock_for_fn{}(*mutex_, timeout);
    return !!state_;
  }

  template <typename Clock, typename Duration>
  bool try_lock_until(
      std::chrono::time_point<Clock, Duration> const& deadline) {
    check<false>();
    state_ = typename Policy::try_lock_until_fn{}(*mutex_, deadline);
    return !!state_;
  }

  void unlock() {
    check<true>();
    if constexpr (has_state_) {
      // prohibit unlock to mutate state_
      typename Policy::unlock_fn{}(*mutex_, std::as_const(state_));
    } else {
      typename Policy::unlock_fn{}(*mutex_);
    }
    state_ = owner_type{};
  }

  mutex_type* release() noexcept {
    state_ = owner_type{};
    return std::exchange(mutex_, nullptr);
  }

  mutex_type* mutex() const noexcept { return mutex_; }

  template <bool C = has_state_, if_<C> = 0>
  state_type state() const noexcept {
    return state_;
  }

  bool owns_lock() const noexcept { return !!state_; }

  explicit operator bool() const noexcept { return !!state_; }

 protected:
  void swap(lock_base& that) noexcept {
    std::swap(mutex_, that.mutex_);
    std::swap(state_, that.state_);
  }

 private:
  template <bool Owns>
  void check() {
    if (!mutex_ || !state_ == Owns) {
      check_fail_<Owns>();
    }
  }

  template <bool Owns>
  [[noreturn]] void check_fail_() {
    auto perm = std::errc::operation_not_permitted;
    auto dead = std::errc::resource_deadlock_would_occur;
    auto code = !mutex_ || !state_ ? perm : dead;
    throw std::system_error(std::make_error_code(code));
  }
};

template <typename Mutex, typename Policy>
class lock_guard_base {
 private:
  using lock_type_ = lock_base<Mutex, Policy>;
  using lock_state_type_ = typename lock_type_::state_type;

  static constexpr bool has_state_ = !std::is_void<lock_state_type_>::value;
  using state_type_ = std::conditional_t<has_state_, lock_state_type_, bool>;
  template <bool C>
  using if_ = std::enable_if_t<C, int>;

 public:
  using mutex_type = Mutex;

  lock_guard_base(lock_guard_base const&) = delete;
  lock_guard_base(lock_guard_base&&) = delete;
  explicit lock_guard_base(mutex_type& mutex) : lock_{mutex} {}
  template <bool C = has_state_, if_<!C> = 0>
  lock_guard_base(mutex_type& mutex, std::adopt_lock_t)
      : lock_{mutex, std::adopt_lock} {}
  template <bool C = has_state_, if_<C> = 0>
  lock_guard_base(
      mutex_type& mutex, std::adopt_lock_t, state_type_ const& state)
      : lock_{mutex, std::adopt_lock, state} {}
  template <bool C = has_state_, if_<C> = 0>
  lock_guard_base(
      mutex_type& mutex, adopt_lock_state_t, state_type_ const& state)
      : lock_{mutex, std::adopt_lock, state} {}
  void operator=(lock_guard_base const&) = delete;
  void operator=(lock_guard_base&&) = delete;

 private:
  lock_type_ lock_;
};

struct lock_policy_unique {
  using lock_fn = access::lock_fn;
  using try_lock_fn = access::try_lock_fn;
  using try_lock_for_fn = access::try_lock_for_fn;
  using try_lock_until_fn = access::try_lock_until_fn;
  using unlock_fn = access::unlock_fn;
};

struct lock_policy_shared {
  using lock_fn = access::lock_shared_fn;
  using try_lock_fn = access::try_lock_shared_fn;
  using try_lock_for_fn = access::try_lock_shared_for_fn;
  using try_lock_until_fn = access::try_lock_shared_until_fn;
  using unlock_fn = access::unlock_shared_fn;
};

struct lock_policy_upgrade {
  using lock_fn = access::lock_upgrade_fn;
  using try_lock_fn = access::try_lock_upgrade_fn;
  using try_lock_for_fn = access::try_lock_upgrade_for_fn;
  using try_lock_until_fn = access::try_lock_upgrade_until_fn;
  using unlock_fn = access::unlock_upgrade_fn;
};

template <typename Mutex>
using lock_policy_hybrid = std::conditional_t<
    std::is_invocable_v<access::lock_shared_fn, Mutex&>,
    lock_policy_shared,
    lock_policy_unique>;

template <typename Mutex>
using lock_base_unique = lock_base<Mutex, lock_policy_unique>;

template <typename Mutex>
using lock_base_shared = lock_base<Mutex, lock_policy_shared>;

template <typename Mutex>
using lock_base_upgrade = lock_base<Mutex, lock_policy_upgrade>;

template <typename Mutex>
using lock_base_hybrid = lock_base<Mutex, lock_policy_hybrid<Mutex>>;

} // namespace detail

//  unique_lock_base
//
//  A lock-holder base which holds exclusive locks, usable with any mutex type.
//
//  Works with both lockable mutex types and lockable-with-state mutex types.
//
//  When defining lockable-with-state mutex types, specialize std::unique_lock
//  to derive this. See the example with upgrade_lock.
//
//  A lockable-with-state mutex type is signalled by the return type of mutex
//  member function lock. Members try_lock, try_lock_for, and try_lock_until
//  all return this type and member unlock accepts this type.
template <typename Mutex>
class unique_lock_base : public detail::lock_base_unique<Mutex> {
 private:
  using base = detail::lock_base_unique<Mutex>;
  using self = unique_lock_base;

 public:
  using base::base;

  void swap(self& that) noexcept { base::swap(that); }

  friend void swap(self& a, self& b) noexcept { a.swap(b); }
};

//  shared_lock_base
//
//  A lock-holder base which holds shared locks, usable with any shared mutex
//  type.
//
//  Works with both shared-lockable mutex types and shared-lockable-with-state
//  mutex types.
//
//  When defining shared-lockable-with-state mutex types, specialize
//  std::shared_lock to derive this. See the example with upgrade_lock.
//
//  A shared-lockable-with-state mutex type is signalled by the return type of
//  mutex member function lock_shared. Members try_lock_shared,
//  try_lock_shared_for, and try_lock_shared_until all return this type and
//  member unlock_shared accepts this type. Likewise for mutex member
//  transition functions.
template <typename Mutex>
class shared_lock_base : public detail::lock_base_shared<Mutex> {
 private:
  using base = detail::lock_base_shared<Mutex>;
  using self = shared_lock_base;

 public:
  using base::base;

  void swap(self& that) noexcept { base::swap(that); }

  friend void swap(self& a, self& b) noexcept { a.swap(b); }
};

//  upgrade_lock_base
//
//  A lock-holder base which holds upgrade locks, usable with any upgrade mutex
//  type.
//
//  Works with both upgrade-lockable mutex types and upgrade-lockable-with-state
//  mutex types.
//
//  There are no use-cases except the one below.
//
//  An upgrade-lockable-with-state mutex type is signalled by the return type of
//  mutex member function lock_upgrade. Members try_lock_upgrade,
//  try_lock_upgrade_for, and try_lock_upgrade_until all return this type and
//  member unlock_upgrade accepts this type. Likewise for mutex member
//  transition functions.
template <typename Mutex>
class upgrade_lock_base : public detail::lock_base_upgrade<Mutex> {
 private:
  using base = detail::lock_base_upgrade<Mutex>;
  using self = upgrade_lock_base;

 public:
  using base::base;

  void swap(self& that) noexcept { base::swap(that); }

  friend void swap(self& a, self& b) noexcept { a.swap(b); }
};

//  hybrid_lock_base
//
//  A lock-holder base which holds shared locks for shared mutex types or
//  exclusive locks otherwise.
//
//  See unique_lock_base and shared_lock_base.
template <typename Mutex>
class hybrid_lock_base : public detail::lock_base_hybrid<Mutex> {
 private:
  using base = detail::lock_base_hybrid<Mutex>;
  using self = hybrid_lock_base;

 public:
  using base::base;

  void swap(self& that) noexcept { base::swap(that); }

  friend void swap(self& a, self& b) noexcept { a.swap(b); }
};

//  unique_lock
//
//  Alias to std::unique_lock.
using std::unique_lock;

//  shared_lock
//
//  Alias to std::shared_lock.
using std::shared_lock;

//  upgrade_lock
//
//  A lock-holder type which holds upgrade locks, usable with any upgrade mutex
//  type. An upgrade mutex is a shared mutex which supports the upgrade state.
//
//  Works with both upgrade-lockable mutex types and upgrade-lockable-with-state
//  mutex types.
//
//  Upgrade locks are not useful by themselves; they are primarily useful since
//  upgrade locks may be transitioned atomically to exclusive locks. This lock-
//  holder type works with the transition_to_... functions below to facilitate
//  atomic transition from ugprade lock to exclusive lock.
template <typename Mutex>
class upgrade_lock : public upgrade_lock_base<Mutex> {
 public:
  using upgrade_lock_base<Mutex>::upgrade_lock_base;
};

template <typename Mutex, typename... A>
explicit upgrade_lock(Mutex&, A const&...) -> upgrade_lock<Mutex>;

//  hybrid_lock
//
//  A lock-holder type which holds shared locks for shared mutex types or
//  exclusive locks otherwise.
//
//  See unique_lock and shared_lock.
template <typename Mutex>
class hybrid_lock : public hybrid_lock_base<Mutex> {
 public:
  using hybrid_lock_base<Mutex>::hybrid_lock_base;
};

template <typename Mutex, typename... A>
explicit hybrid_lock(Mutex&, A const&...) -> hybrid_lock<Mutex>;

//  lock_guard_base
//
//  A lock-guard which holds exclusive locks, usable with any mutex type.
//
//  Works with both lockable mutex types and lockable-with-state mutex types.
//
//  When defining lockable-with-state mutex types, specialize std::lock_guard
//  to derive this.
template <typename Mutex>
class unique_lock_guard_base
    : public detail::lock_guard_base<Mutex, detail::lock_policy_unique> {
 private:
  using base = detail::lock_guard_base<Mutex, detail::lock_policy_unique>;

 public:
  using base::base;
};

//  unique_lock_guard
//
//  Alias to std::lock_guard.
template <typename Mutex>
using unique_lock_guard = std::lock_guard<Mutex>;

//  shared_lock_guard
//
//  A lock-guard which holds shared locks, usable with any shared mutex type.
//
//  Works with both lockable mutex types and lockable-with-state mutex types.
template <typename Mutex>
class shared_lock_guard
    : public detail::lock_guard_base<Mutex, detail::lock_policy_shared> {
 private:
  using base = detail::lock_guard_base<Mutex, detail::lock_policy_shared>;

 public:
  using base::base;
};

//  hybrid_lock_guard
//
//  For shared mutex types, effectively shared_lock_guard; otherwise,
//  effectively unique_lock_guard.
template <typename Mutex>
class hybrid_lock_guard
    : public detail::lock_guard_base<Mutex, detail::lock_policy_hybrid<Mutex>> {
 private:
  using base =
      detail::lock_guard_base<Mutex, detail::lock_policy_hybrid<Mutex>>;

 public:
  using base::base;
};

template <typename Mutex, typename S>
hybrid_lock_guard(Mutex&, adopt_lock_state_t, S) -> hybrid_lock_guard<Mutex>;

} // namespace folly

#if defined(_LIBCPP_VERSION)
_LIBCPP_BEGIN_NAMESPACE_STD
#else
namespace std {
#endif

template <typename Mutex, typename S>
unique_lock(Mutex&, folly::adopt_lock_state_t, S) -> unique_lock<Mutex>;

template <typename Mutex, typename S>
shared_lock(Mutex&, folly::adopt_lock_state_t, S) -> shared_lock<Mutex>;

template <typename Mutex, typename S>
lock_guard(Mutex&, folly::adopt_lock_state_t, S) -> lock_guard<Mutex>;

#if defined(_LIBCPP_VERSION)
_LIBCPP_END_NAMESPACE_STD
#else
}
#endif

namespace folly {

namespace detail {

template <typename L>
using lock_state_type_of_t_ = typename L::state_type;
template <typename L>
using lock_state_type_of_t = detected_or_t<void, lock_state_type_of_t_, L>;

template <typename State>
struct transition_lock_result_ {
  template <typename Transition, typename Mutex, typename... A>
  using apply = std::invoke_result_t<Transition, Mutex&, State const&, A const&...>;
};
template <>
struct transition_lock_result_<void> {
  template <typename Transition, typename Mutex, typename... A>
  using apply = std::invoke_result_t<Transition, Mutex&, A const&...>;
};
template <typename From, typename Transition, typename... A>
using transition_lock_result_t_ =
    typename transition_lock_result_<lock_state_type_of_t<From>>::
        template apply<Transition, typename From::mutex_type&, A...>;

template <typename From, typename Transition, typename... A>
auto transition_lock_2_(From& lock, Transition transition, A const&... a) {
  using FromState = lock_state_type_of_t<From>;
  if constexpr (std::is_void_v<FromState>) {
    // release() may check or mutate mutex state to support the dissociation;
    // call it before performing the transition.
    return transition(*lock.release(), a...);
  } else {
    auto state = lock.state();
    // release() may check or mutate mutex state to support the dissociation;
    // call it before performing the transition.
    return transition(*lock.release(), std::move(state), a...);
  }
}
template <typename From, typename Transition, typename... A>
auto transition_lock_1_(From& lock, Transition transition, A const&... a) {
  using Result = transition_lock_result_t_<From, Transition, A...>;
  if constexpr (std::is_void_v<Result>) {
    return detail::transition_lock_2_(lock, transition, a...), true;
  } else {
    return detail::transition_lock_2_(lock, transition, a...);
  }
}
template <typename To, typename From, typename Transition, typename... A>
auto transition_lock_0_(From& lock, Transition transition, A const&... a) {
  using ToState = lock_state_type_of_t<To>;
  auto& mutex = *lock.mutex();
  auto s = detail::transition_lock_1_(lock, transition, a...);
  if constexpr (std::is_void_v<ToState>) {
    return !s ? To{} : To{mutex, std::adopt_lock};
  } else {
    return !s ? To{} : To{mutex, folly::adopt_lock_state, s};
  }
}
template <
    template <typename>
    class To,
    template <typename>
    class From,
    typename Mutex,
    typename Transition,
    typename... A>
auto transition_lock_(From<Mutex>& lock, Transition transition, A const&... a) {
  // clang-format off
  return
      !lock.mutex() ? To<Mutex>{} :
      !lock.owns_lock() ? To<Mutex>{*lock.release(), std::defer_lock} :
      detail::transition_lock_0_<To<Mutex>>(lock, transition, a...);
  // clang-format on
}

template <typename, typename>
struct transition_lock_policy;

template <typename Mutex>
struct transition_lock_policy<unique_lock<Mutex>, shared_lock<Mutex>> {
  using transition_fn = access::unlock_and_lock_shared_fn;
};
template <typename Mutex>
struct transition_lock_policy<unique_lock<Mutex>, upgrade_lock<Mutex>> {
  using transition_fn = access::unlock_and_lock_upgrade_fn;
};
template <typename Mutex>
struct transition_lock_policy<shared_lock<Mutex>, unique_lock<Mutex>> {
  using try_transition_fn = access::try_unlock_shared_and_lock_fn;
  using try_transition_for_fn = access::try_unlock_shared_and_lock_for_fn;
  using try_transition_until_fn = access::try_unlock_shared_and_lock_until_fn;
};
template <typename Mutex>
struct transition_lock_policy<shared_lock<Mutex>, upgrade_lock<Mutex>> {
  using try_transition_fn = access::try_unlock_shared_and_lock_upgrade_fn;
  using try_transition_for_fn =
      access::try_unlock_shared_and_lock_upgrade_for_fn;
  using try_transition_until_fn =
      access::try_unlock_shared_and_lock_upgrade_until_fn;
};
template <typename Mutex>
struct transition_lock_policy<upgrade_lock<Mutex>, unique_lock<Mutex>> {
  using transition_fn = access::unlock_upgrade_and_lock_fn;
  using try_transition_fn = access::try_unlock_upgrade_and_lock_fn;
  using try_transition_for_fn = access::try_unlock_upgrade_and_lock_for_fn;
  using try_transition_until_fn = access::try_unlock_upgrade_and_lock_until_fn;
};
template <typename Mutex>
struct transition_lock_policy<upgrade_lock<Mutex>, shared_lock<Mutex>> {
  using transition_fn = access::unlock_upgrade_and_lock_shared_fn;
};

} // namespace detail

//  transition_lock
//
//  Represents an atomic transition from the from-lock to the to-lock. Waits
//  unboundedly for the transition to become available.
template <
    template <typename>
    class ToLock,
    typename Mutex,
    template <typename>
    class FromLock>
ToLock<Mutex> transition_lock(FromLock<Mutex>& lock) {
  using policy = detail::transition_lock_policy<FromLock<Mutex>, ToLock<Mutex>>;
  auto _ = typename policy::transition_fn{};
  return detail::transition_lock_<ToLock>(lock, _);
}

//  try_transition_lock
//
//  Represents an atomic transition attempt from the from-lock to the to-lock.
//  Does not wait if the transition is not immediately available.
template <
    template <typename>
    class ToLock,
    typename Mutex,
    template <typename>
    class FromLock>
ToLock<Mutex> try_transition_lock(FromLock<Mutex>& lock) {
  using policy = detail::transition_lock_policy<FromLock<Mutex>, ToLock<Mutex>>;
  auto _ = typename policy::try_transition_fn{};
  return detail::transition_lock_<ToLock>(lock, _);
}

//  try_transition_lock_for
//
//  Represents an atomic transition attempt from the from-lock to the to-lock
//  bounded by a timeout. Waits up to the timeout for the transition to become
//  available.
template <
    template <typename>
    class ToLock,
    typename Mutex,
    template <typename>
    class FromLock,
    typename Rep,
    typename Period>
ToLock<Mutex> try_transition_lock_for(
    FromLock<Mutex>& lock, std::chrono::duration<Rep, Period> const& timeout) {
  using policy = detail::transition_lock_policy<FromLock<Mutex>, ToLock<Mutex>>;
  auto _ = typename policy::try_transition_for_fn{};
  return detail::transition_lock_<ToLock>(lock, _, timeout);
}

//  try_transition_lock_until
//
//  Represents an atomic transition attempt from the from-lock to the to-lock
//  bounded by a deadline. Waits up to the deadline for the transition to become
//  available.
template <
    template <typename>
    class ToLock,
    typename Mutex,
    template <typename>
    class FromLock,
    typename Clock,
    typename Duration>
ToLock<Mutex> try_transition_lock_until(
    FromLock<Mutex>& lock,
    std::chrono::time_point<Clock, Duration> const& deadline) {
  using policy = detail::transition_lock_policy<FromLock<Mutex>, ToLock<Mutex>>;
  auto _ = typename policy::try_transition_until_fn{};
  return detail::transition_lock_<ToLock>(lock, _, deadline);
}

//  transition_to_shared_lock(unique_lock)
//
//  Wraps mutex member function unlock_and_lock_shared.
//
//  Represents an immediate atomic downgrade transition from exclusive lock to
//  to shared lock.
template <typename Mutex>
shared_lock<Mutex> transition_to_shared_lock(unique_lock<Mutex>& lock) {
  return transition_lock<shared_lock>(lock);
}

//  transition_to_shared_lock(upgrade_lock)
//
//  Wraps mutex member function unlock_upgrade_and_lock_shared.
//
//  Represents an immediate atomic downgrade transition from upgrade lock to
//  shared lock.
template <typename Mutex>
shared_lock<Mutex> transition_to_shared_lock(upgrade_lock<Mutex>& lock) {
  return transition_lock<shared_lock>(lock);
}

//  transition_to_upgrade_lock(unique_lock)
//
//  Wraps mutex member function unlock_and_lock_upgrade.
//
//  Represents an immediate atomic downgrade transition from unique lock to
//  upgrade lock.
template <typename Mutex>
upgrade_lock<Mutex> transition_to_upgrade_lock(unique_lock<Mutex>& lock) {
  return transition_lock<upgrade_lock>(lock);
}

//  transition_to_unique_lock(upgrade_lock)
//
//  Wraps mutex member function unlock_upgrade_and_lock.
//
//  Represents an eventual atomic upgrade transition from upgrade lock to unique
//  lock.
template <typename Mutex>
unique_lock<Mutex> transition_to_unique_lock(upgrade_lock<Mutex>& lock) {
  return transition_lock<unique_lock>(lock);
}

//  try_transition_to_unique_lock(upgrade_lock)
//
//  Wraps mutex member function try_unlock_upgrade_and_lock.
//
//  Represents an immediate attempted atomic upgrade transition from upgrade
//  lock to unique lock.
template <typename Mutex>
unique_lock<Mutex> try_transition_to_unique_lock(upgrade_lock<Mutex>& lock) {
  return transition_lock<unique_lock>(lock);
}

//  try_transition_to_unique_lock_for(upgrade_lock)
//
//  Wraps mutex member function try_unlock_upgrade_and_lock_for.
//
//  Represents an eventual attempted atomic upgrade transition from upgrade
//  lock to unique lock.
template <typename Mutex, typename Rep, typename Period>
unique_lock<Mutex> try_transition_to_unique_lock_for(
    upgrade_lock<Mutex>& lock,
    std::chrono::duration<Rep, Period> const& timeout) {
  return try_transition_lock_for<unique_lock>(lock, timeout);
}

//  try_transition_to_unique_lock_until(upgrade_lock)
//
//  Wraps mutex member function try_unlock_upgrade_and_lock_until.
//
//  Represents an eventual attempted atomic upgrade transition from upgrade
//  lock to unique lock.
template <typename Mutex, typename Clock, typename Duration>
unique_lock<Mutex> try_transition_to_unique_lock_until(
    upgrade_lock<Mutex>& lock,
    std::chrono::time_point<Clock, Duration> const& deadline) {
  return try_transition_lock_until<unique_lock>(lock, deadline);
}

//  try_transition_to_unique_lock(shared_lock)
//
//  Wraps mutex member function try_unlock_shared_and_lock.
//
//  Represents an immediate attempted atomic upgrade transition from shared
//  lock to unique lock.
template <typename Mutex>
unique_lock<Mutex> try_transition_to_unique_lock(shared_lock<Mutex>& lock) {
  return try_transition_lock<unique_lock>(lock);
}

//  try_transition_to_unique_lock_for(shared_lock)
//
//  Wraps mutex member function try_unlock_shared_and_lock_for.
//
//  Represents an eventual attempted atomic upgrade transition from shared
//  lock to unique lock.
template <typename Mutex, typename Rep, typename Period>
unique_lock<Mutex> try_transition_to_unique_lock_for(
    shared_lock<Mutex>& lock,
    std::chrono::duration<Rep, Period> const& timeout) {
  return try_transition_lock_for<unique_lock>(lock, timeout);
}

//  try_transition_to_unique_lock_until(shared_lock)
//
//  Wraps mutex member function try_unlock_shared_and_lock_until.
//
//  Represents an eventual attempted atomic upgrade transition from shared
//  lock to unique lock.
template <typename Mutex, typename Clock, typename Duration>
unique_lock<Mutex> try_transition_to_unique_lock_until(
    shared_lock<Mutex>& lock,
    std::chrono::time_point<Clock, Duration> const& deadline) {
  return try_transition_lock_until<unique_lock>(lock, deadline);
}

//  try_transition_to_upgrade_lock(shared_lock)
//
//  Wraps mutex member function try_unlock_shared_and_lock_upgrade.
//
//  Represents an immediate attempted atomic upgrade transition from shared
//  lock to upgrade lock.
template <typename Mutex>
upgrade_lock<Mutex> try_transition_to_upgrade_lock(shared_lock<Mutex>& lock) {
  return try_transition_lock<upgrade_lock>(lock);
}

//  try_transition_to_upgrade_lock_for(shared_lock)
//
//  Wraps mutex member function try_unlock_shared_and_lock_upgrade_for.
//
//  Represents an eventual attempted atomic upgrade transition from shared
//  lock to upgrade lock.
template <typename Mutex, typename Rep, typename Period>
upgrade_lock<Mutex> try_transition_to_upgrade_lock_for(
    shared_lock<Mutex>& lock,
    std::chrono::duration<Rep, Period> const& timeout) {
  return try_transition_lock_for<upgrade_lock>(lock, timeout);
}

//  try_transition_to_upgrade_lock_until(shared_lock)
//
//  Wraps mutex member function try_unlock_shared_and_lock_upgrade_until.
//
//  Represents an eventual attempted atomic upgrade transition from shared
//  lock to upgrade lock.
template <typename Mutex, typename Clock, typename Duration>
upgrade_lock<Mutex> try_transition_to_upgrade_lock_until(
    shared_lock<Mutex>& lock,
    std::chrono::time_point<Clock, Duration> const& deadline) {
  return try_transition_lock_until<upgrade_lock>(lock, deadline);
}

} // namespace folly
