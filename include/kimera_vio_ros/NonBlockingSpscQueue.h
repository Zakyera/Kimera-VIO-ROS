#pragma once

#include <atomic>
#include <cstddef>
#include <utility>
#include <vector>

namespace VIO {

// Fixed-capacity SPSC ring. tryPush/tryPop never wait; the caller owns the
// drop policy. Capacity is fixed before either endpoint starts.
template <typename T>
class NonBlockingSpscQueue {
 public:
  explicit NonBlockingSpscQueue(std::size_t capacity)
      : slots_(capacity + 1u) {}

  bool tryPush(T value) {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = increment(head);
    if (next == tail_.load(std::memory_order_acquire)) {
      return false;
    }
    slots_[head] = std::move(value);
    head_.store(next, std::memory_order_release);
    return true;
  }

  bool tryPop(T* value) {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
      return false;
    }
    *value = std::move(slots_[tail]);
    tail_.store(increment(tail), std::memory_order_release);
    return true;
  }

  std::size_t approximateSize() const {
    const std::size_t head = head_.load(std::memory_order_acquire);
    const std::size_t tail = tail_.load(std::memory_order_acquire);
    return head >= tail ? head - tail : slots_.size() - tail + head;
  }

  std::size_t capacity() const { return slots_.size() - 1u; }

 private:
  std::size_t increment(std::size_t index) const {
    return (index + 1u) % slots_.size();
  }

  std::vector<T> slots_;
  alignas(64) std::atomic<std::size_t> head_{0u};
  alignas(64) std::atomic<std::size_t> tail_{0u};
};

}  // namespace VIO
