#ifndef WHISTLE_RING_BUFFER_H
#define WHISTLE_RING_BUFFER_H

#include <array>

namespace whistle {

template <typename T, std::size_t N>
class RingBuffer {
 public:
  explicit RingBuffer(const T& init) { vals.fill(init); }

  void reset(const T& val) { vals.fill(val); }

  void push(const T& val) {
    pos = (pos + 1) % N;
    vals[pos] = val;
  }

  const T& oldest() const { return at(N - 1); }

 private:
  const T& at(std::size_t n) const {
    if (pos < n) {
      return vals[pos + N - n];
    }
    return vals[pos - n];
  }

  std::array<T, N> vals;
  std::size_t pos = N - 1;
};

}  // namespace whistle

#endif  // WHISTLE_RING_BUFFER_H
