#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <cstddef>

// Fixed-size single-producer / single-consumer ring buffer
// Producer: CAN RX task
// Consumer: main loop

template<typename T, size_t SIZE>
class RingBuffer {
public:
    RingBuffer() : head_(0), count_(0) {
        static_assert(SIZE > 0, "RingBuffer size must be > 0");
    }

    void push(const T& item) {
        buffer_[head_] = item;
        if (count_ < SIZE) {
            count_++;
        }
        head_ = (head_ + 1) % SIZE;
    }

    const T* getLatest() const {
        if (count_ == 0) {
            return nullptr;
        }
        size_t latestIndex = (head_ == 0) ? (SIZE - 1) : (head_ - 1);
        return &buffer_[latestIndex];
    }

    const T* getHistory(size_t index) const {
        if (index >= count_) {
            return nullptr;
        }
        size_t offset = index + 1;
        size_t actualIndex = (head_ + SIZE - offset) % SIZE;
        return &buffer_[actualIndex];
    }

    size_t getCount() const { return count_; }
    bool isEmpty() const { return count_ == 0; }
    bool isFull() const { return count_ == SIZE; }

    void clear() {
        head_ = 0;
        count_ = 0;
    }

private:
    T buffer_[SIZE];
    volatile size_t head_;
    volatile size_t count_;
};

#endif // RING_BUFFER_H
