/**
 * @file RingBuffer.h
 * @brief Template-based circular buffer for BroadcastDataStore v2
 * 
 * @details
 * Fixed-size ring buffer for storing timestamped CAN message history.
 * - Zero dynamic allocation (compile-time sized)
 * - O(1) push and latest access
 * - Automatic wraparound (oldest evicted)
 * - Thread-safe single producer (Core 0 write, Core 1 read)
 * 
 * Usage:
 *   RingBuffer<TimestampedPosition, 10> encoderHistory;
 *   encoderHistory.push(newData);
 *   auto latest = encoderHistory.getLatest();
 * 
 * @author PP-motorized-injector
 * @date January 16, 2026
 */

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <Arduino.h>

/**
 * @brief Fixed-size circular buffer template
 * @tparam T Data type to store (must be copyable)
 * @tparam SIZE Buffer capacity (number of elements)
 */
template<typename T, size_t SIZE>
class RingBuffer {
public:
    /**
     * @brief Constructor - initializes empty buffer
     */
    RingBuffer() : head_(0), count_(0) {
        static_assert(SIZE > 0, "RingBuffer size must be greater than 0");
    }

    /**
     * @brief Add new element to buffer (overwrites oldest if full)
     * @param item Element to store
     * 
     * Thread-safe: Single producer (Core 0), single consumer (Core 1)
     * Write head_ last to ensure reader sees consistent state.
     */
    void push(const T& item) {
        buffer_[head_] = item;
        
        // Update count first (reader-safe)
        if (count_ < SIZE) {
            count_++;
        }
        
        // Update head last (atomic on ESP32)
        head_ = (head_ + 1) % SIZE;
    }

    /**
     * @brief Get most recent element
     * @return Pointer to latest element, or nullptr if empty
     * 
     * Note: Returns pointer to avoid copy overhead. Valid until next push().
     */
    const T* getLatest() const {
        if (count_ == 0) {
            return nullptr;
        }
        
        // Latest is one position before head (with wraparound)
        size_t latestIndex = (head_ == 0) ? (SIZE - 1) : (head_ - 1);
        return &buffer_[latestIndex];
    }

    /**
     * @brief Get element at index (0 = latest, 1 = previous, etc.)
     * @param index History offset (0 = newest, count-1 = oldest)
     * @return Pointer to element, or nullptr if index out of range
     * 
     * Usage: getHistory(0) = latest, getHistory(1) = previous, etc.
     */
    const T* getHistory(size_t index) const {
        if (index >= count_) {
            return nullptr;  // Out of range
        }
        
        // Calculate position relative to head (walking backwards)
        size_t offset = index + 1;  // +1 because head points to NEXT write position
        size_t actualIndex = (head_ + SIZE - offset) % SIZE;
        return &buffer_[actualIndex];
    }

    /**
     * @brief Check if buffer is full
     * @return true if buffer contains SIZE elements
     */
    bool isFull() const {
        return count_ == SIZE;
    }

    /**
     * @brief Check if buffer is empty
     * @return true if no elements stored
     */
    bool isEmpty() const {
        return count_ == 0;
    }

    /**
     * @brief Get number of elements currently stored
     * @return Element count (0 to SIZE)
     */
    size_t getCount() const {
        return count_;
    }

    /**
     * @brief Get buffer capacity
     * @return Maximum number of elements (SIZE)
     */
    size_t getCapacity() const {
        return SIZE;
    }

    /**
     * @brief Clear all elements (reset to empty)
     */
    void clear() {
        head_ = 0;
        count_ = 0;
        // Note: Don't zero buffer_ - unnecessary overhead
    }

    /**
     * @brief Iterate through buffer from newest to oldest
     * @param callback Function to call for each element (bool callback(const T& item, size_t index))
     * 
     * Callback receives element and its index (0=newest).
     * If callback returns false, iteration stops.
     * 
     * Usage:
     *   buffer.forEach([](const auto& item, size_t idx) {
     *       Serial.print(item.position);
     *       return true;  // Continue
     *   });
     */
    template<typename Func>
    void forEach(Func callback) const {
        for (size_t i = 0; i < count_; i++) {
            const T* item = getHistory(i);
            if (item && !callback(*item, i)) {
                break;  // Callback returned false, stop iteration
            }
        }
    }

private:
    T buffer_[SIZE];       ///< Fixed-size storage array
    volatile size_t head_; ///< Next write position (volatile for thread safety)
    volatile size_t count_; ///< Number of elements stored (0 to SIZE)
};

#endif // RINGBUFFER_H

// =============================================================================
// UNIT TEST CODE (Phase 1 Validation)
// =============================================================================
/*
 * Add this code to main.cpp TEST_MODE_PHASE1 section to validate RingBuffer:
 * 
 * #include "RingBuffer.h"
 * 
 * void testRingBuffer() {
 *     Serial.println("\n=== RingBuffer Unit Test ===");
 *     
 *     // Test structure
 *     struct TestData {
 *         float value;
 *         uint64_t timestamp;
 *     };
 *     
 *     RingBuffer<TestData, 5> buffer;  // Small buffer for testing
 *     
 *     // Test 1: Empty buffer
 *     Serial.print("Test 1 - Empty: ");
 *     if (buffer.isEmpty() && buffer.getCount() == 0 && buffer.getLatest() == nullptr) {
 *         Serial.println("PASS");
 *     } else {
 *         Serial.println("FAIL");
 *     }
 *     
 *     // Test 2: Push elements (not full)
 *     Serial.print("Test 2 - Push 3 elements: ");
 *     for (int i = 1; i <= 3; i++) {
 *         buffer.push({(float)i * 10.0f, (uint64_t)i * 1000});
 *     }
 *     if (buffer.getCount() == 3 && !buffer.isFull()) {
 *         Serial.println("PASS");
 *     } else {
 *         Serial.println("FAIL");
 *     }
 *     
 *     // Test 3: Get latest
 *     Serial.print("Test 3 - Latest value: ");
 *     const TestData* latest = buffer.getLatest();
 *     if (latest && latest->value == 30.0f && latest->timestamp == 3000) {
 *         Serial.println("PASS (30.0 @ 3000us)");
 *     } else {
 *         Serial.println("FAIL");
 *     }
 *     
 *     // Test 4: Get history
 *     Serial.print("Test 4 - History access: ");
 *     const TestData* prev = buffer.getHistory(1);  // Previous value
 *     const TestData* oldest = buffer.getHistory(2);  // Oldest value
 *     if (prev && prev->value == 20.0f && oldest && oldest->value == 10.0f) {
 *         Serial.println("PASS (20.0, 10.0)");
 *     } else {
 *         Serial.println("FAIL");
 *     }
 *     
 *     // Test 5: Fill buffer (reaches capacity)
 *     Serial.print("Test 5 - Fill to capacity: ");
 *     buffer.push({40.0f, 4000});
 *     buffer.push({50.0f, 5000});
 *     if (buffer.isFull() && buffer.getCount() == 5) {
 *         Serial.println("PASS");
 *     } else {
 *         Serial.println("FAIL");
 *     }
 *     
 *     // Test 6: Wraparound (oldest evicted)
 *     Serial.print("Test 6 - Wraparound: ");
 *     buffer.push({60.0f, 6000});  // Should evict 10.0
 *     buffer.push({70.0f, 7000});  // Should evict 20.0
 *     latest = buffer.getLatest();
 *     oldest = buffer.getHistory(4);  // Oldest still in buffer
 *     if (latest && latest->value == 70.0f && oldest && oldest->value == 30.0f) {
 *         Serial.println("PASS (latest=70.0, oldest=30.0)");
 *     } else {
 *         Serial.println("FAIL");
 *     }
 *     
 *     // Test 7: forEach iteration
 *     Serial.print("Test 7 - forEach: ");
 *     Serial.print("[");
 *     int count = 0;
 *     buffer.forEach([&count](const TestData& item, size_t idx) {
 *         if (count > 0) Serial.print(", ");
 *         Serial.print(item.value, 1);
 *         count++;
 *         return true;
 *     });
 *     Serial.println("] (newest to oldest)");
 *     
 *     // Test 8: Clear and reuse
 *     Serial.print("Test 8 - Clear: ");
 *     buffer.clear();
 *     if (buffer.isEmpty() && buffer.getCount() == 0) {
 *         Serial.println("PASS");
 *     } else {
 *         Serial.println("FAIL");
 *     }
 *     
 *     Serial.println("=== RingBuffer Test Complete ===\n");
 * }
 * 
 * // In setup():
 * testRingBuffer();
 */
