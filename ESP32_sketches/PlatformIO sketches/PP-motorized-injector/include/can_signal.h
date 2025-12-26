#ifndef CAN_SIGNAL_H
#define CAN_SIGNAL_H

#include <Arduino.h>
#include <string.h>

/**
 * Minimal CAN signal packing/unpacking helpers
 * Based on ODrive's can_simple.hpp implementation
 * Handles bit-level packing into CAN frames
 */

// Pack a signal into a CAN message at bit position
template <typename T>
void can_setSignal(uint8_t* data, T value, uint32_t bit_offset, uint32_t bit_width, bool is_signed = true) {
    uint64_t val = *reinterpret_cast<uint64_t*>(&value);
    
    // Shift value to bit position
    val <<= bit_offset;
    
    // Write to data buffer
    for (uint32_t i = bit_offset / 8; i < (bit_offset + bit_width + 7) / 8; i++) {
        data[i] |= (val >> (i * 8)) & 0xFF;
    }
}

// Extract a signal from a CAN message at bit position
template <typename T>
T can_getSignal(const uint8_t* data, uint32_t bit_offset, uint32_t bit_width, bool is_signed = true) {
    uint64_t val = 0;
    
    // Extract bits from data buffer
    for (uint32_t i = bit_offset / 8; i < (bit_offset + bit_width + 7) / 8; i++) {
        val |= ((uint64_t)data[i]) << (i * 8);
    }
    
    // Shift to align with bit_offset
    val >>= (bit_offset % 8);
    
    // Mask to bit_width
    val &= (1ULL << bit_width) - 1;
    
    // Sign extend if needed
    if (is_signed && (val & (1ULL << (bit_width - 1)))) {
        val |= ~((1ULL << bit_width) - 1);
    }
    
    return *reinterpret_cast<T*>(&val);
}

#endif
