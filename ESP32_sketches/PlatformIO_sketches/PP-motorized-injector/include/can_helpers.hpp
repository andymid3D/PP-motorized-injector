#pragma once

#include <stdint.h>
#include <algorithm>
#include <cstring>

/**
 * Official ODrive CAN bit-packing helpers
 * Source: https://github.com/odriverobotics/ODrive/blob/master/Firmware/communication/can/can_helpers.hpp
 * 
 * These provide type-safe bit-level packing/unpacking for CAN message payloads.
 * Supports both Intel (little-endian) and Motorola (big-endian) byte order.
 */

struct can_Message_t {
    uint32_t id = 0x000;  // 11-bit standard (0x7ff), 29-bit extended (0x1FFFFFFF)
    bool isExt = false;
    bool rtr = false;
    uint8_t len = 8;
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
};

/**
 * Extract a signal from CAN message buffer
 * @tparam T Return type (uint8_t, uint16_t, uint32_t, int32_t, float, etc.)
 * @param msg CAN message to extract from
 * @param startBit Bit position to start at (0-63)
 * @param length Number of bits to extract (1-64)
 * @param isIntel True for Intel (little-endian), false for Motorola (big-endian)
 * @return Extracted value, cast to type T
 */
template <typename T>
constexpr T can_getSignal(can_Message_t msg, const uint8_t startBit, 
                          const uint8_t length, const bool isIntel) {
    uint64_t tempVal = 0;
    uint64_t mask = length < 64 ? (1ULL << length) - 1ULL : -1ULL;

    if (isIntel) {
        std::memcpy(&tempVal, msg.buf, sizeof(tempVal));
        tempVal = (tempVal >> startBit) & mask;
    } else {
        std::reverse(msg.buf, msg.buf + 8);  // Reverse C array using pointer arithmetic
        std::memcpy(&tempVal, msg.buf, sizeof(tempVal));
        tempVal = (tempVal >> (64 - startBit - length)) & mask;
    }

    T retVal;
    std::memcpy(&retVal, &tempVal, sizeof(T));
    return retVal;
}

/**
 * Insert a signal into CAN message buffer
 * @tparam T Type of value to insert (uint8_t, uint16_t, uint32_t, int32_t, float, etc.)
 * @param msg CAN message to insert into (modified)
 * @param val Value to insert
 * @param startBit Bit position to start at (0-63)
 * @param length Number of bits to use (1-64)
 * @param isIntel True for Intel (little-endian), false for Motorola (big-endian)
 */
template <typename T>
constexpr void can_setSignal(can_Message_t& msg, const T& val, 
                             const uint8_t startBit, const uint8_t length, const bool isIntel) {
    uint64_t valAsBits = 0;
    std::memcpy(&valAsBits, &val, sizeof(val));

    uint64_t mask = length < 64 ? (1ULL << length) - 1ULL : -1ULL;

    if (isIntel) {
        uint64_t data = 0;
        std::memcpy(&data, msg.buf, sizeof(data));
        data &= ~(mask << startBit);
        data |= valAsBits << startBit;
        std::memcpy(msg.buf, &data, sizeof(data));
    } else {
        uint64_t data = 0;
        std::reverse(msg.buf, msg.buf + 8);  // Reverse C array using pointer arithmetic
        std::memcpy(&data, msg.buf, sizeof(data));
        data &= ~(mask << (64 - startBit - length));
        data |= valAsBits << (64 - startBit - length);
        std::memcpy(msg.buf, &data, sizeof(data));
        std::reverse(msg.buf, msg.buf + 8);  // Reverse back
    }
}

/**
 * Extract a scaled signal from CAN message
 * @param msg CAN message to extract from
 * @param startBit Bit position to start at
 * @param length Number of bits to extract
 * @param isIntel True for Intel byte order
 * @param factor Scale factor: output = (raw * factor) + offset
 * @param offset Scale offset: output = (raw * factor) + offset
 * @return Scaled value as float
 */
template<typename T>
float can_getSignal(can_Message_t msg, const uint8_t startBit, 
                    const uint8_t length, const bool isIntel, 
                    const float factor, const float offset) {
    T retVal = can_getSignal<T>(msg, startBit, length, isIntel);
    return (retVal * factor) + offset;
}

/**
 * Insert a scaled signal into CAN message
 * @param msg CAN message to insert into (modified)
 * @param val Value to insert
 * @param startBit Bit position to start at
 * @param length Number of bits to use
 * @param isIntel True for Intel byte order
 * @param factor Scale factor: raw = (value - offset) / factor
 * @param offset Scale offset: raw = (value - offset) / factor
 */
template<typename T>
void can_setSignal(can_Message_t& msg, const T& val, 
                   const uint8_t startBit, const uint8_t length, const bool isIntel, 
                   const float factor, const float offset) {
    T scaledVal = static_cast<T>((val - offset) / factor);
    can_setSignal<T>(msg, scaledVal, startBit, length, isIntel);
}