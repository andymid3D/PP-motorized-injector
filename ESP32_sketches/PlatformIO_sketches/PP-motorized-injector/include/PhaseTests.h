/**
 * @file PhaseTests.h
 * @brief Modular test harness for Phase 1 development validation
 * 
 * @details
 * Provides centralized test infrastructure for Phase 1 modules:
 * - Individual enable flags in config.h (TEST_*_ENABLED)
 * - Setup tests (run once at boot)
 * - Loop tests (continuous validation)
 * - Clean separation from main.cpp
 * 
 * Architecture:
 * - Each test checks its enable flag before running
 * - Disabled tests compile out (zero overhead)
 * - Easy to enable/disable for isolated testing
 * 
 * Usage in main.cpp:
 *   #if TEST_MODE_PHASE1
 *     PhaseTests::runSetupTests();  // In setup()
 *     PhaseTests::runLoopTests();   // In loop()
 *   #endif
 * 
 * @author PP-motorized-injector
 * @date January 16, 2026
 */

#ifndef PHASETESTS_H
#define PHASETESTS_H

#include <Arduino.h>
#include "config.h"

// Forward declaration for BufferedOutput (Phase 2)
class BufferedOutput;

class PhaseTests {
public:
    /**
     * @brief Run all enabled one-time setup tests
     * 
     * Called once in setup() after hardware initialization.
     * Checks enable flags and runs active tests.
     * 
     * Tests:
     * - TEST_RINGBUFFER_ENABLED: RingBuffer template validation
     */
    static void runSetupTests();

    /**
     * @brief Run all enabled continuous loop tests
     * 
     * Called every loop() iteration.
     * Checks enable flags and runs active tests.
     * 
     * @param serialOut BufferedOutput instance for non-blocking output
     * 
     * Tests:
     * - TEST_LOOPTIMER_ENABLED: Performance monitoring (5sec reports)
     * - TEST_STRESS_QUEUE_ENABLED: Dual-core stress testing
     * - TEST_BDS_STORAGE_ENABLED: BDS v2 storage validation
     * - TEST_BDS_INTEGRATION_ENABLED: Live ODrive integration
     * - TEST_BASELINE_TIMING_ENABLED: Pre-SafeString baseline
     */
    static void runLoopTests(); // Removed BufferedOutput& serialOut parameter

private:
    // ===== PHASE 1.7: RingBuffer Validation =====
    /**
     * @brief Validate RingBuffer template implementation
     * 
     * Tests: empty, push, latest, history, wraparound, forEach, clear
     * Expected: All 8 tests PASS
     */
    static void testRingBuffer();

    // ===== PHASE 1.6b: Queue Stress Testing =====
    /**
     * @brief Stress test CanRxHandler queue with simulated FSM loads
     * 
     * Simulates FSM overhead with delays (30µs → 700ms)
     * Measures queue depth, drain cycles, overflows
     * Validates 4,000× safety margin and 700ms breaking point
     */
    static void testStressQueue();

    /**
     * @brief Test BDS v2 ring buffer storage
     * 
     * Validates:
     * - storeXXX() methods populate ring buffers correctly
     * - getLatestXXX() retrieves newest timestamped data
     * - Timestamps preserved with µs precision
     * - Ring buffer wraparound (history > size)
     * - Staleness detection works
     * - TX correlation flag preserved
     * - V1 API backward compatible
     */
    static void testBDSStorage();
    
    /**
     * @brief Test BDS v2 integration with live ODrive messages
     * 
     * Validates:
     * - drainAndStore() processes real CAN messages
     * - BDS ring buffers populated from ODrive broadcasts
     * - Position/velocity data matches expectations
     * - Message rate ~180 msg/s (10ms encoder + 100ms others)
     * - Loop time remains low with BDS processing
     */
    static void testBDSIntegration();
    
    /**
     * @brief Baseline timing test (before SafeString integration)
     * 
     * Measures loop performance WITH Serial.print() blocking
     * Purpose: Capture "before" metrics for comparison
     * 
     * @param serialOut BufferedOutput instance for stats output
     * 
     * Metrics:
     * - Loop time with Serial calls (expect 1-5ms spikes)
     * - Message capture rate (~180 msg/s)
     * - Queue depth (expect 0-2 during Serial output)
     * 
     * Duration: 60 seconds
     * Expected: Identify Serial.print() as blocking bottleneck
     */
    static void testBaselineTiming(BufferedOutput& serialOut);
    
    // Future tests:
    // static void testTxCorrelation();  // Phase 1.8: Response vs broadcast detection
};

#endif // PHASETESTS_H
