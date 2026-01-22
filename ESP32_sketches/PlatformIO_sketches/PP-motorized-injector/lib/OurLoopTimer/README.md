# OurLoopTimer - ESP32 Dual-Core Loop Timer

Interrupt-independent loop timing for ESP32 using dual-core measurement technique.

## Overview

Measures one CPU's loop time from the other CPU without interference. The measuring CPU does nothing except watch a toggle flag and calculate timing, providing accurate measurements without adding overhead to the code under test.

## Features

- **Interrupt-independent**: Uses `esp_timer` (or your GPTimer) for consistent timing
- **Zero interference**: Measuring CPU only observes, doesn't interfere with work CPU
- **5-second rolling average**: Shows trends over time, filters outliers
- **Maximum tracking**: Records worst-case loop time since boot
- **Preprocessor optimized**: Only compiles code for active measurement direction
- **Manual watchdog control**: Uses microsecond delays, not millisecond yields

## Configuration

In `OurLoopTimer.h`, set which CPU does what:

```cpp
#define CPU_WORKING 0        // CPU0 runs code under test
#define CPU_MEASURING 1      // CPU1 measures CPU0's loop time
```

Swap the values to measure the other CPU.

## Integration

1. Include the header:
```cpp
#include <OurLoopTimer.h>
```

2. Initialize in `setup()` after Serial.begin():
```cpp
initLoopTimer();
```

3. In your work loop, toggle at START or END (be consistent):
```cpp
void workLoop(void *parameter) {
  while (true) {
    toggleLoopFlag();  // At start of loop
    
    // Your production code here
    
    esp_task_wdt_reset();  // Feed watchdog manually
  }
}
```

4. Print stats periodically:
```cpp
printLoopStats();  // Output: CX:avg5s/max (microseconds)
```

5. Create work task pinned to CPU_WORKING:
```cpp
TaskHandle_t workTaskHandle = NULL;
xTaskCreatePinnedToCore(workLoop, "Working", 4096, NULL, 1, 
                        &workTaskHandle, CPU_WORKING);
esp_task_wdt_add(workTaskHandle);
```

## Output Format

```
C1:5/174
```
- `C1`: CPU1 is being measured
- `5`: Average loop time over last 5 seconds (microseconds)
- `174`: Maximum loop time since boot (microseconds)

## API Reference

### Initialization
- `void initLoopTimer()` - Initialize loop timer system, create measuring task

### Runtime
- `void toggleLoopFlag()` - Toggle loop flag (call once per loop)
- `void printLoopStats()` - Print avg5s/max, reset 5s window

### Accessors
- `uint32_t getLoopAvg5s()` - Get 5-second average (microseconds)
- `uint64_t getLoopMax()` - Get maximum since boot (microseconds)
- `uint32_t getLoopCount()` - Get total loop count

## Custom Timer Source

To use your custom GPTimer instead of esp_timer, modify `getTimerMicros()` in `OurLoopTimer.h`:

```cpp
inline uint64_t getTimerMicros() {
  return yourGPTimerFunction();  // Must return microseconds
}
```

## Delay Documentation

Document ALL delays in your code with `[DELAY]` comments:

```cpp
const unsigned long LED_INTERVAL = 1000;  // [DELAY] Toggle every 1000ms
delayMicroseconds(100);  // [DELAY] Work delay
```

This helps interpret loop timing results.

## Testing Production Code

1. Test with debug prints enabled to verify logic
2. Set `ENABLE_DEBUG_PRINTS false` in your code
3. Run final test to measure production loop time (no Serial.print overhead)
4. Record both debug and production times in your budget table

## Example

See `examples/basic_usage.cpp` for complete integration example.

## Notes

- Measuring CPU uses ~1µs delay to prevent watchdog timeout
- Work CPU must call `esp_task_wdt_reset()` periodically
- IDLE0 task removed from watchdog (we manage it manually)
- 5-second window resets after each print for rolling average
