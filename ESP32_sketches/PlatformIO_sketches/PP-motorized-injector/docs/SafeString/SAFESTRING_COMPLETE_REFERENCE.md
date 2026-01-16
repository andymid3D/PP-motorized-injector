# SafeString Library V4.1.42 - Complete Reference
**Author:** Matthew Ford (Forward Computing and Control Pty. Ltd.)  
**Website:** https://www.forward.com.au/pfod/ArduinoProgramming/SafeString/docs/html/index.html

---

## LIBRARY OVERVIEW

SafeString is a safe, robust, debuggable replacement for Arduino String processing.
- **Safe:** Never causes reboots, always in valid state even with null pointers or capacity exceeded
- **Robust:** Completely avoids memory fragmentation, never makes extra copies when passed as arguments
- **Debuggable:** Detailed error messages including SafeString name to help debug
- **Faster:** No multiple copies, no short-lived objects, no unnecessary data copying
- **Non-blocking:** Includes BufferedOutput, millisDelay, SafeStringReader for non-blocking I/O
- **Can wrap existing char[] and char*:** Safely manipulate existing data without copying

---

## CORE CLASSES

### 1. SafeString - Safe String Manipulation
**Purpose:** Replaces Arduino String with safe, non-fragmenting string processing

**Creation Macros:**
```cpp
createSafeString(name, capacity)             // cSF(name, capacity) - creates internal buffer
createSafeStringFromCharArray(name, charArray)  // cSFA(name, charArray) - wraps char[size]
createSafeStringFromCharPtr(name, charPtr)      // cSFP(name, charPtr) - wraps char*, capacity = strlen()
createSafeStringFromCharPtrWithSize(name, charPtr, size) // cSFPS - wraps char* with specified size
```

**String Operations (200+ methods):**
- **Concatenation:** `concat()`, `prefix()`, `+=`, `-=`
- **Substring:** `substring()`, `remove()`, `removeBefore()`, `removeFrom()`, `keepLast()`
- **Tokenizing:** `nextToken()`, `firstToken()`, `stoken()` with delimiter support
- **Search:** `indexOf()`, `lastIndexOf()`, `indexOfCharFrom()`, `startsWith()`, `endsWith()`
- **Modification:** `replace()`, `toLowerCase()`, `toUpperCase()`, `trim()`, `setCharAt()`
- **Comparison:** `==`, `!=`, `<`, `>`, `<=`, `>=`, `equals()`, `equalsIgnoreCase()`, `compareTo()`
- **Type Conversion:**
  - `toInt()`, `toLong()`, `toFloat()`, `toDouble()`, `toInt64_t()`
  - `hexToLong()`, `binToLong()`, `octToLong()`
- **Stream I/O (non-blocking):**
  - `read()`, `readUntil()`, `readUntilToken()`
  - `readFrom()`, `writeTo()`
- **UTF-8 Support:**
  - `utf8index(idx)` - find start of UTF-8 code point <= idx
  - `utf8nextIndex(idx)` - find start of next UTF-8 code point
- **Print Interface:** `print()`, `println()` with formatted output (width, precision)
- **Debugging:** `debug()`, `hasError()`, `errorDetected()`, `setOutput()`, `setVerbose()`

**Safety Features:**
- All methods check capacity, prevent buffer overflow
- Set error flags on failures
- Never crash program
- Detailed error messages with SafeString name

**Key Differences from Arduino String:**
- No `+` operator (use `+=` or `concat()`)
- No dynamic resizing (capacity fixed at creation)
- No const SafeString& arguments allowed
- Must pass SafeString& and update in method (no return SafeString)
- Stricter type conversion (returns bool, updates arg only if valid)

---

### 2. millisDelay - Non-Blocking Timers
**Purpose:** Replace delay() with non-blocking timers

**Methods:**
```cpp
start(unsigned long delay)   // Start timer with delay in ms
stop()                       // Stop timer (justFinished() never returns true until start/repeat/restart)
repeat()                     // Repeat same delay, accounts for call delay (no drift)
restart()                    // Start same delay from now (may drift)
finish()                     // Force immediate end
justFinished()               // Returns true ONCE when timer expires, then false
isRunning()                  // Check if timer active
remaining()                  // Returns ms left until finish, 0 if stopped/finished
getStartTime()               // Returns last start time in ms (0 if never started)
delay()                      // Returns the delay value set in start()
```

**Usage Pattern:**
```cpp
millisDelay ledDelay;

setup() {
  ledDelay.start(10000);  // start 10sec delay
}

loop() {
  if (ledDelay.justFinished()) {  // MUST call every loop
    // do something once
    ledDelay.repeat();  // or restart() to repeat
  }
}
```

**Important:**
- `justFinished()` MUST be called every loop() for timer to timeout
- `repeat()` adjusts for drift, `restart()` does not
- Returns true ONCE after timeout, then false

---

### 3. BufferedOutput - Non-Blocking Serial Output
**Purpose:** Non-blocking replacement for Serial.print() to prevent loop delays

**Creation:**
```cpp
createBufferedOutput(name, size, mode)
createBufferedOutput(name, size, mode, allOrNothing)
```

**Modes:**
- `BLOCK_IF_FULL` - Blocks when full (not recommended, defeats purpose)
- `DROP_UNTIL_EMPTY` - Drop output until buffer completely empties, inserts ~~ marker (recommended)
- `DROP_IF_FULL` - Drop output until space available, inserts ~~ marker

**Critical Setup:**
```cpp
BufferedOutput output;
setup() {
  Serial.begin(115200);  // Use high baud rate
  output.connect(Serial);  // REQUIRED - connect to Serial
}

loop() {
  output.nextByteOut();  // REQUIRED - call at least once per loop to release buffered bytes
  output.println("data");
}
```

**Key Methods:**
```cpp
connect(HardwareSerial& _serial)  // REQUIRED - connect to Serial
connect(Stream& _stream, baudRate)  // For boards without tx buffer (e.g., NanoBLE)
nextByteOut()                     // REQUIRED - call every loop() to release bytes
availableForWrite()               // Space in buffer + Serial TX space
clear()                           // Empty buffer (Serial TX unchanged)
flush()                           // Block until buffer empties (use sparingly!)
clearSpace(size_t len)            // Make space by removing old data (stops at protect mark)
protect()                         // Add marker to prevent clearSpace() from removing earlier data
terminateLastLine()               // Add \r\n if last char isn't \n
```

**allOrNothing Flag:**
- `true` (default): Drop entire print() if won't fit
- `false`: Allow partial output

**Best Practices:**
1. Use highest possible Serial baud rate (115200)
2. Use DROP_UNTIL_EMPTY mode
3. Make buffer large enough for important messages
4. Call clearSpace() before important messages
5. Call protect() after critical messages
6. Avoid flush() (blocks loop)

---

### 4. loopTimer (loopTimerClass) - Performance Monitoring
**Purpose:** Track min/max/avg loop() execution times to detect blocking code

**Included in SafeString library V3+** (part of SafeString, NOT SR Library)

**Global Instance:**
```cpp
#include <loopTimer.h>
// Global instance 'loopTimer' automatically created
```

**Usage:**
```cpp
#include <loopTimer.h>

setup() {
  Serial.begin(9600);
  // ... other setup
}

loop() {
  loopTimer.check(Serial);  // Prints stats every 5sec to Serial
  // OR
  loopTimer.check(bufferedOut);  // Better - prints to BufferedOutput
  // OR
  loopTimer.check();  // Silent mode - no output, use print() manually later
  
  // your loop code
}
```

**Manual Printing:**
```cpp
loopTimer.check();  // Update stats silently
// ... later
loopTimer.print(Serial);  // Print current stats
```

**Output Format:**
```
loop us Latency
 5sec max:1408 avg:254
 sofar max:1408 avg:254 max - prt:1872
```
**Output Fields:**
- `5sec max`: Maximum loop time in last 5 seconds (microseconds)
- `5sec avg`: Average loop time in last 5 seconds (microseconds)
- `sofar max`: Maximum loop time since program start (microseconds)
- `sofar avg`: Average loop time since program start (microseconds)
- `max - prt`: Time taken to print stats (microseconds) - **excluded from loop time**

**Named Timers:**
```cpp
loopTimerClass task1Timer("task1");  // Create named timer
task1Timer.check(bufferedOut);       // Prints "task1 us Latency"
```

**Methods:**
```cpp
check()               // Update stats silently, no output
check(Print& out)     // Update stats and print every 5sec to 'out'
print(Print& out)     // Print current stats to 'out'
```

**Key Features:**
- Automatically excludes print time from measurements (prt: value)
- Tracks both 5-second window and overall statistics
- Global instance available without declaration
- Can create multiple named instances for different sections
- Prints automatically every 5 seconds when output stream provided

**Performance Impact:**
- Adds ~1-2ms overhead every 5 seconds when printing
- Negligible overhead when printing to BufferedOutput
- **IMPORTANT:** Remove loopTimer.check() after testing/debugging

**Benchmarking Example:**
```cpp
// Simple Multi-tasking vs FreeRTOS comparison (from docs):
// - FreeRTOS: reads analog every 17ms, uses 9996 bytes Flash, 456 bytes RAM
// - frt RTOS: reads analog every 17ms, uses 10988 bytes Flash, 453 bytes RAM  
// - Simple Multi-tasking: reads analog every 0.1ms, uses 3822 bytes Flash, 307 bytes RAM
// (loopTimer was used to measure these results)
```

**Troubleshooting with loopTimer:**
1. Loop time > 1ms: Check for blocking code (delay(), Serial.print(), etc.)
2. Sudden spikes: Identify which task caused delay
3. Gradual increase: Memory leak or buffer overflow
4. High avg: Too much processing per loop
5. High max, low avg: Occasional blocking (e.g., print statements)

---

### 5. SafeStringReader - Non-Blocking Text Input
**Purpose:** Non-blocking high-level reader for delimited text input

**Creation:**
```cpp
createSafeStringReader(name, maxCmdLength, delimiters)
```

**Setup:**
```cpp
SafeStringReader sfReader;
setup() {
  sfReader.connect(Serial);  // where to read from
  sfReader.echoOn();         // echo input (optional)
  sfReader.setTimeout(2000); // 2sec non-blocking timeout (optional)
}
```

**Usage:**
```cpp
loop() {
  if (sfReader.read()) {  // non-blocking, returns true when delimiter found or timeout
    // process sfReader as SafeString
    if (sfReader == "command") {
      // handle command
    }
  }
}
```

**Key Methods:**
```cpp
connect(Stream& stream)       // Connect to input stream
echoOn()                      // Echo input back
setTimeout(unsigned long ms)  // Non-blocking timeout (returns token after ms of no input)
skipToDelimiter()             // Discard input up to next delimiter
getDelimiter()                // Returns delimiter that terminated token
```

**Features:**
- Handles unlimited length input without overflow
- Only maxCmdLength buffer needed
- Automatically handles empty fields
- Never blocks loop()

---

### 6. SafeStringStream - Automated Testing
**Purpose:** Stream for test inputs, releases data at specified baud rate

**Creation:**
```cpp
cSF(testData, 200);           // Create test data buffer
cSF(rxBuf, 64);               // Create rx buffer (64 for Uno, 128 for ESP32)
SafeStringStream sfStream(testData, rxBuf);
```

**Setup:**
```cpp
setup() {
  testData = F("test data here\nmore data\n");
  sfStream.begin(testData, 9600);  // Release at 9600 baud
}
```

**Usage:**
```cpp
sfReader.connect(sfStream);  // Connect reader to stream instead of Serial
sfReader.echoOn();           // Echo back to stream for continuous testing
```

**Statistics:**
```cpp
int overflowCount = sfStream.RxBufferOverflow();  // Returns count and clears it
```

---

### 7. BufferedInput - Extra Input Buffering
**Purpose:** Add extra input buffering when processing delays cause missed data

**Creation:**
```cpp
createBufferedInput(name, size)
```

**Setup:**
```cpp
BufferedInput bufferedIn;
setup() {
  bufferedIn.connect(Serial);  // Add buffering to Serial
  sfReader.connect(bufferedIn); // Reader reads from buffered input
}

loop() {
  bufferedIn.nextByteIn();  // REQUIRED - call every loop to read data into buffer
  // ... rest of loop
}
```

**Statistics:**
```cpp
int used = bufferedIn.maxBufferUsed();    // Max chars stored, clears after read
int avail = bufferedIn.maxStreamAvailable(); // Max chars available from stream, clears after read
```

**Sizing Guide:**
- If maxBufferUsed() == size: increase buffer
- If maxStreamAvailable() == HW buffer size AND space in BufferedInput: call nextByteIn() more often

---

### 8. PinFlasher - Non-Blocking Pin Flashing
**Purpose:** Simple class to flash/toggle output pin at set rate

**Creation:**
```cpp
#include <PinFlasher.h>
PinFlasher flasher(13);        // Pin 13, on = HIGH (default)
PinFlasher f(4, true);         // Pin 4, inverted logic (on = LOW)
```

**Usage:**
```cpp
loop() {
  flasher.update();  // REQUIRED - call every loop
  
  if (error) {
    flasher.setOnOff(100);    // Fast flash (100ms on, 100ms off)
  } else if (warning) {
    flasher.setOnOff(1000);   // Slow flash (1sec on, 1sec off)
  } else {
    flasher.setOnOff(PIN_OFF); // Turn off
  }
}
```

**Methods:**
```cpp
setOnOff(unsigned long ms)  // Set on/off time (ms), or PIN_ON, PIN_OFF
update()                    // Check if should toggle (call every loop)
setPin(int pin)             // Change pin
invertOutput(bool invert)   // Invert on/off logic
```

**Features:**
- Repeated calls with same value ignored (no restart)
- PIN_ON, PIN_OFF for readability
- Separate on/off times supported: `setOnOff(onTime, offTime)`

---

### 9. SerialComs - Arduino to Arduino/PC Messages
**Purpose:** Structured messaging between Arduinos or PC
**Note:** Tutorial redirected to: https://www.forward.com.au/pfod/ArduinoProgramming/SerialComs/index.html

---

## UTF-8 SUPPORT

SafeString handles UTF-8 encoded text (1-4 bytes per code point):

```cpp
cSF(sfStr, 15, "-℃+℉*%.");  // Assign UTF-8 chars

// indexOf works with UTF-8
int idx = sfStr.indexOf("℉");  // Returns 5

// Extract UTF-8 substring preserving code points
sfStr.substring(sfSubstr, 0, sfStr.utf8nextIndex(3));  // "-℃"

// Extract single code point at index
sfStr.substring(sfSubstr, sfStr.utf8index(6), sfStr.utf8nextIndex(6));  // "℉"

// Iterate through UTF-8 code points
size_t idx = 0;
while (idx < sf.length()) {
  size_t nextIdx = sf.utf8nextIndex(idx);
  sf.substring(sfChar, idx, nextIdx);  // Extract one UTF-8 char
  idx = nextIdx;
}
```

---

## ERROR HANDLING

**Global Error Flag:**
```cpp
SafeString::errorDetected()  // Returns true if any error since last call, clears flag
```

**Per-SafeString Error Flag:**
```cpp
sfStr.hasError()  // Returns true if error on this SafeString since last call
```

**Error Messages:**
```cpp
setup() {
  SafeString::setOutput(Serial);  // Enable error messages to Serial
}
```

**Error Message Control:**
```cpp
SafeString::setVerbose(false);  // Compact error messages (omit contents)
SafeString::turnOutputOff();    // Disable all error messages
```

**Debug Output:**
```cpp
sfStr.debug();                       // Print SafeString state + contents
sfStr.debug(F("After operation"));   // Print with title
sfStr.debug(false);                  // Print state only (no contents)
```

**Custom Debug Messages:**
```cpp
SafeString::Output.println("debug msg");  // Only prints if setOutput() called
```

**Remove All Error Code:**
```cpp
// Comment out in SafeString.h to save program space:
//#define SSTRING_DEBUG
// Error checking remains, messages removed
// debug() still works but without variable name
```

---

## PASSING SAFESTRINGS TO METHODS

**Correct:**
```cpp
void test(SafeString& str) {  // Reference, no const
  str = "modified";
}
```

**Incorrect (compile error):**
```cpp
void test(SafeString str) { }       // Missing &
void test(const SafeString& str) { } // const not allowed (SafeString cleans wrapped arrays)
SafeString test() { return str; }    // Cannot return SafeString
```

**Return via Reference:**
```cpp
void buildString(int num, SafeString& result) {
  result = "value: ";
  result += num;
}

cSF(str, 20);
buildString(42, str);  // str updated
```

---

## WRAPPING EXISTING CHAR ARRAYS

**createSafeStringFromCharArray (cSFA):**
```cpp
char testData[25] = "initial";
cSFA(sfData, testData);  // Wraps char[25], capacity = 24

sfData.toUpperCase();
// testData now contains "INITIAL"
```

**createSafeStringFromCharPtr (cSFP):**
```cpp
char* dataPtr = testData;
cSFP(sfData, dataPtr);  // Capacity = strlen(dataPtr)
// Can process but not extend
```

**createSafeStringFromCharPtrWithSize (cSFPS):**
```cpp
char testData[25] = "";
char* dataPtr = testData;
cSFPS(sfData, dataPtr, 25);  // Capacity = 24
// Can process and extend up to capacity
```

**Important:**
- Changes via SafeString update underlying char[]
- Use min 5 byte char[] to avoid sizeof(char*) detection error
- Capacity calculated as size - 1 (for terminating '\0')

**Memory vs Processing Trade-off:**
```cpp
// Wrapped - saves memory, slower (strlen() each method call)
cSFP(data, OBDdata);

// Copied - uses stack memory, faster (no strlen() calls)
cSF(data, strlen(OBDdata));
data = OBDdata;
```

---

## EXAMPLE WORKFLOWS

### 1. Non-Blocking Blink (millisDelay)
```cpp
#include <millisDelay.h>
int led = 13;
bool ledOn = false;
millisDelay ledDelay;

setup() {
  pinMode(led, OUTPUT);
  ledDelay.start(1000);
}

loop() {
  if (ledDelay.justFinished()) {
    ledDelay.repeat();
    ledOn = !ledOn;
    digitalWrite(led, ledOn ? HIGH : LOW);
  }
}
```

### 2. Non-Blocking Serial I/O
```cpp
#include <BufferedOutput.h>
#include <SafeStringReader.h>

createBufferedOutput(output, 80, DROP_UNTIL_EMPTY);
createSafeStringReader(sfReader, 15, " ,\r\n");

setup() {
  Serial.begin(115200);
  output.connect(Serial);
  sfReader.connect(Serial);
  sfReader.echoOn();
}

loop() {
  output.nextByteOut();  // Release buffered chars
  
  if (sfReader.read()) {
    if (sfReader == "start") {
      output.println("Started");
    }
  }
}
```

### 3. String Processing
```cpp
cSF(input, 50, "23.5, 44a ,, , -5. , 7a");
cSF(token, 15);
float nums[10];
int count = 0;

while (input.length() > 0) {
  if (input.nextToken(token, ", ")) {
    float f;
    if (token.toFloat(f)) {
      nums[count++] = f;
    }
  }
}
```

### 4. Simple Multi-Tasking
```cpp
#include <loopTimer.h>
#include <millisDelay.h>
#include <BufferedOutput.h>

createBufferedOutput(output, 80, DROP_UNTIL_EMPTY);
millisDelay task1Delay, task2Delay;

setup() {
  Serial.begin(115200);
  output.connect(Serial);
  task1Delay.start(1000);
  task2Delay.start(5000);
}

void task1() {
  if (task1Delay.justFinished()) {
    task1Delay.repeat();
    output.println("Task 1");
  }
}

void task2() {
  if (task2Delay.justFinished()) {
    task2Delay.repeat();
    output.println("Task 2");
  }
}

loop() {
  output.nextByteOut();
  loopTimer.check(output);
  task1();
  task2();
}
```

---

## INSTALLATION

**Arduino Library Manager:**
1. Open Arduino IDE
2. Tools → Manage Libraries
3. Search "SafeString"
4. Click Install

**Manual Installation:**
1. Download https://www.forward.com.au/pfod/ArduinoProgramming/SafeString/SafeString.zip
2. Sketch → Include Library → Add .ZIP Library

**PlatformIO:**
```ini
[env]
lib_deps = SafeString
```

---

## KEY PRINCIPLES

1. **Never use delay()** - Use millisDelay instead
2. **Never use Serial.print() in loop()** - Use BufferedOutput instead
3. **Never use Serial.readString() etc** - Use SafeStringReader instead
4. **Always call nextByteOut() in loop()** - Required for BufferedOutput
5. **Always call justFinished() in loop()** - Required for millisDelay
6. **Pass SafeString& not SafeString** - Reference required
7. **No const SafeString&** - SafeString cleans wrapped arrays
8. **Use unsigned long for millis()** - Handles 50-day overflow
9. **Enable error messages during development** - SafeString::setOutput(Serial)
10. **Remove loopTimer after testing** - Adds 1-2ms overhead

---

## COMMON ERRORS & SOLUTIONS

**Error:** "SafeString(const SafeString& other) is private"
**Fix:** Change `void test(SafeString str)` to `void test(SafeString& str)`

**Error:** "passing 'const SafeString' as 'this' argument discards qualifiers"
**Fix:** Remove const: `void test(const SafeString& str)` → `void test(SafeString& str)`

**Error:** "reference to local variable returned"
**Fix:** Don't return SafeString. Pass SafeString& as argument and update it.

**Error:** Buffer overflow after 65 seconds
**Fix:** Use `unsigned long` not `unsigned int` for millis() variables

**Error:** Loop running slow
**Fix:** Check loopTimer output, remove Serial.print(), use BufferedOutput

**Error:** Missing Serial input
**Fix:** Use higher baud rate, add BufferedInput, check BufferedInput stats

---

## PERFORMANCE TIPS

1. Use highest baud rate possible (115200)
2. Call important tasks multiple times per loop
3. Keep loop() < 1ms for responsive control
4. Use BufferedOutput to prevent blocking
5. Use millisDelay.repeat() not restart() to avoid drift
6. Monitor loop time with loopTimer
7. Add extra calls to critical tasks in slow sections
8. Use SafeString wrapping (cSFP) to avoid copying
9. Copy to local SafeString for intensive processing (cSF)
10. Disable error messages in production (//#define SSTRING_DEBUG)

---

## COMPLETE REFERENCE LINKS

**Main Documentation:**
https://www.forward.com.au/pfod/ArduinoProgramming/SafeString/docs/html/index.html

**Tutorials:**
1. SafeString Tutorial: https://www.forward.com.au/pfod/ArduinoProgramming/SafeString/index.html
2. Serial I/O for Real World: https://www.forward.com.au/pfod/ArduinoProgramming/Serial_IO/index.html
3. Simple Multi-tasking Arduino: https://www.forward.com.au/pfod/ArduinoProgramming/RealTimeArduino/index.html
4. How to Code Timers and Delays: https://www.forward.com.au/pfod/ArduinoProgramming/TimingDelaysInArduino.html

**Example Sketches:**
Under File → Examples → SafeString in Arduino IDE

---

## VERSION HISTORY

- V4.1.42 (Sept 2025): Added utf8index(), utf8nextIndex(), prevent recursive nextByteOut()
- V4.1.41 (June 2025): Fixed millisDelay repeat() after stop()/finish()
- V4.1.40 (April 2025): Added PinFlasher on/off time support, int64_t for += and -=
- V4.1.37 (March 2025): Added toInt64_t() for time_t parsing
- V3.0.0: Added loopTimer, millisDelay, BufferedInput
- V2.0.5: Added non-blocking Serial I/O, SafeStringStreams
- V2.0.0: Added wrapping existing char[] and char*

---

**© Copyright Forward Computing and Control Pty. Ltd.**
**All documentation sourced from official SafeString library website**
