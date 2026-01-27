# Motor Movement Timing Analysis

## Purpose
Document timing characteristics of motor movements for injection cycle optimization.

## Test Setup
- **Command**: `set_position 1.0` (1.0 turn movement)
- **Control Mode**: POSITION_CONTROL + PASSTHROUGH
- **Data Rate**: 100ms (10Hz) → **UPGRADING TO 10ms (100Hz)**
- **Capture Window**: 1s pre-roll + 1s post-command → **0.5s pre-roll + 0.5s post-command (1s total)**
- **Buffer Size**: 50 slots → **250 slots (for 10ms rates)**
- **Analysis**: Command timestamp vs response timing

## CAN Bus Load Analysis
- **Current (100ms rate)**: 2×100ms = 20 msg/s (1.1% load at 250kbps)
- **Proposed (10ms rate)**: 2×10ms + 4×100ms = 200 + 40 = **240 msg/s (13.3% load at 250kbps)**
- **Capacity**: ~1800 msg/s at 250kbps → **240 msg/s = 13.3% utilization** ✅

## Timing Measurements

### Production Analysis Results (10ms Rate, Setpoint Detection)

#### Test: set_position 1.0 (1.0 turn movement)
- **Command**: POSITION_CONTROL + PASSTHROUGH
- **Analysis Window**: 200ms chase from command timestamp
- **Detection Method**: Setpoint spike (5.0A threshold from baseline)
- **Data Rate**: 10ms (100Hz)

**Results Summary:**
| Test | Latency | Peak Setpoint | Peak Measured | Status |
|------|---------|---------------|---------------|---------|
| 1 | 25ms | 17.3A | 0.2A | SUCCESS |
| 2 | 31ms | 16.8A | 13.8A | SUCCESS |
| 3 | 29ms | 15.6A | 13.2A | SUCCESS |

**Key Findings:**
- **Consistent latency**: 25-31ms (avg ~28ms)
- **Setpoint response**: Fast, immediate spikes (15-17A)
- **Measured response**: Slower, dampened (10-13A with lag)
- **Detection reliability**: 100% success rate across tests
- **Chronological processing**: Fixed buffer order issues

**Production Validation:**
- ✅ **Automation-ready**: `[DATA] LATENCY=29,PEAK=13.2,SETPOINT=15.6,POINTS=20,STATUS=OK`
- ✅ **Consistent baseline**: 0.0-1.5A at T+0-9ms
- ✅ **Reliable spike detection**: First significant increase from baseline
- ✅ **No centering issues**: Chase approach eliminates timing problems

### Technical Implementation Details

#### Algorithm: Production Chase Approach
```cpp
// Chase window: command+0ms to command+200ms
uint64_t chaseStart = commandSendTime_;
uint64_t chaseEnd = commandSendTime_ + 200000;

// Collect data → Sort chronologically → Detect spike
// Baseline from first point, spike when > baseline + 5A
```

#### Key Improvements
1. **No centering required**: Chase from known command time
2. **Chronological processing**: Sort buffer data by timestamp
3. **Setpoint detection**: Faster response than measured current
4. **Production output**: Structured data for automation
5. **Consistent window**: Fixed 200ms analysis period

#### Future Production Options
- **Early exit**: Return at spike detection (~30ms total)
- **Confirmation window**: Continue 50ms after spike for validation
- **Adaptive thresholds**: Per movement type calibration
- **Extended testing**: Different move types (trap_traj, ramps)

### Test Plan for Tomorrow
1. **set_position 0.5** - Compare current draw vs 1.0 turn
2. **trap_traj moves** - Trajectory-based positioning
3. **Ramped velocity** - Velocity-controlled movements
4. **Ramped torque** - Torque-controlled movements
5. **Early exit optimization** - Per-move-type timing tables

### Test 1: Position Movement in Passthrough Mode (1.0 turn) - 100ms Rate
| Metric | Value | Notes |
|--------|-------|-------|
| **Command Sent** | T+1000011 us | Command sent at 1s into capture |
| **IQ Setpoint Spike** | T+1042ms (16.692A) | First significant current increase |
| **IQ Measured Response** | T+1042ms (11.943A) | Actual current response |
| **Encoder Movement Start** | T+1142ms (0.019→0.605) | Position begins to change |
| **Command → IQ Spike Latency** | ~41ms | Primary metric of interest |
| **IQ Spike → Encoder Latency** | ~100ms | Current to position response |
| **Total Command → Movement** | ~141ms | End-to-end latency |
| **Data Rate** | 100ms (10Hz) | Current resolution |
| **Timing Precision** | ±50ms | Limited by 100ms intervals |

### Test 2: Position Movement (1.0 turn) - 10ms Rate
| Metric | Value | Notes |
|--------|-------|-------|
| **Command Type** | set_position 1.0 | |
| **Control Mode** | POSITION_CONTROL + PASSTHROUGH | |
| **Data Rate** | 10ms (100Hz) | Upgraded for precision |
| **Timing Precision** | ±5ms | 10x better resolution |
| **Command → IQ Spike** | TBD | |
| **IQ Spike → Encoder** | TBD | |
| **Total Latency** | TBD | |

### Test 3: Position Movement (1.0 turn) - 10ms Rate
| Metric | Value | Notes |
|--------|-------|-------|
| **Command Type** | set_position 1.0 | |
| **Control Mode** | POSITION_CONTROL + PASSTHROUGH | |
| **Data Rate** | 10ms (100Hz) | |
| **Timing Precision** | ±5ms | |
| **Command → IQ Spike** | TBD | |
| **IQ Spike → Encoder** | TBD | |
| **Total Latency** | TBD | |

### Test 4: Position Movement (1.0 turn) - 10ms Rate
| Metric | Value | Notes |
|--------|-------|-------|
| **Command Type** | set_position 1.0 | |
| **Control Mode** | POSITION_CONTROL + PASSTHROUGH | |
| **Data Rate** | 10ms (100Hz) | |
| **Timing Precision** | ±5ms | |
| **Command → IQ Spike** | TBD | |
| **IQ Spike → Encoder** | TBD | |
| **Total Latency** | TBD | |

## Analysis Notes

### Key Observations

#### From Test 1 (100ms rate):
- **Large current spike**: 16.7A jump from baseline -0.056A
- **Fast IQ response**: ~41ms after command (within first 100ms window)
- **Slower position response**: ~100ms after IQ spike
- **Precision limitation**: Can't determine exact timing within 100ms window
- **42ms latency**: Could be anywhere from 1ms to 99ms in reality

#### Expected Improvements with 10ms rate:
- **10x timing precision**: ±5ms instead of ±50ms
- **Better spike detection**: Multiple data points during response
- **Accurate latency measurement**: Exact response timing
- **Adaptive threshold development**: Data-driven detection parameters
- **42ms precision**: Will know if it's 35ms, 42ms, or 49ms exactly

### Variability
- [Document any timing variations between runs]

### Factors Affecting Timing
- **Current Limits**: Higher limits may response faster
- **Velocity Limits**: May constrain acceleration
- **Mechanical Load**: Affects actual movement time
- **CAN Bus Load**: May affect command processing

## Future Tests Planned

### Injection Cycle Movements
1. **Rapid Fill** - Small, fast movements
2. **Pack Pressure** - High current, small movements  
3. **Hold Position** - Maintaining against pressure
4. **Retract** - Return movements
5. **Large Traverse** - Major position changes

### Different Distances
- **0.1 turn** - Small adjustments
- **0.5 turn** - Medium movements
- **1.0 turn** - Large movements (current test)
- **2.0 turn** - Maximum practical movements

### Different Control Modes
- **POSITION_CONTROL + PASSTHROUGH**
- **POSITION_CONTROL + TRAP_TRAJ**
- **VELOCITY_CONTROL**
- **TORQUE_CONTROL**

## Data Rate Impact Analysis

### Current: 100ms (10Hz)
- **Resolution**: 100ms between data points
- **Precision**: ±50ms timing uncertainty
- **Adequate for**: Gross timing analysis

### Proposed: 10ms (100Hz)
- **Resolution**: 10ms between data points  
- **Precision**: ±5ms timing uncertainty
- **Benefits**: 10x better timing resolution
- **Considerations**: 
  - Increased CAN bus load (2x current)
  - Larger buffer requirements
  - More processing overhead

## Recommendations
[To be developed based on timing analysis]

---

*Document created: 2026-01-26*
*Last updated: 2026-01-26*
