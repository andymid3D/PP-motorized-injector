# Trap Trajectory Timing Analysis

## Test Configuration
- **Move**: 10.0 turns trap_traj
- **Velocity Limit**: 15.0 rps (REFILL setting)
- **Analysis Window**: 500ms
- **Detection**: IQ setpoint spike > 2.0A

## Results Summary

### Acceleration vs Latency & Peak Current

| Acceleration | Latency | Peak Current | Peak Setpoint | Notes |
|--------------|---------|--------------|---------------|-------|
| 20.0 turns/s² | 126-246ms | 11.0-11.7A | 12.6-12.8A | Variable response |
| 50.0 turns/s² | 102ms | 15.6A | 17.2A | Faster, higher peak |
| 500.0 turns/s² | 78ms | 16.6A | 17.2A | Fastest, similar peak |

### Key Observations

#### 1. **Latency Reduction with Acceleration**
- **20→50 accel**: 126-246ms → 102ms (~20% improvement)
- **50→500 accel**: 102ms → 78ms (~24% improvement)
- **Diminishing returns**: Higher acceleration gives less benefit

#### 2. **Current Spike Characteristics**
- **Low accel (20)**: Gradual ramp, lower peak (11-12A)
- **High accel (500)**: Sharp spike, higher peak (16-17A)
- **10A threshold timing**:
  - 50 accel: 10A at 152ms
  - 500 accel: 10A at 88ms

#### 3. **Consistent 70ms Startup Delay**
**Critical Finding**: All tests show ~70ms delay before any current increase
```
T+0ms: Command sent
T+70ms: First current increase detected
```

**Possible Causes**:
- ODrive trajectory planner calculation time
- Internal setpoint generation algorithm
- CAN message processing pipeline
- Control loop initialization

## Production Implications

### Detection Threshold
- **Current 2.0A**: Too low for production
- **Motor cogging friction**: 3.4-6.0A required to move
- **Holding current**: Can exceed 2.0A in CLC
- **Recommendation**: 5.0A threshold for production

### Acceleration Strategy
- **500 accel**: Best performance (78ms latency)
- **Trade-off**: Higher current peaks (16-17A vs 11-12A)
- **Production choice**: Balance between speed and current stress

## Next Steps

### 1. ODrive Configuration Investigation
Look for settings that reduce 70ms startup delay:
- Trajectory planner parameters
- Control loop timing
- CAN processing settings
- Buffer configurations

### 2. Velocity Ramp Tests
Implement vel_ramp tests with:
- 2-second duration
- Stop command analysis
- Baseline detection for stop

### 3. Stop Detection Logic
For production stop detection:
- Detect current reduction to < 2.0A
- Return to baseline levels
- Confirm motor deceleration

## Error Handling Note

Observed random ODrive errors:
```
axis: UNKNOWN ERROR: 0x00000100
encoder: ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH
```

These should be caught by SafetyManager and cleared with clear_errors() command.
