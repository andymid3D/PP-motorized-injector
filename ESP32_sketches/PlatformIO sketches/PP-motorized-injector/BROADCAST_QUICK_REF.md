# Active Broadcast Messages - Quick Reference

**ODrive Configuration:** All 7 broadcast messages are enabled via CAN config.

## Message Schedule

```
Every 10ms:
  ├─ CYCLIC_ENCODER_ESTIMATES (0x09): position, velocity
  ├─ CYCLIC_MOTOR_ERROR (0x03): motor error flags
  ├─ CYCLIC_ENCODER_ERROR (0x04): encoder error flags
  └─ CYCLIC_CONTROLLER_ERROR (0x1D): controller error flags

Every 100ms:
  ├─ CYCLIC_HEARTBEAT (0x01): axis state, axis error, state flags
  ├─ CYCLIC_IQ (0x14): Iq setpoint and measured current
  └─ CYCLIC_BUS_VI (0x17): bus voltage and current
```

## Why Each Message?

| Message | Purpose | Used By | Interval |
|---------|---------|---------|----------|
| **CYCLIC_HEARTBEAT** | Axis state & overall error status | Homing FSM, all states | 100ms |
| **CYCLIC_ENCODER_ESTIMATES** | Motor position & velocity feedback | Position control, compression force | 10ms |
| **CYCLIC_MOTOR_ERROR** | Motor-specific faults | Error recovery, mechanical diagnostics | 10ms |
| **CYCLIC_ENCODER_ERROR** | Encoder health & calibration issues | Index search, homing reliability | 10ms |
| **CYCLIC_CONTROLLER_ERROR** | Firmware-level issues | System health monitoring | 10ms |
| **CYCLIC_IQ** | Motor current for load monitoring | Force feedback, parameter tuning | 100ms |
| **CYCLIC_BUS_VI** | Supply voltage/current health | Power diagnostics, failure prediction | 100ms |

## CAN Bandwidth Usage

- 10ms messages: 4 @ 8 bytes = **3.2 kbps of 250 kbps** available
- 100ms messages: 3 @ 8 bytes = **240 bps overhead**
- **Total: <2% of CAN bandwidth** - plenty of headroom

## Storage Location

All messages are cached in two places:

1. **CanBusHandlerV2** (backward compatibility)
   - Via getter methods: `getPosition()`, `getVelocity()`, `getMotorErrorDetails()`, etc.
   
2. **BroadcastDataStore** (recommended for new code)
   - Via methods: `getMotorError()`, `getEncoderError()`, `getControllerError()`, etc.
   - Includes timestamps for staleness detection

## Serial Logging (1Hz)

All error codes logged at 1Hz via SerialMessaging:
```
MotorErr:0xXXXXXXXX EncoderErr:0xXXXXXXXX CtrlErr:0xXXXXXXXX
```

## Integration Status

✅ **All broadcast messages integrated into BroadcastDataStore**

- Parsing: All cyclic message parsers available in ODriveCANProtocol
- Dispatch: CanBusHandlerV2::loop() handles all message types
- Storage: BroadcastDataStore caches all values with timestamps
- Logging: SafeString + SerialMessaging for 1Hz output

## No More Missing Data!

Before integration, only 4/7 messages were being captured:
- ❌ Motor error (ignored) → Now ✅ tracked
- ❌ Encoder error (ignored) → Now ✅ tracked
- ❌ Controller error (ignored) → Now ✅ tracked
- ❌ Sensorless error (ignored) → Now ✅ tracked

---

See [ERROR_CODES.md](./ERROR_CODES.md) for detailed error code meanings.
See [BROADCAST_INTEGRATION.md](./BROADCAST_INTEGRATION.md) for integration details.
