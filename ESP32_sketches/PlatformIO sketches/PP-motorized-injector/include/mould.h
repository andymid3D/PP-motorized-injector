#pragma once

typedef struct actualMouldParams {
  char mouldName[20];      
  
  // --- PHASE 1: FILL (Injection) ---
  float fillVolume;        // (cm3 or grams) Primary size of the mould.
  float fillSpeed;         // (Turns/Sec) Injection speed.
  float fillPressure;      // (Amps) The Safety Limit. Low for 2D, High for 3D.
  
  // --- PHASE 2: PACK (Hold / Last-Fill) ---
  float packVolume;        // (cm3 or grams) Small extra amount to compensate shrinkage.
  float packSpeed;         // (Turns/Sec) Slow speed.
  float packPressure;      // (Amps) Maintenance pressure.
  float packTime;          // (Seconds) Duration of packing.
  
  // --- PHASE 3: COOLING ---
  float coolingTime;       // (Seconds) Time to wait before release.
  
  // --- TRAP_TRAJ PARAMETERS (Fine-tuning for injection cycle) ---
  float fillTrapAccel;     // (Turns/Sec²) Acceleration during fill phase (default: TRAP_ACCEL_NORMAL)
  float fillTrapDecel;     // (Turns/Sec²) Deceleration during fill phase (default: TRAP_DECEL_NORMAL)
  float packTrapAccel;     // (Turns/Sec²) Acceleration during pack phase (default: TRAP_ACCEL_SLOW)
  float packTrapDecel;     // (Turns/Sec²) Deceleration during pack phase (default: TRAP_DECEL_SLOW)
  
} actualMouldParams_t;