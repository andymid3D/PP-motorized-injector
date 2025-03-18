#pragma once
/// @brief struct to hold ActualMouldParams

typedef struct actualMouldParams
{
  const char *mouldName;          // unique string name for stuct of mould parameters for below variables
  int fillMouldMoveSpeed;         /* speed of next move, sent from ESP32, test, is possible different for 2D and 3D moulds,
                                     higher speeds will lead to earlier over-current errors */
  int fillMouldAccel;             // possibly not needed, have to discover whether small 2D/2.5D moulds benefit from slower accel on fill..?
  int fillMouldMoveDistSteps;     /* distance to move plunger to fill mould, sent from ESP32 at the start of each injection, as
                                     per mould in use */
  int holdMouldMoveSpeed;         /* ... and how fast (will be a low speed, test.. this will have to translate into a timed move,
                                     for example has to last 15s to allow 25g of plastic to cool enough, then distSteps * speed has to take 15s.
                                     make formula that can take time requiered, distance to move, and divide to get speed OR use ContMove for Time..? */      
  int holdMouldMoveDistSteps;     // after mould fill, apply a little more move/pressure to get good surface finish
  int holdMouldAcccel;        // as this will be a very slow speed and small distance motor move, either use fillMouldAccel, or leave as default..?
} actualMouldParams_t;

// FIXME commented out for testing
// struct actualMouldParams mouldNow = { "Posavasos", maxSpeedLimit, motorAcceleration, 2160, (maxSpeedLimit / 10), 80 };
/* struct to hold received message with above variable parameters... can I include in {variable name1, variable name2, etc}..?
={fillMouldMoveSpeed, fillMouldMoveDistSteps, holdMouldMoveSpeed, holdMouldMoveDistSteps}, better practice would be struct including name
of mould and list of valies ofr the variables above, which would then be written to these variables ... include check of
message (5 comma separated values with special starting char?) length */
