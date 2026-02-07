#ifndef MOULD_H
#define MOULD_H

struct actualMouldParams {
    char mouldName[20];

    float fillVolume;
    float fillSpeed;
    float fillPressure;

    float packVolume;
    float packSpeed;
    float packPressure;
    float packTime;

    float coolingTime;

    float fillTrapAccel;
    float fillTrapDecel;
    float packTrapAccel;
    float packTrapDecel;
};

using actualMouldParams_t = actualMouldParams;

#endif // MOULD_H
