// ZcCoupledKinematics.cpp
#include "Movement/Kinematics/ZcCoupledKinematics.h"
#include "Platform/Platform.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"

#ifndef AXIS_INDEX_Z
# define AXIS_INDEX_Z 2
#endif
#ifndef AXIS_INDEX_C
# define AXIS_INDEX_C 8
#endif
static_assert(MaxAxes > AXIS_INDEX_C, "MaxAxes must exceed C axis index");

bool ZcCoupledKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException)
{
    if (mCode != 669) { return false; }

    bool seenAny = false;
    if (gb.Seen('L')) { leadMmPerRev     = gb.GetFValue(); seenAny = true; }
    if (gb.Seen('H')) { handedness       = (gb.GetFValue() >= 0.0f) ? +1.0f : -1.0f; seenAny = true; }
    if (gb.Seen('W')) { wobbleEnable     = (gb.GetIValue() != 0); seenAny = true; }
    if (gb.Seen('A')) { wobbleAmpMm      = gb.GetFValue(); seenAny = true; }
    if (gb.Seen('N')) { wobbleCycles     = gb.GetFValue(); seenAny = true; }
    if (gb.Seen('P')) { wobblePhaseDeg   = gb.GetFValue(); seenAny = true; }
    if (gb.Seen('G')) { guardThresholdMm = gb.GetFValue(); seenAny = true; }
    if (gb.Seen('E')) { guardAsError     = (gb.GetIValue() != 0); seenAny = true; }

    if (leadMmPerRev < 0.0f) { leadMmPerRev = 0.0f; }

    reply.catf("Kinematics=ZC L=%.6f H=%.0f W=%d A=%.6f N=%.6f P=%.3f",
               (double)leadMmPerRev, (double)handedness,
               wobbleEnable ? 1 : 0,
               (double)wobbleAmpMm, (double)wobbleCycles, (double)wobblePhaseDeg);
    if (guardThresholdMm > 0.0f) {
        reply.catf(" G=%.6f E=%d", (double)guardThresholdMm, (int)guardAsError);
    }
    reply.cat('\n');

    error = false;
    return seenAny;
}

MovementError ZcCoupledKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[],
                                                         size_t numVisibleAxes, size_t numTotalAxes,
                                                         int32_t motorPos[], bool isCoordinated) const noexcept
{
    if (baseK == nullptr) {
        for (size_t i = 0; i < numTotalAxes; ++i) {
            motorPos[i] = lrintf(machinePos[i] * stepsPerMm[i]);
        }
        return MovementError::ok;
    }

    float tmp[MaxAxes];
    for (size_t i = 0; i < numTotalAxes; ++i) tmp[i] = machinePos[i];

    const float cDeg = tmp[AXIS_INDEX_C];
    tmp[AXIS_INDEX_Z] = ZPrime(tmp[AXIS_INDEX_Z], cDeg);

    return baseK->CartesianToMotorSteps(tmp, stepsPerMm, numVisibleAxes, numTotalAxes, motorPos, isCoordinated);
}

void ZcCoupledKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[],
                                                size_t numVisibleAxes, size_t numTotalAxes,
                                                float machinePos[]) const noexcept
{
    if (baseK == nullptr) {
        for (size_t i = 0; i < numTotalAxes; ++i) {
            machinePos[i] = (float)motorPos[i] / stepsPerMm[i];
        }
        return;
    }

    float tmp[MaxAxes];
    baseK->MotorStepsToCartesian(motorPos, stepsPerMm, numVisibleAxes, numTotalAxes, tmp);

    const float cDeg   = tmp[AXIS_INDEX_C];
    const float zPrime = tmp[AXIS_INDEX_Z];
    tmp[AXIS_INDEX_Z]  = ZFromZPrime(zPrime, cDeg);

    for (size_t i = 0; i < numTotalAxes; ++i) machinePos[i] = tmp[i];
}
