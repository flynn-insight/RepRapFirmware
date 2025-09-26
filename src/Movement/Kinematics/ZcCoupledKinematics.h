// ZcCoupledKinematics.h
// Robust Z↔C coupling kinematics with optional sinusoidal wobble & guardrails.
// RRF 3.6-compatible. Designed as a thin decorator over the active base kinematics.
// Copyright (c) 2025
#pragma once

#include "RepRapFirmware.h"
#include "Movement/Kinematics/Kinematics.h"   // <-- brings in Kinematics, MovementError, HomingMode
#include "General/StringRef.h"

class Platform;
class GCodeBuffer;

class ZcCoupledKinematics final : public Kinematics
{
public:
    ZcCoupledKinematics(Platform& p, Kinematics* baseOwned) noexcept
    : Kinematics(KinematicsType::ZC, SegmentationType(false, false, false)),  // ctor signature in 3.6
      platform(p), baseK(baseOwned) {}

    ~ZcCoupledKinematics() override { delete baseK; }

    // RRF 3.6 pure-virtuals we must implement:
    const char* GetName(bool forStatusReport = false) const noexcept override { return "ZC"; }

    bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;

    MovementError CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[],
                                        size_t numVisibleAxes, size_t numTotalAxes,
                                        int32_t motorPos[], bool isCoordinated) const noexcept override;

    void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[],
                               size_t numVisibleAxes, size_t numTotalAxes,
                               float machinePos[]) const noexcept override;

    HomingMode GetHomingMode() const noexcept override
    {
        // Delegate to base if present; otherwise fall back to the first enum (matches most base kin defaults).
        return (baseK != nullptr) ? baseK->GetHomingMode() : (HomingMode)0;
    }

private:
    Platform&    platform;
    Kinematics*  baseK = nullptr;    // OWNED

    // Parameters (M669)
    float leadMmPerRev     = 10.0f;  // L
    float handedness       = +1.0f;  // H (+1 or -1)
    bool  wobbleEnable     = false;  // W
    float wobbleAmpMm      = 0.0f;   // A
    float wobbleCycles     = 1.0f;   // N (cycles per 360°)
    float wobblePhaseDeg   = 0.0f;   // P
    float guardThresholdMm = -1.0f;  // G (<0 disabled)
    bool  guardAsError     = false;  // E

    inline float LinearAt(float cDeg) const
    {
        return handedness * (leadMmPerRev / 360.0f) * cDeg;
    }
    inline float WobbleAt(float cDeg) const
    {
        if (!wobbleEnable || wobbleAmpMm == 0.0f || wobbleCycles == 0.0f) { return 0.0f; }
        const float argRad = ((wobbleCycles * cDeg) + wobblePhaseDeg) * (float)M_PI / 180.0f;
        return wobbleAmpMm * sinf(argRad);
    }
    inline float ZPrime(float z, float cDeg) const
    {
        return z + LinearAt(cDeg) + WobbleAt(cDeg);
    }
    inline float ZFromZPrime(float zp, float cDeg) const
    {
        return zp - (LinearAt(cDeg) + WobbleAt(cDeg));
    }
};
