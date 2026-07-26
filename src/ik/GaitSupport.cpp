#include "GaitSupport.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "ArmIK.h"

namespace frik::ik
{
    float wrapAngle(const float radians)
    {
        if (!isFinite(radians)) {
            return 0.0f;
        }

        constexpr float twoPi = 2.0f * std::numbers::pi_v<float>;
        float wrapped = std::remainder(radians, twoPi);
        // std::remainder already lands in [-pi, pi], but guard the boundary so the
        // result never reads back as exactly +pi on one frame and -pi on the next.
        if (wrapped <= -std::numbers::pi_v<float>) {
            wrapped += twoPi;
        } else if (wrapped > std::numbers::pi_v<float>) {
            wrapped -= twoPi;
        }
        return wrapped;
    }

    float stopBlend(const float elapsedSeconds, const float durationSeconds)
    {
        if (!isFinite(elapsedSeconds) || !isFinite(durationSeconds) || durationSeconds <= 0.0f) {
            return 1.0f;
        }
        return smoothStep(0.0f, durationSeconds, elapsedSeconds);
    }

    void TurnAccumulator::reset()
    {
        accumulated = 0.0f;
        lastYaw = 0.0f;
        hasYaw = false;
    }

    bool TurnAccumulator::update(const float bodyYaw, const float thresholdRadians)
    {
        if (!isFinite(bodyYaw)) {
            reset();
            return false;
        }

        if (!hasYaw) {
            lastYaw = bodyYaw;
            hasYaw = true;
            accumulated = 0.0f;
            return false;
        }

        accumulated += std::abs(wrapAngle(bodyYaw - lastYaw));
        lastYaw = bodyYaw;

        if (!isFinite(thresholdRadians) || thresholdRadians <= 0.0f) {
            return false;
        }

        // wrapAngle bounds a single delta to pi, but a recenter or teleport can still
        // deliver most of that at once. Bounding the backlog to two thresholds means
        // one bad frame can queue at most two steps instead of a long shuffle.
        accumulated = (std::min)(accumulated, thresholdRadians * 2.0f);
        if (accumulated < thresholdRadians) {
            return false;
        }

        // Consume one threshold's worth rather than clearing, so a continuous spin
        // keeps producing steps at a steady angular cadence.
        accumulated -= thresholdRadians;
        return true;
    }
}
