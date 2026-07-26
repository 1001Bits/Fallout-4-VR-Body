#include "../src/ik/TorsoIK.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numbers>

namespace
{
    constexpr float kTolerance = 1.0e-4f;

    bool close(const float lhs, const float rhs, const float tolerance = kTolerance) { return std::abs(lhs - rhs) <= tolerance; }

    void require(const bool condition, const char* message)
    {
        if (!condition) {
            std::cerr << "TorsoIK test failed: " << message << '\n';
            std::exit(EXIT_FAILURE);
        }
    }

    float degrees(const float value) { return value * std::numbers::pi_v<float> / 180.0f; }

    frik::ik::TorsoTwistSettings makeSettings(const float share)
    {
        return { .share = share, .spineFraction = 0.4f, .spineLimit = degrees(20.0f), .chestLimit = degrees(30.0f) };
    }
}

int main()
{
    using namespace frik::ik;

    // A zero share must reproduce the legacy root-only rotation bit for bit, because
    // that is the shipped default and must not change any pose.
    for (const float yaw : { -degrees(50.0f), -degrees(12.0f), 0.0f, degrees(7.0f), degrees(50.0f) }) {
        const auto off = distributeTorsoTwist(yaw, makeSettings(0.0f));
        require(off.root == yaw, "zero share leaves the whole yaw on the root");
        require(off.spine == 0.0f && off.chest == 0.0f, "zero share does not twist the spine");
    }

    // The chest must land where the legacy root-only rotation put it, for any share.
    constexpr std::array shares{ 0.0f, 0.15f, 0.35f, 0.7f, 1.0f };
    for (const float share : shares) {
        for (const float yaw : { -degrees(50.0f), -degrees(30.0f), -degrees(3.0f), 0.0f, degrees(3.0f), degrees(30.0f), degrees(50.0f) }) {
            const auto twist = distributeTorsoTwist(yaw, makeSettings(share));
            require(close(twist.root + twist.spine + twist.chest, yaw), "distribution preserves the total body yaw");
            require(std::abs(twist.spine) <= degrees(20.0f) + kTolerance, "spine twist respects its limit");
            require(std::abs(twist.chest) <= degrees(30.0f) + kTolerance, "chest twist respects its limit");
        }
    }

    // Mirrored input must mirror exactly, so left and right turns are symmetric.
    const auto right = distributeTorsoTwist(degrees(40.0f), makeSettings(0.35f));
    const auto left = distributeTorsoTwist(-degrees(40.0f), makeSettings(0.35f));
    require(close(right.root, -left.root), "mirrored yaw mirrors the root share");
    require(close(right.spine, -left.spine), "mirrored yaw mirrors the spine share");
    require(close(right.chest, -left.chest), "mirrored yaw mirrors the chest share");

    // A share actually moves yaw off the root, which is the whole point: the pelvis
    // and therefore the feet stop swinging with the upper body.
    const auto moved = distributeTorsoTwist(degrees(40.0f), makeSettings(0.35f));
    require(std::abs(moved.root) < std::abs(degrees(40.0f)), "a non-zero share unloads the root");
    require(moved.spine > 0.0f && moved.chest > 0.0f, "a positive yaw twists both spine joints positively");
    require(close(moved.spine + moved.chest, degrees(40.0f) * 0.35f), "the spine takes exactly the configured share");

    // Clamping must spill back onto the root rather than losing the rotation.
    const TorsoTwistSettings tight{ .share = 1.0f, .spineFraction = 0.5f, .spineLimit = degrees(2.0f), .chestLimit = degrees(3.0f) };
    const auto clamped = distributeTorsoTwist(degrees(50.0f), tight);
    require(close(clamped.spine, degrees(2.0f)), "spine saturates at its limit");
    require(close(clamped.chest, degrees(3.0f)), "chest saturates at its limit");
    require(close(clamped.root, degrees(50.0f) - degrees(5.0f)), "saturated yaw returns to the root");

    // Degenerate settings and inputs must never produce a non-finite pose.
    const auto nan = distributeTorsoTwist(std::numeric_limits<float>::quiet_NaN(), makeSettings(0.35f));
    require(nan.root == 0.0f && nan.spine == 0.0f && nan.chest == 0.0f, "non-finite yaw yields no rotation");

    const TorsoTwistSettings garbage{ .share = std::numeric_limits<float>::quiet_NaN(),
        .spineFraction = -5.0f,
        .spineLimit = std::numeric_limits<float>::infinity(),
        .chestLimit = -1.0f };
    const auto sanitized = distributeTorsoTwist(degrees(30.0f), garbage);
    require(close(sanitized.root, degrees(30.0f)), "non-finite share falls back to legacy behaviour");
    require(sanitized.spine == 0.0f && sanitized.chest == 0.0f, "non-finite share does not twist the spine");

    // An out-of-range share must stay bounded instead of counter-rotating the root.
    const auto over = distributeTorsoTwist(degrees(20.0f), makeSettings(5.0f));
    require(close(over.root + over.spine + over.chest, degrees(20.0f)), "clamped share still preserves the total yaw");

    std::cout << "TorsoIK tests passed\n";
    return 0;
}
