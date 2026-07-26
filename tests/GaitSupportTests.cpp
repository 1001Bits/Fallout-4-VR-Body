#include "../src/ik/GaitSupport.h"

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
            std::cerr << "GaitSupport test failed: " << message << '\n';
            std::exit(EXIT_FAILURE);
        }
    }

    float degrees(const float value) { return value * std::numbers::pi_v<float> / 180.0f; }
}

int main()
{
    using namespace frik::ik;

    constexpr float pi = std::numbers::pi_v<float>;

    // Wrapping must be stable across the +/-pi seam, which a turning body crosses
    // constantly - an unwrapped difference there reads as a full turn.
    require(close(wrapAngle(0.0f), 0.0f), "zero stays zero");
    require(close(wrapAngle(degrees(90.0f)), degrees(90.0f)), "in-range angle is unchanged");
    require(close(wrapAngle(3.0f * pi), pi), "odd multiples of pi wrap to pi");
    require(close(wrapAngle(2.0f * pi + degrees(10.0f)), degrees(10.0f)), "full turn is removed");
    require(close(wrapAngle(-2.0f * pi - degrees(10.0f)), -degrees(10.0f)), "negative full turn is removed");
    require(wrapAngle(std::numeric_limits<float>::quiet_NaN()) == 0.0f, "non-finite angle yields zero");
    for (const float a : { -pi, pi, -pi + 1.0e-6f, pi - 1.0e-6f }) {
        require(wrapAngle(a) > -pi - kTolerance && wrapAngle(a) <= pi + kTolerance, "wrap stays inside the range");
    }

    // Crossing the seam by a small step must report a small turn, not ~2pi.
    require(close(std::abs(wrapAngle(-pi + degrees(5.0f) - (pi - degrees(5.0f)))), degrees(10.0f)), "seam crossing measures the short way");

    // Stop blend ramps monotonically and saturates.
    require(close(stopBlend(0.0f, 0.2f), 0.0f), "blend starts at zero");
    require(close(stopBlend(0.2f, 0.2f), 1.0f), "blend finishes at one");
    require(stopBlend(0.1f, 0.2f) > 0.0f && stopBlend(0.1f, 0.2f) < 1.0f, "blend is partial mid-way");
    float previous = -1.0f;
    for (int i = 0; i <= 20; ++i) {
        const float value = stopBlend(0.01f * static_cast<float>(i), 0.2f);
        require(value >= previous - kTolerance, "blend is monotonic");
        previous = value;
    }
    require(close(stopBlend(1.0f, 0.0f), 1.0f), "zero duration completes immediately");
    require(close(stopBlend(std::numeric_limits<float>::quiet_NaN(), 0.2f), 1.0f), "non-finite elapsed completes rather than stalling");

    // Turn accumulator: first sample only primes, it must not fire.
    TurnAccumulator turn;
    require(!turn.update(degrees(90.0f), degrees(30.0f)), "first sample only primes the accumulator");

    // Turning past the threshold fires exactly once, then needs a fresh turn.
    require(!turn.update(degrees(100.0f), degrees(30.0f)), "10 degrees is below the threshold");
    require(!turn.update(degrees(110.0f), degrees(30.0f)), "20 degrees is still below");
    require(turn.update(degrees(125.0f), degrees(30.0f)), "35 degrees crosses the threshold");
    require(!turn.update(degrees(130.0f), degrees(30.0f)), "the threshold is consumed, not latched");

    // Direction does not matter - turning back also accumulates.
    TurnAccumulator both;
    both.update(0.0f, degrees(30.0f));
    require(!both.update(degrees(-20.0f), degrees(30.0f)), "reverse turn accumulates below threshold");
    require(both.update(degrees(-45.0f), degrees(30.0f)), "reverse turn fires at the threshold");

    // A continuous spin should produce steps at a steady cadence, not one burst.
    TurnAccumulator spin;
    spin.update(0.0f, degrees(30.0f));
    int fired = 0;
    for (int i = 1; i <= 36; ++i) {
        if (spin.update(degrees(10.0f * static_cast<float>(i)), degrees(30.0f))) {
            ++fired;
        }
    }
    require(fired == 12, "a full turn at 30 degrees per step fires twelve times");

    // A teleport-sized jump must not bank up a long queue of steps. The backlog is
    // bounded to two thresholds, so it may settle over at most one more frame.
    TurnAccumulator jump;
    jump.update(0.0f, degrees(30.0f));
    require(jump.update(degrees(1000.0f), degrees(30.0f)), "a huge delta fires");
    require(jump.accumulated <= degrees(30.0f) + kTolerance, "backlog after firing is at most one threshold");
    int residual = 0;
    for (int i = 0; i < 10; ++i) {
        if (jump.update(degrees(1000.0f), degrees(30.0f))) { // no further turning
            ++residual;
        }
    }
    require(residual <= 1, "a huge delta queues at most one extra step");
    require(!jump.update(degrees(1000.0f), degrees(30.0f)), "the backlog drains and stops firing");

    // Non-finite input resets instead of poisoning the state.
    TurnAccumulator poisoned;
    poisoned.update(0.0f, degrees(30.0f));
    poisoned.update(degrees(20.0f), degrees(30.0f));
    require(!poisoned.update(std::numeric_limits<float>::quiet_NaN(), degrees(30.0f)), "non-finite yaw does not fire");
    require(!poisoned.hasYaw && poisoned.accumulated == 0.0f, "non-finite yaw resets the accumulator");

    std::cout << "GaitSupport tests passed\n";
    return 0;
}
