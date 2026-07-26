#include "../src/ik/ArmIK.h"

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <numbers>

namespace
{
    constexpr float kTolerance = 1.0e-3f;

    bool close(const float lhs, const float rhs, const float tolerance = kTolerance)
    {
        return std::abs(lhs - rhs) <= tolerance;
    }

    void require(const bool condition, const char* message)
    {
        if (!condition) {
            std::cerr << "ArmIK test failed: " << message << '\n';
            std::exit(EXIT_FAILURE);
        }
    }

    frik::ik::ArmSolveInput makeInput()
    {
        return {
            .shoulder = { 0.0f, 0.0f, 0.0f },
            .hand = { 8.0f, 24.0f, 4.0f },
            .bodyForward = { 0.0f, 1.0f, 0.0f },
            .bodyOutward = { 1.0f, 0.0f, 0.0f },
            .bodyUp = { 0.0f, 0.0f, 1.0f },
            .handBack = { 0.0f, -1.0f, 0.0f },
            .handSide = { 1.0f, 0.0f, 0.0f },
            .upperLength = 16.0f,
            .lowerLength = 15.0f,
            .deltaTime = 1.0f / 90.0f
        };
    }
}

int main()
{
    using namespace frik::ik;

    require(close(safeAcos(2.0f), 0.0f), "safeAcos clamps above one");
    require(close(safeAcos(-2.0f), std::numbers::pi_v<float>), "safeAcos clamps below minus one");
    require(isFinite(safeNormalize({ 0.0f, 0.0f, 0.0f })), "zero-vector normalization stays finite");

    ArmContinuityState state;
    const auto input = makeInput();
    const auto solved = solveArm(input, state);
    require(solved.valid, "ordinary pose solves");
    require(close(length(solved.elbow - input.shoulder), solved.upperLength, 0.01f), "ordinary upper length is preserved");
    const auto solvedHand = input.shoulder + safeNormalize(input.hand - input.shoulder) * solved.solvedReach;
    require(close(length(solvedHand - solved.elbow), solved.lowerLength, 0.01f), "ordinary lower length is preserved");

    // Mirroring both the tracked hand and outward body axis should mirror the
    // solved elbow without requiring handedness-specific solver code.
    auto mirroredInput = input;
    mirroredInput.hand.x *= -1.0f;
    mirroredInput.bodyOutward.x *= -1.0f;
    ArmContinuityState mirroredState;
    const auto mirrored = solveArm(mirroredInput, mirroredState);
    require(mirrored.valid, "mirrored pose solves");
    require(close(solved.elbow.x, -mirrored.elbow.x, 0.01f), "mirrored elbow has opposite lateral coordinate");
    require(close(solved.elbow.y, mirrored.elbow.y, 0.01f), "mirrored elbow preserves forward coordinate");
    require(close(solved.elbow.z, mirrored.elbow.z, 0.01f), "mirrored elbow preserves vertical coordinate");

    // Small movements across the shoulder-up singularity must keep the pole
    // continuous rather than flipping to the opposite side.
    auto verticalInput = input;
    verticalInput.hand = { 0.01f, 0.01f, 25.0f };
    ArmContinuityState verticalState;
    const auto verticalA = solveArm(verticalInput, verticalState);
    verticalInput.hand = { -0.01f, -0.01f, 25.0f };
    const auto verticalB = solveArm(verticalInput, verticalState);
    require(verticalA.valid && verticalB.valid, "near-vertical poses solve");
    require(dot(verticalA.pole, verticalB.pole) > 0.95f, "near-vertical pole remains continuous");

    // A straight wrist commonly makes handBack parallel to the reach axis.
    // The orthogonal tracked hand axis must keep the late correction finite.
    auto wristDegenerateInput = input;
    wristDegenerateInput.handBack = safeNormalize(input.hand - input.shoulder);
    wristDegenerateInput.handSide = { 1.0f, 0.0f, 0.0f };
    ArmContinuityState wristDegenerateState;
    const auto wristDegenerate = solveArm(wristDegenerateInput, wristDegenerateState);
    require(wristDegenerate.valid, "parallel hand-back pose uses secondary hand axis");
    require(isFinite(wristDegenerate.wristCorrection), "secondary-axis wrist correction stays finite");
    require(isFinite(wristDegenerate.pole), "secondary-axis pole stays finite");

    // Reach beyond the calibrated arm is handled without invalid cosine-rule
    // inputs, while clearly bad tracking remains rejected.
    auto stretchedInput = input;
    stretchedInput.hand = { 0.0f, 35.0f, 0.0f };
    ArmContinuityState stretchedState;
    const auto stretched = solveArm(stretchedInput, stretchedState);
    require(stretched.valid && stretched.stretched, "moderate overreach solves with soft extension");
    require(
        stretched.upperLength + stretched.lowerLength <=
            (stretchedInput.upperLength + stretchedInput.lowerLength) * 1.061f,
        "soft extension is bounded to six percent");
    require(
        stretched.solvedReach < length(stretchedInput.hand - stretchedInput.shoulder),
        "analytic reach clamps before unreachable tracked target");
    require(
        close(length(stretched.elbow - stretchedInput.shoulder), stretched.upperLength, 0.01f),
        "bounded overreach preserves upper length");
    const auto stretchedHand =
        stretchedInput.shoulder + safeNormalize(stretchedInput.hand - stretchedInput.shoulder) * stretched.solvedReach;
    require(close(length(stretchedHand - stretched.elbow), stretched.lowerLength, 0.01f), "bounded overreach preserves lower length");

    stretchedInput.hand = { 0.0f, 80.0f, 0.0f };
    const auto invalid = solveArm(stretchedInput, stretchedState);
    require(!invalid.valid, "egregious tracking target is rejected");

    // Exponential smoothing composes consistently across frame rates.
    float at45Hz = 0.0f;
    float at90Hz = 0.0f;
    for (int i = 0; i < 45; ++i) {
        at45Hz += (1.0f - at45Hz) * smoothingAlpha(1.0f / 45.0f, 0.05f);
    }
    for (int i = 0; i < 90; ++i) {
        at90Hz += (1.0f - at90Hz) * smoothingAlpha(1.0f / 90.0f, 0.05f);
    }
    require(close(at45Hz, at90Hz, 1.0e-5f), "time-based smoothing is frame-rate independent");
}
