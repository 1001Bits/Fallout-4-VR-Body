#include "../src/ik/ArmIK.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numbers>

namespace
{
    constexpr float kTolerance = 1.0e-3f;

    bool close(const float lhs, const float rhs, const float tolerance = kTolerance) { return std::abs(lhs - rhs) <= tolerance; }

    void require(const bool condition, const char* message)
    {
        if (!condition) {
            std::cerr << "ArmIK test failed: " << message << '\n';
            std::exit(EXIT_FAILURE);
        }
    }

    frik::ik::ArmSolveInput makeInput()
    {
        return { .shoulder = { 0.0f, 0.0f, 0.0f },
            .hand = { 8.0f, 24.0f, 4.0f },
            .bodyForward = { 0.0f, 1.0f, 0.0f },
            .bodyOutward = { 1.0f, 0.0f, 0.0f },
            .bodyUp = { 0.0f, 0.0f, 1.0f },
            .handBack = { 0.0f, -1.0f, 0.0f },
            .handSide = { 1.0f, 0.0f, 0.0f },
            .upperLength = 16.0f,
            .lowerLength = 15.0f,
            .deltaTime = 1.0f / 90.0f };
    }

    float smoothForOneSecond(const int refreshRate)
    {
        float value = 0.0f;
        for (int frame = 0; frame < refreshRate; ++frame) {
            value += (1.0f - value) * frik::ik::smoothingAlpha(1.0f / static_cast<float>(refreshRate), 0.05f);
        }
        return value;
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

    // Exact lateral T-poses used to choose opposite tangent hemispheres for
    // left and right because tangent dot outward is zero in this pose.
    auto rightTposeInput = makeInput();
    rightTposeInput.hand = { 25.0f, 0.0f, 0.0f };
    rightTposeInput.bodyOutward = { 1.0f, 0.0f, 0.0f };
    rightTposeInput.handBack = { 0.0f, -1.0f, 0.0f };
    ArmContinuityState rightTposeState;
    const auto rightTpose = solveArm(rightTposeInput, rightTposeState);

    auto leftTposeInput = rightTposeInput;
    leftTposeInput.hand.x *= -1.0f;
    leftTposeInput.bodyOutward.x *= -1.0f;
    ArmContinuityState leftTposeState;
    const auto leftTpose = solveArm(leftTposeInput, leftTposeState);
    require(rightTpose.valid && leftTpose.valid, "exact lateral T-poses solve");
    require(close(rightTpose.elbow.x, -leftTpose.elbow.x, 0.01f), "T-pose elbow mirrors laterally");
    require(close(rightTpose.elbow.y, leftTpose.elbow.y, 0.01f), "T-pose elbows choose the same fore-aft hemisphere");
    require(close(rightTpose.elbow.z, leftTpose.elbow.z, 0.01f), "T-pose elbows preserve vertical symmetry");

    auto crossingInput = rightTposeInput;
    ArmContinuityState crossingState;
    crossingInput.hand.y = -0.01f;
    const auto crossingA = solveArm(crossingInput, crossingState);
    crossingInput.hand.y = 0.01f;
    const auto crossingB = solveArm(crossingInput, crossingState);
    require(crossingA.valid && crossingB.valid, "near-lateral sweep solves");
    require(dot(crossingA.pole, crossingB.pole) > 0.95f, "pole does not flip as a T-pose crosses body-forward zero");

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

    // Unequal segments (notably Power Armor) remain continuous at the inner
    // reachable radius instead of abruptly becoming two equal segments.
    auto foldedInput = input;
    foldedInput.upperLength = 20.0f;
    foldedInput.lowerLength = 12.0f;
    foldedInput.hand = { 7.99f, 0.0f, 0.0f };
    ArmContinuityState foldedState;
    const auto foldedInside = solveArm(foldedInput, foldedState);
    foldedInput.hand = { 8.01f, 0.0f, 0.0f };
    const auto foldedOutside = solveArm(foldedInput, foldedState);
    require(foldedInside.valid && foldedOutside.valid, "poses around unequal-segment inner radius solve");
    require(std::abs(foldedInside.upperLength - foldedInside.lowerLength) > 7.9f, "inner-radius handling preserves unequal segment character");
    require(std::abs(foldedInside.upperLength - foldedOutside.upperLength) < 0.03f, "inner-radius segment adjustment is continuous");
    require(close(foldedInside.solvedReach, 7.99f, 0.001f) && close(foldedOutside.solvedReach, 8.01f, 0.001f), "folded poses preserve the tracked endpoint");

    // Reach beyond the calibrated arm is handled without invalid cosine-rule
    // inputs, while clearly bad tracking remains rejected.
    auto stretchedInput = input;
    stretchedInput.hand = { 0.0f, 32.0f, 0.0f };
    ArmContinuityState stretchedState;
    const auto stretched = solveArm(stretchedInput, stretchedState);
    require(stretched.valid && stretched.stretched, "moderate overreach solves with soft extension");
    require(stretched.upperLength + stretched.lowerLength <= (stretchedInput.upperLength + stretchedInput.lowerLength) * 1.061f, "soft extension is bounded to six percent");
    require(close(stretched.solvedReach, length(stretchedInput.hand - stretchedInput.shoulder), 0.001f), "every valid solve preserves the tracked hand target");
    require(close(length(stretched.elbow - stretchedInput.shoulder), stretched.upperLength, 0.01f), "bounded overreach preserves upper length");
    const auto stretchedHand = stretchedInput.shoulder + safeNormalize(stretchedInput.hand - stretchedInput.shoulder) * stretched.solvedReach;
    require(close(length(stretchedHand - stretched.elbow), stretched.lowerLength, 0.01f), "bounded overreach preserves lower length");

    stretchedInput.hand = { 0.0f, 35.0f, 0.0f };
    const auto invalid = solveArm(stretchedInput, stretchedState);
    require(!invalid.valid, "overreach beyond bounded extension is rejected");

    stretchedInput.hand = { 0.0f, 80.0f, 0.0f };
    require(!solveArm(stretchedInput, stretchedState).valid, "egregious tracking target is rejected");

    auto nonFiniteInput = input;
    nonFiniteInput.hand.x = std::numeric_limits<float>::quiet_NaN();
    const auto priorPole = state.pole;
    const auto nonFinite = solveArm(nonFiniteInput, state);
    require(!nonFinite.valid, "non-finite tracking input is rejected");
    require(state.hasPole && dot(state.pole, priorPole) > 0.999f, "rejected input does not poison continuity state");

    auto zeroBasisInput = input;
    zeroBasisInput.bodyForward = {};
    zeroBasisInput.bodyOutward = {};
    zeroBasisInput.bodyUp = {};
    zeroBasisInput.handBack = {};
    zeroBasisInput.handSide = {};
    ArmContinuityState zeroBasisState;
    const auto zeroBasis = solveArm(zeroBasisInput, zeroBasisState);
    require(zeroBasis.valid && isFinite(zeroBasis.elbow), "degenerate tracking axes use finite fallbacks");

    // Exponential smoothing composes consistently across common VR refresh rates.
    constexpr std::array refreshRates{ 45, 72, 90, 120 };
    const float reference = smoothForOneSecond(refreshRates.front());
    for (const int refreshRate : refreshRates) {
        require(close(reference, smoothForOneSecond(refreshRate), 1.0e-5f), "time-based smoothing is frame-rate independent");
    }
}
