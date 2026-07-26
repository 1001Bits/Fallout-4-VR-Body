#include "ArmIK.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace
{
    constexpr float kEpsilon = 1.0e-5f;
    constexpr float kMinimumElbowAngle = 13.0f;
    constexpr float kMaximumElbowAngle = 175.0f;
    constexpr float kElbowAngleOffset = 135.0f;
    constexpr float kVerticalAxisCorrectionStart = 0.5f;
    constexpr float kBehindShoulderCorrectionRange = 0.1f;
    constexpr float kWristDeadZone = 54.0f;
    constexpr float kMaximumWristCorrection = 35.0f;
    constexpr float kMaximumArmStretch = 0.06f;
    constexpr float kPoleTimeConstant = 0.045f;
    constexpr float kSingularPoleTimeConstant = 0.085f;

    float degreesToRadians(const float degrees)
    {
        return degrees * std::numbers::pi_v<float> / 180.0f;
    }

    float clamp01(const float value)
    {
        return std::clamp(value, 0.0f, 1.0f);
    }

    frik::ik::Vec3 projectOnPlane(const frik::ik::Vec3& value, const frik::ik::Vec3& normal)
    {
        return value - normal * frik::ik::dot(value, normal);
    }

    frik::ik::Vec3 lerp(const frik::ik::Vec3& from, const frik::ik::Vec3& to, const float amount)
    {
        return from + (to - from) * clamp01(amount);
    }

    frik::ik::Vec3 rotateAroundAxis(const frik::ik::Vec3& value, const frik::ik::Vec3& axis, const float angle)
    {
        const float sine = std::sin(angle);
        const float cosine = std::cos(angle);
        return value * cosine + frik::ik::cross(axis, value) * sine + axis * (frik::ik::dot(axis, value) * (1.0f - cosine));
    }

    float signedAngle(const frik::ik::Vec3& from, const frik::ik::Vec3& to, const frik::ik::Vec3& axis)
    {
        const auto safeFrom = frik::ik::safeNormalize(from);
        const auto safeTo = frik::ik::safeNormalize(to, safeFrom);
        return std::atan2(frik::ik::dot(axis, frik::ik::cross(safeFrom, safeTo)), frik::ik::dot(safeFrom, safeTo));
    }

    float getPositionElbowAngle(const frik::ik::Vec3& localHand)
    {
        // Parameters from Parger's published VRArmIK implementation. The input
        // is dimensionless: shoulder-to-hand position divided by arm length.
        float angle = kElbowAngleOffset - 60.0f * localHand.z;

        const float forwardDistance = std::max(0.6f - localHand.y, 0.0f);
        if (localHand.z > 0.0f) {
            angle += 260.0f * forwardDistance * localHand.z;
        } else {
            angle -= 100.0f * forwardDistance * -localHand.z;
        }

        // bodyOutward is positive away from the torso; only crossing inward
        // should activate this term.
        angle -= 50.0f * std::max(-localHand.x + 0.1f, 0.0f);
        return std::clamp(angle, kMinimumElbowAngle, kMaximumElbowAngle);
    }

    float getWristCorrection(
        const frik::ik::Vec3& reachAxis,
        const frik::ik::Vec3& currentHandToElbow,
        const frik::ik::Vec3& trackedHandBack,
        const frik::ik::Vec3& trackedHandSide)
    {
        const auto current = projectOnPlane(currentHandToElbow, reachAxis);
        const auto backProjection = projectOnPlane(trackedHandBack, reachAxis);
        const auto sideProjection = projectOnPlane(trackedHandSide, reachAxis);
        const float currentLength = frik::ik::length(current);
        const float backWeight = frik::ik::length(backProjection);
        const float sideWeight = frik::ik::length(sideProjection);
        if (currentLength <= kEpsilon || (backWeight <= kEpsilon && sideWeight <= kEpsilon)) {
            return 0.0f;
        }

        const auto currentDirection = frik::ik::safeNormalize(current);
        auto sidePole = frik::ik::safeNormalize(frik::ik::cross(reachAxis, sideProjection), currentDirection);
        if (frik::ik::dot(sidePole, currentDirection) < 0.0f) {
            sidePole = sidePole * -1.0f;
        }

        // Both the hand's longitudinal and side axes constrain the one
        // available elbow-swivel degree of freedom. Weight them by projection
        // length so the side axis takes over when handBack is parallel to the
        // shoulder-hand line (the common straight-wrist degeneracy).
        frik::ik::Vec3 desired{};
        if (backWeight > kEpsilon) {
            desired = desired + frik::ik::safeNormalize(backProjection) * (backWeight * backWeight);
        }
        if (sideWeight > kEpsilon) {
            desired = desired + sidePole * (sideWeight * sideWeight);
        }
        desired = frik::ik::safeNormalize(desired, currentDirection);

        const float angle = signedAngle(currentDirection, desired, reachAxis);
        const float deadZone = degreesToRadians(kWristDeadZone);
        const float excess = std::abs(angle) - deadZone;
        if (excess <= 0.0f) {
            return 0.0f;
        }

        // Parger applies a quadratic correction outside the wrist joint-limit
        // dead zone. Cap the swivel contribution so hand orientation remains
        // a late, soft cue rather than taking control of the elbow.
        const float scale = degreesToRadians(135.0f);
        const float correction = std::min(excess * excess / scale, degreesToRadians(kMaximumWristCorrection));
        return std::copysign(correction, angle);
    }
}

namespace frik::ik
{
    Vec3 Vec3::operator+(const Vec3& rhs) const
    {
        return { x + rhs.x, y + rhs.y, z + rhs.z };
    }

    Vec3 Vec3::operator-(const Vec3& rhs) const
    {
        return { x - rhs.x, y - rhs.y, z - rhs.z };
    }

    Vec3 Vec3::operator*(const float scalar) const
    {
        return { x * scalar, y * scalar, z * scalar };
    }

    Vec3 Vec3::operator/(const float scalar) const
    {
        return std::abs(scalar) > kEpsilon ? *this * (1.0f / scalar) : Vec3{};
    }

    bool isFinite(const float value)
    {
        return std::isfinite(value);
    }

    bool isFinite(const Vec3& value)
    {
        return isFinite(value.x) && isFinite(value.y) && isFinite(value.z);
    }

    float dot(const Vec3& lhs, const Vec3& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    Vec3 cross(const Vec3& lhs, const Vec3& rhs)
    {
        return {
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x
        };
    }

    float length(const Vec3& value)
    {
        const float squaredLength = dot(value, value);
        return squaredLength > 0.0f && isFinite(squaredLength) ? std::sqrt(squaredLength) : 0.0f;
    }

    Vec3 safeNormalize(const Vec3& value, const Vec3& fallback)
    {
        const float valueLength = length(value);
        if (valueLength > kEpsilon) {
            return value / valueLength;
        }

        const float fallbackLength = length(fallback);
        return fallbackLength > kEpsilon ? fallback / fallbackLength : Vec3{ 1.0f, 0.0f, 0.0f };
    }

    float safeAcos(const float value)
    {
        return std::acos(std::clamp(value, -1.0f, 1.0f));
    }

    float smoothingAlpha(const float deltaTime, const float timeConstant)
    {
        if (!isFinite(deltaTime) || deltaTime <= 0.0f) {
            return 1.0f;
        }
        if (!isFinite(timeConstant) || timeConstant <= kEpsilon) {
            return 1.0f;
        }
        return 1.0f - std::exp(-std::min(deltaTime, 0.1f) / timeConstant);
    }

    float smoothStep(const float edge0, const float edge1, const float value)
    {
        if (edge1 <= edge0) {
            return value >= edge1 ? 1.0f : 0.0f;
        }
        const float normalized = clamp01((value - edge0) / (edge1 - edge0));
        return normalized * normalized * (3.0f - 2.0f * normalized);
    }

    ArmSolveResult solveArm(const ArmSolveInput& input, ArmContinuityState& continuity)
    {
        ArmSolveResult result;
        if (!isFinite(input.shoulder) || !isFinite(input.hand) ||
            !isFinite(input.upperLength) || !isFinite(input.lowerLength) ||
            input.upperLength <= kEpsilon || input.lowerLength <= kEpsilon) {
            return result;
        }

        const Vec3 shoulderToHand = input.hand - input.shoulder;
        const float targetDistance = length(shoulderToHand);
        const float restLength = input.upperLength + input.lowerLength;
        if (targetDistance <= kEpsilon || targetDistance > restLength * 2.25f) {
            return result;
        }

        const Vec3 reachAxis = safeNormalize(shoulderToHand);
        const Vec3 forward = safeNormalize(input.bodyForward, { 0.0f, 1.0f, 0.0f });
        const Vec3 outward = safeNormalize(input.bodyOutward, { 1.0f, 0.0f, 0.0f });
        const Vec3 up = safeNormalize(input.bodyUp, { 0.0f, 0.0f, 1.0f });

        result.upperLength = input.upperLength;
        result.lowerLength = input.lowerLength;
        result.reachRatio = targetDistance / restLength;

        // Use a small soft extension near full reach, but never stretch the
        // avatar to an arbitrary tracking target. The tracked/controller hand
        // remains untouched; only the cosmetic arm chain is reach-limited.
        if (targetDistance >= restLength - kEpsilon) {
            const float stretchWeight = smoothStep(1.0f, 1.08f, result.reachRatio);
            const float scale = 1.0f + kMaximumArmStretch * stretchWeight;
            result.upperLength *= scale;
            result.lowerLength *= scale;
            result.stretched = scale > 1.0f + kEpsilon;
        } else if (targetDistance <= std::abs(result.upperLength - result.lowerLength) + kEpsilon) {
            // Equal segments remain solvable when the hand is very close to the
            // shoulder and avoid invalid cosine-rule triangles.
            result.upperLength = result.lowerLength = restLength * 0.5f;
        }
        result.solvedReach = std::min(targetDistance, result.upperLength + result.lowerLength - 0.01f);

        const Vec3 localHand = {
            dot(shoulderToHand, outward) / restLength,
            dot(shoulderToHand, forward) / restLength,
            dot(shoulderToHand, up) / restLength
        };

        Vec3 previousPole = continuity.hasPole ? projectOnPlane(continuity.pole, reachAxis) : Vec3{};
        const Vec3 fixedDirection = safeNormalize(outward * 0.133f - up * 0.443f - forward * 0.886f);
        const Vec3 fixedPole = safeNormalize(projectOnPlane(fixedDirection, reachAxis), outward);

        Vec3 baseUp = projectOnPlane(up, reachAxis);
        if (length(baseUp) <= kEpsilon) {
            baseUp = continuity.hasPole ? previousPole : fixedPole;
        }
        baseUp = safeNormalize(baseUp, fixedPole);

        Vec3 swivelTangent = safeNormalize(cross(reachAxis, baseUp), fixedPole);
        if (dot(swivelTangent, outward) < 0.0f) {
            swivelTangent = swivelTangent * -1.0f;
        }

        const float elbowAngle = degreesToRadians(getPositionElbowAngle(localHand));
        Vec3 desiredPole = safeNormalize(baseUp * std::cos(elbowAngle) + swivelTangent * std::sin(elbowAngle), fixedPole);

        // The swivel model is singular near the shoulder's vertical axis.
        // Blend toward the paper's fixed direction and the last valid pole,
        // and use the same correction when the hand moves behind the shoulder.
        const float verticalDistance = std::sqrt(localHand.x * localHand.x + localHand.y * localHand.y);
        const float verticalWeight = 1.0f - smoothStep(0.0f, kVerticalAxisCorrectionStart, verticalDistance);
        const float behindWeight = smoothStep(0.0f, kBehindShoulderCorrectionRange, -localHand.y);
        const float singularWeight = std::max(verticalWeight, behindWeight);

        Vec3 stableReference = fixedPole;
        if (continuity.hasPole && length(previousPole) > kEpsilon) {
            stableReference = safeNormalize(lerp(fixedPole, safeNormalize(previousPole, fixedPole), 0.55f), fixedPole);
        }
        desiredPole = safeNormalize(lerp(desiredPole, stableReference, singularWeight), stableReference);

        const float upperSquared = result.upperLength * result.upperLength;
        const float lowerSquared = result.lowerLength * result.lowerLength;
        const float along = (upperSquared - lowerSquared + result.solvedReach * result.solvedReach) / (2.0f * result.solvedReach);
        const float height = std::sqrt(std::max(upperSquared - along * along, 0.0f));
        Vec3 provisionalElbow = input.shoulder + reachAxis * along + desiredPole * height;
        const Vec3 solvedHand = input.shoulder + reachAxis * result.solvedReach;

        // Hand orientation is deliberately only a late joint-limit correction.
        result.wristCorrection = getWristCorrection(
            reachAxis,
            provisionalElbow - solvedHand,
            safeNormalize(input.handBack, provisionalElbow - solvedHand),
            safeNormalize(input.handSide, outward));
        desiredPole = safeNormalize(rotateAroundAxis(desiredPole, reachAxis, result.wristCorrection), desiredPole);

        if (continuity.hasPole && length(previousPole) > kEpsilon) {
            previousPole = safeNormalize(previousPole, desiredPole);
            const float timeConstant = std::lerp(kPoleTimeConstant, kSingularPoleTimeConstant, singularWeight);
            desiredPole = safeNormalize(
                lerp(previousPole, desiredPole, smoothingAlpha(input.deltaTime, timeConstant)),
                desiredPole);
        }

        result.pole = desiredPole;
        result.elbow = input.shoulder + reachAxis * along + desiredPole * height;
        result.valid = isFinite(result.elbow) && isFinite(result.pole);
        if (result.valid) {
            continuity.pole = result.pole;
            continuity.hasPole = true;
        }
        return result;
    }
}
