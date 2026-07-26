#pragma once

namespace frik::ik
{
    struct Vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;

        Vec3 operator+(const Vec3& rhs) const;
        Vec3 operator-(const Vec3& rhs) const;
        Vec3 operator*(float scalar) const;
        Vec3 operator/(float scalar) const;
    };

    struct ArmContinuityState
    {
        Vec3 pole;
        bool hasPole = false;
    };

    struct ArmSolveInput
    {
        Vec3 shoulder;
        Vec3 hand;
        Vec3 bodyForward;
        Vec3 bodyOutward;
        Vec3 bodyUp;
        Vec3 handBack;
        Vec3 handSide;
        float upperLength = 0.0f;
        float lowerLength = 0.0f;
        float deltaTime = 0.0f;

        // Parger's empirical elbow-placement parameters, exposed so the resting
        // elbow can be tuned against a real body instead of trusting the published
        // constants. Defaults are the published values.
        float elbowAngleOffset = 135.0f; // neutral swivel angle about the reach axis, degrees
        float elbowUpWeight = 60.0f; // how strongly hand height swings the elbow
    };

    struct ArmSolveResult
    {
        Vec3 elbow;
        Vec3 pole;
        float upperLength = 0.0f;
        float lowerLength = 0.0f;
        float reachRatio = 0.0f;
        float solvedReach = 0.0f;
        float wristCorrection = 0.0f;
        bool stretched = false;
        bool valid = false;
    };

    bool isFinite(float value);
    bool isFinite(const Vec3& value);
    float dot(const Vec3& lhs, const Vec3& rhs);
    Vec3 cross(const Vec3& lhs, const Vec3& rhs);
    float length(const Vec3& value);
    Vec3 safeNormalize(const Vec3& value, const Vec3& fallback = { 1.0f, 0.0f, 0.0f });
    float safeAcos(float value);
    float smoothingAlpha(float deltaTime, float timeConstant);
    float smoothStep(float edge0, float edge1, float value);

    /**
     * Analytic two-bone arm solve with Parger-style elbow positioning.
     *
     * bodyForward/bodyOutward/bodyUp form the shoulder-local frame. bodyOutward
     * always points away from the torso for the arm being solved, which makes the
     * position model symmetric for left and right arms.
     */
    ArmSolveResult solveArm(const ArmSolveInput& input, ArmContinuityState& continuity);
}
