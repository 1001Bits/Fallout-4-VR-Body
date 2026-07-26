#include "calibration/HmdPivotCalibrator.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numbers>

using namespace frik::calibration;

namespace
{
    Mat3 multiply(const Mat3& lhs, const Mat3& rhs)
    {
        Mat3 result;
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t column = 0; column < 3; ++column) {
                for (std::size_t inner = 0; inner < 3; ++inner) {
                    result(row, column) += lhs(row, inner) * rhs(inner, column);
                }
            }
        }
        return result;
    }

    Mat3 rotationX(const double angle)
    {
        auto result = Mat3::identity();
        result(1, 1) = std::cos(angle);
        result(1, 2) = -std::sin(angle);
        result(2, 1) = std::sin(angle);
        result(2, 2) = std::cos(angle);
        return result;
    }

    Mat3 rotationZ(const double angle)
    {
        auto result = Mat3::identity();
        result(0, 0) = std::cos(angle);
        result(0, 1) = -std::sin(angle);
        result(1, 0) = std::sin(angle);
        result(1, 1) = std::cos(angle);
        return result;
    }

    Vec3 transform(const Mat3& rotation, const Vec3& vector)
    {
        return { rotation(0, 0) * vector.x + rotation(0, 1) * vector.y + rotation(0, 2) * vector.z,
            rotation(1, 0) * vector.x + rotation(1, 1) * vector.y + rotation(1, 2) * vector.z, rotation(2, 0) * vector.x + rotation(2, 1) * vector.y + rotation(2, 2) * vector.z };
    }

    double distance(const Vec3& lhs, const Vec3& rhs)
    {
        const auto x = lhs.x - rhs.x;
        const auto y = lhs.y - rhs.y;
        const auto z = lhs.z - rhs.z;
        return std::sqrt(x * x + y * y + z * z);
    }

    void require(const bool condition, const char* message)
    {
        if (!condition) {
            std::cerr << "FAILED: " << message << '\n';
            std::exit(1);
        }
    }

    HmdPoseSample makeSample(const Vec3& pivot, const Vec3& offset, const double yaw, const double pitch, const double timestamp, const Vec3& noise = {},
        const double worldScale = 1.0)
    {
        const auto rotation = multiply(rotationZ(yaw), rotationX(pitch));
        const auto rotatedOffset = transform(rotation, offset);
        return { { pivot.x + rotatedOffset.x * worldScale + noise.x, pivot.y + rotatedOffset.y * worldScale + noise.y, pivot.z + rotatedOffset.z * worldScale + noise.z }, rotation,
            timestamp, worldScale };
    }

    void recoversKnownPivotWithOutliers()
    {
        HmdPivotCalibrator calibrator;
        const Vec3 expectedPivot{ 100.0, -40.0, 125.0 };
        const Vec3 expectedOffset{ 0.35, 5.8, 9.2 };

        std::size_t sampleIndex = 0;
        for (int pitchIndex = -3; pitchIndex <= 3; ++pitchIndex) {
            for (int yawIndex = -5; yawIndex <= 5; ++yawIndex) {
                const auto yaw = static_cast<double>(yawIndex) * 8.0 * std::numbers::pi / 180.0;
                const auto pitch = static_cast<double>(pitchIndex) * 7.0 * std::numbers::pi / 180.0;
                Vec3 noise{ 0.025 * std::sin(static_cast<double>(sampleIndex)), 0.02 * std::cos(static_cast<double>(sampleIndex) * 0.7),
                    0.02 * std::sin(static_cast<double>(sampleIndex) * 0.4) };
                if (sampleIndex == 18 || sampleIndex == 51) {
                    noise = { 8.0, -6.0, 5.0 };
                }
                require(calibrator.addSample(makeSample(expectedPivot, expectedOffset, yaw, pitch, static_cast<double>(sampleIndex) * 0.05, noise)) != SampleStatus::Invalid,
                    "synthetic sample should be valid");
                ++sampleIndex;
            }
        }

        const auto result = calibrator.solve();
        require(result.succeeded(), describeCalibrationFailure(result.failure));
        require(result.inlierCount < result.sampleCount, "gross outliers should be rejected");
        require(distance(result.pivotToHmdOffset, expectedOffset) < 0.15, "offset fit should recover ground truth");
        require(distance(result.worldPivot, expectedPivot) < 0.15, "pivot fit should recover ground truth");
        require(result.residualRmse < 0.1, "inlier residual should remain low");
    }

    void rejectsSingleAxisMotion()
    {
        HmdPivotCalibrator calibrator;
        const Vec3 pivot{ 10.0, 20.0, 30.0 };
        const Vec3 offset{ 0.0, 5.5, 9.0 };
        for (int index = 0; index < 60; ++index) {
            const auto yaw = (-40.0 + static_cast<double>(index) * 80.0 / 59.0) * std::numbers::pi / 180.0;
            calibrator.addSample(makeSample(pivot, offset, yaw, 0.0, static_cast<double>(index) * 0.05));
        }
        require(calibrator.solve().failure == CalibrationFailure::PoorConditioning, "single-axis motion must be underconstrained");
    }

    void resetsAfterRecenter()
    {
        HmdPivotCalibrator calibrator;
        auto first = makeSample({ 0.0, 0.0, 0.0 }, { 0.0, 5.5, 9.0 }, 0.0, 0.0, 0.0);
        require(calibrator.addSample(first) == SampleStatus::Accepted, "first sample should be accepted");
        first.position.x += 30.0;
        first.timestampSeconds = 0.05;
        require(calibrator.addSample(first) == SampleStatus::ResetAfterTrackingDiscontinuity, "recenter jump should reset capture");
        require(calibrator.sampleCount() == 1, "new tracking epoch should retain only its first sample");
    }

    void handlesRuntimeMatrixConventionAndScale()
    {
        const Vec3 pivot{ 12.0, -5.0, 80.0 };
        const Vec3 offset{ 0.2, 5.5, 9.0 };
        const auto localToWorld = multiply(rotationZ(0.45), rotationX(-0.25));
        Mat3 runtimeWorldToLocal;
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t column = 0; column < 3; ++column) {
                runtimeWorldToLocal(row, column) = localToWorld(column, row);
            }
        }
        Mat3 adaptedLocalToWorld;
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t column = 0; column < 3; ++column) {
                adaptedLocalToWorld(row, column) = runtimeWorldToLocal(column, row);
            }
        }
        constexpr double worldScale = 1.25;
        const auto rotatedOffset = transform(localToWorld, offset);
        const Vec3 tracked{ pivot.x + rotatedOffset.x * worldScale, pivot.y + rotatedOffset.y * worldScale, pivot.z + rotatedOffset.z * worldScale };
        require(distance(computeWorldPivot(tracked, adaptedLocalToWorld, worldScale, offset), pivot) < 1.0e-9, "runtime transpose and world scale must reconstruct the pivot");
    }

    void keepsPivotInvariantUnderRotationAndTranslation()
    {
        const Vec3 offset{ 0.4, 6.2, 8.7 };
        constexpr double worldScale = 1.17;
        constexpr std::array yawDegrees{ -85.0, -45.0, 0.0, 38.0, 90.0 };
        constexpr std::array pitchDegrees{ -50.0, -20.0, 0.0, 25.0, 55.0 };

        for (std::size_t index = 0; index < yawDegrees.size(); ++index) {
            const Vec3 pivot{ 30.0 + static_cast<double>(index) * 4.0, -15.0 - static_cast<double>(index) * 2.5, 110.0 + static_cast<double>(index) };
            const auto rotation = multiply(rotationZ(yawDegrees[index] * std::numbers::pi / 180.0), rotationX(pitchDegrees[index] * std::numbers::pi / 180.0));
            const auto apparentTranslation = transform(rotation, offset);
            const Vec3 tracked{ pivot.x + apparentTranslation.x * worldScale, pivot.y + apparentTranslation.y * worldScale, pivot.z + apparentTranslation.z * worldScale };

            require(distance(computeWorldPivot(tracked, rotation, worldScale, offset), pivot) < 1.0e-9, "pure HMD rotation must not move the recovered anatomical pivot");
        }
    }
}

int main()
{
    recoversKnownPivotWithOutliers();
    rejectsSingleAxisMotion();
    resetsAfterRecenter();
    handlesRuntimeMatrixConventionAndScale();
    keepsPivotInvariantUnderRotationAndTranslation();
    std::cout << "HmdPivotCalibrator tests passed\n";
    return 0;
}
