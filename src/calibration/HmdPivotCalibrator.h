#pragma once

#include <array>
#include <cstddef>
#include <deque>
#include <vector>

namespace frik::calibration
{
    struct Vec3
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct Mat3
    {
        std::array<double, 9> values{};

        double& operator()(const std::size_t row, const std::size_t column) { return values[row * 3 + column]; }
        double operator()(const std::size_t row, const std::size_t column) const { return values[row * 3 + column]; }

        static Mat3 identity();
    };

    struct HmdPoseSample
    {
        Vec3 position;
        // Always local -> world. Fallout's NiMatrix world rotation is the
        // inverse convention and must be transposed by the runtime adapter.
        Mat3 rotation;
        double timestampSeconds = 0.0;
        double worldScale = 1.0;
    };

    enum class SampleStatus
    {
        Accepted,
        IgnoredTooSoon,
        Invalid,
        ResetAfterTrackingDiscontinuity
    };

    enum class CalibrationFailure
    {
        None,
        NotEnoughSamples,
        NotEnoughTime,
        InsufficientRotationCoverage,
        PoorConditioning,
        TooManyOutliers,
        ExcessiveResidual,
        ImplausibleOffset
    };

    struct HmdPivotCalibrationResult
    {
        CalibrationFailure failure = CalibrationFailure::NotEnoughSamples;
        Vec3 pivotToHmdOffset;
        Vec3 worldPivot;
        std::size_t sampleCount = 0;
        std::size_t inlierCount = 0;
        double durationSeconds = 0.0;
        double orientationSpanRadians = 0.0;
        double conditionNumber = 0.0;
        double residualRmse = 0.0;

        bool succeeded() const { return failure == CalibrationFailure::None; }
    };

    struct HmdPivotCalibratorSettings
    {
        std::size_t minimumSamples = 24;
        std::size_t maximumSamples = 600;
        double minimumDurationSeconds = 2.0;
        double retainedDurationSeconds = 10.0;
        double minimumSampleIntervalSeconds = 1.0 / 30.0;
        double trackingGapResetSeconds = 1.0;
        double recenterJumpDistance = 20.0;
        double minimumOrientationSpanRadians = 0.55;
        double maximumConditionNumber = 10000.0;
        double minimumNormalEigenvalue = 1.0e-4;
        double minimumInlierRatio = 0.65;
        double outlierSigma = 3.5;
        double maximumResidualRmse = 1.5;
    };

    /**
     * Fits p_i = c + R_i r, where p_i/R_i are tracked HMD world poses,
     * c is the fixed anatomical pivot, and r is pivot -> tracked HMD origin
     * expressed in HMD-local axes.
     *
     * This class is engine-independent so the numerical and rejection behavior
     * can be exercised without Fallout 4 or F4SE.
     */
    class HmdPivotCalibrator
    {
    public:
        explicit HmdPivotCalibrator(HmdPivotCalibratorSettings settings = {});

        SampleStatus addSample(const HmdPoseSample& sample);
        HmdPivotCalibrationResult solve() const;
        void reset();

        std::size_t sampleCount() const { return _samples.size(); }
        double durationSeconds() const;

    private:
        HmdPivotCalibratorSettings _settings;
        std::deque<HmdPoseSample> _samples;
    };

    const char* describeCalibrationFailure(CalibrationFailure failure);

    /**
     * Runtime integration API. Rigid pivot correction is absolute/unsmoothed:
     * c = p - R * (worldScale * pivotToHmdOffset).
     */
    Vec3 computeWorldPivot(
        const Vec3& trackedHmdPosition, const Mat3& localToWorldRotation, double worldScale, const Vec3& pivotToHmdOffset);
}
