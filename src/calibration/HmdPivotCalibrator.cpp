#include "HmdPivotCalibrator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace frik::calibration
{
    namespace
    {
        constexpr double PI = 3.14159265358979323846;

        struct Fit
        {
            bool solved = false;
            Vec3 offset;
            Vec3 pivot;
            std::vector<double> residuals;
            double conditionNumber = std::numeric_limits<double>::infinity();
            double minimumEigenvalue = 0.0;
            double rmse = std::numeric_limits<double>::infinity();
        };

        Vec3 operator+(const Vec3& lhs, const Vec3& rhs)
        {
            return { lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
        }

        Vec3 operator-(const Vec3& lhs, const Vec3& rhs)
        {
            return { lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
        }

        Vec3 operator*(const Vec3& value, const double scale)
        {
            return { value.x * scale, value.y * scale, value.z * scale };
        }

        Vec3& operator+=(Vec3& lhs, const Vec3& rhs)
        {
            lhs = lhs + rhs;
            return lhs;
        }

        double squaredLength(const Vec3& value)
        {
            return value.x * value.x + value.y * value.y + value.z * value.z;
        }

        double length(const Vec3& value)
        {
            return std::sqrt(squaredLength(value));
        }

        Vec3 multiply(const Mat3& matrix, const Vec3& vector)
        {
            return {
                matrix(0, 0) * vector.x + matrix(0, 1) * vector.y + matrix(0, 2) * vector.z,
                matrix(1, 0) * vector.x + matrix(1, 1) * vector.y + matrix(1, 2) * vector.z,
                matrix(2, 0) * vector.x + matrix(2, 1) * vector.y + matrix(2, 2) * vector.z
            };
        }

        bool isFinite(const Vec3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        double determinant(const Mat3& matrix)
        {
            return matrix(0, 0) * (matrix(1, 1) * matrix(2, 2) - matrix(1, 2) * matrix(2, 1))
                - matrix(0, 1) * (matrix(1, 0) * matrix(2, 2) - matrix(1, 2) * matrix(2, 0))
                + matrix(0, 2) * (matrix(1, 0) * matrix(2, 1) - matrix(1, 1) * matrix(2, 0));
        }

        bool isRotationMatrix(const Mat3& matrix)
        {
            for (const auto value : matrix.values) {
                if (!std::isfinite(value)) {
                    return false;
                }
            }

            for (std::size_t row = 0; row < 3; ++row) {
                double normSquared = 0.0;
                for (std::size_t column = 0; column < 3; ++column) {
                    normSquared += matrix(row, column) * matrix(row, column);
                }
                if (normSquared < 0.8 || normSquared > 1.2) {
                    return false;
                }
            }
            const auto det = determinant(matrix);
            return det > 0.8 && det < 1.2;
        }

        double rotationDistance(const Mat3& lhs, const Mat3& rhs)
        {
            // trace(lhs^T rhs) is also the Frobenius inner product.
            double trace = 0.0;
            for (std::size_t row = 0; row < 3; ++row) {
                for (std::size_t column = 0; column < 3; ++column) {
                    trace += lhs(row, column) * rhs(row, column);
                }
            }
            return std::acos(std::clamp((trace - 1.0) * 0.5, -1.0, 1.0));
        }

        std::array<double, 3> symmetricEigenvalues(std::array<std::array<double, 3>, 3> matrix)
        {
            // Jacobi iterations are stable and ample for a 3x3 symmetric normal matrix.
            for (int iteration = 0; iteration < 24; ++iteration) {
                std::size_t p = 0;
                std::size_t q = 1;
                double largest = std::abs(matrix[p][q]);
                for (std::size_t row = 0; row < 3; ++row) {
                    for (std::size_t column = row + 1; column < 3; ++column) {
                        if (std::abs(matrix[row][column]) > largest) {
                            largest = std::abs(matrix[row][column]);
                            p = row;
                            q = column;
                        }
                    }
                }
                if (largest < 1.0e-12) {
                    break;
                }

                const auto angle = 0.5 * std::atan2(2.0 * matrix[p][q], matrix[q][q] - matrix[p][p]);
                const auto cosine = std::cos(angle);
                const auto sine = std::sin(angle);
                const auto app = matrix[p][p];
                const auto aqq = matrix[q][q];
                const auto apq = matrix[p][q];

                matrix[p][p] = cosine * cosine * app - 2.0 * sine * cosine * apq + sine * sine * aqq;
                matrix[q][q] = sine * sine * app + 2.0 * sine * cosine * apq + cosine * cosine * aqq;
                matrix[p][q] = matrix[q][p] = 0.0;

                for (std::size_t index = 0; index < 3; ++index) {
                    if (index == p || index == q) {
                        continue;
                    }
                    const auto aip = matrix[index][p];
                    const auto aiq = matrix[index][q];
                    matrix[index][p] = matrix[p][index] = cosine * aip - sine * aiq;
                    matrix[index][q] = matrix[q][index] = sine * aip + cosine * aiq;
                }
            }

            std::array<double, 3> result{ matrix[0][0], matrix[1][1], matrix[2][2] };
            std::sort(result.begin(), result.end());
            return result;
        }

        bool solve3x3(
            std::array<std::array<double, 3>, 3> matrix, const std::array<double, 3>& rightHandSide, Vec3& solution)
        {
            std::array<std::array<double, 4>, 3> augmented{};
            for (std::size_t row = 0; row < 3; ++row) {
                for (std::size_t column = 0; column < 3; ++column) {
                    augmented[row][column] = matrix[row][column];
                }
                augmented[row][3] = rightHandSide[row];
            }

            for (std::size_t pivot = 0; pivot < 3; ++pivot) {
                auto bestRow = pivot;
                for (std::size_t row = pivot + 1; row < 3; ++row) {
                    if (std::abs(augmented[row][pivot]) > std::abs(augmented[bestRow][pivot])) {
                        bestRow = row;
                    }
                }
                if (std::abs(augmented[bestRow][pivot]) < 1.0e-12) {
                    return false;
                }
                std::swap(augmented[pivot], augmented[bestRow]);

                for (std::size_t row = pivot + 1; row < 3; ++row) {
                    const auto factor = augmented[row][pivot] / augmented[pivot][pivot];
                    for (std::size_t column = pivot; column < 4; ++column) {
                        augmented[row][column] -= factor * augmented[pivot][column];
                    }
                }
            }

            std::array<double, 3> values{};
            for (int row = 2; row >= 0; --row) {
                auto value = augmented[static_cast<std::size_t>(row)][3];
                for (std::size_t column = static_cast<std::size_t>(row) + 1; column < 3; ++column) {
                    value -= augmented[static_cast<std::size_t>(row)][column] * values[column];
                }
                values[static_cast<std::size_t>(row)] = value / augmented[static_cast<std::size_t>(row)][static_cast<std::size_t>(row)];
            }
            solution = { values[0], values[1], values[2] };
            return isFinite(solution);
        }

        Fit fitSamples(const std::deque<HmdPoseSample>& samples, const std::vector<std::size_t>& indices)
        {
            Fit fit;
            if (indices.empty()) {
                return fit;
            }

            Vec3 meanPosition;
            Mat3 meanRotation;
            for (const auto index : indices) {
                meanPosition += samples[index].position;
                for (std::size_t element = 0; element < 9; ++element) {
                    meanRotation.values[element] += samples[index].rotation.values[element] * samples[index].worldScale;
                }
            }
            const auto inverseCount = 1.0 / static_cast<double>(indices.size());
            meanPosition = meanPosition * inverseCount;
            for (auto& element : meanRotation.values) {
                element *= inverseCount;
            }

            std::array<std::array<double, 3>, 3> normal{};
            std::array<double, 3> rightHandSide{};
            for (const auto index : indices) {
                const auto positionDelta = samples[index].position - meanPosition;
                double deltaRotation[3][3]{};
                for (std::size_t row = 0; row < 3; ++row) {
                    for (std::size_t column = 0; column < 3; ++column) {
                        deltaRotation[row][column] =
                            samples[index].rotation(row, column) * samples[index].worldScale - meanRotation(row, column);
                    }
                }

                const double positionValues[3]{ positionDelta.x, positionDelta.y, positionDelta.z };
                for (std::size_t column = 0; column < 3; ++column) {
                    for (std::size_t otherColumn = 0; otherColumn < 3; ++otherColumn) {
                        for (std::size_t row = 0; row < 3; ++row) {
                            normal[column][otherColumn] += deltaRotation[row][column] * deltaRotation[row][otherColumn];
                        }
                    }
                    for (std::size_t row = 0; row < 3; ++row) {
                        rightHandSide[column] += deltaRotation[row][column] * positionValues[row];
                    }
                }
            }

            const auto eigenvalues = symmetricEigenvalues(normal);
            fit.minimumEigenvalue = eigenvalues.front();
            if (fit.minimumEigenvalue > 0.0) {
                fit.conditionNumber = eigenvalues.back() / fit.minimumEigenvalue;
            }
            if (!solve3x3(normal, rightHandSide, fit.offset)) {
                return fit;
            }

            for (const auto index : indices) {
                fit.pivot += computeWorldPivot(
                    samples[index].position, samples[index].rotation, samples[index].worldScale, fit.offset);
            }
            fit.pivot = fit.pivot * inverseCount;

            double residualSquaredSum = 0.0;
            fit.residuals.reserve(indices.size());
            for (const auto index : indices) {
                const auto predictedPosition =
                    fit.pivot + multiply(samples[index].rotation, fit.offset) * samples[index].worldScale;
                const auto residual = length(samples[index].position - predictedPosition);
                fit.residuals.push_back(residual);
                residualSquaredSum += residual * residual;
            }
            fit.rmse = std::sqrt(residualSquaredSum * inverseCount);
            fit.solved = std::isfinite(fit.rmse);
            return fit;
        }

        double median(std::vector<double> values)
        {
            if (values.empty()) {
                return 0.0;
            }
            const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
            std::nth_element(values.begin(), middle, values.end());
            auto result = *middle;
            if (values.size() % 2 == 0) {
                result = (*std::max_element(values.begin(), middle) + result) * 0.5;
            }
            return result;
        }

        bool plausibleOffset(const Vec3& offset)
        {
            return offset.x >= -8.0 && offset.x <= 8.0
                && offset.y >= 1.0 && offset.y <= 16.0
                && offset.z >= 2.0 && offset.z <= 20.0;
        }
    }

    Mat3 Mat3::identity()
    {
        Mat3 result;
        result(0, 0) = 1.0;
        result(1, 1) = 1.0;
        result(2, 2) = 1.0;
        return result;
    }

    HmdPivotCalibrator::HmdPivotCalibrator(HmdPivotCalibratorSettings settings) :
        _settings(settings)
    {
    }

    SampleStatus HmdPivotCalibrator::addSample(const HmdPoseSample& sample)
    {
        if (!std::isfinite(sample.timestampSeconds) || !std::isfinite(sample.worldScale)
            || sample.worldScale < 0.5 || sample.worldScale > 2.0
            || !isFinite(sample.position) || !isRotationMatrix(sample.rotation)) {
            return SampleStatus::Invalid;
        }

        auto status = SampleStatus::Accepted;
        if (!_samples.empty()) {
            const auto elapsed = sample.timestampSeconds - _samples.back().timestampSeconds;
            if (elapsed <= 0.0) {
                return SampleStatus::Invalid;
            }
            if (elapsed < _settings.minimumSampleIntervalSeconds) {
                return SampleStatus::IgnoredTooSoon;
            }
            if (elapsed > _settings.trackingGapResetSeconds
                || length(sample.position - _samples.back().position) > _settings.recenterJumpDistance) {
                reset();
                status = SampleStatus::ResetAfterTrackingDiscontinuity;
            }
        }

        _samples.push_back(sample);
        while (_samples.size() > _settings.maximumSamples) {
            _samples.pop_front();
        }
        while (_samples.size() > 1
               && _samples.back().timestampSeconds - _samples.front().timestampSeconds > _settings.retainedDurationSeconds) {
            _samples.pop_front();
        }
        return status;
    }

    HmdPivotCalibrationResult HmdPivotCalibrator::solve() const
    {
        HmdPivotCalibrationResult result;
        result.sampleCount = _samples.size();
        result.durationSeconds = durationSeconds();

        if (_samples.size() < _settings.minimumSamples) {
            result.failure = CalibrationFailure::NotEnoughSamples;
            return result;
        }
        if (result.durationSeconds < _settings.minimumDurationSeconds) {
            result.failure = CalibrationFailure::NotEnoughTime;
            return result;
        }

        // Farthest pair is a simple, interpretable overall coverage metric.
        for (std::size_t first = 0; first < _samples.size(); ++first) {
            for (std::size_t second = first + 1; second < _samples.size(); ++second) {
                result.orientationSpanRadians = std::max(
                    result.orientationSpanRadians, rotationDistance(_samples[first].rotation, _samples[second].rotation));
            }
        }
        if (result.orientationSpanRadians < _settings.minimumOrientationSpanRadians) {
            result.failure = CalibrationFailure::InsufficientRotationCoverage;
            return result;
        }

        std::vector<std::size_t> allIndices(_samples.size());
        std::iota(allIndices.begin(), allIndices.end(), std::size_t{ 0 });
        const auto initialFit = fitSamples(_samples, allIndices);
        result.conditionNumber = initialFit.conditionNumber;
        if (!initialFit.solved || initialFit.minimumEigenvalue < _settings.minimumNormalEigenvalue
            || initialFit.conditionNumber > _settings.maximumConditionNumber) {
            result.failure = CalibrationFailure::PoorConditioning;
            return result;
        }

        const auto residualMedian = median(initialFit.residuals);
        std::vector<double> absoluteDeviations;
        absoluteDeviations.reserve(initialFit.residuals.size());
        for (const auto residual : initialFit.residuals) {
            absoluteDeviations.push_back(std::abs(residual - residualMedian));
        }
        const auto robustSigma = 1.4826 * median(std::move(absoluteDeviations));
        const auto inlierThreshold = std::max(0.35, residualMedian + _settings.outlierSigma * std::max(robustSigma, 0.1));

        std::vector<std::size_t> inlierIndices;
        inlierIndices.reserve(allIndices.size());
        for (std::size_t index = 0; index < initialFit.residuals.size(); ++index) {
            if (initialFit.residuals[index] <= inlierThreshold) {
                inlierIndices.push_back(index);
            }
        }
        result.inlierCount = inlierIndices.size();
        if (inlierIndices.size() < _settings.minimumSamples
            || static_cast<double>(inlierIndices.size()) / static_cast<double>(_samples.size()) < _settings.minimumInlierRatio) {
            result.failure = CalibrationFailure::TooManyOutliers;
            return result;
        }

        const auto finalFit = fitSamples(_samples, inlierIndices);
        result.conditionNumber = finalFit.conditionNumber;
        result.residualRmse = finalFit.rmse;
        if (!finalFit.solved || finalFit.minimumEigenvalue < _settings.minimumNormalEigenvalue
            || finalFit.conditionNumber > _settings.maximumConditionNumber) {
            result.failure = CalibrationFailure::PoorConditioning;
            return result;
        }
        if (finalFit.rmse > _settings.maximumResidualRmse) {
            result.failure = CalibrationFailure::ExcessiveResidual;
            return result;
        }
        if (!plausibleOffset(finalFit.offset)) {
            result.failure = CalibrationFailure::ImplausibleOffset;
            return result;
        }

        result.pivotToHmdOffset = finalFit.offset;
        result.worldPivot = finalFit.pivot;
        result.failure = CalibrationFailure::None;
        return result;
    }

    void HmdPivotCalibrator::reset()
    {
        _samples.clear();
    }

    double HmdPivotCalibrator::durationSeconds() const
    {
        return _samples.size() > 1 ? _samples.back().timestampSeconds - _samples.front().timestampSeconds : 0.0;
    }

    const char* describeCalibrationFailure(const CalibrationFailure failure)
    {
        switch (failure) {
        case CalibrationFailure::None:
            return "calibration succeeded";
        case CalibrationFailure::NotEnoughSamples:
            return "not enough samples; keep moving your head slowly";
        case CalibrationFailure::NotEnoughTime:
            return "capture was too short; keep moving your head slowly";
        case CalibrationFailure::InsufficientRotationCoverage:
            return "not enough head rotation; look left, right, up, and down";
        case CalibrationFailure::PoorConditioning:
            return "motion did not cover independent axes; look left/right and up/down";
        case CalibrationFailure::TooManyOutliers:
            return "too much body movement or tracking noise; keep shoulders still and retry";
        case CalibrationFailure::ExcessiveResidual:
            return "head motion was inconsistent; keep shoulders still and retry";
        case CalibrationFailure::ImplausibleOffset:
            return "fitted offset was outside safe anatomical limits";
        }
        return "unknown calibration error";
    }

    Vec3 computeWorldPivot(
        const Vec3& trackedHmdPosition, const Mat3& localToWorldRotation, const double worldScale, const Vec3& pivotToHmdOffset)
    {
        return trackedHmdPosition - multiply(localToWorldRotation, pivotToHmdOffset) * worldScale;
    }
}
