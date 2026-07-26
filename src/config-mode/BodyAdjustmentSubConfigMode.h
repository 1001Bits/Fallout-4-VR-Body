#pragma once

#include <chrono>
#include <vector>

#include "calibration/HmdPivotCalibrator.h"
#include "vrui/UIContainer.h"
#include "vrui/UIToggleGroupContainer.h"

namespace frik
{
    /**
     * What is currently being configured
     */
    enum class BodyAdjustmentConfigTarget : uint8_t
    {
        None = 0,
        BodyHeight,
        BodyForwardOffset,
        BodyArmsLength,
        VRScale,
    };

    class BodyAdjustmentSubConfigMode
    {
    public:
        explicit BodyAdjustmentSubConfigMode(const std::function<void()>& onClose);

        void onFrameUpdate();
        static void updateLegSlack(float rightLegSlack, float leftLegSlack);

    private:
        struct HeightCalibrationSample
        {
            float rawUprightHeight = 0.0f;
            float pivotCoefficientX = 0.0f;
            float pivotCoefficientY = 0.0f;
            float pivotCoefficientZ = 0.0f;
            float hmdToPlayerScale = 1.0f;
        };

        void createConfigUI();
        void clearConfigTarget();
        void beginHmdPivotCalibration();
        void captureHmdPivotSample();
        bool finishHmdPivotCalibration();
        void captureHeightSample();
        bool finishHeightCalibration();
        void beginArmSpanCalibration();
        void captureArmSpanSample();
        bool finishArmSpanCalibration();
        void togglePlayingSeated(bool seated);
        void toggleHideHeadEquipment(bool hide);
        void closeConfig();
        void handleAdjustment();
        void handleHeightAdjustment();
        static void handleForwardAdjustment();
        static void handleArmsLengthAdjustment();
        static void handleVRScaleAdjustment();
        void saveConfig();
        void resetConfig();

        // A complete same-frame pair prevents one planted/bent leg from
        // driving an unsafe whole-body correction.
        inline static float _bilateralLegSlackLow = 0.0f;
        inline static float _bilateralLegSlackHigh = 0.0f;
        inline static bool _hasBilateralLegSlack = false;

        std::function<void()> _onClose;

        BodyAdjustmentConfigTarget _configTarget = BodyAdjustmentConfigTarget::None;
        calibration::HmdPivotCalibrator _hmdPivotCalibrator;
        std::vector<HeightCalibrationSample> _heightSamples;
        std::vector<float> _wristSpanSamples;
        std::vector<std::chrono::steady_clock::time_point> _wristSpanSampleTimes;
        std::chrono::steady_clock::time_point _heightCaptureStartedAt;
        std::chrono::steady_clock::time_point _armSpanCaptureStartedAt;
        std::chrono::steady_clock::time_point _lastHeightSampleAt;
        std::chrono::steady_clock::time_point _lastArmSpanSampleAt;
        std::chrono::steady_clock::time_point _lastLegSlackUpdate = std::chrono::steady_clock::now();

        // configuration UI
        std::shared_ptr<vrui::UIContainer> _configUI;

        std::shared_ptr<vrui::UIToggleGroupContainer> _row2Container;
        std::shared_ptr<vrui::UIWidget> _noneMsg;
        std::shared_ptr<vrui::UIWidget> _heightMsg;
        std::shared_ptr<vrui::UIWidget> _forwardMsg;
        std::shared_ptr<vrui::UIWidget> _armsLengthMsg;
        std::shared_ptr<vrui::UIWidget> _vrScaleMsg;
    };
}
