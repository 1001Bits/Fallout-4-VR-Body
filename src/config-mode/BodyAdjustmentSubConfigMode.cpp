#include "BodyAdjustmentSubConfigMode.h"

#include <algorithm>
#include <cmath>

#include "Config.h"
#include "f4vr/PlayerNodes.h"
#include "skeleton/HandPose.h"
#include "skeleton/Skeleton.h"
#include "utils.h"
#include "vrui/UIButton.h"
#include "vrui/UIManager.h"
#include "vrui/UIToggleGroupContainer.h"
#include "vrui/UIWidget.h"

using namespace vrui;
using namespace common;

namespace
{
    void updateVRScaleGameConfig()
    {
        const auto set = RE::GetINISetting("fVrScale:VR");
        set->SetFloat(frik::g_config.fVrScale);
    }

    frik::calibration::HmdPivotCalibratorSettings getHmdPivotCalibratorSettings()
    {
        frik::calibration::HmdPivotCalibratorSettings settings;
        settings.retainedDurationSeconds = frik::g_config.hmdPivotCalibrationDuration;
        return settings;
    }

    float median(std::vector<float> values)
    {
        if (values.empty()) {
            return 0.0f;
        }
        const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
        std::nth_element(values.begin(), middle, values.end());
        auto value = *middle;
        if (values.size() % 2 == 0) {
            value = (*std::max_element(values.begin(), middle) + value) * 0.5f;
        }
        return value;
    }

    std::vector<float> rejectScalarOutliers(const std::vector<float>& samples)
    {
        const auto center = median(samples);
        std::vector<float> deviations;
        deviations.reserve(samples.size());
        for (const auto sample : samples) {
            deviations.push_back(std::abs(sample - center));
        }
        const auto threshold = (std::max)(0.5f, 3.5f * 1.4826f * median(std::move(deviations)));

        std::vector<float> inliers;
        inliers.reserve(samples.size());
        for (const auto sample : samples) {
            if (std::abs(sample - center) <= threshold) {
                inliers.push_back(sample);
            }
        }
        return inliers;
    }
}

namespace frik
{
    BodyAdjustmentSubConfigMode::BodyAdjustmentSubConfigMode(const std::function<void()>& onClose) : _onClose(onClose), _hmdPivotCalibrator(getHmdPivotCalibratorSettings())
    {
        createConfigUI();
    }

    void BodyAdjustmentSubConfigMode::onFrameUpdate()
    {
        _configUI->setPosition(0, 0, f4vr::isNodeVisible(f4vr::getWeaponNode()) ? 6.0f : 0.0f);

        _noneMsg->setVisibility(_configTarget == BodyAdjustmentConfigTarget::None);
        _heightMsg->setVisibility(_configTarget == BodyAdjustmentConfigTarget::BodyHeight);
        _forwardMsg->setVisibility(_configTarget == BodyAdjustmentConfigTarget::BodyForwardOffset);
        _armsLengthMsg->setVisibility(_configTarget == BodyAdjustmentConfigTarget::BodyArmsLength);
        _vrScaleMsg->setVisibility(_configTarget == BodyAdjustmentConfigTarget::VRScale);

        handleAdjustment();
    }

    /**
     * Create all the config elements.
     */
    void BodyAdjustmentSubConfigMode::createConfigUI()
    {
        const auto playSeattedBtn = std::make_shared<UIToggleButton>("FRIK\\UI_Main_Config\\btn_play_seated.nif");
        playSeattedBtn->setToggleState(g_config.isPlayingSeated);
        playSeattedBtn->setOnToggleHandler([this](UIWidget*, const bool enabled) { togglePlayingSeated(enabled); });

        const auto hideHeadBtn = std::make_shared<UIToggleButton>("FRIK\\UI_Main_Config\\btn_hide_head.nif");
        hideHeadBtn->setToggleState(g_config.hideHeadEquipment);
        hideHeadBtn->setOnToggleHandler([this](UIWidget*, const bool enabled) { toggleHideHeadEquipment(enabled); });

        const auto row1Container = std::make_shared<UIContainer>("Row1", UIContainerLayout::HorizontalCenter, 0.5f);
        row1Container->addElement(playSeattedBtn);
        row1Container->addElement(hideHeadBtn);

        const auto heightToggleBtn = std::make_shared<UIToggleButton>("FRIK\\UI_Main_Config\\btn_body_vertical.nif");
        heightToggleBtn->setOnToggleHandler([this](UIWidget*, const bool enabled) {
            if (enabled) {
                _configTarget = BodyAdjustmentConfigTarget::BodyHeight;
                beginHmdPivotCalibration();
            }
        });

        const auto forwardToggleBtn = std::make_shared<UIToggleButton>("FRIK\\UI_Main_Config\\btn_body_forward.nif");
        forwardToggleBtn->setOnToggleHandler([this](UIWidget*, bool) { _configTarget = BodyAdjustmentConfigTarget::BodyForwardOffset; });

        const auto armsLengthToggleBtn = std::make_shared<UIToggleButton>("FRIK\\UI_Main_Config\\btn_arms_length.nif");
        armsLengthToggleBtn->setOnToggleHandler([this](UIWidget*, const bool enabled) {
            if (enabled) {
                _configTarget = BodyAdjustmentConfigTarget::BodyArmsLength;
                beginArmSpanCalibration();
            }
        });

        const auto vrScaleToggleBtn = std::make_shared<UIToggleButton>("FRIK\\UI_Main_Config\\btn_vr_scale.nif");
        vrScaleToggleBtn->setOnToggleHandler([this](UIWidget*, bool) { _configTarget = BodyAdjustmentConfigTarget::VRScale; });

        _row2Container = std::make_shared<UIToggleGroupContainer>("Row2", UIContainerLayout::HorizontalCenter, 0.3f);
        _row2Container->addElement(heightToggleBtn);
        _row2Container->addElement(forwardToggleBtn);
        _row2Container->addElement(armsLengthToggleBtn);
        _row2Container->addElement(vrScaleToggleBtn);

        const auto saveBtn = std::make_shared<UIButton>("FRIK\\UI_Common\\btn_save.nif");
        saveBtn->setOnPressHandler([this](UIWidget*) { saveConfig(); });

        const auto resetBtn = std::make_shared<UIButton>("FRIK\\UI_Common\\btn_reset.nif");
        resetBtn->setOnPressHandler([this](UIWidget*) { resetConfig(); });

        const auto exitBtn = std::make_shared<UIButton>("FRIK\\UI_Common\\btn_back.nif");
        exitBtn->setOnPressHandler([this](UIWidget*) { closeConfig(); });

        const auto row3Container = std::make_shared<UIContainer>("Row3", UIContainerLayout::HorizontalCenter, 0.3f);
        row3Container->addElement(saveBtn);
        row3Container->addElement(resetBtn);
        row3Container->addElement(exitBtn);

        _noneMsg = std::make_shared<UIWidget>("FRIK\\UI_Main_Config\\msg_node_selected.nif");
        _heightMsg = std::make_shared<UIWidget>("FRIK\\UI_Main_Config\\msg_body_vertical.nif");
        _forwardMsg = std::make_shared<UIWidget>("FRIK\\UI_Main_Config\\msg_body_forward.nif");
        _armsLengthMsg = std::make_shared<UIWidget>("FRIK\\UI_Main_Config\\msg_arms_length.nif");
        _vrScaleMsg = std::make_shared<UIWidget>("FRIK\\UI_Main_Config\\msg_vr_scale.nif");
        const auto toggleSelfieMsg = std::make_shared<UIWidget>("FRIK\\UI_Main_Config\\msg_toggle_selfie.nif");

        const auto row4Container = std::make_shared<UIContainer>("Row4", UIContainerLayout::HorizontalCenter, 0.3f, 0.7f);
        row4Container->addElement(_noneMsg);
        row4Container->addElement(_heightMsg);
        row4Container->addElement(_forwardMsg);
        row4Container->addElement(_armsLengthMsg);
        row4Container->addElement(_vrScaleMsg);
        row4Container->addElement(toggleSelfieMsg);

        const auto header = std::make_shared<UIWidget>("FRIK\\UI_Main_Config\\title_body_adjust.nif", 0.5f);

        _configUI = std::make_shared<UIContainer>("BodyAdjustConfig", UIContainerLayout::VerticalUp, 0.35f, 1.8f);
        _configUI->addElement(row4Container);
        _configUI->addElement(row3Container);
        _configUI->addElement(_row2Container);
        _configUI->addElement(row1Container);
        _configUI->addElement(header);

        g_uiManager->attachPresetToPrimaryWandTop(_configUI, { 0, 0, 0 });
    }

    /**
     * Toggle seated play.
     */
    void BodyAdjustmentSubConfigMode::togglePlayingSeated(const bool seated)
    {
        g_config.saveIsPlayingSeated(seated);
        clearConfigTarget();
    }

    /**
     * Toggle head hiding on\off.
     * Show notification regarding not hiding in selfie mode for player not to be confused.
     */
    void BodyAdjustmentSubConfigMode::toggleHideHeadEquipment(const bool hide)
    {
        g_config.saveHideHeadEquipment(hide);
        clearConfigTarget();
        if (hide) {
            std::string msg = "Player head equipment is now hidden";
            if (g_config.selfieIgnoreHideFlags) {
                msg = msg + "\nNote: The head equipment is NOT hidden in selfie mode!";
            }
            f4vr::showNotification(msg);
        }
    }

    /**
     * On close of the body adjustment UI we clear unsaved config, cleanup, and close.
     */
    void BodyAdjustmentSubConfigMode::closeConfig()
    {
        _hmdPivotCalibrator.reset();
        _heightSamples.clear();
        _wristSpanSamples.clear();
        _wristSpanSampleTimes.clear();
        _hasBilateralLegSlack = false;

        // reload config to revert unsaved values
        g_config.loadIniOnly();

        // redo VR Scale if changed
        updateVRScaleGameConfig();

        // close the UI
        g_uiManager->detachElement(_configUI, true);
        _configUI.reset();

        // notify parent
        _onClose();
    }

    /**
     * Delegate adjustment to the right target.
     * Read the thumbstick value and change the taget values accordingly.
     */
    void BodyAdjustmentSubConfigMode::handleAdjustment()
    {
        switch (_configTarget) {
        case BodyAdjustmentConfigTarget::BodyHeight:
            captureHmdPivotSample();
            captureHeightSample();
            handleHeightAdjustment();
            break;
        case BodyAdjustmentConfigTarget::BodyForwardOffset:
            handleForwardAdjustment();
            break;
        case BodyAdjustmentConfigTarget::BodyArmsLength:
            captureArmSpanSample();
            handleArmsLengthAdjustment();
            break;
        case BodyAdjustmentConfigTarget::VRScale:
            handleVRScaleAdjustment();
            break;
        case BodyAdjustmentConfigTarget::None:
            break;
        }
    }

    void BodyAdjustmentSubConfigMode::handleHeightAdjustment()
    {
        const auto primAxisY = vrcf::VRControllers.getThumbstickValue(vrcf::Hand::Primary).y;
        g_config.setPlayerHMDOffsetUp(g_config.getPlayerHMDOffsetUp() + correctAdjustmentValue(primAxisY, 4));
        g_config.setPlayerBodyOffsetUp(g_config.getPlayerBodyOffsetUp() + 0.2f * correctAdjustmentValue(primAxisY, 4));

        const auto offAxisY = vrcf::VRControllers.getThumbstickValue(vrcf::Hand::Offhand).y;
        g_config.setPlayerBodyOffsetUp(g_config.getPlayerBodyOffsetUp() - correctAdjustmentValue(offAxisY, 4));
        g_config.setPlayerLegSlackAdjustOffset(g_config.getPlayerLegSlackAdjustOffset() + 3 * correctAdjustmentValue(offAxisY, 4));

        const auto now = std::chrono::steady_clock::now();
        const auto deltaTime = std::clamp(std::chrono::duration<float>(now - _lastLegSlackUpdate).count(), 0.0f, 0.05f);
        _lastLegSlackUpdate = now;

        constexpr float deadzone = 0.1f;
        float signedError = 0.0f;
        if (_hasBilateralLegSlack) {
            // Only move when both legs agree on which side of the target they
            // are. The nearer leg bounds the correction and prevents overshoot.
            if (_bilateralLegSlackLow > g_config.skeletonLegSlackTarget + deadzone) {
                signedError = -(_bilateralLegSlackLow - g_config.skeletonLegSlackTarget - deadzone);
            } else if (_bilateralLegSlackHigh < g_config.skeletonLegSlackTarget - deadzone) {
                signedError = g_config.skeletonLegSlackTarget - deadzone - _bilateralLegSlackHigh;
            }
            _hasBilateralLegSlack = false;
        }

        if (signedError != 0.0f && deltaTime > 0.0f && g_config.legSlackAutoAdjustRate > 0.0f) {
            const auto maximumStep = g_config.legSlackAutoAdjustRate * deltaTime;
            const auto adjustment = std::copysign((std::min)(std::abs(signedError), maximumStep), signedError);
            const auto adjustedOffset = std::clamp(g_config.getPlayerLegSlackAdjustOffset() + adjustment, -50.0f, 50.0f);
            logger::debug("Bilateral dt leg slack adjustment: low={}, high={}, dt={}, delta={}, offset={}", _bilateralLegSlackLow, _bilateralLegSlackHigh, deltaTime, adjustment,
                adjustedOffset);
            g_config.setPlayerLegSlackAdjustOffset(adjustedOffset);
        }
    }

    void BodyAdjustmentSubConfigMode::handleForwardAdjustment()
    {
        const auto axisY = vrcf::VRControllers.getThumbstickValue(vrcf::Hand::Primary).y;
        g_config.setPlayerBodyOffsetForward(g_config.getPlayerBodyOffsetForward() + correctAdjustmentValue(axisY, 4));
    }

    void BodyAdjustmentSubConfigMode::handleArmsLengthAdjustment()
    {
        const auto axisY = vrcf::VRControllers.getThumbstickValue(vrcf::Hand::Primary).y;
        const auto adjustment = correctAdjustmentValue(axisY, 5);

        // Keep the thumbstick inside the validated range. validateSolverCalibration
        // Config() resets an out-of-range length to the default rather than clamping
        // it, so without this an adjustment past the limit would silently snap arm
        // length back to 36.74 instead of simply stopping at the limit.
        constexpr float minimumArmLength = 15.0f;
        constexpr float maximumArmLength = 80.0f;
        g_config.armLength = std::clamp(g_config.armLength + adjustment, minimumArmLength, maximumArmLength);
        g_config.leftArmLength = std::clamp(g_config.leftArmLength + adjustment, minimumArmLength, maximumArmLength);
        g_config.rightArmLength = std::clamp(g_config.rightArmLength + adjustment, minimumArmLength, maximumArmLength);
    }

    void BodyAdjustmentSubConfigMode::handleVRScaleAdjustment()
    {
        const auto axisY = vrcf::VRControllers.getThumbstickValue(vrcf::Hand::Primary).y;
        g_config.fVrScale += correctAdjustmentValue(axisY, 5);
        updateVRScaleGameConfig();
    }

    void BodyAdjustmentSubConfigMode::saveConfig()
    {
        const auto pivotUpdated = _configTarget == BodyAdjustmentConfigTarget::BodyHeight && finishHmdPivotCalibration();
        const auto heightUpdated = _configTarget == BodyAdjustmentConfigTarget::BodyHeight && finishHeightCalibration();
        const auto armSpanUpdated = _configTarget == BodyAdjustmentConfigTarget::BodyArmsLength && finishArmSpanCalibration();
        if (_configTarget != BodyAdjustmentConfigTarget::BodyHeight) {
            if (_configTarget != BodyAdjustmentConfigTarget::BodyArmsLength || !armSpanUpdated) {
                f4vr::showNotification("Saving all body adjustment configs");
            }
        } else if (!pivotUpdated && !heightUpdated) {
            f4vr::showNotification("Saving manual body adjustments; automatic measurements were not changed");
        }
        g_config.save();
        clearConfigTarget();
    }

    /**
     * Reset the current config adjusting value to the default in the embedded config.
     * Use a small hack to load ONLY the embedded config into a temp config object.
     */
    void BodyAdjustmentSubConfigMode::resetConfig()
    {
        Config defaultConfig;
        defaultConfig.loadEmbeddedDefaultOnly();
        defaultConfig.isPlayingSeated = g_config.isPlayingSeated;

        switch (_configTarget) {
        case BodyAdjustmentConfigTarget::BodyHeight:
            f4vr::showNotification("Reset height body adjustment config");
            g_config.setPlayerHMDOffsetUp(defaultConfig.getPlayerHMDOffsetUp());
            g_config.setPlayerLegSlackAdjustOffset(defaultConfig.getPlayerLegSlackAdjustOffset());
            g_config.setPlayerBodyOffsetUp(defaultConfig.getPlayerBodyOffsetUp());
            g_config.resetHmdPivotOffset();
            g_config.calibratedPlayerHeight = defaultConfig.calibratedPlayerHeight;
            g_config.shoulderWidth = defaultConfig.shoulderWidth;
            _hmdPivotCalibrator.reset();
            _heightSamples.clear();
            break;
        case BodyAdjustmentConfigTarget::BodyForwardOffset:
            f4vr::showNotification("Reset forward body adjustment config");
            g_config.setPlayerBodyOffsetForward(defaultConfig.getPlayerBodyOffsetForward());
            break;
        case BodyAdjustmentConfigTarget::BodyArmsLength:
            f4vr::showNotification("Reset arms length body adjustment config");
            g_config.armLength = defaultConfig.armLength;
            g_config.leftArmLength = defaultConfig.leftArmLength;
            g_config.rightArmLength = defaultConfig.rightArmLength;
            _wristSpanSamples.clear();
            _wristSpanSampleTimes.clear();
            break;
        case BodyAdjustmentConfigTarget::VRScale:
            f4vr::showNotification("Reset VR Scale body adjustment config");
            g_config.fVrScale = defaultConfig.fVrScale;
            updateVRScaleGameConfig();
            break;
        case BodyAdjustmentConfigTarget::None:
            f4vr::showNotification("Please select body adjustment to reset");
            break;
        }

        clearConfigTarget();
    }

    void BodyAdjustmentSubConfigMode::clearConfigTarget()
    {
        _configTarget = BodyAdjustmentConfigTarget::None;
        _hasBilateralLegSlack = false;
        _row2Container->clearToggleState();
    }

    void BodyAdjustmentSubConfigMode::updateLegSlack(const float rightLegSlack, const float leftLegSlack)
    {
        if (!std::isfinite(rightLegSlack) || !std::isfinite(leftLegSlack)) {
            _hasBilateralLegSlack = false;
            return;
        }

        _bilateralLegSlackLow = (std::min)(rightLegSlack, leftLegSlack);
        _bilateralLegSlackHigh = (std::max)(rightLegSlack, leftLegSlack);
        _hasBilateralLegSlack = true;
    }

    void BodyAdjustmentSubConfigMode::beginHmdPivotCalibration()
    {
        _hmdPivotCalibrator = calibration::HmdPivotCalibrator(getHmdPivotCalibratorSettings());
        _heightSamples.clear();
        const auto now = std::chrono::steady_clock::now();
        _heightCaptureStartedAt = now;
        _lastHeightSampleAt = {};
        _lastLegSlackUpdate = now;
        _hasBilateralLegSlack = false;
        f4vr::showNotification("Height/HMD capture started\nStand upright, keep shoulders still; slowly look left/right/up/down, then press Save");
    }

    void BodyAdjustmentSubConfigMode::captureHmdPivotSample()
    {
        const auto playerNodes = f4vr::getPlayerNodes();
        if (!playerNodes || !playerNodes->HmdNode) {
            return;
        }

        const auto& world = playerNodes->HmdNode->world;
        calibration::HmdPoseSample sample;
        sample.position = { world.translate.x, world.translate.y, world.translate.z };
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t column = 0; column < 3; ++column) {
                // Runtime NiMatrix world rotation maps world -> local; the
                // calibration equation and correction API use local -> world.
                sample.rotation(row, column) = world.rotate.entry[column][row];
            }
        }
        sample.worldScale = world.scale;
        sample.timestampSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();

        if (_hmdPivotCalibrator.addSample(sample) == calibration::SampleStatus::ResetAfterTrackingDiscontinuity) {
            f4vr::showNotification("Tracking recentered; HMD pivot capture restarted");
        }
    }

    bool BodyAdjustmentSubConfigMode::finishHmdPivotCalibration()
    {
        const auto result = _hmdPivotCalibrator.solve();
        if (!result.succeeded()) {
            logger::info("HMD pivot calibration rejected: {} (samples={}, inliers={}, duration={}, span={}, condition={}, rmse={})",
                calibration::describeCalibrationFailure(result.failure), result.sampleCount, result.inlierCount, result.durationSeconds, result.orientationSpanRadians,
                result.conditionNumber, result.residualRmse);
            f4vr::showNotification(std::string("HMD pivot not updated: ") + calibration::describeCalibrationFailure(result.failure));
            return false;
        }

        g_config.setHmdPivotOffset(static_cast<float>(result.pivotToHmdOffset.x), static_cast<float>(result.pivotToHmdOffset.y), static_cast<float>(result.pivotToHmdOffset.z));
        g_config.enableHmdPivotCorrection = true;
        logger::info("HMD pivot calibrated: offset=({}, {}, {}), samples={}/{}, condition={}, rmse={}", result.pivotToHmdOffset.x, result.pivotToHmdOffset.y,
            result.pivotToHmdOffset.z, result.inlierCount, result.sampleCount, result.conditionNumber, result.residualRmse);
        f4vr::showNotification("HMD pivot calibrated and saved with body adjustments");
        return true;
    }

    void BodyAdjustmentSubConfigMode::captureHeightSample()
    {
        if (g_config.isPlayingSeated) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (_lastHeightSampleAt.time_since_epoch().count() != 0 && now - _lastHeightSampleAt < std::chrono::milliseconds(30)) {
            return;
        }
        const auto nodes = f4vr::getPlayerNodes();
        if (!nodes || !nodes->playerworldnode || !nodes->UprightHmdNode || !nodes->HmdNode) {
            return;
        }

        // FRIK's UprightHmdNode local Z is the established unrotated playspace
        // height (the legacy default is ~120.48). It avoids world/root scale
        // and translation leaking back into this solver-only measurement.
        const auto rawHeight = nodes->UprightHmdNode->local.translate.z;
        const auto& hmdWorld = nodes->HmdNode->world;
        const auto& playerWorld = nodes->playerworldnode->world;
        if (!std::isfinite(rawHeight) || rawHeight < 60.0f || rawHeight > 250.0f || !std::isfinite(hmdWorld.scale) || !std::isfinite(playerWorld.scale) ||
            std::abs(hmdWorld.scale) <= 0.0001f || std::abs(playerWorld.scale) <= 0.0001f) {
            return;
        }

        // Cache the linear contribution of every candidate tracker-local
        // lever component. Save recomputes height after the pivot fitter has
        // produced its final offset, keeping pure head rotation out of height.
        const auto verticalPivotCoefficient = [&](const RE::NiPoint3& localAxis) {
            const auto leverWorld = hmdWorld.rotate.Transpose() * (localAxis * hmdWorld.scale);
            const auto correctionPlayer = playerWorld.rotate * ((leverWorld * -1.0f) / playerWorld.scale);
            return correctionPlayer.z;
        };

        HeightCalibrationSample sample;
        sample.rawUprightHeight = rawHeight;
        sample.pivotCoefficientX = verticalPivotCoefficient(RE::NiPoint3(1.0f, 0.0f, 0.0f));
        sample.pivotCoefficientY = verticalPivotCoefficient(RE::NiPoint3(0.0f, 1.0f, 0.0f));
        sample.pivotCoefficientZ = verticalPivotCoefficient(RE::NiPoint3(0.0f, 0.0f, 1.0f));
        sample.hmdToPlayerScale = hmdWorld.scale / playerWorld.scale;
        if (!std::isfinite(sample.pivotCoefficientX) || !std::isfinite(sample.pivotCoefficientY) || !std::isfinite(sample.pivotCoefficientZ) ||
            !std::isfinite(sample.hmdToPlayerScale)) {
            return;
        }

        if (_heightSamples.size() >= 600) {
            _heightSamples.erase(_heightSamples.begin());
        }
        _heightSamples.push_back(sample);
        _lastHeightSampleAt = now;
    }

    void BodyAdjustmentSubConfigMode::beginArmSpanCalibration()
    {
        _wristSpanSamples.clear();
        _wristSpanSampleTimes.clear();
        const auto now = std::chrono::steady_clock::now();
        _armSpanCaptureStartedAt = now;
        _lastArmSpanSampleAt = {};
        f4vr::showNotification("Arm-span capture started\nStand upright and hold both arms straight out horizontally, then press Save");
    }

    void BodyAdjustmentSubConfigMode::captureArmSpanSample()
    {
        if (g_config.isPlayingSeated) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (_lastArmSpanSampleAt.time_since_epoch().count() != 0 && now - _lastArmSpanSampleAt < std::chrono::milliseconds(30)) {
            return;
        }
        const auto nodes = f4vr::getPlayerNodes();
        if (!nodes || !nodes->playerworldnode || !nodes->UprightHmdNode || !nodes->primaryWandNode || !nodes->SecondaryWandNode) {
            return;
        }
        const auto& playerWorld = nodes->playerworldnode->world;
        if (!std::isfinite(playerWorld.scale) || std::abs(playerWorld.scale) <= 0.0001f) {
            return;
        }
        const auto toPlayerSpace = [&](const RE::NiPoint3& worldPosition) { return playerWorld.rotate * ((worldPosition - playerWorld.translate) / playerWorld.scale); };
        const auto primary = toPlayerSpace(nodes->primaryWandNode->world.translate);
        const auto secondary = toPlayerSpace(nodes->SecondaryWandNode->world.translate);
        const auto uprightHmd = toPlayerSpace(nodes->UprightHmdNode->world.translate);
        const auto hmdZ = uprightHmd.z;
        const auto meanHandZ = (primary.z + secondary.z) * 0.5f;
        const auto delta = primary - secondary;
        const auto span = std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);

        // A near-horizontal T-pose is required; arbitrary controller positions
        // must not silently become body dimensions.
        if (std::isfinite(span) && std::abs(primary.z - secondary.z) <= 8.0f && meanHandZ <= hmdZ + 5.0f && meanHandZ >= hmdZ - 35.0f && span >= 40.0f && span <= 180.0f) {
            if (_wristSpanSamples.size() >= 600) {
                _wristSpanSamples.erase(_wristSpanSamples.begin());
                _wristSpanSampleTimes.erase(_wristSpanSampleTimes.begin());
            }
            _wristSpanSamples.push_back(span);
            _wristSpanSampleTimes.push_back(now);
            _lastArmSpanSampleAt = now;
        }
    }

    bool BodyAdjustmentSubConfigMode::finishHeightCalibration()
    {
        if (g_config.isPlayingSeated) {
            f4vr::showNotification("Solver height unchanged: standing capture is required");
            return false;
        }

        const auto captureDuration = std::chrono::duration<float>(std::chrono::steady_clock::now() - _heightCaptureStartedAt).count();
        std::vector<float> neutralHeightSamples;
        neutralHeightSamples.reserve(_heightSamples.size());
        const auto hmdPlacementOffset = Skeleton::getAdjustedPlayerHMDOffset();
        for (const auto& sample : _heightSamples) {
            auto neutralHeight = sample.rawUprightHeight + hmdPlacementOffset;
            if (g_config.enableHmdPivotCorrection) {
                neutralHeight += sample.pivotCoefficientX * g_config.hmdPivotOffsetX + sample.pivotCoefficientY * g_config.hmdPivotOffsetY +
                    sample.pivotCoefficientZ * g_config.hmdPivotOffsetZ + sample.hmdToPlayerScale * g_config.hmdPivotOffsetZ;
            }
            if (std::isfinite(neutralHeight)) {
                neutralHeightSamples.push_back(neutralHeight);
            }
        }
        const auto heightInliers = rejectScalarOutliers(neutralHeightSamples);
        if (captureDuration < 1.5f || heightInliers.size() < 45) {
            f4vr::showNotification("Solver height unchanged: stand upright for a longer capture");
            return false;
        }
        // The final pivot fit removes each sample's rotated tracker lever, then
        // adds its neutral upright Z lever back. Runtime subtracts that neutral
        // lever from this reference, so rotation alone cannot change posture.
        const auto measuredHeight = median(heightInliers);
        const auto measuredShoulderWidth = std::clamp(DEFAULT_SHOULDER_WIDTH * measuredHeight / DEFAULT_CAMERA_HEIGHT, 15.0f, 70.0f);
        if (!std::isfinite(measuredHeight) || measuredHeight < 60.0f || measuredHeight > 250.0f || !std::isfinite(measuredShoulderWidth)) {
            f4vr::showNotification("Solver height unchanged: fitted dimensions were outside safe limits");
            return false;
        }

        g_config.calibratedPlayerHeight = measuredHeight;
        g_config.shoulderWidth = measuredShoulderWidth;
        logger::info("Solver height calibrated: height={}, shoulderWidth={}", measuredHeight, measuredShoulderWidth);
        f4vr::showNotification("Solver height and normalized shoulder width calibrated");
        return true;
    }

    bool BodyAdjustmentSubConfigMode::finishArmSpanCalibration()
    {
        if (g_config.isPlayingSeated) {
            f4vr::showNotification("Solver arm lengths unchanged: standing T-pose capture is required");
            return false;
        }
        const auto captureDuration = std::chrono::duration<float>(std::chrono::steady_clock::now() - _armSpanCaptureStartedAt).count();
        if (captureDuration < 1.0f || _wristSpanSamples.size() < 30 || _wristSpanSamples.size() != _wristSpanSampleTimes.size()) {
            f4vr::showNotification("Solver arm lengths unchanged: hold both arms out horizontally");
            return false;
        }
        auto sortedSpans = _wristSpanSamples;
        std::sort(sortedSpans.begin(), sortedSpans.end());
        const auto percentileIndex = static_cast<std::size_t>(0.9 * static_cast<double>(sortedSpans.size() - 1));
        const auto extendedSpan = sortedSpans[percentileIndex];
        const auto minimumExtendedSpan = g_config.calibratedPlayerHeight * 0.72f;
        const auto maximumExtendedSpan = g_config.calibratedPlayerHeight * 1.25f;
        if (extendedSpan < minimumExtendedSpan || extendedSpan > maximumExtendedSpan) {
            f4vr::showNotification("Solver arm lengths unchanged: controller span was not an outstretched pose");
            return false;
        }

        const auto plateauTolerance = (std::max)(2.0f, extendedSpan * 0.03f);
        std::vector<float> plateau;
        std::vector<float> currentPlateau;
        auto currentPlateauStartedAt = _wristSpanSampleTimes.front();
        auto previousAcceptedAt = currentPlateauStartedAt;
        float bestPlateauDuration = 0.0f;
        for (std::size_t index = 0; index < _wristSpanSamples.size(); ++index) {
            if (std::abs(_wristSpanSamples[index] - extendedSpan) > plateauTolerance) {
                currentPlateau.clear();
                continue;
            }

            const auto sampleTime = _wristSpanSampleTimes[index];
            if (currentPlateau.empty() || sampleTime - previousAcceptedAt > std::chrono::milliseconds(100)) {
                currentPlateau.clear();
                currentPlateauStartedAt = sampleTime;
            }
            currentPlateau.push_back(_wristSpanSamples[index]);
            previousAcceptedAt = sampleTime;
            const auto duration = std::chrono::duration<float>(sampleTime - currentPlateauStartedAt).count();
            if (currentPlateau.size() > plateau.size() || (currentPlateau.size() == plateau.size() && duration > bestPlateauDuration)) {
                plateau = currentPlateau;
                bestPlateauDuration = duration;
            }
        }
        if (plateau.size() < 30 || bestPlateauDuration < 0.9f) {
            f4vr::showNotification("Solver arm lengths unchanged: hold the outstretched pose steadily");
            return false;
        }

        const auto measuredSpan = median(plateau);
        const auto measuredShoulderWidth = DEFAULT_SHOULDER_WIDTH * g_config.calibratedPlayerHeight / DEFAULT_CAMERA_HEIGHT;
        const auto symmetricArmLength = (measuredSpan - measuredShoulderWidth) * 0.5f;
        if (!std::isfinite(symmetricArmLength) || measuredShoulderWidth < 15.0f || measuredShoulderWidth > 70.0f || symmetricArmLength < 15.0f || symmetricArmLength > 80.0f) {
            f4vr::showNotification("Solver arm lengths unchanged: fitted dimensions were outside safe limits");
            return false;
        }

        g_config.shoulderWidth = measuredShoulderWidth;
        // A wrist-span observation cannot identify asymmetric arms, so use one
        // explicit symmetric estimate instead of inventing left/right lengths.
        g_config.leftArmLength = symmetricArmLength;
        g_config.rightArmLength = symmetricArmLength;
        g_config.armLength = symmetricArmLength;
        logger::info("Solver arm span calibrated: wristSpan={}, shoulderWidth={}, armLength={}", measuredSpan, measuredShoulderWidth, symmetricArmLength);
        f4vr::showNotification("Solver shoulder width and symmetric arm lengths calibrated");
        return true;
    }
}
