#include "Skeleton.h"

#include <array>
#include <cmath>
#include <cstring>
#include <span>
#include <utility>

#include "Config.h"
#include "FRIK.h"
#include "HandPose.h"
#include "common/MatrixUtils.h"
#include "common/Quaternion.h"
#include "f4vr/BSFlattenedBoneTree.h"
#include "f4vr/F4VRSkelly.h"
#include "f4vr/F4VRUtils.h"
#include "ik/ArmIK.h"
#include "ik/GaitSupport.h"
#include "ik/TorsoIK.h"
#include "world/GroundQuery.h"
#include "vrcf/VRControllersManager.h"

using namespace common;
using namespace f4vr;
using namespace vrcf;

namespace
{
    constexpr float kVectorEpsilon = 0.0001f;
    constexpr float kMaximumTrackedPosition = 1000000.0f;
    constexpr float kTrackingDiscontinuityDistance = 100.0f;
    constexpr float kDirectionChangeDelaySeconds = 2.0f / 90.0f;
    constexpr float kStopBlendSeconds = 0.18f;
    // How far a planted foot may drift from its rest position before it is re-planted
    // regardless of turn, so slow drift cannot stretch a leg past its reach.
    constexpr float kStanceFootHoldLimit = 20.0f;
    constexpr float kStepRetargetDeceleration = -20.0f * 90.0f;

    /**
     * Set the cull flag on every descendant matching one of the given names using a
     * single traversal.  Searching per name walked the same subtree once per name.
     */
    void hideNodesByName(RE::NiAVObject* node, const std::span<const char* const> names)
    {
        if (!node) {
            return;
        }

        const auto* nodeName = node->name.c_str();
        for (const auto* name : names) {
            if (_stricmp(name, nodeName) == 0) {
                node->flags.flags |= 0x1; // first bit is the cull flag, so the node is hidden
                break;
            }
        }

        if (const auto niNode = node->IsNode()) {
            for (const auto& child : niNode->children) {
                if (child) {
                    hideNodesByName(child.get(), names);
                }
            }
        }
    }

    frik::ik::Vec3 toIKVector(const RE::NiPoint3& value) { return { value.x, value.y, value.z }; }

    RE::NiPoint3 toNiPoint(const frik::ik::Vec3& value) { return { value.x, value.y, value.z }; }

    bool isFinite(const RE::NiPoint3& value)
    {
        return frik::ik::isFinite(toIKVector(value)) && std::abs(value.x) < kMaximumTrackedPosition && std::abs(value.y) < kMaximumTrackedPosition &&
            std::abs(value.z) < kMaximumTrackedPosition;
    }

    bool isFinite(const RE::NiMatrix3& value)
    {
        for (std::uint32_t row = 0; row < 3; ++row) {
            for (std::uint32_t column = 0; column < 3; ++column) {
                if (!frik::ik::isFinite(value.entry[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

    bool isFinite(const RE::NiTransform& transform)
    {
        if (!isFinite(transform.translate) || !isFinite(transform.rotate) || !std::isfinite(transform.scale) || transform.scale <= kVectorEpsilon) {
            return false;
        }

        // Reject a finite but collapsed/corrupt orientation.  Thresholds are
        // intentionally loose because engine matrices can contain small drift.
        const RE::NiPoint3 basisX(transform.rotate.entry[0][0], transform.rotate.entry[0][1], transform.rotate.entry[0][2]);
        const RE::NiPoint3 basisY(transform.rotate.entry[1][0], transform.rotate.entry[1][1], transform.rotate.entry[1][2]);
        const RE::NiPoint3 basisZ(transform.rotate.entry[2][0], transform.rotate.entry[2][1], transform.rotate.entry[2][2]);
        const float lengthX = MatrixUtils::vec3Len(basisX);
        const float lengthY = MatrixUtils::vec3Len(basisY);
        const float lengthZ = MatrixUtils::vec3Len(basisZ);
        return lengthX > 0.5f && lengthX < 1.5f && lengthY > 0.5f && lengthY < 1.5f && lengthZ > 0.5f && lengthZ < 1.5f;
    }

    bool isNearlyOrthonormal(const RE::NiMatrix3& value)
    {
        if (!isFinite(value)) {
            return false;
        }

        const RE::NiPoint3 rowX(value.entry[0][0], value.entry[0][1], value.entry[0][2]);
        const RE::NiPoint3 rowY(value.entry[1][0], value.entry[1][1], value.entry[1][2]);
        const RE::NiPoint3 rowZ(value.entry[2][0], value.entry[2][1], value.entry[2][2]);
        constexpr float lengthTolerance = 0.03f;
        constexpr float dotTolerance = 0.03f;
        constexpr float determinantTolerance = 0.06f;
        const float determinant = MatrixUtils::vec3Dot(rowX, MatrixUtils::vec3Cross(rowY, rowZ));
        return std::abs(MatrixUtils::vec3Len(rowX) - 1.0f) <= lengthTolerance && std::abs(MatrixUtils::vec3Len(rowY) - 1.0f) <= lengthTolerance &&
            std::abs(MatrixUtils::vec3Len(rowZ) - 1.0f) <= lengthTolerance && std::abs(MatrixUtils::vec3Dot(rowX, rowY)) <= dotTolerance &&
            std::abs(MatrixUtils::vec3Dot(rowX, rowZ)) <= dotTolerance && std::abs(MatrixUtils::vec3Dot(rowY, rowZ)) <= dotTolerance && std::isfinite(determinant) &&
            std::abs(determinant - 1.0f) <= determinantTolerance;
    }

    float maximumMatrixDifference(const RE::NiMatrix3& lhs, const RE::NiMatrix3& rhs)
    {
        float maximumDifference = 0.0f;
        for (std::uint32_t row = 0; row < 3; ++row) {
            for (std::uint32_t column = 0; column < 3; ++column) {
                maximumDifference = (std::max)(maximumDifference, std::abs(lhs.entry[row][column] - rhs.entry[row][column]));
            }
        }
        return maximumDifference;
    }

    bool tryNormalize(const RE::NiPoint3& input, RE::NiPoint3& output)
    {
        const float length = MatrixUtils::vec3Len(input);
        if (!std::isfinite(length) || length <= kVectorEpsilon) {
            return false;
        }

        output = input / length;
        return isFinite(output);
    }

    RE::NiPoint3 safeNormalize(const RE::NiPoint3& value, const RE::NiPoint3& fallback = { 1.0f, 0.0f, 0.0f })
    {
        RE::NiPoint3 normalized;
        if (tryNormalize(value, normalized)) {
            return normalized;
        }
        if (tryNormalize(fallback, normalized)) {
            return normalized;
        }
        return RE::NiPoint3(1.0f, 0.0f, 0.0f);
    }

    bool tryGetRotationFromVectors(const RE::NiPoint3& toVector, const RE::NiPoint3& fromVector, RE::NiMatrix3& result)
    {
        RE::NiPoint3 to;
        RE::NiPoint3 from;
        if (!tryNormalize(toVector, to) || !tryNormalize(fromVector, from)) {
            return false;
        }

        const float dot = std::clamp(MatrixUtils::vec3Dot(from, to), -1.0f, 1.0f);
        if (dot >= 0.99999f) {
            result = MatrixUtils::getIdentityMatrix();
            return true;
        }

        RE::NiPoint3 axis = MatrixUtils::vec3Cross(to, from);
        if (!tryNormalize(axis, axis)) {
            // Antiparallel vectors have infinitely many valid axes. Pick a
            // deterministic cardinal axis least parallel to the source.
            const RE::NiPoint3 cardinal = std::abs(from.x) < 0.8f ? RE::NiPoint3(1.0f, 0.0f, 0.0f) : RE::NiPoint3(0.0f, 1.0f, 0.0f);
            if (!tryNormalize(MatrixUtils::vec3Cross(cardinal, from), axis)) {
                return false;
            }
        }

        const float angle = std::acos(dot);
        const float cosine = std::cos(angle);
        const float sine = std::sin(angle);
        const float oneMinusCosine = 1.0f - cosine;
        // Assign rows directly. MatrixUtils::getMatrix accepts column-major
        // arguments and would transpose this world-to-local delta.
        result.entry[0][0] = cosine + axis.x * axis.x * oneMinusCosine;
        result.entry[0][1] = -axis.z * sine + axis.x * axis.y * oneMinusCosine;
        result.entry[0][2] = axis.y * sine + axis.x * axis.z * oneMinusCosine;
        result.entry[1][0] = axis.z * sine + axis.y * axis.x * oneMinusCosine;
        result.entry[1][1] = cosine + axis.y * axis.y * oneMinusCosine;
        result.entry[1][2] = -axis.x * sine + axis.y * axis.z * oneMinusCosine;
        result.entry[2][0] = -axis.y * sine + axis.z * axis.x * oneMinusCosine;
        result.entry[2][1] = axis.x * sine + axis.z * axis.y * oneMinusCosine;
        result.entry[2][2] = cosine + axis.z * axis.z * oneMinusCosine;
        return isNearlyOrthonormal(result);
    }

    template <class Function>
    class ScopeExit
    {
    public:
        explicit ScopeExit(Function function) : _function(std::move(function)) {}

        ScopeExit(const ScopeExit&) = delete;
        ScopeExit& operator=(const ScopeExit&) = delete;

        ~ScopeExit()
        {
            if (_active) {
                _function();
            }
        }

        void release() noexcept { _active = false; }

    private:
        Function _function;
        bool _active = true;
    };

    bool tryNormalizePlanar(const RE::NiPoint3& input, RE::NiPoint3& output) { return tryNormalize(RE::NiPoint3(input.x, input.y, 0.0f), output); }

    bool tryLawOfCosinesAngle(const float adjacentA, const float adjacentB, const float opposite, float& angle)
    {
        if (!std::isfinite(adjacentA) || !std::isfinite(adjacentB) || !std::isfinite(opposite) || adjacentA <= kVectorEpsilon || adjacentB <= kVectorEpsilon || opposite < 0.0f) {
            return false;
        }

        const float denominator = 2.0f * adjacentA * adjacentB;
        if (!std::isfinite(denominator) || denominator <= kVectorEpsilon) {
            return false;
        }

        const float cosine = std::clamp((adjacentA * adjacentA + adjacentB * adjacentB - opposite * opposite) / denominator, -1.0f, 1.0f);
        angle = std::acos(cosine);
        return std::isfinite(angle);
    }

    bool approximatelyEqual(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs) { return MatrixUtils::vec3Len(lhs - rhs) <= kVectorEpsilon; }

    std::optional<bool> queryOpenVrHmdPoseValidity()
    {
        const auto vrSystem = vr::VRSystem();
        if (!vrSystem) {
            return std::nullopt;
        }

        static_assert(vr::k_unTrackedDeviceIndex_Hmd == 0);
        vr::TrackedDevicePose_t hmdPose{};
        vrSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0f, &hmdPose, 1);
        return hmdPose.bPoseIsValid;
    }

    float frameRateIndependentRetention(const float legacyRetention, const float deltaTime)
    {
        if (!std::isfinite(legacyRetention)) {
            return 0.0f;
        }

        // Existing damping values were tuned as per-frame retention at 90 Hz.
        // Exponentiation preserves that response at 90 Hz while giving the
        // same time constant at other headset refresh rates.
        const float retention = std::clamp(legacyRetention, 0.0f, 1.0f);
        return std::pow(retention, (std::max)(deltaTime, 0.0f) * 90.0f);
    }

    /**
     * Hack to handle comfort sneak affecting the height of the player without real-world body change.
     * By setting static body pitch the body position doesn't change, making it easier to handle skeleton
     * related things like Virtual Holsters.
     */
    bool isComfortSneakHackEnabled() { return frik::g_config.comfortSneakHackStaticBodyPitchAngle > 0 && isComfortSneakMode() && isPlayerSneaking(); }
}

namespace frik
{
    /**
     * Get the player camera height offset adjusted for power armor, sneaking, and dynamic height from external API.
     * The height needs to be adjusted for comfort sneaking because the player physical height doesn't change but
     * the player avatar does. So the camera offset has to be reduced by the same amount as the game changes the height
     * which is 70% of the normal height.
     */
    float Skeleton::getAdjustedPlayerHMDOffset()
    {
        auto offset = g_config.getPlayerHMDOffsetUp() + g_frik.getDynamicCameraHeight();
        if (isComfortSneakMode() && isPlayerSneaking()) {
            offset *= _comfortSneakCameraOffsetAdjustment;
        }
        return offset;
    }

    /**
     * Initialize all the skeleton nodes for quick access during frame update.
     * Setup known defaults where relevant.
     */
    void Skeleton::initializeNodes()
    {
        QueryPerformanceFrequency(&_freqCounter);
        QueryPerformanceCounter(&_timer);

        _prevSpeed = 0.0;

        _playerNodes = getPlayerNodes();

        const auto fpSkeleton = getFirstPersonSkeleton();
        if (fpSkeleton) {
            _rightHand = findNode(fpSkeleton, "RArm_Hand");
            _leftHand = findNode(fpSkeleton, "LArm_Hand");
        }
        if (_rightHand) {
            _rightHandPrevFrame = _rightHand->world;
        }
        if (_leftHand) {
            _leftHandPrevFrame = _leftHand->world;
        }

        _head = findNode(_root, "Head");
        _spine = findNode(_root, "SPINE2");
        _chest = findNode(_root, "Chest");
        _com = findNode(_root, "COM");
        _neck = findNode(_root, "Neck");
        _spine1 = findNode(_root, "SPINE1");
        _leftLeg.hip = findNode(_root, "LLeg_Thigh");
        _leftLeg.knee = findNode(_root, "LLeg_Calf");
        _leftLeg.foot = findNode(_root, "LLeg_Foot");
        _rightLeg.hip = findNode(_root, "RLeg_Thigh");
        _rightLeg.knee = findNode(_root, "RLeg_Calf");
        _rightLeg.foot = findNode(_root, "RLeg_Foot");

        // Setup Arms
        initArmsNodes();

        initSkeletonNodesDefaults();

        _handBones = handOpen;

        Skelly::initBoneTreeMap();

        setBodyLen();

        initHandPoses(_inPowerArmor);

        _comfortSneakCameraOffsetAdjustment = getIniSetting("fComfortSneakHeight:VR")->GetFloat();
    }

    void Skeleton::initArmsNodes()
    {
        const std::vector<std::pair<RE::BSFixedString, RE::NiAVObject**>> armNodes = { { "RArm_Collarbone", &_rightArm.shoulder }, { "RArm_UpperArm", &_rightArm.upper },
            { "RArm_UpperTwist1", &_rightArm.upperT1 }, { "RArm_ForeArm1", &_rightArm.forearm1 }, { "RArm_ForeArm2", &_rightArm.forearm2 },
            { "RArm_ForeArm3", &_rightArm.forearm3 }, { "RArm_Hand", &_rightArm.hand }, { "LArm_Collarbone", &_leftArm.shoulder }, { "LArm_UpperArm", &_leftArm.upper },
            { "LArm_UpperTwist1", &_leftArm.upperT1 }, { "LArm_ForeArm1", &_leftArm.forearm1 }, { "LArm_ForeArm2", &_leftArm.forearm2 }, { "LArm_ForeArm3", &_leftArm.forearm3 },
            { "LArm_Hand", &_leftArm.hand } };
        const auto commonNode = getCommonNode();
        for (const auto& [name, node] : armNodes) {
            *node = findAVObject(commonNode, name.c_str());
        }
    }

    /**
     * Setup default skeleton nodes collection for quick reset on every frame
     * instead of looking up the skeleton nodes every time.
     */
    void Skeleton::initSkeletonNodesDefaults()
    {
        const auto defaultBonesMap = _inPowerArmor ? _skeletonNodesDefaultTransformInPA : _skeletonNodesDefaultTransform;
        for (const auto& [boneName, defaultTransform] : defaultBonesMap) {
            if (auto node = findAVObject(_root, boneName)) {
                auto transform = node->local; // use node transform to keep scale
                transform.translate = defaultTransform.translate;
                transform.rotate = defaultTransform.rotate;
                _skeletonNodesToDefaultTransforms.emplace_back(node, transform);
            } else {
                logger::warn("Skeleton bone node not found for '{}'", boneName.c_str());
            }
        }
    }

    void Skeleton::setBodyLen()
    {
        const auto camera = findNode(_root, "Camera");
        if (camera && _com) {
            _torsoLen = MatrixUtils::vec3Len(camera->world.translate - _com->world.translate);
        }

        const auto pelvis = findNode(_root, "Pelvis");
        if (pelvis && _leftLeg.hip && _leftLeg.knee && _leftLeg.foot) {
            _legLen = MatrixUtils::vec3Len(_leftLeg.hip->world.translate - pelvis->world.translate);
            _legLen += MatrixUtils::vec3Len(_leftLeg.knee->world.translate - _leftLeg.hip->world.translate);
            _legLen += MatrixUtils::vec3Len(_leftLeg.foot->world.translate - _leftLeg.knee->world.translate);
        }
    }

    /**
     * Runs on every game frame to calculate and update the skeleton transform.
     */
    void Skeleton::onFrameUpdate()
    {
        setTime();

        if (!hasRequiredNodes()) {
            logger::sample("Cannot update IK: required skeleton/player nodes are unavailable");
            _trackingWasValid = false;
            resetMotionState();
            return;
        }

        // Sample translation and rotation from the same HMD transform.  All IK
        // consumers use the resulting virtual anatomical pivot for this frame.
        if (!sampleTrackedHeadPose()) {
            return;
        }

        logger::trace("Hide Wands...");
        setWandsVisibility(false, true);
        setWandsVisibility(false, false);

        if (g_config.checkDebugDumpDataOnceFor("bonemap")) {
            dumpAnimationBoneMap();
        }

        _solveLegsThisFrame = canUseProceduralLegs();

        logger::trace("Restore locals of skeleton");
        restoreNodesToDefault();
        updateDownFromRoot();

        const float neckYaw = getNeckYaw();
        const float neckPitch = getNeckPitch();

        // Historically the avatar root absorbed this entire yaw, which swung the
        // pelvis, legs, and feet whenever only the upper body turned.  Split it so
        // the spine can carry part of it while the chest still lands where the
        // root-only rotation put it.
        const auto torsoTwist = ik::distributeTorsoTwist(neckYaw * 0.7f, torsoTwistSettings());

        if (!g_config.hideHead || (g_frik.isSelfieModeOn() && g_config.selfieIgnoreHideFlags)) {
            logger::trace("Setup Head");
            setupHead(neckYaw, neckPitch);
        }

        logger::trace("Set body under HMD");
        setBodyUnderHMD(torsoTwist.root);
        updateDownFromRoot(); // Do world update now so that IK calculations have proper world reference

        // Now Set up body Posture and hook up the legs
        logger::trace("Set body posture...");
        setBodyPosture(neckPitch);
        applyTorsoTwist(torsoTwist);
        updateDownFromRoot(); // Do world update now so that IK calculations have proper world reference

        if (_solveLegsThisFrame) {
            logger::trace("Set knee posture...");
            setKneePos();
        }

        logger::trace("Set walk...");
        walk();

        if (_solveLegsThisFrame) {
            logger::trace("Set legs...");
            const auto rightLegSlack = setSingleLeg(false);
            const auto leftLegSlack = setSingleLeg(true);
            if (rightLegSlack && leftLegSlack) {
                BodyAdjustmentSubConfigMode::updateLegSlack(*rightLegSlack, *leftLegSlack);
            }
        }

        // Do another update before setting arms
        updateDownFromRoot(); // Do world update now so that IK calculations have proper world reference

        // do arm IK - Right then Left
        logger::trace("Set Arms...");
        handleLeftHandedWeaponNodesSwitch();
        setArms(false);
        setArms(true);
        updateDownFromRoot(); // Do world update now so that IK calculations have proper world reference

        // Misc stuff to show/hide things
        logger::trace("Pipboy and Weapons...");
        hide3rdPersonWeapon();
        hideFistHelpers();
        showHidePAHud();

        logger::trace("Cull geometry...");
        _cullGeometry.cullPlayerGeometry();

        // project body out in front of the camera for debug purposes
        logger::trace("Selfie Time");
        _selfieHandler.onFrameUpdate(_trackedHeadPose.pivot, _forwardDir);

        logger::trace("Operate hands...");
        setHandPose();

        if (g_frik.isInScopeMenu()) {
            hideHands();
        }

        if (_inPowerArmor) {
            fixArmor();
        }
    }

    void Skeleton::setTime()
    {
        _prevTime = _timer;
        QueryPerformanceFrequency(&_freqCounter);
        QueryPerformanceCounter(&_timer);
        _frameTime = static_cast<float>(_timer.QuadPart - _prevTime.QuadPart) / _freqCounter.QuadPart;
        _timeDiscontinuity = !std::isfinite(_frameTime) || _frameTime <= 0.0f || _frameTime > 0.1f;
        if (!std::isfinite(_frameTime) || _frameTime <= 0.0f) {
            _frameTime = 1.0f / 90.0f;
        }
        _frameTime = (std::min)(_frameTime, 0.1f);
    }

    bool Skeleton::sampleTrackedHeadPose()
    {
        RE::NiTransform rawPose;
        bool usedCameraFallback = false;

        const auto openVrPoseValid = queryOpenVrHmdPoseValidity();
        if (openVrPoseValid && !*openVrPoseValid) {
            if (_trackingWasValid) {
                logger::warn("OpenVR reports invalid HMD tracking; retaining the last valid IK pose");
            }
            _trackingWasValid = false;
            resetMotionState();
            return false;
        }

        if (_playerNodes->HmdNode && isFinite(_playerNodes->HmdNode->world)) {
            rawPose = _playerNodes->HmdNode->world;
        } else {
            const auto playerCamera = getPlayerCamera();
            if (!playerCamera || !playerCamera->cameraNode || !isFinite(playerCamera->cameraNode->world)) {
                if (_trackingWasValid) {
                    logger::warn("HMD tracking became invalid; retaining the last valid IK pose");
                } else {
                    logger::sample("Cannot update IK: no valid coherent HMD/camera transform");
                }
                _trackingWasValid = false;
                resetMotionState();
                return false;
            }

            rawPose = playerCamera->cameraNode->world;
            usedCameraFallback = true;
            logger::sample(3000, "HmdNode transform is invalid; using the complete camera transform for IK");
        }

        const auto playerCamera = getPlayerCamera();
        if (!usedCameraFallback && playerCamera && playerCamera->cameraNode && isFinite(playerCamera->cameraNode->world)) {
            const float cameraDelta = MatrixUtils::vec3Len(playerCamera->cameraNode->world.translate - rawPose.translate);
            if (std::isfinite(cameraDelta) && cameraDelta > 5.0f) {
                logger::sample(3000, "HMD and camera origins differ by {:.2f} units; IK is using the coherent HmdNode transform", cameraDelta);
            }
        }

        const RE::NiPoint3 configuredOffset(g_config.hmdPivotOffsetX, g_config.hmdPivotOffsetY, g_config.hmdPivotOffsetZ);
        bool correctionEnabled = g_config.enableHmdPivotCorrection;

        RE::NiPoint3 pivot = rawPose.translate;
        if (correctionEnabled) {
            if (!isFinite(configuredOffset)) {
                logger::sample(3000, "HMD pivot correction offset is invalid; using the raw HMD origin");
                correctionEnabled = false;
            } else {
                // NiMatrix world rotations are world-to-local.  Convert the
                // pivot->tracker lever arm to world space with Transpose, then
                // subtract it from the tracked origin to recover the pivot.
                const RE::NiPoint3 leverWorld = rawPose.rotate.Transpose() * (configuredOffset * rawPose.scale);
                pivot -= leverWorld;
            }
        }

        if (!isFinite(pivot)) {
            if (_trackingWasValid) {
                logger::warn("Corrected HMD pivot became invalid; retaining the last valid IK pose");
            }
            _trackingWasValid = false;
            resetMotionState();
            return false;
        }

        const bool correctionChanged =
            !_hasLastPivotConfig || correctionEnabled != _lastPivotCorrectionEnabled || (correctionEnabled && !approximatelyEqual(configuredOffset, _lastPivotOffset));
        const bool trackingSourceChanged = _hasLastTrackingSource && usedCameraFallback != _lastUsedCameraFallback;
        const bool trackingScaleChanged = _hasValidTrackedHeadPose && std::abs(rawPose.scale - _trackedHeadPose.raw.scale) > 0.01f;
        const bool solverCalibrationChanged = _hasLastSolverCalibration &&
            (std::abs(g_config.calibratedPlayerHeight - _lastCalibratedPlayerHeight) > 0.001f || std::abs(g_config.shoulderWidth - _lastShoulderWidth) > 0.001f ||
                std::abs(g_config.leftArmLength - _lastLeftArmLength) > 0.001f || std::abs(g_config.rightArmLength - _lastRightArmLength) > 0.001f);
        const bool trackingReacquired = !_trackingWasValid;
        const bool positionDiscontinuity = _hasValidTrackedHeadPose && MatrixUtils::vec3Len(pivot - _curentPosition) > kTrackingDiscontinuityDistance;

        _lastPosition = _hasValidTrackedHeadPose ? _curentPosition : pivot;
        _curentPosition = pivot;
        _trackedHeadPose.raw = rawPose;
        _trackedHeadPose.pivot = pivot;
        _hasValidTrackedHeadPose = true;
        _trackingWasValid = true;
        _lastPivotCorrectionEnabled = correctionEnabled;
        _lastPivotOffset = configuredOffset;
        _hasLastPivotConfig = true;
        _lastUsedCameraFallback = usedCameraFallback;
        _hasLastTrackingSource = true;
        _lastCalibratedPlayerHeight = g_config.calibratedPlayerHeight;
        _lastShoulderWidth = g_config.shoulderWidth;
        _lastLeftArmLength = g_config.leftArmLength;
        _lastRightArmLength = g_config.rightArmLength;
        _hasLastSolverCalibration = true;

        if (trackingReacquired || correctionChanged || trackingSourceChanged || trackingScaleChanged || solverCalibrationChanged || positionDiscontinuity || _timeDiscontinuity) {
            if (positionDiscontinuity) {
                logger::sample(3000, "HMD position discontinuity detected; resetting IK motion history");
            }
            if (trackingSourceChanged) {
                logger::sample(3000, "HMD tracking source changed; resetting IK motion history");
            }
            resetMotionState();
        }

        return true;
    }

    /**
     * One-shot dump answering whether Fallout 4 VR can supply animated legs.
     *
     * PlayerCharacter::PostUpdateAnimationGraphManager copies an animated pose from
     * the first-person bone tree onto the third-person body through
     * PlayerCharacter::boneMapping1stTo3rd, a BSTArray<BSTTuple<int,int>> of
     * {firstPersonIndex, thirdPersonIndex}. Reverse engineered from
     * Fallout4VR.exe 1.2.72: the array data sits at player+0x1238 and its size at
     * player+0x1248 (see docs/ANIMATION_DRIVEN_LEGS.md).
     *
     * Whether the legs are in that map decides whether an animation-driven gait is
     * possible at all, and the map is runtime data, so it cannot be read statically.
     * Enable by adding "bonemap" to sDebugDumpDataOnceNames in FRIK.ini.
     */
    void Skeleton::dumpAnimationBoneMap() const
    {
        const auto player = RE::PlayerCharacter::GetSingleton();
        if (!player || !_root) {
            logger::info("[BONEMAP] no player or root node");
            return;
        }

        const auto playerBase = reinterpret_cast<const std::byte*>(player);
        const auto mapEntries = *reinterpret_cast<const std::int32_t* const*>(playerBase + 0x1238);
        const auto mapCount = *reinterpret_cast<const std::uint32_t*>(playerBase + 0x1248);

        const auto thirdPerson = reinterpret_cast<BSFlattenedBoneTree*>(_root);
        const auto firstPerson = reinterpret_cast<BSFlattenedBoneTree*>(getFirstPersonSkeleton());

        logger::info("[BONEMAP] entries={} first-person tree={} bones={} third-person tree={} bones={}", mapCount, firstPerson ? "yes" : "no",
            firstPerson ? firstPerson->numTransforms : -1, thirdPerson ? "yes" : "no", thirdPerson ? thirdPerson->numTransforms : -1);

        // A malformed count would walk off the heap; 4096 is far above any skeleton.
        if (!mapEntries || mapCount == 0 || mapCount > 4096) {
            logger::info("[BONEMAP] map is empty or implausible - the sync is not populated");
            return;
        }

        const auto nameAt = [](const BSFlattenedBoneTree* tree, const std::int32_t index) -> const char* {
            if (!tree || !tree->transforms || index < 0 || index >= tree->numTransforms) {
                return "<out of range>";
            }
            const auto name = tree->transforms[index].name.c_str();
            return name ? name : "<null>";
        };

        int legEntries = 0;
        for (std::uint32_t entry = 0; entry < mapCount; ++entry) {
            const auto firstIndex = mapEntries[entry * 2];
            const auto thirdIndex = mapEntries[entry * 2 + 1];
            const auto thirdName = nameAt(thirdPerson, thirdIndex);
            logger::info("[BONEMAP] {:3}: 1st[{:3}]={} -> 3rd[{:3}]={}", entry, firstIndex, nameAt(firstPerson, firstIndex), thirdIndex, thirdName);
            const std::string_view name(thirdName);
            if (name.find("Leg") != std::string_view::npos || name.find("Thigh") != std::string_view::npos || name.find("Calf") != std::string_view::npos ||
                name.find("Foot") != std::string_view::npos) {
                ++legEntries;
            }
        }
        logger::info("[BONEMAP] leg-related entries: {} -> animation-driven legs are {}", legEntries, legEntries > 0 ? "POSSIBLE" : "not available via this sync");

        // Independently: does the first-person skeleton contain legs at all? This is
        // the ceiling on what the sync can ever carry.
        if (firstPerson && firstPerson->transforms && firstPerson->numTransforms > 0 && firstPerson->numTransforms < 4096) {
            int firstPersonLegBones = 0;
            for (int bone = 0; bone < firstPerson->numTransforms; ++bone) {
                const auto boneName = firstPerson->transforms[bone].name.c_str();
                if (!boneName) {
                    continue;
                }
                const std::string_view name(boneName);
                if (name.find("Leg") != std::string_view::npos || name.find("Thigh") != std::string_view::npos || name.find("Calf") != std::string_view::npos ||
                    name.find("Foot") != std::string_view::npos) {
                    ++firstPersonLegBones;
                    logger::info("[BONEMAP] first-person leg bone: {}", boneName);
                }
            }
            logger::info("[BONEMAP] first-person skeleton leg bones: {}", firstPersonLegBones);
        }
    }

    void Skeleton::resetMotionState()
    {
        _lastPosition = _curentPosition;
        _lastNeckYaw = 0.0f;
        _armIKContinuity = {};
        resetWalkingState();

        // The damped objects are weapon offset nodes, not the first-person
        // hand bones cached above.  Prime from the actual node on its next use.
        _rightHandDampingPrimed = false;
        _leftHandDampingPrimed = false;
    }

    /**
     * Ground height under a foot, falling back to the body-root plane.
     *
     * FRIK forces both feet onto a single plane through the root, so on a slope or
     * stairs one foot floats while the other sinks. The probe is deliberately short
     * and the result is clamped near the root plane so a ray that hits something
     * unexpected (a ledge above, a hole below) cannot pull a leg past its reach -
     * setSingleLeg would reject that pose and drop the leg entirely.
     */
    float Skeleton::groundedFootHeight(const RE::NiPoint3& footPosition, const float fallbackZ) const
    {
        if (!g_config.groundAwareFeet || !std::isfinite(fallbackZ) || !isFinite(footPosition)) {
            return fallbackZ;
        }

        // Enough to clear a stair rise above and a short drop below, no more.
        constexpr float probeUp = 45.0f;
        constexpr float probeDown = 75.0f;
        constexpr float maximumRise = 35.0f;
        constexpr float maximumDrop = 55.0f;

        const RE::NiPoint3 probeOrigin(footPosition.x, footPosition.y, fallbackZ);
        const auto ground = world::findGround(probeOrigin, probeUp, probeDown);
        if (!ground) {
            return fallbackZ;
        }

        const float groundZ = ground->position.z;
        if (!std::isfinite(groundZ)) {
            return fallbackZ;
        }
        return std::clamp(groundZ, fallbackZ - maximumDrop, fallbackZ + maximumRise);
    }

    /**
     * Hold the standing feet in world space, re-planting once the body has turned far
     * enough. Called only while standing still and only when the feature is enabled.
     *
     * `_leftFootPos`/`_rightFootPos` arrive holding this frame's rest-derived
     * positions, which is what a re-plant snaps to.
     */
    void Skeleton::holdStanceFeet()
    {
        const float threshold = MatrixUtils::degreesToRads(std::clamp(g_config.turnInPlaceStepDegrees, 5.0f, 90.0f));

        RE::NiPoint3 planarForward;
        const bool haveFacing = tryNormalizePlanar(_forwardDir, planarForward);
        const bool turned = haveFacing && _turnAccumulator.update(std::atan2(planarForward.y, planarForward.x), threshold);

        // Re-plant on a turn, on first use, or if a held foot has drifted too far to
        // still be reachable - otherwise setSingleLeg would just reject the pose.
        const bool drifted = _stanceFeetPlanted &&
            (MatrixUtils::vec3Len(_leftFootPos - _leftFootPlanted) > kStanceFootHoldLimit ||
                MatrixUtils::vec3Len(_rightFootPos - _rightFootPlanted) > kStanceFootHoldLimit);

        if (!_stanceFeetPlanted || turned || drifted || !haveFacing || !isFinite(_leftFootPlanted) || !isFinite(_rightFootPlanted)) {
            _leftFootPlanted = _leftFootPos;
            _rightFootPlanted = _rightFootPos;
            _stanceFeetPlanted = true;
            return;
        }

        // Keep the planted world position, but track the ground under it so a moving
        // floor (lift, vertibird) does not leave the feet hanging.
        _leftFootPos = _leftFootPlanted;
        _rightFootPos = _rightFootPlanted;
        _leftFootPos.z = groundedFootHeight(_leftFootPos, _root->world.translate.z);
        _rightFootPos.z = groundedFootHeight(_rightFootPos, _root->world.translate.z);
        _leftFootPlanted.z = _leftFootPos.z;
        _rightFootPlanted.z = _rightFootPos.z;
    }

    void Skeleton::resetWalkingState()
    {
        _prevSpeed = 0.0f;
        _walkingState = 0;
        _currentStepTime = 0.0f;
        _stepTimeinStep = 0.0f;
        _footStepping = 0;
        _directionChangeDelayRemaining = 0.0f;
        _spineAngle = 0.0f;
        _stepDir = _forwardDir;
        _solveLegsThisFrame = false;
        _stopBlendElapsed = 0.0f;
        _stanceFeetPlanted = false;
        _turnAccumulator.reset();
    }

    /**
     * Whether the procedural gait should run this frame.
     *
     * This gate must fail OPEN. 0.77.12 had no gating at all and worked; anything
     * here that misfires disables the legs entirely, and because
     * restoreNodesToDefault() also skips leg bones while disabled, they simply stop
     * moving. The reason is logged whenever it changes so a bad condition is
     * identifiable from FRIK.log rather than by guesswork.
     */
    bool Skeleton::canUseProceduralLegs()
    {
        const auto player = RE::PlayerCharacter::GetSingleton();
        const char* reason = nullptr;

        const bool legNodesAvailable = _leftLeg.hip && _leftLeg.knee && _leftLeg.foot && _rightLeg.hip && _rightLeg.knee && _rightLeg.foot;
        if (!player) {
            reason = "no player";
        } else if (!legNodesAvailable) {
            reason = "leg bones missing";
            // NOT gated on g_config.isPlayingSeated. That setting selects which
            // height/posture offsets apply when the player is physically sitting in a
            // real chair; it says nothing about whether the avatar should have legs.
            // Upstream 0.77.12 never referenced it in Skeleton.cpp, and gating on it
            // disabled the legs outright for every seated player.
        } else if (g_frik.isPauseMenuOpen()) {
            reason = "pause menu";
        } else if (isJumpingOrInAir()) {
            reason = "jumping or in air";
        } else if (isSwimming(player)) {
            reason = "swimming";
        } else if (isUnderwater(player)) {
            reason = "underwater";
        } else if (player->IsDead(false)) {
            reason = "dead";
        } else if (player->DoGetSitSleepState() != RE::SIT_SLEEP_STATE::kNormal) {
            reason = "sitting or sleeping";
        } else {
            // Only veto camera states where a procedural gait is clearly wrong. The
            // previous code allowlisted FirstPerson/IronSights and failed closed on
            // everything else, so any state Fallout 4 VR reports outside that pair
            // killed the legs permanently. Unknown states are now allowed through.
            const auto playerCamera = getPlayerCamera();
            if (playerCamera && playerCamera->cameraState) {
                switch (playerCamera->cameraState->stateID) {
                case F4SEVR::PlayerCamera::kCameraState_ThirdPerson1:
                case F4SEVR::PlayerCamera::kCameraState_ThirdPerson2:
                case F4SEVR::PlayerCamera::kCameraState_AutoVanity:
                case F4SEVR::PlayerCamera::kCameraState_Free:
                case F4SEVR::PlayerCamera::kCameraState_TweenMenu:
                case F4SEVR::PlayerCamera::kCameraState_Furniture:
                case F4SEVR::PlayerCamera::kCameraState_Horse:
                case F4SEVR::PlayerCamera::kCameraState_Bleedout:
                case F4SEVR::PlayerCamera::kCameraState_Dialogue:
                case F4SEVR::PlayerCamera::kCameraState_VATS:
                    reason = "camera state";
                    break;
                default:
                    break;
                }
            }
        }

        // Report only on change, so this is usable at the default info log level.
        if (reason != _lastLegGateReason) {
            if (reason) {
                logger::info("Procedural legs disabled: {}", reason);
            } else {
                logger::info("Procedural legs enabled");
            }
            _lastLegGateReason = reason;
        }
        return reason == nullptr;
    }

    bool Skeleton::hasRequiredNodes() const
    {
        const bool armsAvailable =
            _leftArm.shoulder && _leftArm.upper && _leftArm.forearm1 && _leftArm.hand && _rightArm.shoulder && _rightArm.upper && _rightArm.forearm1 && _rightArm.hand;
        return _root && _root->parent && _playerNodes && _playerNodes->playerworldnode && _playerNodes->UprightHmdNode && _playerNodes->primaryWandNode &&
            _playerNodes->SecondaryWandNode && _head && _spine && _chest && _com && _neck && _spine1 && _rightHand && _leftHand && armsAvailable;
    }

    /**
     * Restore the skeleton main 25 nodes to their default transforms.
     * To wipe out any local transform changes the game might have made since last update
     */
    void Skeleton::restoreNodesToDefault()
    {
        for (const auto& [boneNode, resetTransform] : _skeletonNodesToDefaultTransforms) {
            const bool isLegNode = boneNode == _leftLeg.hip || boneNode == _leftLeg.knee || boneNode == _leftLeg.foot || boneNode == _rightLeg.hip || boneNode == _rightLeg.knee ||
                boneNode == _rightLeg.foot;
            if (!_solveLegsThisFrame && isLegNode) {
                continue;
            }
            boneNode->local = resetTransform;
        }
    }

    /**
     * Moves head up and back out of the player view and handle head movement.
     * It's still not good enough to prevent seeing hats and stuff, but it's a step forward in case
     * someone wants to tackle it further.
     */
    void Skeleton::setupHead(const float neckYaw, const float neckPitch) const
    {
        const float headBackAdj = g_frik.isSelfieModeOn() && g_config.selfieIgnoreHideFlags ? 0 : g_config.headBackPositionOffset + (neckPitch > 0 ? 2 * neckPitch : 0);
        _head->local.translate -= RE::NiPoint3(headBackAdj, 2 * headBackAdj, 0);
        _head->local.rotate = _head->local.rotate * MatrixUtils::getMatrixFromEulerAngles(neckYaw, 0, neckPitch);
        RE::NiUpdateData* ud = nullptr;
        _head->UpdateWorldData(ud);
    }

    // Estimate head yaw relative to the body from the two pivot-to-controller
    // directions.  Per Parger 3.1, each projected direction is normalized
    // before the sum so controller distance cannot dominate the estimate.
    float Skeleton::getNeckYaw()
    {
        if (!_hasValidTrackedHeadPose) {
            return _lastNeckYaw;
        }

        const RE::NiPoint3 hmdToLeft = _playerNodes->SecondaryWandNode->world.translate - _trackedHeadPose.pivot;
        const RE::NiPoint3 hmdToRight = _playerNodes->primaryWandNode->world.translate - _trackedHeadPose.pivot;
        float weight = 1.0f;

        if (!isFinite(hmdToLeft) || !isFinite(hmdToRight) || MatrixUtils::vec3Len(hmdToLeft) < 10.0f || MatrixUtils::vec3Len(hmdToRight) < 10.0f) {
            return _lastNeckYaw;
        }

        // handle excessive angles when hand is above the head.
        if (hmdToLeft.z > 0) {
            weight = (std::max)(weight - 0.05f * hmdToLeft.z, 0.0f);
        }

        if (hmdToRight.z > 0) {
            weight = (std::max)(weight - 0.05f * hmdToRight.z, 0.0f);
        }

        // hands moving across the chest rotate too much.   try to handle with below
        // wp = parWp + parWr * lp =>   lp = (wp - parWp) * parWr'
        const RE::NiPoint3 locLeft = _trackedHeadPose.raw.rotate * hmdToLeft;
        const RE::NiPoint3 locRight = _trackedHeadPose.raw.rotate * hmdToRight;

        if (locLeft.x > locRight.x) {
            const float delta = locRight.x - locLeft.x;
            weight = (std::max)(weight + 0.02f * delta, 0.0f);
        }

        RE::NiPoint3 leftDirection;
        RE::NiPoint3 rightDirection;
        if (!tryNormalizePlanar(hmdToLeft, leftDirection) || !tryNormalizePlanar(hmdToRight, rightDirection)) {
            return _lastNeckYaw;
        }

        RE::NiPoint3 handForward;
        if (!tryNormalizePlanar(leftDirection + rightDirection, handForward)) {
            return _lastNeckYaw;
        }

        RE::NiPoint3 hmdForward;
        if (!tryNormalizePlanar(_trackedHeadPose.raw.rotate.Transpose() * RE::NiPoint3(0, 1, 0), hmdForward)) {
            hmdForward = _forwardDir;
        }

        const float dot = std::clamp(MatrixUtils::vec3Dot(hmdForward, handForward), -1.0f, 1.0f);
        const float determinant = handForward.x * hmdForward.y - handForward.y * hmdForward.x;
        const float relativeYaw = atan2f(determinant, dot);
        const float neckYaw = std::clamp(-relativeYaw * weight, MatrixUtils::degreesToRads(-50.0f), MatrixUtils::degreesToRads(50.0f));

        if (std::isfinite(neckYaw)) {
            _lastNeckYaw = neckYaw;
        }
        return _lastNeckYaw;
    }

    float Skeleton::getNeckPitch() const
    {
        if (!_hasValidTrackedHeadPose) {
            return 0.0f;
        }

        RE::NiPoint3 lookDirection;
        if (!tryNormalize(_trackedHeadPose.raw.rotate.Transpose() * RE::NiPoint3(0, 1, 0), lookDirection)) {
            return 0.0f;
        }

        const float horizontalLength = std::sqrt(lookDirection.x * lookDirection.x + lookDirection.y * lookDirection.y);
        return atan2f(lookDirection.z, horizontalLength);
    }

    float Skeleton::getBodyPitch(const float neckPitch) const
    {
        if (isComfortSneakHackEnabled()) {
            return MatrixUtils::degreesToRads(g_config.comfortSneakHackStaticBodyPitchAngle);
        }

        constexpr float basePitch = 105.3f;
        constexpr float weight = 0.1f;

        // PlayerHeight is a deprecated migration value.  The calibrated
        // measurement participates only in dimensionless solver math and is
        // never applied to the scene graph or weapon hierarchy.
        const float curHeight = (std::max)(std::abs(g_config.calibratedPlayerHeight), kVectorEpsilon);
        float neutralPivotLeverZ = 0.0f;
        if (_lastPivotCorrectionEnabled) {
            float parentScale = 1.0f;
            if (_playerNodes && _playerNodes->playerworldnode && std::isfinite(_playerNodes->playerworldnode->world.scale) &&
                std::abs(_playerNodes->playerworldnode->world.scale) > kVectorEpsilon) {
                parentScale = _playerNodes->playerworldnode->world.scale;
            }
            neutralPivotLeverZ = g_config.hmdPivotOffsetZ * _trackedHeadPose.raw.scale / parentScale;
        }
        const float referenceHeight = (std::max)(curHeight - neutralPivotLeverZ, kVectorEpsilon);
        const float correctedHeight = getCorrectedUprightHmdHeight() + getAdjustedPlayerHMDOffset();
        const float heightCalc = std::clamp((referenceHeight - correctedHeight) / referenceHeight, 0.0f, 1.0f);
        const float angle = heightCalc * (basePitch + weight * MatrixUtils::radsToDegrees(neckPitch));
        return MatrixUtils::degreesToRads(angle);
    }

    float Skeleton::getCorrectedUprightHmdHeight() const
    {
        // playerworldnode is dereferenced below for its world rotation, so it belongs
        // in this guard.  The per-frame path reaches here only after
        // hasRequiredNodes(), but the guard should match what the body actually uses.
        if (!_hasValidTrackedHeadPose || !_playerNodes || !_playerNodes->UprightHmdNode || !_playerNodes->playerworldnode) {
            return 0.0f;
        }

        float parentScale = 1.0f;
        if (std::isfinite(_playerNodes->playerworldnode->world.scale) && std::abs(_playerNodes->playerworldnode->world.scale) > kVectorEpsilon) {
            parentScale = _playerNodes->playerworldnode->world.scale;
        }

        const RE::NiPoint3 pivotCorrectionWorld = _trackedHeadPose.pivot - _trackedHeadPose.raw.translate;
        const RE::NiPoint3 pivotCorrectionLocal = _playerNodes->playerworldnode->world.rotate * (pivotCorrectionWorld / parentScale);
        return _playerNodes->UprightHmdNode->local.translate.z + pivotCorrectionLocal.z;
    }

    /**
     * Build the settings for splitting body yaw between the root and the spine.
     * A zero share reproduces the legacy root-only rotation exactly.
     */
    ik::TorsoTwistSettings Skeleton::torsoTwistSettings()
    {
        return { .share = g_config.torsoTwistShare,
            .spineFraction = 0.4f,
            // Soft anatomical caps for thoracic/lumbar axial rotation.
            .spineLimit = MatrixUtils::degreesToRads(20.0f),
            .chestLimit = MatrixUtils::degreesToRads(30.0f) };
    }

    /**
     * Twist the spine about its own local bone axis.  Every spine-chain bone is
     * offset from its parent purely along local X, so X is the chain/twist axis.
     *
     * The euler matrix is pre-multiplied, which is what rotates a node in its own
     * local frame under this codebase's transposed matrix convention (world =
     * local * parentWorld).  This matches how walk() already twists this very same
     * node for gait sway, so the two compose about one consistent axis.
     *
     * The yaw applied here was taken off the avatar root, so the chest reaches the
     * same orientation as before while the pelvis, legs, and feet stay planted.
     */
    void Skeleton::applyTorsoTwist(const ik::TorsoTwist& twist) const
    {
        const auto twistNode = [](RE::NiAVObject* node, const float angle) {
            if (!node || !std::isfinite(angle) || std::abs(angle) <= kVectorEpsilon) {
                return;
            }
            const RE::NiMatrix3 rotated = MatrixUtils::getMatrixFromEulerAngles(angle, 0, 0) * node->local.rotate;
            if (isFinite(rotated) && isNearlyOrthonormal(rotated)) {
                node->local.rotate = rotated;
            }
        };

        twistNode(_spine, twist.spine);
        twistNode(_chest, twist.chest);
    }

    /**
     * set up the body underneath the headset in a proper scale and orientation
     */
    void Skeleton::setBodyUnderHMD(const float rootYaw)
    {
        if (g_config.disableSmoothMovement) {
            _playerNodes->playerworldnode->local.translate.z = getAdjustedPlayerHMDOffset();
            updateDown(_playerNodes->playerworldnode, true);
        }

        const float z = _root->local.translate.z;

        RE::NiPoint3 planarHmdForward;
        const RE::NiPoint3 hmdForward = _trackedHeadPose.raw.rotate.Transpose() * RE::NiPoint3(0, 1, 0);
        if (tryNormalizePlanar(hmdForward, planarHmdForward)) {
            // rootYaw is already the caller's share of the body yaw; the historical
            // 0.7 weighting is applied before the root/spine split.
            _forwardDir = MatrixUtils::rotateXY(planarHmdForward, rootYaw);
        }
        _sidewaysRDir = RE::NiPoint3(_forwardDir.y, -_forwardDir.x, 0);

        RE::NiNode* body = _root->parent;
        body->local.translate *= 0.0f;
        body->world.translate.x = _curentPosition.x;
        body->world.translate.y = _curentPosition.y;
        body->world.translate.z += _playerNodes->playerworldnode->local.translate.z;

        const RE::NiPoint3 back = safeNormalize(RE::NiPoint3(_forwardDir.x, _forwardDir.y, 0), RE::NiPoint3(0, 1, 0));
        const auto bodyDir = RE::NiPoint3(0, 1, 0);

        RE::NiMatrix3 bodyFacing;
        if (!tryGetRotationFromVectors(back, bodyDir, bodyFacing)) {
            return;
        }
        _root->local.rotate = bodyFacing * body->world.rotate.Transpose();
        _root->local.translate = body->world.translate - _curentPosition;
        _root->local.translate.z = z;
        // PlayerHeight scaling is legacy and breaks weapon alignment.  Body
        // dimensions are calibration inputs only; the rendered skeleton stays
        // at its authored scale.
        _root->local.scale = 1.0f;
    }

    void Skeleton::setBodyPosture(const float neckPitch)
    {
        const float requestedBodyPitch = _inPowerArmor ? getBodyPitch(neckPitch) : getBodyPitch(neckPitch) / 1.2f;
        const float bodyPitch = std::clamp(requestedBodyPitch, MatrixUtils::degreesToRads(-85.0f), MatrixUtils::degreesToRads(85.0f));

        if (!_leftLeg.knee || !_rightLeg.knee) {
            return;
        }

        _leftKneePos = _leftLeg.knee->world.translate;
        _rightKneePos = _rightLeg.knee->world.translate;

        _com->local.translate.x = 0.0f;
        _com->local.translate.y = 0.0f;

        // comfort sneak changes the height of the avatar without the player changing height in the real world, need to adjust for it
        const float comfortSneakAdjustZ = isComfortSneakMode() && isPlayerSneaking() ? _comfortSneakCameraOffsetAdjustment * _comfortSneakCameraOffsetAdjustment : 1.0f;

        // Preserve the old approximation only when pivot correction is
        // explicitly disabled.  With correction enabled, the full rotated
        // three-dimensional lever arm replaces both pitch-only offsets.
        const float legacyForwardOffsetByPitch = !_lastPivotCorrectionEnabled ? fmaxf(0, (isComfortSneakHackEnabled() ? 2.0f : 5.0f) * fabs(neckPitch)) : 0.0f;
        const float legacyVerticalOffsetByPitch = !_lastPivotCorrectionEnabled ? 6.0f * neckPitch : 0.0f;
        const float playerAdjustZ =
            (4 * g_config.getPlayerBodyOffsetUp() - g_config.getPlayerHMDOffsetUp() + g_config.getPlayerLegSlackAdjustOffset()) * comfortSneakAdjustZ + legacyVerticalOffsetByPitch;

        // In corrected mode the rotated HMD lever arm has already been removed
        // from the pivot, so the former pitch approximations are both zero.
        const auto neckPos = _curentPosition +
            RE::NiPoint3(-_forwardDir.x * (g_config.getPlayerBodyOffsetForward() / 2 - legacyForwardOffsetByPitch),
                -_forwardDir.y * (g_config.getPlayerBodyOffsetForward() / 2 - legacyForwardOffsetByPitch), -playerAdjustZ);

        _torsoLen = MatrixUtils::vec3Len(_neck->world.translate - _com->world.translate);
        if (!std::isfinite(_torsoLen) || _torsoLen <= kVectorEpsilon) {
            return;
        }

        const RE::NiPoint3 hmdToHip = neckPos - _com->world.translate;
        const auto dir = RE::NiPoint3(-_forwardDir.x, -_forwardDir.y, 0);
        RE::NiPoint3 normalizedDir;
        if (!tryNormalize(dir, normalizedDir)) {
            return;
        }

        const float dist = tanf(bodyPitch) * MatrixUtils::vec3Len(hmdToHip);
        if (!std::isfinite(dist)) {
            return;
        }
        RE::NiPoint3 tmpHipPos = _com->world.translate + normalizedDir * dist;
        tmpHipPos.z = _com->world.translate.z;

        const RE::NiPoint3 hmdToNewHip = tmpHipPos - neckPos;
        RE::NiPoint3 normalizedHmdToNewHip;
        if (!tryNormalize(hmdToNewHip, normalizedHmdToNewHip)) {
            return;
        }
        const RE::NiPoint3 newHipPos = neckPos + normalizedHmdToNewHip * _torsoLen;

        const RE::NiPoint3 newPos = _com->local.translate + _root->world.rotate * (newHipPos - _com->world.translate);
        if (!isFinite(newPos)) {
            return;
        }
        _com->local.translate.y += newPos.y + g_config.getPlayerBodyOffsetForward() - 2 * legacyForwardOffsetByPitch;
        _com->local.translate.z = _inPowerArmor ? newPos.z / 1.7f : newPos.z / 1.5f;

        // ???
        _root->parent->world.translate.z -= g_config.getPlayerBodyOffsetUp() + getAdjustedPlayerHMDOffset();

        if (MatrixUtils::vec3Len(neckPos - tmpHipPos) > kVectorEpsilon && MatrixUtils::vec3Len(hmdToHip) > kVectorEpsilon) {
            RE::NiMatrix3 postureRotation;
            if (tryGetRotationFromVectors(neckPos - tmpHipPos, hmdToHip, postureRotation)) {
                const RE::NiMatrix3 mat = postureRotation * _spine1->parent->world.rotate.Transpose();
                _spine1->local.rotate = _spine1->world.rotate * mat;
            }
        }
    }

    void Skeleton::setKneePos()
    {
        if (!_leftLeg.knee || !_rightLeg.knee || !isFinite(_leftLeg.knee->world) || !isFinite(_rightLeg.knee->world) || !isFinite(_leftKneePos) || !isFinite(_rightKneePos)) {
            return;
        }

        _leftLeg.knee->world.translate.z = _leftKneePos.z;
        _rightLeg.knee->world.translate.z = _rightKneePos.z;

        _leftKneePos = _leftLeg.knee->world.translate;
        _rightKneePos = _rightLeg.knee->world.translate;

        updateDown(_leftLeg.knee, false);
        updateDown(_rightLeg.knee, false);
    }

    // TODO: does it do anything? check if it works at all
    void Skeleton::fixArmor() const
    {
        // "LArm_UpperArm" is already resolved into the cached arm binding, so reuse
        // it instead of searching the tree again on every Power Armor frame.  The
        // previous code dereferenced the search result unchecked, which crashed on
        // any skeleton that had pauldrons but no upper-arm bone.
        if (!_leftArm.upper || !_root || !isFinite(_leftArm.upper->world) || !isFinite(_root->world)) {
            return;
        }

        auto lPauldron = findNode(_root, "L_Pauldron");
        auto rPauldron = findNode(_root, "R_Pauldron");
        if (!lPauldron || !rPauldron) {
            return;
        }

        const float delta = _leftArm.upper->world.translate.z - _root->world.translate.z;
        if (!std::isfinite(delta)) {
            return;
        }

        lPauldron->local.translate.z = delta - 15.0f;
        rPauldron->local.translate.z = delta - 15.0f;
    }

    void Skeleton::walk()
    {
        if (!_solveLegsThisFrame) {
            resetWalkingState();
            return;
        }

        const auto lHip = _leftLeg.hip;
        const auto rHip = _rightLeg.hip;
        const auto lKnee = _leftLeg.knee;
        const auto rKnee = _rightLeg.knee;
        const auto lFoot = _leftLeg.foot;
        const auto rFoot = _rightLeg.foot;

        if (!lHip || !rHip || !lKnee || !rKnee || !lFoot || !rFoot) {
            resetWalkingState();
            return;
        }

        if (!isFinite(_root->world) || !isFinite(lHip->world) || !isFinite(rHip->world) || !isFinite(lKnee->world) || !isFinite(rKnee->world) || !isFinite(lFoot->world) ||
            !isFinite(rFoot->world)) {
            resetWalkingState();
            return;
        }

        // want to calculate direction vector first.     Will only concern with x-y vector to start.
        RE::NiPoint3 lastPos = _lastPosition;
        RE::NiPoint3 curPos = _curentPosition;
        curPos.z = 0;
        lastPos.z = 0;

        RE::NiPoint3 dir = curPos - lastPos;
        const float movementDistance = MatrixUtils::vec3Len(dir);
        if (!isFinite(dir) || !std::isfinite(movementDistance) || _frameTime <= kVectorEpsilon) {
            resetWalkingState();
            return;
        }

        float curSpeed = std::clamp(movementDistance / _frameTime, 0.0f, 350.0f);
        if (_prevSpeed > 20.0f) {
            const float speedRetention = frameRateIndependentRetention(0.5f, _frameTime);
            curSpeed = curSpeed * (1.0f - speedRetention) + _prevSpeed * speedRetention;
        }

        const float stepTime = std::clamp(std::cos(curSpeed / 140.0f), 0.28f, 0.50f);
        RE::NiPoint3 normalizedDirection;
        if (movementDistance > kVectorEpsilon) {
            if (!tryNormalize(dir, normalizedDirection)) {
                resetWalkingState();
                return;
            }
            dir = normalizedDirection;
        } else if (!tryNormalizePlanar(_stepDir, dir) && !tryNormalizePlanar(_forwardDir, dir)) {
            dir = RE::NiPoint3(0, 1, 0);
        }

        // move feet closer together after all motion inputs have validated
        const RE::NiPoint3 leftToRight = _inPowerArmor ? (rFoot->world.translate - lFoot->world.translate) * -0.15f : (rFoot->world.translate - lFoot->world.translate) * 0.3f;
        lFoot->world.translate += leftToRight;
        rFoot->world.translate -= leftToRight;

        // Preserve the old 20-units-per-90-Hz-frame behavior as a
        // frame-rate-independent acceleration threshold.
        const float acceleration = (curSpeed - _prevSpeed) / _frameTime;
        if (acceleration < kStepRetargetDeceleration) {
            _walkingState = 3;
        }

        _prevSpeed = curSpeed;

        // setup current walking state based on velocity and previous state
        switch (_walkingState) {
        case 0: {
            if (curSpeed >= 35.0) {
                _walkingState = 1; // start walking
                // Step with whichever foot is further back along the direction of
                // travel. That is the foot which would naturally swing through, and
                // unlike the previous std::rand() it is deterministic and thread-safe.
                _footStepping = MatrixUtils::vec3Dot(rFoot->world.translate, dir) <= MatrixUtils::vec3Dot(lFoot->world.translate, dir) ? 1 : 2;
                _stepDir = dir;
                _stepTimeinStep = stepTime;
                _directionChangeDelayRemaining = kDirectionChangeDelaySeconds;

                if (_footStepping == 1) {
                    _rightFootTarget = rFoot->world.translate + _stepDir * (curSpeed * stepTime * 1.5f);
                    _rightFootStart = rFoot->world.translate;
                    _leftFootTarget = lFoot->world.translate;
                    _leftFootStart = lFoot->world.translate;
                    _leftFootPos = _leftFootStart;
                    _rightFootPos = _rightFootStart;
                } else {
                    _rightFootTarget = rFoot->world.translate;
                    _rightFootStart = rFoot->world.translate;
                    _leftFootTarget = lFoot->world.translate + _stepDir * (curSpeed * stepTime * 1.5f);
                    _leftFootStart = lFoot->world.translate;
                    _leftFootPos = _leftFootStart;
                    _rightFootPos = _rightFootStart;
                }
                _currentStepTime = stepTime / 2;
                break;
            }
            _currentStepTime = 0.0;
            _footStepping = 0;
            _spineAngle = 0.0f;
            break;
        }
        case 1: {
            if (curSpeed < 20.0) {
                _walkingState = 2; // begin process to stop walking
                _currentStepTime = 0.0;
                // Capture where the feet actually are so they can be blended back to
                // the rest pose rather than teleported there.
                _stopBlendElapsed = 0.0f;
                _leftFootStopFrom = _leftFootPos;
                _rightFootStopFrom = _rightFootPos;
            }
            break;
        }
        case 2: {
            if (curSpeed >= 20.0) {
                _walkingState = 1; // resume walking
                _currentStepTime = 0.0;
                _stopBlendElapsed = 0.0f;
            }
            break;
        }
        case 3: {
            _stepDir = dir;
            if (_footStepping == 1) {
                _rightFootTarget = rFoot->world.translate + _stepDir * (curSpeed * stepTime * 0.1f);
            } else {
                _leftFootTarget = lFoot->world.translate + _stepDir * (curSpeed * stepTime * 0.1f);
            }
            _walkingState = 1;
            break;
        }
        default: {
            _walkingState = 0;
            break;
        }
        }

        if (_walkingState == 0) {
            // we're standing still so just set foot positions accordingly.
            _leftFootPos = lFoot->world.translate;
            _rightFootPos = rFoot->world.translate;
            _leftFootPos.z = groundedFootHeight(_leftFootPos, _root->world.translate.z);
            _rightFootPos.z = groundedFootHeight(_rightFootPos, _root->world.translate.z);

            if (!isFinite(_leftFootPos) || !isFinite(_rightFootPos)) {
                resetWalkingState();
                return;
            }

            // The rest pose rotates with the body, so standing feet slide round with
            // it instead of stepping. Hold them where they were planted and re-plant
            // once enough turn has built up.
            if (g_config.turnInPlaceStepDegrees > 0.0f) {
                holdStanceFeet();
            } else if (_stanceFeetPlanted) {
                _stanceFeetPlanted = false;
                _turnAccumulator.reset();
            }
            return;
        }
        if (_walkingState == 1) {
            RE::NiPoint3 dirOffset = dir - _stepDir;
            const float dot = MatrixUtils::vec3Dot(dir, _stepDir);
            const float scale = (std::min)(curSpeed * stepTime * 1.5f, 140.0f);
            if (!isFinite(dirOffset) || !std::isfinite(dot) || !std::isfinite(scale) || !isFinite(_leftFootTarget) || !isFinite(_rightFootTarget) || !isFinite(_leftFootStart) ||
                !isFinite(_rightFootStart)) {
                resetWalkingState();
                return;
            }
            dirOffset = dirOffset * scale;

            float sign = 1.0f;

            _currentStepTime += _frameTime;

            if (!std::isfinite(_stepTimeinStep) || _stepTimeinStep <= kVectorEpsilon) {
                resetWalkingState();
                return;
            }
            const float interp = std::clamp(_currentStepTime / _stepTimeinStep, 0.0f, 1.0f);

            if (_footStepping == 1) {
                sign = -1.0f;
                if (dot < 0.9) {
                    _directionChangeDelayRemaining = (std::max)(0.0f, _directionChangeDelayRemaining - _frameTime);
                    if (_directionChangeDelayRemaining <= 0.0f) {
                        _rightFootTarget += dirOffset;
                        _stepDir = dir;
                        _directionChangeDelayRemaining = kDirectionChangeDelaySeconds;
                    }
                } else {
                    _directionChangeDelayRemaining = kDirectionChangeDelaySeconds;
                }
                _rightFootTarget.z = groundedFootHeight(_rightFootTarget, _root->world.translate.z);
                _rightFootStart.z = groundedFootHeight(_rightFootStart, _root->world.translate.z);
                _rightFootPos = _rightFootStart + (_rightFootTarget - _rightFootStart) * interp;
                const float stepAmount = std::clamp(MatrixUtils::vec3Len(_rightFootTarget - _rightFootStart) / 150.0f, 0.0f, 1.0f);
                const float stepHeight = (std::max)(stepAmount * 9.0f, 1.0f);
                const float up = sinf(interp * std::numbers::pi_v<float>) * stepHeight;
                _rightFootPos.z += up;
            } else {
                if (dot < 0.9f) {
                    _directionChangeDelayRemaining = (std::max)(0.0f, _directionChangeDelayRemaining - _frameTime);
                    if (_directionChangeDelayRemaining <= 0.0f) {
                        _leftFootTarget += dirOffset;
                        _stepDir = dir;
                        _directionChangeDelayRemaining = kDirectionChangeDelaySeconds;
                    }
                } else {
                    _directionChangeDelayRemaining = kDirectionChangeDelaySeconds;
                }
                _leftFootTarget.z = groundedFootHeight(_leftFootTarget, _root->world.translate.z);
                _leftFootStart.z = groundedFootHeight(_leftFootStart, _root->world.translate.z);
                _leftFootPos = _leftFootStart + (_leftFootTarget - _leftFootStart) * interp;
                const float stepAmount = std::clamp(MatrixUtils::vec3Len(_leftFootTarget - _leftFootStart) / 150.0f, 0.0f, 1.0f);
                const float stepHeight = (std::max)(stepAmount * 9.0f, 1.0f);
                const float up = sinf(interp * std::numbers::pi_v<float>) * stepHeight;
                _leftFootPos.z += up;
            }

            if (!isFinite(_leftFootPos) || !isFinite(_rightFootPos) || !isFinite(_leftFootTarget) || !isFinite(_rightFootTarget)) {
                resetWalkingState();
                return;
            }

            _spineAngle = sign * sinf(interp * std::numbers::pi_v<float>) * 3.0f;

            _spine->local.rotate = MatrixUtils::getMatrixFromEulerAngles(MatrixUtils::degreesToRads(_spineAngle), 0.0f, 0.0f) * _spine->local.rotate;

            if (_currentStepTime > stepTime) {
                _currentStepTime = 0.0;
                _stepDir = dir;
                _stepTimeinStep = stepTime;
                //logger::info("%2f %2f", curSpeed, stepTime);

                if (_footStepping == 1) {
                    _footStepping = 2;
                    _leftFootTarget = lFoot->world.translate + _stepDir * scale;
                    _leftFootStart = _leftFootPos;
                } else {
                    _footStepping = 1;
                    _rightFootTarget = rFoot->world.translate + _stepDir * scale;
                    _rightFootStart = _rightFootPos;
                }
            }
            return;
        }
        if (_walkingState == 2) {
            // Stopping used to drop both feet onto the rest pose in a single frame,
            // which pops every time you stop walking. Ease them back instead.
            RE::NiPoint3 leftRest = lFoot->world.translate;
            RE::NiPoint3 rightRest = rFoot->world.translate;
            leftRest.z = groundedFootHeight(leftRest, _root->world.translate.z);
            rightRest.z = groundedFootHeight(rightRest, _root->world.translate.z);
            if (!isFinite(leftRest) || !isFinite(rightRest) || !isFinite(_leftFootStopFrom) || !isFinite(_rightFootStopFrom)) {
                resetWalkingState();
                return;
            }

            _stopBlendElapsed += _frameTime;
            const float blend = ik::stopBlend(_stopBlendElapsed, kStopBlendSeconds);
            _leftFootPos = _leftFootStopFrom + (leftRest - _leftFootStopFrom) * blend;
            _rightFootPos = _rightFootStopFrom + (rightRest - _rightFootStopFrom) * blend;
            if (!isFinite(_leftFootPos) || !isFinite(_rightFootPos)) {
                resetWalkingState();
                return;
            }

            if (blend >= 1.0f) {
                _walkingState = 0;
                _stanceFeetPlanted = false;
                _turnAccumulator.reset();
            }
        }
    }

    // adapted solver from VRIK.  Thanks prog!
    std::optional<float> Skeleton::setSingleLeg(const bool isLeft) const
    {
        const auto leg = isLeft ? _leftLeg : _rightLeg;
        const auto footNode = leg.foot;
        const auto kneeNode = leg.knee;
        const auto hipNode = leg.hip;
        if (!footNode || !kneeNode || !hipNode || !hipNode->parent || !isFinite(footNode->local) || !isFinite(footNode->world) || !isFinite(kneeNode->local) ||
            !isFinite(kneeNode->world) || !isFinite(hipNode->local) || !isFinite(hipNode->world) || !isFinite(hipNode->parent->world)) {
            return std::nullopt;
        }

        const RE::NiPoint3 footPos = isLeft ? _leftFootPos : _rightFootPos;
        const RE::NiPoint3 hipPos = hipNode->world.translate;
        const RE::NiPoint3 footToHip = hipNode->world.translate - footPos;
        if (!isFinite(footPos) || !isFinite(hipPos) || !isFinite(footToHip)) {
            return std::nullopt;
        }

        const float rawFootToHipLength = MatrixUtils::vec3Len(footToHip);
        RE::NiPoint3 xDir;
        if (!std::isfinite(rawFootToHipLength) || !tryNormalize(footToHip, xDir)) {
            return std::nullopt;
        }

        auto rotV = RE::NiPoint3(0, 1, 0);
        if (_inPowerArmor) {
            rotV.y = 0;
            rotV.z = isLeft ? 1.0f : -1.0f;
        }
        const RE::NiPoint3 hipDir = hipNode->world.rotate.Transpose() * (rotV);
        RE::NiPoint3 yDir;
        RE::NiPoint3 projectedBendDirection = hipDir - xDir * MatrixUtils::vec3Dot(hipDir, xDir);
        if (!tryNormalize(projectedBendDirection, yDir)) {
            // Full extension makes the original pole direction ambiguous.
            // Prefer the body-forward plane, then a stable world axis.
            const RE::NiPoint3 bodyForward(_forwardDir.x, _forwardDir.y, 0.0f);
            projectedBendDirection = bodyForward - xDir * MatrixUtils::vec3Dot(bodyForward, xDir);
            if (!tryNormalize(projectedBendDirection, yDir)) {
                const RE::NiPoint3 fallbackAxis = std::abs(xDir.z) < 0.9f ? RE::NiPoint3(0, 0, 1) : RE::NiPoint3(0, 1, 0);
                projectedBendDirection = fallbackAxis - xDir * MatrixUtils::vec3Dot(fallbackAxis, xDir);
                if (!tryNormalize(projectedBendDirection, yDir)) {
                    return std::nullopt;
                }
            }
        }

        const float thighLenOrig = MatrixUtils::vec3Len(kneeNode->local.translate);
        const float calfLenOrig = MatrixUtils::vec3Len(footNode->local.translate);
        if (!std::isfinite(thighLenOrig) || !std::isfinite(calfLenOrig) || thighLenOrig <= kVectorEpsilon || calfLenOrig <= kVectorEpsilon) {
            return std::nullopt;
        }

        float thighLen = thighLenOrig;
        float calfLen = calfLenOrig;

        const float ftLen = (std::max)(rawFootToHipLength, 0.1f);

        if (ftLen > thighLen + calfLen) {
            const float diff = ftLen - thighLen - calfLen;
            const float combinedLength = calfLen + thighLen;
            if (!std::isfinite(diff) || combinedLength <= kVectorEpsilon) {
                return std::nullopt;
            }
            const float ratio = calfLen / combinedLength;
            calfLen += ratio * diff + 0.1f;
            thighLen += (1.0f - ratio) * diff + 0.1f;
        }

        // Clamp the cosine before acos.  This covers unreachable targets and
        // floating-point drift at full extension without producing NaNs.
        float footAngle = 0.0f;
        if (!tryLawOfCosinesAngle(calfLen, ftLen, thighLen, footAngle)) {
            return std::nullopt;
        }

        // Get the desired world coordinate of the knee
        const float xDist = cosf(footAngle) * calfLen;
        const float yDist = sinf(footAngle) * calfLen;
        const RE::NiPoint3 kneePos = footPos + xDir * xDist + yDir * yDist;
        if (!std::isfinite(xDist) || !std::isfinite(yDist) || !isFinite(kneePos)) {
            return std::nullopt;
        }

        const RE::NiPoint3 pos = kneePos - hipPos;
        RE::NiPoint3 normalizedHipToKnee;
        if (!tryNormalize(pos, normalizedHipToKnee)) {
            return std::nullopt;
        }
        const RE::NiPoint3 upperLocalDirection = hipNode->world.rotate * normalizedHipToKnee;
        RE::NiMatrix3 hipSwing;
        if (!tryGetRotationFromVectors(upperLocalDirection, kneeNode->local.translate, hipSwing)) {
            return std::nullopt;
        }
        const RE::NiMatrix3 hipLocalRotate = hipSwing * hipNode->local.rotate;
        if (!isNearlyOrthonormal(hipLocalRotate)) {
            return std::nullopt;
        }

        const RE::NiMatrix3 hipWR = hipLocalRotate * hipNode->parent->world.rotate;
        if (!isFinite(hipWR)) {
            return std::nullopt;
        }

        const RE::NiMatrix3 baseCalfWR = kneeNode->local.rotate * hipWR;

        RE::NiPoint3 normalizedKneeToFoot;
        if (!tryNormalize(footPos - kneePos, normalizedKneeToFoot)) {
            return std::nullopt;
        }
        const RE::NiPoint3 calfLocalDirection = baseCalfWR * normalizedKneeToFoot;
        RE::NiMatrix3 kneeSwing;
        if (!tryGetRotationFromVectors(calfLocalDirection, footNode->local.translate, kneeSwing)) {
            return std::nullopt;
        }
        const RE::NiMatrix3 kneeLocalRotate = kneeSwing * kneeNode->local.rotate;
        if (!isNearlyOrthonormal(kneeLocalRotate)) {
            return std::nullopt;
        }

        const RE::NiMatrix3 calfWR = kneeLocalRotate * hipWR;
        if (!isFinite(calfWR)) {
            return std::nullopt;
        }

        // Calculate Clp:  Cwp = Twp + Twr * (Clp * Tws) = kneePos   ===>   Clp = Twr' * (kneePos - Twp) / Tws
        RE::NiPoint3 kneeLocalTranslate = hipWR * ((kneePos - hipPos) / hipNode->world.scale);
        const float computedThighLength = MatrixUtils::vec3Len(kneeLocalTranslate);
        if (!isFinite(kneeLocalTranslate) || !std::isfinite(computedThighLength)) {
            return std::nullopt;
        }
        if (computedThighLength > thighLenOrig) {
            RE::NiPoint3 normalizedKneeLocal;
            if (!tryNormalize(kneeLocalTranslate, normalizedKneeLocal)) {
                return std::nullopt;
            }
            kneeLocalTranslate = normalizedKneeLocal * thighLenOrig;
        }

        // Calculate Flp:  Fwp = Cwp + Cwr * (Flp * Cws) = footPos   ===>   Flp = Cwr' * (footPos - Cwp) / Cws
        RE::NiPoint3 footLocalTranslate = calfWR * ((footPos - kneePos) / kneeNode->world.scale);
        const float computedCalfLength = MatrixUtils::vec3Len(footLocalTranslate);
        if (!isFinite(footLocalTranslate) || !std::isfinite(computedCalfLength)) {
            return std::nullopt;
        }
        if (computedCalfLength > calfLenOrig) {
            RE::NiPoint3 normalizedFootLocal;
            if (!tryNormalize(footLocalTranslate, normalizedFootLocal)) {
                return std::nullopt;
            }
            footLocalTranslate = normalizedFootLocal * calfLenOrig;
        }

        // Commit only after the complete candidate pose validates.
        hipNode->local.rotate = hipLocalRotate;
        kneeNode->local.rotate = kneeLocalRotate;
        kneeNode->local.translate = kneeLocalTranslate;
        footNode->local.translate = footLocalTranslate;

        return (thighLenOrig + calfLenOrig) - ftLen;
    }

    void Skeleton::rotateLeg(const uint32_t pos, const float angle) const
    {
        const auto rt = reinterpret_cast<BSFlattenedBoneTree*>(_root);

        auto& transform = rt->transforms[pos];
        transform.local.rotate = MatrixUtils::getMatrixFromEulerAngles(MatrixUtils::degreesToRads(angle), 0, 0) * transform.local.rotate;

        const auto& parentTransform = rt->transforms[transform.parPos];
        const RE::NiPoint3 p = parentTransform.world.rotate * (transform.local.translate * parentTransform.world.scale);
        transform.world.translate = parentTransform.world.translate + p;

        transform.world.rotate = transform.local.rotate * parentTransform.world.rotate;
    }

    /**
     * Hide the 3rd-person weapon that comes with the skeleton as we are using the 1st-person weapon model.
     */
    void Skeleton::hide3rdPersonWeapon() const
    {
        if (RE::NiAVObject* weapon = find1StChildNode(_rightArm.hand, "Weapon")) {
            setNodeVisibility(weapon, false);
        }
    }

    void Skeleton::hideFistHelpers() const
    {
        // Handedness only decides which wand holds which triplet, and every helper is
        // hidden either way, so the union of names is handedness-independent.  Hiding
        // the whole set in one traversal per wand replaces six traversals per frame.
        static constexpr const char* fistHelperNames[] = { "fist_M_Right_HELPER", "fist_F_Right_HELPER", "PA_fist_R_HELPER", "fist_M_Left_HELPER", "fist_F_Left_HELPER",
            "PA_fist_L_HELPER" };

        hideNodesByName(_playerNodes->primaryWandNode, fistHelperNames);
        hideNodesByName(_playerNodes->SecondaryWandNode, fistHelperNames);

        if (const auto uiNode = findNode(_playerNodes->SecondaryWandNode, "Point002")) {
            uiNode->local.scale = 0.0;
        }
    }

    void Skeleton::showHidePAHud() const
    {
        if (const auto hud = findNode(_playerNodes->roomnode, "PowerArmorHelmetRoot")) {
            hud->local.scale = g_config.showPAHUD ? 1.0f : 0.0f;
        }
    }

    /**
     * Switch right and left weapon nodes if left-handed mode is enabled to correctly the hands.
     * Remember the setting to set back if settings change while game is running.
     */
    void Skeleton::handleLeftHandedWeaponNodesSwitch()
    {
        if (_lastLeftHandedModeSwitch == isLeftHandedMode()) {
            return;
        }

        _lastLeftHandedModeSwitch = isLeftHandedMode();
        logger::warn("Left-handed mode weapon nodes switch (LeftHanded:{})", _lastLeftHandedModeSwitch);

        RE::NiNode* rightWeapon = getWeaponNode();
        RE::NiNode* leftWeapon = _playerNodes->WeaponLeftNode;
        const auto rHand = findNode(getFirstPersonSkeleton(), "RArm_Hand");
        const auto lHand = findNode(getFirstPersonSkeleton(), "LArm_Hand");

        if (!rightWeapon || !rHand || !leftWeapon || !lHand) {
            logger::sample("Cannot set up weapon nodes for left-handed mode switch");
            _lastLeftHandedModeSwitch = isLeftHandedMode();
            return;
        }

        rHand->DetachChild(rightWeapon);
        rHand->DetachChild(leftWeapon);
        lHand->DetachChild(rightWeapon);
        lHand->DetachChild(leftWeapon);

        if (isLeftHandedMode()) {
            rHand->AttachChild(leftWeapon, true);
            lHand->AttachChild(rightWeapon, true);
        } else {
            rHand->AttachChild(rightWeapon, true);
            lHand->AttachChild(leftWeapon, true);
        }
    }

    // This is the main arm IK solver function - Algo credit to prog from SkyrimVR VRIK mod - what a beast!
    void Skeleton::setArms(bool isLeft)
    {
        // This first part is to handle the game calculating the first person hand based off two offset nodes
        // PrimaryWeaponOffset and PrimaryMeleeOffset
        // Unfortunately neither of these two nodes are that close to each other so when you equip a melee or ranged weapon
        // the hand will jump which completely messes up the solver and looks bad to boot.
        // So this code below does a similar operation as the in game function that solves the first person arm by forcing
        // everything to go to the PrimaryWeaponNode.  I have hardcoded a rotation below based off one of the guns that
        // matches my real life hand pose with an index controller very well.   I use this as the baseline for everything

        auto& continuity = _armIKContinuity[isLeft ? 0 : 1];
        if (getFirstPersonSkeleton() == nullptr) {
            continuity = {};
            return;
        }

        const auto arm = isLeft ? _leftArm : _rightArm;
        if (!arm.shoulder || !arm.upper || !arm.forearm1 || !arm.hand || (!_inPowerArmor && (!arm.forearm2 || !arm.forearm3))) {
            logger::sample("Cannot solve {} arm: required skeleton nodes are missing", isLeft ? "left" : "right");
            continuity = {};
            return;
        }

        RE::NiNode* rightWeapon = getWeaponNode();
        //RE::NiNode* rightWeapon = _playerNodes->primaryWandNode;
        RE::NiNode* leftWeapon = _playerNodes->WeaponLeftNode; // "WeaponLeft" can return incorrect node for left-handed with throwable weapons

        // handle the NON-primary hand (i.e. the hand that is NOT holding the weapon)
        bool handleOffhand = isLeftHandedMode() ^ isLeft;

        RE::NiNode* weaponNode = handleOffhand ? leftWeapon : rightWeapon;
        RE::NiNode* offsetNode = handleOffhand ? _playerNodes->SecondaryMeleeWeaponOffsetNode2 : _playerNodes->primaryWeaponOffsetNOde;
        if (!weaponNode || !offsetNode || (handleOffhand && !_playerNodes->primaryWeaponOffsetNOde)) {
            logger::sample("Cannot solve {} arm: tracked weapon nodes are missing", isLeft ? "left" : "right");
            continuity = {};
            return;
        }

        if (handleOffhand) {
            _playerNodes->SecondaryMeleeWeaponOffsetNode2->local = _playerNodes->primaryWeaponOffsetNOde->local;
            _playerNodes->SecondaryMeleeWeaponOffsetNode2->local.rotate =
                _playerNodes->SecondaryMeleeWeaponOffsetNode2->local.rotate * MatrixUtils::getMatrixFromEulerAngles(0, MatrixUtils::degreesToRads(180.0f), 0);
            _playerNodes->SecondaryMeleeWeaponOffsetNode2->local.translate = RE::NiPoint3(-2, -9, 2);
            updateTransforms(_playerNodes->SecondaryMeleeWeaponOffsetNode2);
        }

        weaponNode->local.rotate = !isLeftHandedMode() ? MatrixUtils::getMatrix(-0.122f, 0.987f, 0.100f, 0.990f, 0.114f, 0.081f, 0.069f, 0.109f, -0.992f)
                                                       : MatrixUtils::getMatrix(-0.122f, 0.987f, 0.100f, -0.990f, -0.114f, -0.081f, -0.069f, -0.109f, 0.992f);

        if (handleOffhand) {
            weaponNode->local.rotate = weaponNode->local.rotate * MatrixUtils::getMatrixFromEulerAngles(0, MatrixUtils::degreesToRads(isLeft ? 45.0f : -45.0f), 0);
        }

        weaponNode->local.translate = isLeftHandedMode() ? (isLeft ? RE::NiPoint3(3.389f, -2.099f, 3.133f) : RE::NiPoint3(0, -4.8f, 0))
            : isLeft                                     ? RE::NiPoint3(0, 0, 0)
                                                         : RE::NiPoint3(4.389f, -1.899f, -3.133f);

        dampenHand(offsetNode, isLeft);

        weaponNode->IncRefCount();
        Update1StPersonArm(RE::PlayerCharacter::GetSingleton(), &weaponNode, &offsetNode);

        RE::NiPoint3 handPos = isLeft ? _leftHand->world.translate : _rightHand->world.translate;
        RE::NiMatrix3 handRot = isLeft ? _leftHand->world.rotate : _rightHand->world.rotate;

        // Detect if the 1st person hand position is invalid. This can happen when a controller loses tracking.
        // If it is, do not handle IK and let Fallout use its normal animations for that arm instead.
        if (!isFinite(handPos) || !isFinite(handRot) || MatrixUtils::vec3Len(arm.upper->world.translate - handPos) > 200.0) {
            continuity = {};
            return;
        }

        // Any failure below restores the complete arm chain. The tracked
        // weapon/controller nodes are deliberately outside this transaction.
        const auto shoulderLocalBefore = arm.shoulder->local;
        const auto upperLocalBefore = arm.upper->local;
        const auto forearm1LocalBefore = arm.forearm1->local;
        const auto forearm2LocalBefore = arm.forearm2 ? arm.forearm2->local : arm.forearm1->local;
        const auto forearm3LocalBefore = arm.forearm3 ? arm.forearm3->local : arm.forearm1->local;
        const auto handLocalBefore = arm.hand->local;
        ScopeExit poseRollback([&]() {
            arm.shoulder->local = shoulderLocalBefore;
            arm.upper->local = upperLocalBefore;
            arm.forearm1->local = forearm1LocalBefore;
            if (arm.forearm2) {
                arm.forearm2->local = forearm2LocalBefore;
            }
            if (arm.forearm3) {
                arm.forearm3->local = forearm3LocalBefore;
            }
            arm.hand->local = handLocalBefore;
            continuity = {};
            updateDown(arm.shoulder, true);
        });

        // Calibrated dimensions are solver-only.  Scaling these individual
        // bone offsets does not touch the actor root or weapon hierarchy.
        constexpr float referenceArmLength = 36.74f;
        float calibratedArmLength = isLeft ? g_config.leftArmLength : g_config.rightArmLength;
        if (!std::isfinite(calibratedArmLength) || calibratedArmLength <= 1.0f) {
            calibratedArmLength = std::isfinite(g_config.armLength) && g_config.armLength > 1.0f ? g_config.armLength : referenceArmLength;
        }
        const float adjustedArmLength = calibratedArmLength / referenceArmLength;

        const float shoulderWidthScale = std::clamp(g_config.shoulderWidth, 15.0f, 70.0f) / DEFAULT_SHOULDER_WIDTH;
        if (!std::isfinite(shoulderWidthScale) || !isFinite(arm.upper->local.translate)) {
            continuity = {};
            return;
        }
        arm.upper->local.translate *= shoulderWidthScale;
        updateDown(arm.shoulder, true);

        const float originalUpperLen = MatrixUtils::vec3Len(arm.forearm1->local.translate);
        const float originalForearmLen = _inPowerArmor
            ? MatrixUtils::vec3Len(arm.hand->local.translate)
            : MatrixUtils::vec3Len(arm.hand->local.translate) + MatrixUtils::vec3Len(arm.forearm2->local.translate) + MatrixUtils::vec3Len(arm.forearm3->local.translate);
        const float restArmLength = (originalUpperLen + originalForearmLen) * adjustedArmLength;
        if (!std::isfinite(restArmLength) || restArmLength <= 0.01f) {
            continuity = {};
            return;
        }

        // Parger distinct-shoulder estimation: use a dimensionless reach ratio,
        // preserving the current FRIK collarbone mechanism and skeleton scale.
        const RE::NiPoint3 shoulderToHand = handPos - arm.upper->world.translate;
        const float reachRatio = MatrixUtils::vec3Len(shoulderToHand) / restArmLength;
        const float shoulderReach = ik::smoothStep(0.5f, 1.05f, reachRatio);
        const RE::NiPoint3 shoulderOffset = safeNormalize(shoulderToHand) * (shoulderReach * restArmLength * 0.08f);

        RE::NiPoint3 clavicalToNewShoulder = arm.upper->world.translate + shoulderOffset - arm.shoulder->world.translate;

        if (!std::isfinite(arm.shoulder->world.scale) || std::abs(arm.shoulder->world.scale) <= 0.0001f) {
            continuity = {};
            return;
        }
        RE::NiPoint3 sLocalDir = arm.shoulder->world.rotate * (clavicalToNewShoulder / arm.shoulder->world.scale);

        if (ik::length(toIKVector(sLocalDir)) > 0.0001f) {
            RE::NiMatrix3 shoulderRotation;
            if (!tryGetRotationFromVectors(sLocalDir, RE::NiPoint3(1, 0, 0), shoulderRotation)) {
                return;
            }
            arm.shoulder->local.rotate = shoulderRotation * arm.shoulder->local.rotate;
        }

        updateDown(arm.shoulder, true);

        float negLeft = isLeft ? -1.0f : 1.0f;
        const RE::NiPoint3 forwardDir = safeNormalize(_forwardDir, RE::NiPoint3(0, 1, 0));
        const RE::NiPoint3 sidewaysDir = safeNormalize(_sidewaysRDir * negLeft, RE::NiPoint3(negLeft, 0, 0));
        const RE::NiPoint3 handBack = safeNormalize(handRot.Transpose() * RE::NiPoint3(-1, 0, 0), forwardDir * -1.0f);
        RE::NiPoint3 handSide = handRot.Transpose() * (RE::NiPoint3(0, -1, 0));
        RE::NiPoint3 handInSide = handSide * negLeft;
        const RE::NiPoint3 Uwp = arm.upper->world.translate;
        const ik::ArmSolveInput solveInput{ .shoulder = toIKVector(Uwp),
            .hand = toIKVector(handPos),
            .bodyForward = toIKVector(forwardDir),
            .bodyOutward = toIKVector(sidewaysDir),
            .bodyUp = { 0.0f, 0.0f, 1.0f },
            .handBack = toIKVector(handBack),
            .handSide = toIKVector(safeNormalize(handInSide, sidewaysDir)),
            .upperLength = originalUpperLen * adjustedArmLength,
            .lowerLength = originalForearmLen * adjustedArmLength,
            .deltaTime = _frameTime };
        auto candidateContinuity = continuity;
        const auto solve = ik::solveArm(solveInput, candidateContinuity);
        if (!solve.valid) {
            continuity = {};
            return;
        }

        const float forearmLen = solve.lowerLength;
        const RE::NiPoint3 elbowWorld = toNiPoint(solve.elbow);

        // This code below rotates and positions the upper arm, forearm, and hand bones
        // Notation: C=Clavicle, U=Upper arm, F=Forearm, H=hand   w=world, l=local   p=position, r=rotation, s=scale
        //    Rules: World position = Parent world pos + Parent world rot * (Local pos * Parent World scale)
        //           World Rotation = Parent world rotation * Local Rotation
        // ---------------------------------------------------------------------------------------------------------

        // The upper arm bone must be rotated from its forward vector to its shoulder-to-elbow vector in its local space
        // Calculate Ulr:  baseUwr * rotTowardElbow = Cwr * Ulr   ===>   Ulr = Cwr' * baseUwr * rotTowardElbow
        RE::NiMatrix3 Uwr = arm.upper->world.rotate;
        RE::NiPoint3 pos = elbowWorld - Uwp;
        if (!std::isfinite(arm.upper->world.scale) || std::abs(arm.upper->world.scale) <= 0.0001f) {
            return;
        }
        RE::NiPoint3 uLocalDir = Uwr * (safeNormalize(pos) / arm.upper->world.scale);

        RE::NiMatrix3 upperSwing;
        if (!tryGetRotationFromVectors(uLocalDir, arm.forearm1->local.translate, upperSwing)) {
            return;
        }
        arm.upper->local.rotate = upperSwing * arm.upper->local.rotate;

        Uwr = arm.upper->local.rotate * arm.shoulder->world.rotate;

        // The forearm arm bone must be rotated from its forward vector to its elbow-to-hand vector in its local space
        // Calculate Flr:  Fwr * rotTowardHand = Uwr * Flr   ===>   Flr = Uwr' * Fwr * rotTowardHand
        RE::NiMatrix3 Fwr = arm.forearm1->local.rotate * Uwr;
        RE::NiPoint3 elbowHand = handPos - elbowWorld;
        RE::NiPoint3 fLocalDir = Fwr * safeNormalize(elbowHand);

        RE::NiMatrix3 forearmSwing;
        if (!tryGetRotationFromVectors(fLocalDir, RE::NiPoint3(1, 0, 0), forearmSwing)) {
            return;
        }
        arm.forearm1->local.rotate = forearmSwing * arm.forearm1->local.rotate;
        Fwr = arm.forearm1->local.rotate * Uwr;

        RE::NiMatrix3 Fwr3;

        if (!_inPowerArmor && arm.forearm2 != nullptr && arm.forearm3 != nullptr) {
            auto Fwr2 = arm.forearm2->local.rotate * Fwr;
            Fwr3 = arm.forearm3->local.rotate * Fwr2;

            // Find the angle the wrist is pointing and twist forearm3 appropriately
            //    Fwr * twist = Uwr * Flr   ===>   Flr = (Uwr' * Fwr) * twist = (Flr) * twist

            RE::NiPoint3 wLocalDir = Fwr3 * safeNormalize(handInSide);
            wLocalDir.x = 0;
            RE::NiPoint3 forearm3Side = Fwr3.Transpose() * (RE::NiPoint3(0, 0, -1));
            // forearm is rotated 90 degrees already from hand so need this vector instead of 0,-1,0
            RE::NiPoint3 floc = Fwr2 * safeNormalize(forearm3Side);
            floc.x = 0;
            const RE::NiPoint3 normalizedWrist = safeNormalize(wLocalDir);
            const RE::NiPoint3 normalizedForearm = safeNormalize(floc);
            float fcos = std::clamp(MatrixUtils::vec3Dot(normalizedWrist, normalizedForearm), -1.0f, 1.0f);
            float fsin = MatrixUtils::vec3Det(normalizedWrist, normalizedForearm, RE::NiPoint3(-1, 0, 0));
            float forearmAngle = -1 * negLeft * atan2f(fsin, fcos);

            arm.forearm2->local.rotate = MatrixUtils::getMatrixFromEulerAngles(negLeft * forearmAngle / 2, 0, 0) * arm.forearm2->local.rotate;
            arm.forearm3->local.rotate = MatrixUtils::getMatrixFromEulerAngles(negLeft * forearmAngle / 2, 0, 0) * arm.forearm3->local.rotate;

            Fwr2 = arm.forearm2->local.rotate * Fwr;
            Fwr3 = arm.forearm3->local.rotate * Fwr2;
        }

        // Calculate Hlr:  Fwr * Hlr = handRot   ===>   Hlr = Fwr' * handRot
        arm.hand->local.rotate = handRot * (_inPowerArmor ? Fwr : Fwr3).Transpose();

        // Calculate Flp:  Fwp = Uwp + Uwr * (Flp * Uws) = elbowWorld   ===>   Flp = Uwr' * (elbowWorld - Uwp) / Uws
        arm.forearm1->local.translate = Uwr * ((elbowWorld - Uwp) / arm.upper->world.scale);

        const float forearmRatio = forearmLen / originalForearmLen;
        if (!std::isfinite(forearmRatio) || forearmRatio <= 0.0f || forearmRatio > 3.0f) {
            return;
        }

        if (arm.forearm2 && !_inPowerArmor) {
            arm.forearm2->local.translate *= forearmRatio;
            arm.forearm3->local.translate *= forearmRatio;
        }
        arm.hand->local.translate *= forearmRatio;

        const bool localPoseValid = isFinite(arm.shoulder->local) && isNearlyOrthonormal(arm.shoulder->local.rotate) && isFinite(arm.upper->local) &&
            isNearlyOrthonormal(arm.upper->local.rotate) && isFinite(arm.forearm1->local) && isNearlyOrthonormal(arm.forearm1->local.rotate) && isFinite(arm.hand->local) &&
            isNearlyOrthonormal(arm.hand->local.rotate) && (!arm.forearm2 || (isFinite(arm.forearm2->local) && isNearlyOrthonormal(arm.forearm2->local.rotate))) &&
            (!arm.forearm3 || (isFinite(arm.forearm3->local) && isNearlyOrthonormal(arm.forearm3->local.rotate)));
        if (!localPoseValid) {
            logger::sample("Rejected {} arm IK pose: invalid candidate local transform", isLeft ? "left" : "right");
            return;
        }

        updateDown(arm.shoulder, true);
        const float handPositionResidual = MatrixUtils::vec3Len(arm.hand->world.translate - handPos);
        const float elbowPositionResidual = MatrixUtils::vec3Len(arm.forearm1->world.translate - elbowWorld);
        const float handRotationResidual = maximumMatrixDifference(arm.hand->world.rotate, handRot);

        const bool geometryValid = isFinite(arm.shoulder->world) && isNearlyOrthonormal(arm.shoulder->world.rotate) && isFinite(arm.upper->world) &&
            isNearlyOrthonormal(arm.upper->world.rotate) && isFinite(arm.forearm1->world) && isNearlyOrthonormal(arm.forearm1->world.rotate) &&
            (!arm.forearm2 || (isFinite(arm.forearm2->world) && isNearlyOrthonormal(arm.forearm2->world.rotate))) &&
            (!arm.forearm3 || (isFinite(arm.forearm3->world) && isNearlyOrthonormal(arm.forearm3->world.rotate))) && isFinite(arm.hand->world) &&
            isNearlyOrthonormal(arm.hand->world.rotate);

        // These residuals exist to catch a pose that did not track the controller at
        // all, not to demand analytic precision from a chain that also applies
        // forearm-ratio and twist-bone scaling. The previous bounds (elbow within
        // 0.25 units, ~3.5mm) were far tighter than the chain can reproduce, and a
        // failure here rolls the whole arm back to its rest pose - which reads in VR
        // as the arm snapping. Only the hand target is worth rejecting over, scaled
        // to the arm rather than a fixed unit count; the elbow and wrist are cosmetic
        // and are reported instead.
        const float handResidualLimit = (std::max)(restArmLength * 0.1f, 2.0f);
        const bool handTracked = std::isfinite(handPositionResidual) && handPositionResidual <= handResidualLimit;
        if (!geometryValid || !handTracked) {
            logger::sample("Rejected {} arm IK pose: {} (hand pos {:.3f}/{:.3f}, elbow pos {:.3f}, hand rot {:.3f})", isLeft ? "left" : "right",
                geometryValid ? "hand missed target" : "invalid world transform", handPositionResidual, handResidualLimit, elbowPositionResidual, handRotationResidual);
            return;
        }

        // Diagnostic for the arm visibly snapping: the elbow pole swinging hard in a
        // single frame is exactly that symptom. Pole smoothing spreads a hemisphere
        // flip over several frames, so a per-frame threshold well under 180 degrees
        // is what catches it. Reported with the inputs that would explain it.
        if (continuity.hasPole && candidateContinuity.hasPole) {
            const float poleDot = std::clamp(ik::dot(continuity.pole, candidateContinuity.pole), -1.0f, 1.0f);
            const float poleSwingDegrees = MatrixUtils::radsToDegrees(std::acos(poleDot));
            if (std::isfinite(poleSwingDegrees) && poleSwingDegrees > 12.0f) {
                logger::sample(500, "[IKDIAG] {} elbow swing {:.1f}deg (reach {:.2f}, wrist {:.1f}deg, stretched {}, handResidual {:.2f})", isLeft ? "left" : "right",
                    poleSwingDegrees, solve.reachRatio, MatrixUtils::radsToDegrees(solve.wristCorrection), solve.stretched ? 1 : 0, handPositionResidual);
            }
        }

        continuity = candidateContinuity;
        poseRollback.release();
    }

    void Skeleton::hideHands() const
    {
        const RE::NiPoint3 rwp = _rightArm.shoulder->world.translate;
        _root->local.scale = 0.00001f;
        updateTransforms(_root);
        _root->world.translate += _forwardDir * -10.0f;
        _root->world.translate.z = rwp.z;
        updateDown(_root, false);
    }

    void Skeleton::calculateHandPose(const std::string& bone, const float gripProx, const bool thumbUp, const bool isLeft)
    {
        Quaternion qc, qt;
        const float sign = isLeft ? -1.0f : 1.0f;

        // if a mod is using the papyrus interface to manually set finger poses
        if (handPapyrusHasControl[bone]) {
            qt.fromMatrix(handOpen[bone].rotate);
            Quaternion qo;
            qo.fromMatrix(handClosed[bone].rotate);
            qo.slerp(std::clamp(handPapyrusPose[bone], -1.0f, 2.0f), qt);
            qt = qo;
        }
        // thumbUp pose
        else if (thumbUp && bone.find("Finger1") != std::string::npos) {
            if (bone.find("Finger11") != std::string::npos) {
                RE::NiMatrix3 wr = handOpen[bone].rotate;
                wr = MatrixUtils::getMatrixFromEulerAngles(sign * 0.5f, sign * 0.4f, -0.3f) * wr;
                qt.fromMatrix(wr);
            } else if (bone.find("Finger13") != std::string::npos) {
                RE::NiMatrix3 wr = handOpen[bone].rotate;
                wr = MatrixUtils::getMatrixFromEulerAngles(0, 0, MatrixUtils::degreesToRads(-35.0f)) * wr;
                qt.fromMatrix(wr);
            }
        } else if (_closedHand[bone]) {
            qt.fromMatrix(handClosed[bone].rotate);
        } else {
            qt.fromMatrix(handOpen[bone].rotate);
            if (_handBonesButton.at(bone) == k_EButton_Grip) {
                Quaternion qo;
                qo.fromMatrix(handClosed[bone].rotate);
                qo.slerp(1.0f - gripProx, qt);
                qt = qo;
            }
        }

        qc.fromMatrix(_handBones[bone].rotate);
        const float blend = ik::smoothingAlpha(_frameTime, 1.0f / 7.0f);
        qc.slerp(blend, qt);
        _handBones[bone].rotate = qc.getMatrix();
    }

    /**
     * Copy the 1st-person bone position for the given hand bone.
     * Useful for different weapons holding hand poses.
     */
    void Skeleton::copy1StPerson(const std::string& bone)
    {
        const auto fpTree = getFirstPersonBoneTree();
        const int pos = fpTree->GetBoneIndex(bone);
        if (pos >= 0) {
            if (fpTree->transforms[pos].refNode) {
                _handBones[bone] = fpTree->transforms[pos].refNode->local;
            } else {
                _handBones[bone] = fpTree->transforms[pos].local;
            }
        }
    }

    /**
     * In left-handed mode the 1st-person skeleton is not using the correct hand so we can't use "copy1StPerson" method.
     * Instead, we just force a specific hand pose that makes sense.
     */
    void Skeleton::setPredefinedHandPose(const std::string& bone)
    {
        Quaternion qo, qt;
        qt.fromMatrix(handOpen[bone].rotate);
        qo.fromMatrix(handClosed[bone].rotate);
        qo.slerp(std::clamp(getHandBonePose(bone, g_frik.isMeleeWeaponDrawn()), -1.0f, 2.0f), qt);
        _handBones[bone].rotate = qo.getMatrix();
    }

    void Skeleton::setHandPose()
    {
        const auto rt = reinterpret_cast<BSFlattenedBoneTree*>(_root);
        for (auto pos = 0; pos < rt->numTransforms; pos++) {
            std::string name = Skelly::getBoneName(pos);
            auto found = _fingerRelations.find(name);
            if (found != _fingerRelations.end()) {
                const bool isLeft = name[0] == 'L';
                const uint64_t reg = isLeft ? VRControllers.getControllerState_DEPRECATED(TrackerType::Left).ulButtonTouched
                                            : VRControllers.getControllerState_DEPRECATED(TrackerType::Right).ulButtonTouched;
                const float gripProx =
                    isLeft ? VRControllers.getControllerState_DEPRECATED(TrackerType::Left).rAxis[2].x : VRControllers.getControllerState_DEPRECATED(TrackerType::Right).rAxis[2].x;
                const bool thumbUp =
                    reg & ButtonMaskFromId(k_EButton_Grip) && reg & ButtonMaskFromId(k_EButton_SteamVR_Trigger) && !(reg & ButtonMaskFromId(k_EButton_SteamVR_Touchpad));
                _closedHand[name] = reg & ButtonMaskFromId(_handBonesButton.at(name));

                if (IsWeaponDrawn() && (isLeftHandedMode() || !g_frik.isPipboyOperatingWithFinger()) // left-handed has pipboy on the hand with the weapon
                    && !(isLeft ^ isLeftHandedMode())) {
                    if (isLeftHandedMode()) {
                        setPredefinedHandPose(name);
                    } else {
                        // use the game hand position for the weapon in hand
                        copy1StPerson(name);
                    }
                } else {
                    // use the forced hand position
                    calculateHandPose(name, gripProx, thumbUp, isLeft);
                }

                const RE::NiTransform trans = _handBones[name];

                rt->transforms[pos].local.rotate = trans.rotate;
                rt->transforms[pos].local.translate = handOpen[name.c_str()].translate;

                if (rt->transforms[pos].refNode) {
                    rt->transforms[pos].refNode->local = rt->transforms[pos].local;
                }
            }

            if (rt->transforms[pos].refNode) {
                rt->transforms[pos].world = rt->transforms[pos].refNode->world;
            } else {
                const short parent = rt->transforms[pos].parPos;
                RE::NiPoint3 p = rt->transforms[pos].local.translate;
                p = rt->transforms[parent].world.rotate.Transpose() * ((p * rt->transforms[parent].world.scale));

                rt->transforms[pos].world.translate = rt->transforms[parent].world.translate + p;

                rt->transforms[pos].world.rotate = rt->transforms[pos].local.rotate * rt->transforms[parent].world.rotate;
            }
        }
    }

    void Skeleton::dampenHand(RE::NiNode* node, const bool isLeft)
    {
        if (!node || !isFinite(node->world)) {
            if (isLeft) {
                _leftHandDampingPrimed = false;
            } else {
                _rightHandDampingPrimed = false;
            }
            return;
        }

        auto& prevFrame = isLeft ? _leftHandPrevFrame : _rightHandPrevFrame;
        auto& dampingPrimed = isLeft ? _leftHandDampingPrimed : _rightHandDampingPrimed;

        if (!g_config.dampenHands) {
            // Keep history current so enabling damping cannot interpolate from
            // a stale transform.
            prevFrame = node->world;
            dampingPrimed = true;
            return;
        }

        const bool isInScopeMenu = g_frik.isInScopeMenu();
        if (isInScopeMenu && !g_config.dampenHandsInVanillaScope) {
            prevFrame = node->world;
            dampingPrimed = true;
            return;
        }

        if (!dampingPrimed || !isFinite(prevFrame)) {
            prevFrame = node->world;
            dampingPrimed = true;
            return;
        }

        const RE::NiTransform currentFrame = node->world;
        const float rotationRetention = frameRateIndependentRetention(isInScopeMenu ? g_config.dampenHandsRotationInVanillaScope : g_config.dampenHandsRotation, _frameTime);
        const float translationRetention =
            frameRateIndependentRetention(isInScopeMenu ? g_config.dampenHandsTranslationInVanillaScope : g_config.dampenHandsTranslation, _frameTime);

        // Spherical interpolation between previous frame and current frame for the world rotation matrix
        Quaternion rq, rt;
        rq.fromMatrix(prevFrame.rotate);
        rt.fromMatrix(node->world.rotate);
        rq.slerp(1.0f - rotationRetention, rt);
        node->world.rotate = rq.getMatrix();
        if (!isFinite(node->world.rotate)) {
            node->world.rotate = currentFrame.rotate;
        }

        // Linear interpolation between the position from the previous frame to current frame
        // Compensate using corrected anatomical-pivot movement, so rotating the
        // HMD around the neck does not masquerade as player translation.
        const RE::NiPoint3 dir = _curentPosition - _lastPosition;
        RE::NiPoint3 deltaPos = node->world.translate - prevFrame.translate - dir;
        deltaPos *= translationRetention;
        node->world.translate -= deltaPos;
        if (!isFinite(node->world.translate)) {
            node->world.translate = currentFrame.translate;
        }

        prevFrame = node->world;

        updateDown(node, false);
    }

    /**
     * Default skeleton nodes position and rotation to be used for resetting skeleton before each frame update manipulations.
     * Required because loading a game does NOT reset the skeleton nodes resulting in incorrect positions/rotations.
     * Entering/Existing power-armor fixes the skeleton but loading the game over and over makes it worse.
     * By forcing the hardcoded default values the issue is prevented as we always start with the same initial values.
     * The values were collected by reading them from the skeleton nodes on first load of a saved game before any manipulations.
     */
    std::unordered_map<std::string, RE::NiTransform> Skeleton::getSkeletonNodesDefaultTransforms()
    {
        return std::unordered_map<std::string, RE::NiTransform>{
            { "Root", MatrixUtils::getTransform(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f) },
            { "COM", MatrixUtils::getTransform(0.0f, 0.0f, 68.91130f, 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f) },
            { "Pelvis", MatrixUtils::getTransform(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f) },
            { "LLeg_Thigh",
                MatrixUtils::getTransform(0.0f, 0.00040f, 6.61510f, -0.99112f, -0.00017f, -0.13297f, -0.03860f, 0.95730f, 0.28650f, 0.12725f, 0.28909f, -0.94881f, 1.0f) },
            { "LLeg_Calf", MatrixUtils::getTransform(31.59520f, 0.0f, 0.0f, 0.99210f, 0.12266f, -0.02618f, -0.12266f, 0.99245f, 0.00159f, 0.02617f, 0.00164f, 0.99966f, 1.0f) },
            { "LLeg_Foot", MatrixUtils::getTransform(31.94290f, 0.0f, 0.0f, 0.45330f, -0.88555f, -0.10159f, 0.88798f, 0.45855f, -0.03499f, 0.07757f, -0.07435f, 0.99421f, 1.0f) },
            { "RLeg_Thigh",
                MatrixUtils::getTransform(0.0f, 0.00040f, -6.61510f, -0.99307f, 0.00520f, 0.11741f, -0.02903f, 0.95721f, -0.28795f, -0.11389f, -0.28936f, -0.95042f, 1.0f) },
            { "RLeg_Calf", MatrixUtils::getTransform(31.59510f, 0.0f, 0.0f, 0.99108f, 0.13329f, 0.00011f, -0.13329f, 0.99108f, 0.00139f, 0.00007f, -0.00140f, 1.0f, 1.0f) },
            { "RLeg_Foot", MatrixUtils::getTransform(31.94260f, 0.0f, 0.0f, 0.44741f, -0.88731f, 0.11181f, 0.89061f, 0.45344f, 0.03463f, -0.08143f, 0.08409f, 0.99313f, 1.0f) },
            { "SPINE1", MatrixUtils::getTransform(3.792f, -0.00290f, 0.0f, 0.99246f, -0.12254f, 0.0f, 0.12254f, 0.99246f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f) },
            { "SPINE2", MatrixUtils::getTransform(8.70470f, 0.0f, 0.0f, 0.98463f, 0.17464f, 0.0f, -0.17464f, 0.98463f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f) },
            { "Chest", MatrixUtils::getTransform(9.95630f, 0.0f, 0.0f, 0.99983f, -0.01837f, 0.0f, 0.01837f, 0.99983f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f) },
            { "LArm_Collarbone",
                MatrixUtils::getTransform(19.15320f, -0.51040f, 1.69510f, -0.40489f, -0.00599f, -0.91434f, -0.26408f, 0.95813f, 0.11066f, 0.87540f, 0.28627f, -0.38952f, 1.0f) },
            { "LArm_UpperArm",
                MatrixUtils::getTransform(12.53660f, 0.0f, 0.0f, 0.91617f, -0.25279f, -0.31102f, 0.25328f, 0.96658f, -0.03954f, 0.31062f, -0.04255f, 0.94958f, 1.0f) },
            { "LArm_ForeArm1",
                MatrixUtils::getTransform(17.96830f, 0.0f, 0.0f, 0.85511f, -0.51462f, -0.06284f, 0.51548f, 0.85690f, -0.00289f, 0.05534f, -0.02992f, 0.99802f, 1.0f) },
            { "LArm_ForeArm2", MatrixUtils::getTransform(6.15160f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.99999f, -0.00536f, 0.0f, 0.00536f, 0.99999f, 1.0f) },
            { "LArm_ForeArm3", MatrixUtils::getTransform(6.15160f, -0.00010f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.99999f, -0.00536f, 0.0f, 0.00536f, 0.99999f, 1.0f) },
            { "LArm_Hand", MatrixUtils::getTransform(6.15160f, 0.0f, -0.00010f, 0.98845f, 0.14557f, -0.04214f, 0.04136f, 0.00839f, 0.99911f, 0.14579f, -0.98931f, 0.00227f, 1.0f) },
            { "RArm_Collarbone",
                MatrixUtils::getTransform(19.15320f, -0.51040f, -1.69510f, -0.40497f, -0.00602f, 0.91431f, -0.26413f, 0.95811f, -0.11069f, -0.87535f, -0.28632f, -0.38960f, 1.0f) },
            { "RArm_UpperArm", MatrixUtils::getTransform(12.53430f, 0.0f, 0.0f, 0.91620f, -0.25314f, 0.31064f, 0.25365f, 0.96649f, 0.03947f, -0.31022f, 0.04263f, 0.94971f, 1.0f) },
            { "RArm_ForeArm1",
                MatrixUtils::getTransform(17.97050f, 0.00010f, -0.00010f, 0.85532f, -0.51419f, 0.06360f, 0.51507f, 0.85714f, 0.00288f, -0.05599f, 0.03030f, 0.99797f, 1.0f) },
            { "RArm_ForeArm2", MatrixUtils::getTransform(6.15280f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.99999f, 0.00536f, 0.0f, -0.00536f, 0.99999f, 1.0f) },
            { "RArm_ForeArm3", MatrixUtils::getTransform(6.15290f, 0.0f, -0.00010f, 1.0f, 0.0f, 0.0f, 0.0f, 0.99999f, 0.00536f, 0.0f, -0.00536f, 0.99999f, 1.0f) },
            { "RArm_Hand", MatrixUtils::getTransform(6.15290f, 0.0f, 0.0f, 0.98845f, 0.14557f, 0.04214f, 0.04136f, 0.00839f, -0.99911f, -0.14579f, 0.98931f, 0.00227f, 1.0f) },
            { "Neck", MatrixUtils::getTransform(22.084f, -3.767f, 0.0f, 0.91268f, -0.40867f, -0.00003f, 0.40867f, 0.91268f, 0.0f, 0.00002f, -0.00001f, 1.0f, 1.0f) },
            { "Head", MatrixUtils::getTransform(8.22440f, 0.0f, 0.0f, 0.94872f, 0.31613f, 0.00002f, -0.31613f, 0.94872f, -0.00001f, -0.00003f, 0.0f, 1.0f, 1.0f) },
        };
    }

    // See "getSkeletonNodesDefaultTransforms" above
    std::unordered_map<std::string, RE::NiTransform> Skeleton::getSkeletonNodesDefaultTransformsInPA()
    {
        return std::unordered_map<std::string, RE::NiTransform>{
            { "Root", MatrixUtils::getTransform(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f) },
            { "COM", MatrixUtils::getTransform(0.0f, -3.74980f, 89.41950f, 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f) },
            { "Pelvis", MatrixUtils::getTransform(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f) },
            { "LLeg_Thigh",
                MatrixUtils::getTransform(4.54870f, -1.33f, 6.90830f, -0.98736f, 0.14491f, 0.06416f, 0.06766f, 0.01940f, 0.99752f, 0.14331f, 0.98925f, -0.02896f, 1.0f) },
            { "LLeg_Calf", MatrixUtils::getTransform(34.298f, 0.0f, 0.0f, 0.99681f, -0.00145f, 0.07983f, 0.00170f, 0.99999f, -0.00305f, -0.07982f, 0.00318f, 0.99680f, 1.0f) },
            { "LLeg_Foot", MatrixUtils::getTransform(52.54120f, 0.0f, 0.0f, 0.63109f, -0.76168f, -0.14685f, -0.07775f, 0.12624f, -0.98895f, 0.77180f, 0.63554f, 0.02045f, 1.0f) },
            { "RLeg_Thigh",
                MatrixUtils::getTransform(4.54760f, -1.32430f, -6.898f, -0.98732f, 0.14533f, -0.06381f, 0.06732f, 0.01938f, -0.99754f, -0.14374f, -0.98919f, -0.02892f, 1.0f) },
            { "RLeg_Calf", MatrixUtils::getTransform(34.29790f, 0.0f, 0.0f, 0.99684f, -0.00096f, -0.07937f, 0.00120f, 0.99999f, 0.00307f, 0.07937f, -0.00316f, 0.99684f, 1.0f) },
            { "RLeg_Foot", MatrixUtils::getTransform(52.54080f, 0.0f, 0.0f, 0.63118f, -0.76162f, 0.14677f, -0.07771f, 0.12618f, 0.98896f, -0.77173f, -0.63562f, 0.02046f, 1.0f) },
            { "SPINE1", MatrixUtils::getTransform(5.75050f, -0.00290f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f) },
            { "SPINE2", MatrixUtils::getTransform(5.62550f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f) },
            { "Chest", MatrixUtils::getTransform(5.53660f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f) },
            { "LArm_Collarbone",
                MatrixUtils::getTransform(22.192f, 0.34820f, 1.00420f, -0.34818f, -0.05435f, -0.93585f, -0.26919f, 0.96207f, 0.04428f, 0.89794f, 0.26734f, -0.34961f, 1.0f) },
            { "LArm_UpperArm",
                MatrixUtils::getTransform(14.59840f, 0.00010f, 0.00010f, 0.77214f, -0.19393f, -0.60514f, 0.08574f, 0.97538f, -0.20318f, 0.62964f, 0.10499f, 0.76976f, 1.0f) },
            { "LArm_ForeArm1",
                MatrixUtils::getTransform(19.53690f, 0.41980f, 0.04580f, 0.92233f, -0.38166f, -0.06030f, 0.38176f, 0.92420f, -0.01042f, 0.05971f, -0.01341f, 0.99813f, 1.0f) },
            { "LArm_ForeArm2", MatrixUtils::getTransform(0.00020f, 0.00020f, 0.00020f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f) },
            { "LArm_ForeArm3", MatrixUtils::getTransform(10.000494f, 0.000162f, -0.000004f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f) },
            { "LArm_Hand",
                MatrixUtils::getTransform(26.96440f, 0.00020f, 0.00040f, 0.98604f, 0.16503f, 0.02218f, 0.00691f, -0.17364f, 0.98479f, 0.16638f, -0.97088f, -0.17236f, 1.0f) },
            { "RArm_Collarbone",
                MatrixUtils::getTransform(22.19190f, 0.34810f, -1.004f, -0.34818f, -0.06482f, 0.93518f, -0.26918f, 0.96251f, -0.03351f, -0.89795f, -0.26340f, -0.35257f, 1.0f) },
            { "RArm_UpperArm",
                MatrixUtils::getTransform(14.59880f, 0.0f, 0.0f, 0.77213f, -0.19339f, 0.60533f, 0.09277f, 0.97667f, 0.19369f, -0.62866f, -0.09340f, 0.77205f, 1.0f) },
            { "RArm_ForeArm1",
                MatrixUtils::getTransform(19.53660f, 0.41990f, -0.04620f, 0.92233f, -0.38166f, 0.06029f, 0.38171f, 0.92422f, 0.01129f, -0.06003f, 0.01260f, 0.99812f, 1.0f) },
            { "RArm_ForeArm2", MatrixUtils::getTransform(-0.00010f, -0.00010f, -0.00010f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f) },
            { "RArm_ForeArm3", MatrixUtils::getTransform(10.00050f, -0.00010f, 0.00010f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f) },
            { "RArm_Hand",
                MatrixUtils::getTransform(26.96460f, 0.00010f, 0.00120f, 0.98604f, 0.16503f, -0.02218f, 0.00691f, -0.17364f, -0.98479f, -0.16638f, 0.97088f, -0.17236f, 1.0f) },
            { "Neck", MatrixUtils::getTransform(24.29350f, -2.84160f, 0.0f, 0.92612f, -0.37723f, -0.00002f, 0.37723f, 0.92612f, 0.00001f, 0.00002f, -0.00002f, 1.0f, 1.0f) },
            { "Head", MatrixUtils::getTransform(8.22440f, 0.0f, 0.0f, 0.94891f, 0.31555f, 0.00002f, -0.31555f, 0.94891f, 0.0f, -0.00002f, -0.00001f, 1.0f, 1.0f) },
        };
    }

    std::map<std::string, std::pair<std::string, std::string>> Skeleton::makeFingerRelations()
    {
        std::map<std::string, std::pair<std::string, std::string>> map;

        auto addFingerRelations = [&](const std::string& hand, const std::string& finger1, const std::string& finger2, const std::string& finger3) {
            map.insert({ finger1, { hand, finger2 } });
            map.insert({ finger2, { finger1, finger3 } });
            map.insert({ finger3, { finger2, std::string() } });
        };

        //left hand
        addFingerRelations("LArm_Hand", "LArm_Finger11", "LArm_Finger12", "LArm_Finger13");
        addFingerRelations("LArm_Hand", "LArm_Finger21", "LArm_Finger22", "LArm_Finger23");
        addFingerRelations("LArm_Hand", "LArm_Finger31", "LArm_Finger32", "LArm_Finger33");
        addFingerRelations("LArm_Hand", "LArm_Finger41", "LArm_Finger42", "LArm_Finger43");
        addFingerRelations("LArm_Hand", "LArm_Finger51", "LArm_Finger52", "LArm_Finger53");

        //right hand
        addFingerRelations("RArm_Hand", "RArm_Finger11", "RArm_Finger12", "RArm_Finger13");
        addFingerRelations("RArm_Hand", "RArm_Finger21", "RArm_Finger22", "RArm_Finger23");
        addFingerRelations("RArm_Hand", "RArm_Finger31", "RArm_Finger32", "RArm_Finger33");
        addFingerRelations("RArm_Hand", "RArm_Finger41", "RArm_Finger42", "RArm_Finger43");
        addFingerRelations("RArm_Hand", "RArm_Finger51", "RArm_Finger52", "RArm_Finger53");

        return map;
    }

    /**
     * setup hand bones to openvr button mapping
     */
    std::unordered_map<std::string, VRButtonId> Skeleton::getHandBonesButtonMap()
    {
        return std::unordered_map<std::string, VRButtonId>{ { "LArm_Finger11", k_EButton_SteamVR_Touchpad }, { "LArm_Finger12", k_EButton_SteamVR_Touchpad },
            { "LArm_Finger13", k_EButton_SteamVR_Touchpad }, { "LArm_Finger21", k_EButton_SteamVR_Trigger }, { "LArm_Finger22", k_EButton_SteamVR_Trigger },
            { "LArm_Finger23", k_EButton_SteamVR_Trigger }, { "LArm_Finger31", k_EButton_Grip }, { "LArm_Finger32", k_EButton_Grip }, { "LArm_Finger33", k_EButton_Grip },
            { "LArm_Finger41", k_EButton_Grip }, { "LArm_Finger42", k_EButton_Grip }, { "LArm_Finger43", k_EButton_Grip }, { "LArm_Finger51", k_EButton_Grip },
            { "LArm_Finger52", k_EButton_Grip }, { "LArm_Finger53", k_EButton_Grip }, { "RArm_Finger11", k_EButton_SteamVR_Touchpad },
            { "RArm_Finger12", k_EButton_SteamVR_Touchpad }, { "RArm_Finger13", k_EButton_SteamVR_Touchpad }, { "RArm_Finger21", k_EButton_SteamVR_Trigger },
            { "RArm_Finger22", k_EButton_SteamVR_Trigger }, { "RArm_Finger23", k_EButton_SteamVR_Trigger }, { "RArm_Finger31", k_EButton_Grip },
            { "RArm_Finger32", k_EButton_Grip }, { "RArm_Finger33", k_EButton_Grip }, { "RArm_Finger41", k_EButton_Grip }, { "RArm_Finger42", k_EButton_Grip },
            { "RArm_Finger43", k_EButton_Grip }, { "RArm_Finger51", k_EButton_Grip }, { "RArm_Finger52", k_EButton_Grip }, { "RArm_Finger53", k_EButton_Grip } };
    }
}
