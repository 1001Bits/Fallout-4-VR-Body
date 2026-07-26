#pragma once

#include <map>

#include "CullGeometryHandler.h"
#include "SelfieHandler.h"
#include "common/CommonUtils.h"
#include "f4vr/PlayerNodes.h"
#include "vrcf/VRControllersManager.h"

namespace frik
{
    struct ArmNodes
    {
        RE::NiAVObject* shoulder = nullptr;
        RE::NiAVObject* upper = nullptr;
        RE::NiAVObject* upperT1 = nullptr;
        RE::NiAVObject* forearm1 = nullptr;
        RE::NiAVObject* forearm2 = nullptr;
        RE::NiAVObject* forearm3 = nullptr;
        RE::NiAVObject* hand = nullptr;
    };

    struct LegNodes
    {
        RE::NiNode* hip = nullptr;
        RE::NiNode* knee = nullptr;
        RE::NiNode* foot = nullptr;
    };

    /**
     * One coherent HMD sample used by every body-IK calculation in a frame.
     *
     * Fallout's NiMatrix convention stores the world-to-local rotation in the
     * world transform.  A local offset is therefore moved to world space with
     * world.rotate.Transpose().
     */
    struct TrackedHeadPose
    {
        RE::NiTransform raw;
        RE::NiPoint3 pivot;
    };

    class Skeleton
    {
    public:
        Skeleton(RE::NiNode* rootNode, const bool inPowerArmor) :
            _root(rootNode), _inPowerArmor(inPowerArmor)
        {
            _curentPosition = RE::NiPoint3(0, 0, 0);
            _lastPosition = _curentPosition;
            _forwardDir = RE::NiPoint3(0, 1, 0);
            _sidewaysRDir = RE::NiPoint3(1, 0, 0);
            _walkingState = 0;
            initializeNodes();
        }

        ArmNodes getLeftArm() const
        {
            return _leftArm;
        }

        ArmNodes getRightArm() const
        {
            return _rightArm;
        }

        static float getAdjustedPlayerHMDOffset();

        void onFrameUpdate();

    private:
        // initialization
        void initializeNodes();
        void initArmsNodes();
        void initSkeletonNodesDefaults();
        void setBodyLen();

        // on frame update - skeleton update
        void setTime();
        bool sampleTrackedHeadPose();
        void resetMotionState();
        void resetWalkingState();
        bool hasRequiredNodes() const;
        bool canUseProceduralLegs() const;
        void restoreNodesToDefault();
        void setupHead(float neckYaw, float neckPitch) const;
        void setBodyUnderHMD(float neckYaw);
        void setBodyPosture(float neckPitch);
        void setKneePos();
        void walk();
        void setSingleLeg(bool isLeft) const;
        void handleLeftHandedWeaponNodesSwitch();
        void setArms(bool isLeft);
        void dampenHand(RE::NiNode* node, bool isLeft);
        void hide3rdPersonWeapon() const;
        void hideFistHelpers() const;
        void showHidePAHud() const;
        void setHandPose();
        void hideHands() const;
        void fixArmor() const;

        // Utils
        void calculateHandPose(const std::string& bone, float gripProx, bool thumbUp, bool isLeft);
        void copy1StPerson(const std::string& bone);
        void setPredefinedHandPose(const std::string& bone);

        // Utils - Body Positioning
        float getNeckYaw();
        float getNeckPitch() const;
        float getBodyPitch(float neckPitch) const;
        float getCorrectedUprightHmdHeight() const;
        void rotateLeg(uint32_t pos, float angle) const;

        // root node and is in power armor define the Skeleton instance
        RE::NiNode* _root;
        bool _inPowerArmor;

        // ???
        LARGE_INTEGER _freqCounter;
        LARGE_INTEGER _timer;
        LARGE_INTEGER _prevTime;
        float _frameTime;
        bool _timeDiscontinuity = false;

        // handle switch of hands for left-handed mode
        bool _lastLeftHandedModeSwitch = false;

        // Camera positions
        RE::NiPoint3 _curentPosition;
        RE::NiPoint3 _lastPosition;
        TrackedHeadPose _trackedHeadPose;
        RE::NiPoint3 _lastPivotOffset;
        bool _hasValidTrackedHeadPose = false;
        bool _trackingWasValid = false;
        bool _hasLastPivotConfig = false;
        bool _lastPivotCorrectionEnabled = false;
        float _lastNeckYaw = 0.0f;
        inline static float _comfortSneakCameraOffsetAdjustment = -1.0f;

        // ???
        RE::NiPoint3 _forwardDir;
        RE::NiPoint3 _sidewaysRDir;
        RE::NiPoint3 _upDir;

        // skeleton nodes
        f4vr::PlayerNodes* _playerNodes;
        RE::NiNode* _rightHand = nullptr;
        RE::NiNode* _leftHand = nullptr;
        RE::NiNode* _head = nullptr;
        RE::NiNode* _spine = nullptr;
        RE::NiNode* _chest = nullptr;
        RE::NiNode* _com = nullptr;
        RE::NiNode* _neck = nullptr;
        RE::NiNode* _spine1 = nullptr;
        float _torsoLen = 0.0f;
        float _legLen = 0.0f;
        ArmNodes _rightArm;
        ArmNodes _leftArm;
        LegNodes _rightLeg;
        LegNodes _leftLeg;

        // Default transform are used to reset the skeleton before each frame update to start from scratch
        std::vector<std::pair<RE::NiAVObject*, const RE::NiTransform>> _skeletonNodesToDefaultTransforms;
        static std::unordered_map<std::string, RE::NiTransform> getSkeletonNodesDefaultTransforms();
        static std::unordered_map<std::string, RE::NiTransform> getSkeletonNodesDefaultTransformsInPA();
        inline static const std::unordered_map<std::string, RE::NiTransform> _skeletonNodesDefaultTransform = getSkeletonNodesDefaultTransforms();
        inline static const std::unordered_map<std::string, RE::NiTransform> _skeletonNodesDefaultTransformInPA = getSkeletonNodesDefaultTransformsInPA();

        // legs walking stuff
        int _walkingState;
        float _currentStepTime;
        RE::NiPoint3 _leftFootPos;
        RE::NiPoint3 _rightFootPos;
        RE::NiPoint3 _rightFootTarget;
        RE::NiPoint3 _leftFootTarget;
        RE::NiPoint3 _rightFootStart;
        RE::NiPoint3 _leftFootStart;
        RE::NiPoint3 _leftKneePosture;
        RE::NiPoint3 _rightKneePosture;
        RE::NiPoint3 _leftKneePos;
        RE::NiPoint3 _rightKneePos;
        int _footStepping;
        RE::NiPoint3 _stepDir;
        float _prevSpeed;
        float _stepTimeinStep;
        float _directionChangeDelayRemaining = 0.0f;
        float _spineAngle = 0.0f;
        bool _solveLegsThisFrame = false;

        std::map<std::string, RE::NiTransform, common::CaseInsensitiveComparator> _handBones;
        std::map<std::string, bool, common::CaseInsensitiveComparator> _closedHand;
        static std::unordered_map<std::string, vrcf::VRButtonId> getHandBonesButtonMap();
        inline static const std::unordered_map<std::string, vrcf::VRButtonId> _handBonesButton = getHandBonesButtonMap();

        RE::NiTransform _rightHandPrevFrame;
        RE::NiTransform _leftHandPrevFrame;
        bool _rightHandDampingPrimed = false;
        bool _leftHandDampingPrimed = false;

        // bones
        static std::map<std::string, std::pair<std::string, std::string>> makeFingerRelations();
        inline static const std::map<std::string, std::pair<std::string, std::string>> _fingerRelations = makeFingerRelations();

        // cull (hide) parts of the skeleton (head, equipment)
        CullGeometryHandler _cullGeometry;

        SelfieHandler _selfieHandler;
    };
}
