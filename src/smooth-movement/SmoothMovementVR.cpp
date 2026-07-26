#include "SmoothMovementVR.h"

#include "Config.h"
#include "FRIK.h"

using namespace common;

// Adapted from original code by Shizof mod with permission.  Thanks Shizof!!

namespace frik
{
    namespace
    {
        bool isFinitePoint(const RE::NiPoint3& point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        }
    }

    void SmoothMovementVR::reset()
    {
        _notMoving = false;
        _activeLastFrame = false;
        _lastPositions.clear();
        _smoothedPos = RE::NiPoint3();
        _lastAppliedLocalX = 0.0f;
        _lastAppliedLocalY = 0.0f;
        _frameTime = 0.0f;
        QueryPerformanceCounter(&_prevTime);
    }

    void SmoothMovementVR::resetAt(const RE::NiPoint3& position)
    {
        reset();
        _smoothedPos = position;
        _lastPositions.emplace_back(position);
        _activeLastFrame = true;
    }

    void SmoothMovementVR::onFrameUpdate()
    {
        if (g_config.disableSmoothMovement) {
            if (_activeLastFrame) {
                reset();
            }
            return;
        }
        const auto playerNodes = f4vr::getPlayerNodes();
        if (!playerNodes || !playerNodes->playerworldnode) {
            reset();
            return;
        }

        const auto player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            reset();
            return;
        }
        const RE::NiPoint3 curPos = player->GetPosition();
        if (!isFinitePoint(curPos)) {
            logger::warn("[SmoothMovement] Ignoring non-finite player position");
            reset();
            return;
        }

        if (!_activeLastFrame || _lastPositions.empty()) {
            resetAt(curPos);
        } else if (MatrixUtils::distanceNoSqrt(curPos, _smoothedPos) > 4000000.0f) {
            logger::info("[SmoothMovement] Reset after teleport/recenter; curPos:({:.2f}, {:.2f}, {:.2f}), previous:({:.2f}, {:.2f}, {:.2f})",
                curPos.x, curPos.y, curPos.z, _smoothedPos.x, _smoothedPos.y, _smoothedPos.z);
            resetAt(curPos);
        }

        if (_lastPositions.size() >= 4) {
            const RE::NiPoint3 pos = _lastPositions.at(0);
            bool same = true;
            for (unsigned int i = 1; i < _lastPositions.size(); i++) {
                if (fNotEqual(_lastPositions.at(i).x, pos.x) || fNotEqual(_lastPositions.at(i).y, pos.y)) {
                    same = false;
                    break;
                }
            }
            _notMoving = same;
        } else {
            _notMoving = false;
        }

        _lastPositions.emplace_back(curPos);
        if (_lastPositions.size() > 5) {
            _lastPositions.pop_front();
        }

        const auto newPos = smoothedValue(curPos, _smoothedPos);
        _smoothedPos = newPos;

        auto& playerLocalTransformPos = playerNodes->playerworldnode->local.translate;
        if (_notMoving && MatrixUtils::distanceNoSqrt2d(newPos.x - curPos.x, newPos.y - curPos.y, _lastAppliedLocalX, _lastAppliedLocalY) > 100) {
            _smoothedPos = curPos;
            playerLocalTransformPos.z = 0;
            logger::sample("[SmoothMovement] Not moving values exceed normal; curPos:({:.2f}, {:.2f}), newPos:({:.2f}, {:.2f}), lastApplied:({:.2f}, {:.2f})",
                curPos.x, curPos.y, newPos.x, newPos.y, _lastAppliedLocalX, _lastAppliedLocalY);
        } else {
            playerLocalTransformPos = newPos - curPos;
            _lastAppliedLocalX = playerLocalTransformPos.x;
            _lastAppliedLocalY = playerLocalTransformPos.y;
        }

        // logger::sample("[SmoothMovement] curPos:({:.2f}, {:.2f}, {:.2f}), newPos:({:.2f}, {:.2f}, {:.2f}), appliedPos:({:.2f}, {:.2f}, {:.2f})",
        // 	curPos.x, curPos.y, curPos.z, newPos.x, newPos.y, newPos.z, playerLocalTransformPos.x, playerLocalTransformPos.y, playerLocalTransformPos.z);

        playerLocalTransformPos.z += Skeleton::getAdjustedPlayerHMDOffset();
    }

    /**
     * Calculate the new smoothed position based on the player current position and the previous smoothed position.
     */
    RE::NiPoint3 SmoothMovementVR::smoothedValue(const RE::NiPoint3& curPos, const RE::NiPoint3& prevPos)
    {
        LARGE_INTEGER newTime;
        QueryPerformanceCounter(&newTime);
        _frameTime = std::clamp(static_cast<float>(newTime.QuadPart - _prevTime.QuadPart) / static_cast<float>(_hpcFrequency.QuadPart), 0.0001f, 0.05f);
        _prevTime = newTime;

        if (g_config.disableInteriorSmoothingHorizontal && f4vr::isInInternalCell()) {
            // don't smooth if in interior cell and smoothing is disabled for it
            return curPos;
        }

        auto newPos = RE::NiPoint3(curPos.x, curPos.y, curPos.z);
        if (fNotEqual(g_config.dampingMultiplierHorizontal, 0) && fNotEqual(g_config.smoothingAmountHorizontal, 0)) {
            // DO smoothing
            const float absValX = min(50, max(0.1f, abs(curPos.x - prevPos.x)));
            const float tauX =
                g_config.smoothingAmountHorizontal * (g_config.dampingMultiplierHorizontal / absValX) * (_notMoving ? g_config.stoppingMultiplierHorizontal : 1.0f);
            const float alphaX = std::clamp(_frameTime / (std::max)(tauX, 0.0001f), 0.0f, 1.0f);
            newPos.x = prevPos.x + alphaX * (curPos.x - prevPos.x);

            const float absValY = min(50, max(0.1f, abs(curPos.y - prevPos.y)));
            const float tauY =
                g_config.smoothingAmountHorizontal * (g_config.dampingMultiplierHorizontal / absValY) * (_notMoving ? g_config.stoppingMultiplierHorizontal : 1.0f);
            const float alphaY = std::clamp(_frameTime / (std::max)(tauY, 0.0001f), 0.0f, 1.0f);
            newPos.y = prevPos.y + alphaY * (curPos.y - prevPos.y);
        } else {
            newPos.x = curPos.x;
            newPos.y = curPos.y;
        }

        // Don't smooth vertical movement if jumping or in air as it will break the jump
        if (!f4vr::isJumpingOrInAir() && fNotEqual(g_config.dampingMultiplier, 0) && fNotEqual(g_config.smoothingAmount, 0)) {
            const float absVal = min(50, max(0.1f, abs(curPos.z - prevPos.z)));
            const float tau = g_config.smoothingAmount * (g_config.dampingMultiplier / absVal) * (_notMoving ? g_config.stoppingMultiplier : 1.0f);
            const float alpha = std::clamp(_frameTime / (std::max)(tau, 0.0001f), 0.0f, 1.0f);
            newPos.z = prevPos.z + alpha * (curPos.z - prevPos.z);
        } else {
            newPos.z = curPos.z;
        }

        return newPos;
    }
}
