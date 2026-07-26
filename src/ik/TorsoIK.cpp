#include "TorsoIK.h"

#include <algorithm>

#include "ArmIK.h"

namespace frik::ik
{
    TorsoTwist distributeTorsoTwist(const float bodyYaw, const TorsoTwistSettings& settings)
    {
        TorsoTwist result;
        if (!isFinite(bodyYaw)) {
            return result;
        }

        // Legacy behaviour: the root carries the whole yaw.
        result.root = bodyYaw;

        const float share = isFinite(settings.share) ? std::clamp(settings.share, 0.0f, 1.0f) : 0.0f;
        if (share <= 0.0f) {
            return result;
        }

        const float spineFraction = isFinite(settings.spineFraction) ? std::clamp(settings.spineFraction, 0.0f, 1.0f) : 0.0f;
        const float spineLimit = isFinite(settings.spineLimit) ? (std::max)(settings.spineLimit, 0.0f) : 0.0f;
        const float chestLimit = isFinite(settings.chestLimit) ? (std::max)(settings.chestLimit, 0.0f) : 0.0f;

        const float torsoYaw = bodyYaw * share;
        result.spine = std::clamp(torsoYaw * spineFraction, -spineLimit, spineLimit);

        // The chest absorbs the rest of the moved yaw, including anything the spine
        // limit rejected, up to its own anatomical cap.
        result.chest = std::clamp(torsoYaw - result.spine, -chestLimit, chestLimit);

        // Whatever neither spine joint could take stays on the root, so the chest
        // still reaches its legacy orientation.
        result.root = bodyYaw - result.spine - result.chest;
        return result;
    }
}
