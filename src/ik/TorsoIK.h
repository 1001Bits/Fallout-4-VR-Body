#pragma once

namespace frik::ik
{
    /**
     * How much of the inferred body yaw is taken off the avatar root and given to
     * the spine.
     *
     * FRIK yaws the whole avatar root toward the inferred body direction, so the
     * pelvis, legs, and feet swing whenever the player turns only their upper body.
     * Moving part of that yaw into the spine leaves the pelvis planted while the
     * chest still reaches the orientation the root-only rotation produced.
     *
     * Every spine-chain bone ("SPINE1" -> "SPINE2" -> "Chest" -> "Head") is offset
     * from its parent purely along local X, so local X is the twist axis and these
     * angles are applied about it.
     */
    struct TorsoTwistSettings
    {
        float share = 0.0f; // fraction of the body yaw moved off the root (0 disables)
        float spineFraction = 0.4f; // portion of the moved yaw taken by SPINE2
        float spineLimit = 0.0f; // radians, absolute cap for SPINE2
        float chestLimit = 0.0f; // radians, absolute cap for Chest
    };

    struct TorsoTwist
    {
        float root = 0.0f; // yaw left on the avatar root
        float spine = 0.0f; // SPINE2 twist about its local bone axis
        float chest = 0.0f; // Chest twist about its local bone axis
    };

    /**
     * Split `bodyYaw` between the avatar root and the spine.
     *
     * root + spine + chest always equals `bodyYaw`, so the chest lands in the
     * orientation the legacy root-only rotation produced whatever the shares and
     * limits are. Yaw that the clamped spine joints cannot absorb stays on the root
     * instead of being silently dropped.
     */
    TorsoTwist distributeTorsoTwist(float bodyYaw, const TorsoTwistSettings& settings);
}
