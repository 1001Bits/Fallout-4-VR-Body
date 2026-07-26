#pragma once

namespace frik::ik
{
    /** Wrap an angle into [-pi, pi]. */
    float wrapAngle(float radians);

    /**
     * Smooth 0 -> 1 ramp used to blend a procedural gait out instead of snapping.
     *
     * FRIK's "stopping" state used to drop both feet onto their rest positions in a
     * single frame, which pops every time you stop walking.
     */
    float stopBlend(float elapsedSeconds, float durationSeconds);

    /**
     * Accumulates absolute body yaw so a turn on the spot can trigger a step.
     *
     * Standing still, the feet are re-derived from the rest pose every frame, so they
     * rotate with the body instead of stepping. Tracking how far the body has turned
     * since the last plant gives a step something to trigger on.
     */
    struct TurnAccumulator
    {
        float accumulated = 0.0f;
        float lastYaw = 0.0f;
        bool hasYaw = false;

        void reset();

        /**
         * Feed the current body yaw. Returns true once `thresholdRadians` of turning
         * has built up, and consumes that amount so the next step needs a fresh turn.
         * Non-finite input resets rather than poisoning the accumulator.
         */
        bool update(float bodyYaw, float thresholdRadians);
    };
}
