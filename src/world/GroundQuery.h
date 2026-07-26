#pragma once

namespace frik::world
{
    struct GroundHit
    {
        RE::NiPoint3 position; // world space, game units
        RE::NiPoint3 normal; // unit length, world space
        bool hit = false;
    };

    /**
     * Cast a ray through the player cell's Havok world.
     *
     * `start` and `end` are in game units; the engine's SetStartEnd applies the
     * Havok unit scale itself. The hit point is rebuilt by interpolating the
     * game-unit endpoints with the dimensionless hit fraction, so no Havok-space
     * position is ever converted back.
     *
     * Returns a miss (rather than throwing or asserting) whenever the player, cell,
     * collision world, or result is unusable, so callers can always fall back.
     */
    GroundHit castRay(const RE::NiPoint3& start, const RE::NiPoint3& end);

    /**
     * Ground height directly under `position`, probing `probeUp` above and
     * `probeDown` below it. Returns std::nullopt when nothing was hit.
     */
    std::optional<GroundHit> findGround(const RE::NiPoint3& position, float probeUp, float probeDown);
}
