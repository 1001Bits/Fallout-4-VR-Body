#include "GroundQuery.h"

#include <array>
#include <cmath>
#include <cstddef>

#include "f4vr/F4VROffsets.h"

namespace frik::world
{
    namespace
    {
        // All offsets below were read out of the VR Address Library and then verified
        // against Fallout4VR.exe 1.2.72 in Ghidra: each one is the exact entry point
        // of the named function. They are declared as direct offsets, matching the
        // rest of F4VROffsets, so FRIK gains no runtime dependency on the address
        // library (CommonLibF4VR's own bhkPickData wrappers use REL::ID, which would
        // hard-fail for users who do not have it installed).

        // bhkPickData's constructor installs the engine's defaults: raycastMode
        // 0xffff, collectorStatus 3, fraction FLT_MAX, empty result, zero filter.
        using _bhkPickData_ctor = RE::bhkPickData* (*)(RE::bhkPickData* pickData);
        inline REL::Relocation<_bhkPickData_ctor> bhkPickData_ctor{ REL::Offset(0x001f930) };

        // Takes game units and multiplies by the Havok unit scale internally.
        using _bhkPickData_SetStartEnd = void (*)(RE::bhkPickData* pickData, const RE::NiPoint3& start, const RE::NiPoint3& end);
        inline REL::Relocation<_bhkPickData_SetStartEnd> bhkPickData_SetStartEnd{ REL::Offset(0x0027170) };

        using _bhkPickData_HasHit = bool (*)(RE::bhkPickData* pickData);
        inline REL::Relocation<_bhkPickData_HasHit> bhkPickData_HasHit{ REL::Offset(0x1dfb6f0) };

        using _bhkPickData_GetHitFraction = float (*)(RE::bhkPickData* pickData);
        inline REL::Relocation<_bhkPickData_GetHitFraction> bhkPickData_GetHitFraction{ REL::Offset(0x1dfb710) };

        using _bhkWorld_PickObject = void (*)(RE::bhkWorld* world, RE::bhkPickData* pickData);
        inline REL::Relocation<_bhkWorld_PickObject> bhkWorld_PickObject{ REL::Offset(0x1df8d60) };

        // RE::CFilter has an intentional user-declared destructor, which under the
        // MSVC x64 ABI forces it to be returned through a hidden pointer rather than
        // in a register. The member function therefore takes `this` in RCX and the
        // return buffer in RDX, which is what the disassembly shows.
        using _Actor_GetCollisionFilter = RE::CFilter* (*)(RE::Actor* actor, RE::CFilter* result);
        inline REL::Relocation<_Actor_GetCollisionFilter> Actor_GetCollisionFilter{ REL::Offset(0x0de6e10) };

        // The layer mask Fallout's own foot IK cast uses
        // (BSLimbIKModifierUtility::CastRayImpl). It selects world geometry and
        // excludes characters, which is exactly what a ground probe wants.
        constexpr std::uint64_t kFootIkCollisionLayer = 0x10002200eull;

        // The engine keeps only the layer half of the caster's filter for IK casts
        // (BSLimbIKModifierUtilityCastInfo::SetCastInfo takes the high word and
        // CastRayImpl shifts it back), which stops the ray hitting the caster.
        constexpr std::uint32_t kCollisionFilterLayerMask = 0xffff0000u;

        bool isFinitePoint(const RE::NiPoint3& point) { return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z); }
    }

    GroundHit castRay(const RE::NiPoint3& start, const RE::NiPoint3& end)
    {
        GroundHit result;
        if (!isFinitePoint(start) || !isFinitePoint(end)) {
            return result;
        }

        const auto player = RE::PlayerCharacter::GetSingleton();
        if (!player || !player->parentCell) {
            return result;
        }

        const auto world = f4vr::TESObjectCell_GetbhkWorld(player->parentCell);
        if (!world) {
            return result;
        }

        // bhkPickData cannot be default-constructed here because CommonLibF4VR's
        // constructor resolves through REL::ID. Build the storage and run the
        // engine's own constructor over it instead. The type is plain data with no
        // destructor, and 16-byte alignment is required by the SIMD writes to
        // rayOrigin/rayDest.
        alignas(16) std::array<std::byte, sizeof(RE::bhkPickData)> storage{};
        const auto pickData = reinterpret_cast<RE::bhkPickData*>(storage.data());
        bhkPickData_ctor(pickData);

        bhkPickData_SetStartEnd(pickData, start, end);

        RE::CFilter casterFilter{};
        Actor_GetCollisionFilter(player, &casterFilter);
        pickData->collisionFilter.filter = casterFilter.filter & kCollisionFilterLayerMask;
        pickData->collisionLayer = kFootIkCollisionLayer;

        bhkWorld_PickObject(world, pickData);
        if (!bhkPickData_HasHit(pickData)) {
            return result;
        }

        const float fraction = bhkPickData_GetHitFraction(pickData);
        if (!std::isfinite(fraction) || fraction < 0.0f || fraction > 1.0f) {
            return result;
        }

        const RE::NiPoint3 hitPosition = start + (end - start) * fraction;
        RE::NiPoint3 hitNormal(pickData->result.normal.x, pickData->result.normal.y, pickData->result.normal.z);
        if (!isFinitePoint(hitPosition) || !isFinitePoint(hitNormal)) {
            return result;
        }

        // The raw result normal is not guaranteed to be unit length - the engine's
        // own CastRayImpl re-normalizes it before handing it out, so do the same.
        const float normalLength = std::sqrt(hitNormal.x * hitNormal.x + hitNormal.y * hitNormal.y + hitNormal.z * hitNormal.z);
        if (!std::isfinite(normalLength) || normalLength <= 1.0e-4f) {
            return result;
        }
        hitNormal /= normalLength;

        result.position = hitPosition;
        result.normal = hitNormal;
        result.hit = true;
        return result;
    }

    std::optional<GroundHit> findGround(const RE::NiPoint3& position, const float probeUp, const float probeDown)
    {
        if (!isFinitePoint(position) || !std::isfinite(probeUp) || !std::isfinite(probeDown) || probeUp < 0.0f || probeDown <= 0.0f) {
            return std::nullopt;
        }

        const RE::NiPoint3 start(position.x, position.y, position.z + probeUp);
        const RE::NiPoint3 end(position.x, position.y, position.z - probeDown);
        const auto hit = castRay(start, end);
        if (!hit.hit) {
            return std::nullopt;
        }
        return hit;
    }
}
