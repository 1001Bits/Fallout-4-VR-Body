#pragma once

namespace frik
{
    class SelfieHandler
    {
    public:
        void onFrameUpdate(const RE::NiPoint3& hmdPivot, const RE::NiPoint3& bodyForward) const;

    private:
        void basicSelfie(const RE::NiPoint3& hmdPivot, const RE::NiPoint3& bodyForward) const;
        void testSelfie();
        void enterSelfieMode();
        void exitSelfieMode() const;
        static void experimental();

        bool _selfieActive = false;
        RE::NiPoint3 _playerStartPosition;
        RE::NiPoint3 _forwardDir;
        RE::NiPoint3 _rootWorldPos;
    };
}
