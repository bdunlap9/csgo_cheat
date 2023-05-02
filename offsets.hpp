#ifndef OFFSETS_HPP
#define OFFSETS_HPP

namespace hazedumper {
    namespace netvars {
        constexpr ::std::ptrdiff_t m_aimPunchAngle = 0x302C;
        constexpr ::std::ptrdiff_t m_iShotsFired = 0xA390;
        constexpr ::std::ptrdiff_t m_iCrosshairId = 0xB3E4;
        constexpr ::std::ptrdiff_t m_iTeamNum = 0xF4;
        constexpr ::std::ptrdiff_t m_iHealth = 0x100;
        constexpr ::std::ptrdiff_t m_bSpottedByMask = 0x980;
    }

    namespace signatures {
        constexpr ::std::ptrdiff_t dwLocalPlayer = 0xD3FD14;
        constexpr ::std::ptrdiff_t dwEntityList = 0x4DA31EC;
        constexpr ::std::ptrdiff_t dwForceAim = 0x31ED7EC;
        constexpr ::std::ptrdiff_t dwForceAttack = 0x31ED810;
        constexpr ::std::ptrdiff_t dwClientState_ViewAngles = 0x4D90;
    }
}

#endif // OFFSETS_HPP
