#ifndef OFFSETS_HPP
#define OFFSETS_HPP

#include <cstddef>

namespace offsets {
    namespace netvars {
        constexpr ::std::ptrdiff_t m_aimPunchAngle = 0x302C;
        constexpr ::std::ptrdiff_t m_iShotsFired = 0xA390;
        constexpr ::std::ptrdiff_t m_iCrosshairId = 0xB3E4;
        constexpr ::std::ptrdiff_t m_iTeamNum = 0xF4;
        int m_iHealth = 0x100;
        bool m_bSpottedByMask = 0x980;
        constexpr ::std::ptrdiff_t m_vecOrigin = 0x138;
        float m_vecVelocity = 0x114;
        constexpr ::std::ptrdiff_t m_flPoseParameter = 0x2764;
        constexpr ::std::ptrdiff_t m_flSimulationTime = 0x268;
        constexpr ::std::ptrdiff_t m_dwBoneMatrix = 0x26A8;
        bool m_bDormant = 0xED;
        constexpr ::std::ptrdiff_t m_vecViewOffset = 0x108;
        constexpr ::std::ptrdiff_t m_lifeState = 0x25F;
        bool m_bHasHelmet = 0xB3E9;
    }

    namespace signatures {
        constexpr ::std::ptrdiff_t dwLocalPlayer = 0xD3FD14;
        constexpr ::std::ptrdiff_t dwEntityList = 0x4DA31EC;
        constexpr ::std::ptrdiff_t dwForceAim = 0x31ED7EC;
        constexpr ::std::ptrdiff_t dwForceAttack = 0x31ED810;
        constexpr ::std::ptrdiff_t dwClientState_ViewAngles = 0x4D90;
        constexpr ::std::ptrdiff_t dwClientState = 0x58ADD4;
        constexpr ::std::ptrdiff_t dwClientState_GetLocalPlayer = 0x180;
        constexpr ::std::ptrdiff_t dwClientState_IsHLTV = 0x4D48;
        constexpr ::std::ptrdiff_t dwClientState_Map = 0x28C;
        constexpr ::std::ptrdiff_t dwClientState_MapDirectory = 0x188;
        int dwClientState_MaxPlayer = 0x388;
        constexpr ::std::ptrdiff_t dwClientState_PlayerInfo = 0x52B8;
        constexpr ::std::ptrdiff_t dwClientState_State = 0x108;
        constexpr ::std::ptrdiff_t dwClientState_ViewAngles = 0x4D90;
        constexpr ::std::ptrdiff_t dwClientState_Map = 0x28C;
        constexpr ::std::ptrdiff_t dwEntityList = 0x4DA31EC;
        constexpr ::std::ptrdiff_t dwForceAttack = 0x31ED810;
        constexpr ::std::ptrdiff_t dwForceAttack2 = 0x31ED81C;
        constexpr ::std::ptrdiff_t dwForceBackward = 0x31ED864;
        constexpr ::std::ptrdiff_t dwForceForward = 0x31ED85C;
        constexpr ::std::ptrdiff_t dwForceJump = 0x524CFAC;
        constexpr ::std::ptrdiff_t dwForceLeft = 0x31ED884;
        constexpr ::std::ptrdiff_t dwForceRight = 0x31ED87C;
        }

#endif // OFFSETS_HPP
