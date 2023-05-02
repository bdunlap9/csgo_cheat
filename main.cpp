#include <Windows.h>
#include <iostream>
#include <thread>
#include <random>
#include "ProcMem.h"
#include "csgo.hpp"
#include "offsets.hpp"

using namespace hazedumper::netvars;
using namespace hazedumper::signatures;

ProcMem mem;
DWORD client, localPlayer, leftShift;

void RCS();
void Triggerbot();

int main() {
    mem.Process(L"csgo.exe");
    client = mem.Module(L"client.dll");

    while (!GetAsyncKeyState(VK_END)) {
        localPlayer = mem.Read<DWORD>(client + dwLocalPlayer);
        leftShift = GetAsyncKeyState(VK_LSHIFT);

        if (localPlayer) {
            if (leftShift) {
                std::thread t1(RCS);
                std::thread t2(Triggerbot);
                t1.join();
                t2.join();
            }
        }
        Sleep(1);
    }
    return 0;
}

float RandomFloat(float min, float max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return static_cast<float>(dis(gen));
}

void RCS() {
    DWORD engine = mem.Module(L"engine.dll");
    int shotsFired = mem.Read<int>(localPlayer + m_iShotsFired);
    if (shotsFired > 1) {
        DWORD forceAim = client + dwForceAim;
        Vector3 viewAngles = mem.Read<Vector3>(engine + dwClientState_ViewAngles);
        Vector3 aimPunch = mem.Read<Vector3>(localPlayer + m_aimPunchAngle);

        float rcsFactor = 2.0f;
        float humanizationFactor = 0.95f;
        float randomX = RandomFloat(-0.2f, 0.2f);
        float randomY = RandomFloat(-0.2f, 0.2f);
        Vector3 newAngles = viewAngles - ((aimPunch * rcsFactor) * humanizationFactor) + Vector3(randomX, randomY, 0.0f);

        mem.Write<Vector3>(forceAim, newAngles);
    }
}

bool IsLineOfSightClear(DWORD entity) {
    return mem.Read<bool>(localPlayer + m_bSpottedByMask + entity * 0x4);
}

void Triggerbot() {
    int crosshairID = mem.Read<int>(localPlayer + m_iCrosshairId);
    DWORD entity = mem.Read<DWORD>(client + dwEntityList + (crosshairID - 1) * 0x10);
    int entityHealth = mem.Read<int>(entity + m_iHealth);
    int entityTeam = mem.Read<int>(entity + m_iTeamNum);
    int playerTeam = mem.Read<int>(localPlayer + m_iTeamNum);

    if (entityHealth > 0 && entityTeam != playerTeam && IsLineOfSightClear(entity)) {
        mem.Write<int>(client + dwForceAttack, 6);
        Sleep(10);
        mem.Write<int>(client + dwForceAttack, 4);
    }
}
