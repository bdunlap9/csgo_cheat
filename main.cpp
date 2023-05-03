#include <Windows.h>
#include <iostream>
#include <thread>
#include <random>
#include <chrono>
#include "ProcMem.h"
#include "csgo.hpp"
#include "offsets.hpp"
#include "resolver.h"

using namespace offsets::netvars;
using namespace offsets::signatures;
using namespace resolver;

ProcMem mem;
DWORD client, localPlayer, leftShift;

#include <cmath>

struct Vector3 {
    float x, y, z;

    Vector3 operator-(const Vector3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }
};

float GetDistance(const Vector3& source, const Vector3& target) {
    Vector3 delta = source - target;
    return sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
}

Vector3 CalculateAimAngles(const Vector3& source, const Vector3& target) {
    Vector3 delta = target - source;
    float distance = GetDistance(source, target);

    Vector3 angles;
    angles.x = -asin(delta.z / distance) * (180.0f / M_PI);
    angles.y = atan2(delta.y, delta.x) * (180.0f / M_PI);
    angles.z = 0.0f;

    return angles;
}

Vector3 SmoothAim(const Vector3& currentAngles, const Vector3& aimAngles, float smoothFactor) {
    Vector3 smoothedAngles;
    smoothedAngles.x = currentAngles.x + (aimAngles.x - currentAngles.x) / smoothFactor;
    smoothedAngles.y = currentAngles.y + (aimAngles.y - currentAngles.y) / smoothFactor;
    smoothedAngles.z = 0.0f;

    return smoothedAngles;
}

bool IsAimingAtTarget(ProMem& mem, uintptr_t localPlayerBase, uintptr_t targetEntityBase, float aimTolerance) {
    Vector3 localPlayerViewAngles = mem.Read<Vector3>(localPlayerBase + /* Offset for view angles */);
    Vector3 localPlayerPosition = mem.Read<Vector3>(localPlayerBase + /* Offset for position */);
    Vector3 targetHitboxPosition = mem.Read<Vector3>(targetEntityBase + /* Offset for the hitbox position */);

    Vector3 aimAngles = CalculateAimAngles(localPlayerPosition, targetHitboxPosition);
    Vector3 angleDelta = localPlayerViewAngles - aimAngles;

    return (std::abs(angleDelta.x) <= aimTolerance) && (std::abs(angleDelta.y) <= aimTolerance);
}

void Shoot(Memory& mem, uintptr_t localPlayerBase, uintptr_t targetEntityBase, float aimTolerance) {
    if (IsAimingAtTarget(mem, localPlayerBase, targetEntityBase, aimTolerance)) {
        int attackValue = 5; // Replace with appropriate value for your game
        mem.Write(localPlayerBase + /* Offset for attack */, attackValue);

        // Optionally, add a delay between shots to simulate a more human-like behavior
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void AimAtTarget(ProcMem& mem, uintptr_t localPlayerBase, uintptr_t entityBase, int hitbox) {
    Vector3 localPlayerPosition = mem.Read<Vector3>(localPlayerBase + /* Offset for position */);
    Vector3 localPlayerAngles = mem.Read<Vector3>(localPlayerBase + /* Offset for view angles */);

    Vector3 targetHitboxPosition = mem.Read<Vector3>(entityBase + /* Offset for the hitbox position */);

    Vector3 aimAngles = CalculateAimAngles(localPlayerPosition, targetHitboxPosition);

    float smoothFactor = 5.0f; // Adjust this value to control the smoothing speed
    Vector3 smoothedAngles = SmoothAim(localPlayerAngles, aimAngles, smoothFactor);

    mem.Write(localPlayerBase + /* Offset for view angles */, smoothedAngles);
}


void RCS();
void Triggerbot();
bool IsVisible(ProcMem& mem, uintptr_t localPlayerBase, uintptr_t entityBase, int hitbox);
void AimAtTarget(ProcMem& mem, uintptr_t localPlayerBase, uintptr_t entityBase, int hitbox);
void Shoot(ProcMem& mem);

int main() {
    mem.Process(L"csgo.exe");
    client = mem.Module(L"client.dll");

    while (!GetAsyncKeyState(VK_END)) {
        try {
            localPlayer = mem.Read<DWORD>(client + dwLocalPlayer);
        } catch (const std::runtime_error& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
        leftShift = GetAsyncKeyState(VK_LSHIFT);

        if (localPlayer) {
            std::thread t1(RCS);
            t1.join();
            if (leftShift) {
                std::thread t2(Triggerbot);
                t2.join();
            }
        }
        Sleep(1);
    }
    return 0;
}

bool IsLineOfSightClear(DWORD entity) {
    try {
        return mem.Read<bool>(localPlayer + m_bSpottedByMask + entity * 0x4);
    } catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

float RandomFloat(float min, float max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return static_cast<float>(dis(gen));
}

void RCS() {
    try {
        int shotsFired = mem.Read<int>(localPlayer + m_iShotsFired);
    } catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    if (shotsFired > 1) {
        DWORD forceAim = client + dwForceAim;
        try {
            Vector3 viewAngles = mem.Read<Vector3>(offsets::signatures::dwClientState_ViewAngles);
            Vector3 aimPunch = mem.Read<Vector3>(localPlayer + m_aimPunchAngle);
        } catch (const std::runtime_error& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }

        float rcsFactor = 2.0f;
        float humanizationFactor = 0.95f;
        float randomX = RandomFloat(-0.2f, 0.2f);
        float randomY = RandomFloat(-0.2f, 0.2f);
        Vector3 newAngles = viewAngles - ((aimPunch * rcsFactor) * humanizationFactor) + Vector3(randomX, randomY, 0.0f);
        try {
            mem.Write<Vector3>(forceAim, newAngles);
        } catch (const std::runtime_error& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }
}

void Triggerbot() {
    try {
        int crosshairID = mem.Read<int>(localPlayer + m_iCrosshairId);
        DWORD entity = mem.Read<DWORD>(client + dwEntityList + (crosshairID - 1) * 0x10);
        int entityHealth = mem.Read<int>(entity + m_iHealth);
        int entityTeam = mem.Read<int>(entity + m_iTeamNum);
        int playerTeam = mem.Read<int>(localPlayer + m_iTeamNum);
    } catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    if (entityHealth > 0 && entityTeam != playerTeam && IsLineOfSightClear(entity)) {
        try {
            mem.Write<int>(client + dwForceAttack, 6);
        } catch (const std::runtime_error& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
        Sleep(10);
        try {
            mem.Write<int>(client + dwForceAttack, 4);
        } catch (const std::runtime_error& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }
}

void RageBot() {
    ProcMem mem; // Replace this with your memory reading/writing library initialization
    // Add your game process name, module name, etc. for initializing the memory library

    while (true) {
        // Read necessary data from memory
        uintptr_t localPlayerBase = mem.Read<uintptr_t>(/* Your local player base address */);
        uintptr_t entityListBase = mem.Read<uintptr_t>(/* Your entity list base address */);

        // Loop through entities
        for (int i = 0; i < 64; ++i) {
            uintptr_t entityBase = mem.Read<uintptr_t>(entityListBase + i * 0x10);

            if (entityBase == 0) {
                continue;
            }

            // Check if the entity is an enemy and alive
            if (/* Check if the entity is an enemy */ && /* Check if the entity is alive */) {
                // Use the resolver before shooting
                resolver::DataDrivenResolver(mem, localPlayerBase, entityBase);

                // Aim at different hitboxes
                std::vector<int> hitboxes = { /* List of hitbox IDs for head, legs, pelvis, upper chest, neck, feet */ };
                for (int hitbox : hitboxes) {
                    // Check if the target is visible or if you want to shoot through walls
                    if (IsVisible(mem, localPlayerBase, entityBase, hitbox) || /* Your condition for shooting through walls */) {
                        AimAtTarget(mem, localPlayerBase, entityBase, hitbox);
                        Shoot(mem);
                    }
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
