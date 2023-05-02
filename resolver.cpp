#include "ProcMem.h"
#include "csgo.hpp"
#include "offsets.hpp"
#include <cmath>
#include <vector>
#include <Windows.h>

float NormalizeYaw(float yaw) {
    while (yaw < -180.0f) yaw += 360.0f;
    while (yaw > 180.0f) yaw -= 360.0f;
    return yaw;
}

float GetDistance(ProcMem& mem, DWORD localPlayerBase, DWORD entityBase) {
    // Read the positions of the local player and the enemy
    float localPlayerX = mem.Read<float>(localPlayerBase + hazedumper::netvars::m_vecOrigin + 0x0);
    float localPlayerY = mem.Read<float>(localPlayerBase + hazedumper::netvars::m_vecOrigin + 0x4);
    float localPlayerZ = mem.Read<float>(localPlayerBase + hazedumper::netvars::m_vecOrigin + 0x8);

    float enemyX = mem.Read<float>(entityBase + hazedumper::netvars::m_vecOrigin + 0x0);
    float enemyY = mem.Read<float>(entityBase + hazedumper::netvars::m_vecOrigin + 0x4);
    float enemyZ = mem.Read<float>(entityBase + hazedumper::netvars::m_vecOrigin + 0x8);

    // Calculate the distance between the local player and the enemy
    float dx = enemyX - localPlayerX;
    float dy = enemyY - localPlayerY;
    float dz = enemyZ - localPlayerZ;
    float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    return distance;
}

// Custom data structure to store enemy behavior data
struct EnemyData {
    float velocity;
    float animationState;
    bool wasHit;
};

// Store enemy data in a buffer
std::vector<EnemyData> enemyDataBuffer;

// Store enemy data in a buffer
std::vector<EnemyData> enemyDataBuffer;

void BruteResolver(ProcMem& mem, DWORD clientBase, DWORD localPlayerBase, DWORD entityBase) {
    // Read the enemy's current yaw
    float currentYaw = mem.Read<float>(entityBase + hazedumper::netvars::m_angEyeAnglesY);
    
    // This is a simple example of a resolver that tries to "bruteforce" the correct yaw
    // You'll need to develop a more sophisticated algorithm to accurately predict the hitbox positions
    for (int i = 0; i < 4; ++i) {
        float testYaw = currentYaw + 45.0f * i;
        float normalizedTestYaw = NormalizeYaw(testYaw);
        
        // Write the testYaw back to the enemy's m_angEyeAnglesY
        mem.Write<float>(entityBase + hazedumper::netvars::m_angEyeAnglesY, normalizedTestYaw);
        
        // Test if the new yaw yields better results (e.g., by checking if the shots hit the enemy)
        // If successful, you can break the loop and use this yaw
    }
}

void CollectEnemyData(ProcMem& mem, DWORD entityBase) {
    EnemyData enemyData;

    // Collect enemy velocity
    float velX = mem.Read<float>(entityBase + hazedumper::netvars::m_vecVelocity + 0x0);
    float velY = mem.Read<float>(entityBase + hazedumper::netvars::m_vecVelocity + 0x4);
    float velZ = mem.Read<float>(entityBase + hazedumper::netvars::m_vecVelocity + 0x8);
    enemyData.velocity = std::sqrt(velX * velX + velY * velY + velZ * velZ);

    // Collect enemy animation state (you'll need to find the correct offset)
    enemyData.animationState = mem.Read<float>(entityBase + /* ANIMATION_STATE_OFFSET */ 0);

    // Collect hit status (you'll need a method to determine if the enemy was hit)
    enemyData.wasHit = /* CHECK_IF_ENEMY_WAS_HIT_METHOD */ false;

    // Store the collected data in the buffer
    enemyDataBuffer.push_back(enemyData);

    // Optionally, limit the buffer size to a certain length
    if (enemyDataBuffer.size() > 100) {
        enemyDataBuffer.erase(enemyDataBuffer.begin());
    }
}

void DataDrivenResolver(ProcMem& mem, DWORD entityBase) {
    try {
        // Read the enemy's current yaw
        float currentYaw = mem.Read<float>(entityBase + hazedumper::netvars::m_angEyeAnglesY);
    } catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    // Initialize variables for the best yaw and the highest hit rate
    float bestYaw = currentYaw;
    float highestHitRate = 0.0f;

    // Increase the resolution of the tested yaws
    const int numYawSteps = 8;
    for (int i = 0; i < numYawSteps; ++i) {
        float testYaw = currentYaw + (360.0f / numYawSteps) * i;
        float normalizedTestYaw = NormalizeYaw(testYaw);
        float localPlayerX = mem.Read<float>(localPlayerBase + hazedumper::netvars::m_vecOrigin + 0x0);
        float localPlayerY = mem.Read<float>(localPlayerBase + hazedumper::netvars::m_vecOrigin + 0x4);
        float localPlayerZ = mem.Read<float>(localPlayerBase + hazedumper::netvars::m_vecOrigin + 0x8);

        float enemyX = mem.Read<float>(entityBase + hazedumper::netvars::m_vecOrigin + 0x0);
        float enemyY = mem.Read<float>(entityBase + hazedumper::netvars::m_vecOrigin + 0x4);
        float enemyZ = mem.Read<float>(entityBase + hazedumper::netvars::m_vecOrigin + 0x8);

        float dx = enemyX - localPlayerX;
        float dy = enemyY - localPlayerY;
        float dz = enemyZ - localPlayerZ;
        float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        // Calculate the hit rate for the current test yaw using the collected data
        float hitCount = 0;
        float totalCount = 0;
        int buffer_size = static_cast<int>(enemyDataBuffer.size());
        for (int i = 0; i < buffer_size; ++i) {
            const auto& enemyData = enemyDataBuffer[i];

            // Introduce a weighting system
            float weight = static_cast<float>(i + 1) / buffer_size;

            // Check if the test yaw would hit the enemy based on the enemy data
            if (enemyData.animationState > 0.5f && std::abs(normalizedTestYaw - currentYaw) < 45.0f) {
                float distance = GetDistance(mem, localPlayerBase, entityBase);
                float weight = 1.0f;
                if (distance > 200.0f && distance < 300.0f) {
                    weight = 1.5f;
                } else if (distance > 300.0f) {
                    weight = 2.0f;
                }
                if (enemyData.wasHit) {
                    hitCount += static_cast<int>(weight);
                }
                totalCount += static_cast<int>(weight);
            }
        }

        float hitRate = hitCount / totalCount;

        // If the hit rate is higher than the previous best, update the best yaw and highest hit rate
        if (hitRate > highestHitRate) {
            highestHitRate = hitRate;
            bestYaw = normalizedTestYaw;
        }
    }

    try {
        // Write the best yaw back to the enemy's m_angEyeAnglesY
        mem.Write<float>(entityBase + hazedumper::netvars::m_angEyeAnglesY, bestYaw);
    } catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}
