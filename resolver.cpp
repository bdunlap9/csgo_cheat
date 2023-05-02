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
    int previousHealth;
};

// Store enemy data in a buffer
std::vector<EnemyData> enemyDataBuffer;

void CollectEnemyData(ProcMem& mem, DWORD entityBase) {
    EnemyData enemyData;

    // Collect enemy velocity
    float velX = mem.Read<float>(entityBase + hazedumper::netvars::m_vecVelocity + 0x0);
    float velY = mem.Read<float>(entityBase + hazedumper::netvars::m_vecVelocity + 0x4);
    float velZ = mem.Read<float>(entityBase + hazedumper::netvars::m_vecVelocity + 0x8);
    enemyData.velocity = std::sqrt(velX * velX + velY * velY + velZ * velZ);

    // Collect enemy animation state (you'll need to find the correct offset)
    enemyData.animationState = mem.Read<float>(entityBase + /* ANIMATION_STATE_OFFSET */ 0);

    // Collect enemy health
    int currentHealth = mem.Read<int>(entityBase + hazedumper::netvars::m_iHealth);

    // Check if the enemy was hit by comparing the current health to the previous health
    bool wasHitByHealth = false;
    if (!enemyDataBuffer.empty()) {
        const EnemyData& previousEnemyData = enemyDataBuffer.back();
        wasHitByHealth = currentHealth < previousEnemyData.previousHealth;
    }

    // Calculate the change in yaw
    float currentYaw = mem.Read<float>(entityBase + hazedumper::netvars::m_angEyeAnglesY);
    float previousYaw = !enemyDataBuffer.empty() ? enemyDataBuffer.back().yaw : currentYaw;
    float yawChange = std::abs(currentYaw - previousYaw);

    // Set the base threshold value
    float aimDiffThreshold = 0.1f;

    // Adjust the threshold based on the change in yaw
    float yawChangeFactor = 2.0f;
    float adjustedAimDiffThreshold = aimDiffThreshold + yawChange * yawChangeFactor;

    // Combine the hit checks (you can modify this logic as needed)
    enemyData.wasHit = wasHitByHealth;

    // Store the current health and yaw in the enemyData
    enemyData.previousHealth = currentHealth;
    enemyData.yaw = currentYaw;

    // Store the collected data in the buffer
    enemyDataBuffer.push_back(enemyData);

    // Optionally, limit the buffer size to a certain length
    if (enemyDataBuffer.size() > 100) {
        enemyDataBuffer.erase(enemyDataBuffer.begin());
    }
}

float CalculateWeightBasedOnVelocityAndAim(float enemyVelX, float enemyVelY, float enemyVelZ,
                                           float localPlayerAimPitch, float localPlayerAimYaw) {
    // Calculate the enemy's speed
    float enemySpeed = std::sqrt(enemyVelX * enemyVelX + enemyVelY * enemyVelY + enemyVelZ * enemyVelZ);

    // Calculate the aim angle difference between local player and enemy
    float aimAngleDiff = std::sqrt((localPlayerAimPitch * localPlayerAimPitch) + (localPlayerAimYaw * localPlayerAimYaw));

    // You can adjust these values based on how much you want the velocity and aim angles to affect the weight
    float velocityWeightFactor = 0.5f;
    float aimAngleWeightFactor = 0.5f;

    // Normalize the enemy's speed and the aim angle difference to the range [0, 1]
    float normalizedSpeed = enemySpeed / 500.0f; // Assuming max speed of 500 units
    float normalizedAimAngleDiff = aimAngleDiff / 360.0f; // Assuming max aim angle difference of 360 degrees

    // Calculate the weight based on the normalized speed and aim angle difference
    float weight = (1.0f - velocityWeightFactor) * normalizedSpeed + (1.0f - aimAngleWeightFactor) * normalizedAimAngleDiff;

    return weight;
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

        // Read the enemy's velocity
        float enemyVelX = mem.Read<float>(entityBase + hazedumper::netvars::m_vecVelocity + 0x0);
        float enemyVelY = mem.Read<float>(entityBase + hazedumper::netvars::m_vecVelocity + 0x4);
        float enemyVelZ = mem.Read<float>(entityBase + hazedumper::netvars::m_vecVelocity + 0x8);

        // Read local player's aim angles
        float localPlayerAimPitch = mem.Read<float>(localPlayerBase + hazedumper::netvars::m_angEyeAnglesX);
        float localPlayerAimYaw = mem.Read<float>(localPlayerBase + hazedumper::netvars::m_angEyeAnglesY);

        // Calculate the hit rate for the current test yaw using the collected data
        float hitCount = 0;
        float totalCount = 0;
        int buffer_size = static_cast<int>(enemyDataBuffer.size());
        for (int i = 0; i < buffer_size; ++i) {
            const auto& enemyData = enemyDataBuffer[i];

            // Introduce a weighting system
            float weight = static_cast<float>(i + 1) / buffer_size;

            // Check if the test yaw would hit the enemy based on the enemy data and additional factors
            if (enemyData.animationState > 0.5f && std::abs(normalizedTestYaw - currentYaw) < 45.0f) {
                float distance = GetDistance(mem, localPlayerBase, entityBase);

                // Adjust the weight based on distance
                if (distance > 200.0f && distance < 300.0f) {
                    weight *= 1.5f;
                } else if (distance > 300.0f) {
                    weight *= 2.0f;
                }

                // Adjust the weight based on enemy's velocity and local player's aim angles
                weight *= CalculateWeightBasedOnVelocityAndAim(enemyVelX, enemyVelY, enemyVelZ, localPlayerAimPitch, localPlayerAimYaw);

                if (enemyData.wasHit) {
                    hitCount += weight;
                }
                totalCount += weight;
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
        mem.Write<float>(entityBase + hazedumper::netvars::m_angEyeAnglesY, bestYaw);
    } catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}
