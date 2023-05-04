#include "ProcMem.h"
#include "csgo.hpp"
#include "offsets.hpp"
#include "resolver.h"

#include <cmath>
#include <unordered_map>
#include <vector>
#include <Windows.h>
#include <iostream>
#include <chrono>

namespace resolver {

    const int TICK_RATE = 64;
    const double LAG_COMPENSATION_TIME = 0.1;  // 100ms

    float CalculateWeaponTypeWeight(WeaponType weaponType, float enemySpeed) {
    float baseWeight;
    float speedFactor;

    switch (weaponType) {
        case PISTOL:
            baseWeight = 1.1f;
            speedFactor = 0.005f;
            break;
        case RIFLE:
            baseWeight = 1.0f;
            speedFactor = 0.003f;
            break;
        case SMG:
            baseWeight = 1.05f;
            speedFactor = 0.008f;
            break;
        case SNIPER:
            baseWeight = 0.9f;
            speedFactor = 0.001f;
            break;
        case SHOTGUN:
            baseWeight = 1.2f;
            speedFactor = 0.01f;
            break;
        case MACHINEGUN:
            baseWeight = 0.95f;
            speedFactor = 0.002f;
            break;
        case KNIFE:
            baseWeight = 0.5f;
            speedFactor = 0.015f;
            break;
        case GRENADE:
            baseWeight = 0.8f;
            speedFactor = 0.007f;
            break;
        case OTHER:
            baseWeight = 1.0f;
            speedFactor = 0.005f;
            break;
    }

    return baseWeight + speedFactor * enemySpeed;
}

WeaponType GetWeaponType(int weaponID) {
    // Implement a function that returns the WeaponType based on the weaponID.
    // You can use the weaponID to determine which type of weapon it is.
}

// Store enemy data in a buffer
std::unordered_map<DWORD, std::vector<EnemyData>> enemyDataBuffers;

// Store enemy data in a buffer
std::vector<EnemyData> enemyDataBuffer;

// Add a new variable to store the last LBY value
float lastLBY = 0.0f;

float NormalizeYaw(float yaw) {
    while (yaw < -180.0f) yaw += 360.0f;
    while (yaw > 180.0f) yaw -= 360.0f;
    return yaw;
}

float GetDistance(ProcMem& mem, DWORD localPlayerBase, DWORD entityBase) {
    // Read the positions of the local player and the enemy
    float localPlayerX = mem.Read<float>(localPlayerBase + offsets::netvars::m_vecOrigin + 0x0);
    float localPlayerY = mem.Read<float>(localPlayerBase + offsets::netvars::m_vecOrigin + 0x4);
    float localPlayerZ = mem.Read<float>(localPlayerBase + offsets::netvars::m_vecOrigin + 0x8);

    float enemyX = mem.Read<float>(entityBase + offsets::netvars::m_vecOrigin + 0x0);
    float enemyY = mem.Read<float>(entityBase + offsets::netvars::m_vecOrigin + 0x4);
    float enemyZ = mem.Read<float>(entityBase + offsets::netvars::m_vecOrigin + 0x8);

    // Calculate the distance between the local player and the enemy
    float dx = enemyX - localPlayerX;
    float dy = enemyY - localPlayerY;
    float dz = enemyZ - localPlayerZ;
    float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    return distance;
}

struct GameState {
    // Add your game state variables here
};

std::vector<std::pair<double, GameState>> gameStates;

double get_current_time() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

GameState get_current_game_state() {
    GameState currentGameState;

    // Update the current game state
    return currentGameState;
}

void update_game_state(const GameState& newGameState) {
    double currentTime = get_current_time();
    gameStates.push_back({currentTime, newGameState});

    // Remove game states older than the lag compensation time
    while (!gameStates.empty() && gameStates.front().first < currentTime - LAG_COMPENSATION_TIME) {
        gameStates.erase(gameStates.begin());
    }
}

bool check_collision(const GameState& serverGameState, const PlayerState& player) {
    // Check for collisions between the player and the game boundary
    if (player.x < serverGameState.boundary.x ||
        player.y < serverGameState.boundary.y ||
        player.x + player.width > serverGameState.boundary.x + serverGameState.boundary.width ||
        player.y + player.height > serverGameState.boundary.y + serverGameState.boundary.height) {
        return true;
    }

    // Return false if no collision is detected
    return false;
}

void update_velocity(PlayerState& player, const InputCommand& inputCommand) {
    float acceleration = player.speed;

    switch (inputCommand) {
        case InputCommand::MOVE_UP:
            player.vx = 0;
            player.vy = acceleration;
            break;
        case InputCommand::MOVE_DOWN:
            player.vx = 0;
            player.vy = -acceleration;
            break;
        case InputCommand::MOVE_LEFT:
            player.vx = -acceleration;
            player.vy = 0;
            break;
        case InputCommand::MOVE_RIGHT:
            player.vx = acceleration;
            player.vy = 0;
            break;
    }
}

void compensate_for_lag(GameState& serverGameState, const ClientInput& clientInput) {
    // Calculate the time elapsed since the server game state was created
    float elapsedTime = static_cast<float>(get_current_time() - serverGameState.timestamp);

    // Update the player's velocity based on the client input
    PlayerState& player = serverGameState.players[clientInput.playerIndex];
    update_velocity(player, clientInput.inputCommand);

    // Update the player's position based on their velocity and elapsed time
    float newX = player.x + player.vx * elapsedTime;
    float newY = player.y + player.vy * elapsedTime;

    // Check for collisions and only update the position if there is no collision
    PlayerState tempPlayer = player;
    tempPlayer.x = newX;
    tempPlayer.y = newY;
    if (!check_collision(serverGameState, tempPlayer)) {
        player.x = newX;
        player.y = newY;
    }
}

void apply_client_input(GameState& lagCompensatedGameState, const ClientInput& clientInput) {
    // Apply client input to the game state
}

void process_client_input(double clientTimestamp, const ClientInput& clientInput) {
    GameState* serverGameState = nullptr;

    // Find the server game state at the time of the client input
    for (size_t i = 0; i < gameStates.size() - 1; ++i) {
        if (gameStates[i].first <= clientTimestamp && clientTimestamp <= gameStates[i + 1].first) {
            serverGameState = &gameStates[i].second;
            break;
        }
    }

    // If we found a server game state, perform lag compensation and process the input
    if (serverGameState) {
        GameState lagCompensatedGameState = *serverGameState;
        compensate_for_lag(lagCompensatedGameState, clientInput);
        apply_client_input(lagCompensatedGameState, clientInput);
    } else {
        // Ignore input if we don't have a matching game state
    }
}

void CollectEnemyData(ProcMem& mem, DWORD entityBase) {
    EnemyData enemyData;

    // Collect enemy velocity
    float velX = mem.Read<float>(entityBase + offsets::netvars::m_vecVelocity + 0x0);
    float velY = mem.Read<float>(entityBase + offsets::netvars::m_vecVelocity + 0x4);
    float velZ = mem.Read<float>(entityBase + offsets::netvars::m_vecVelocity + 0x8);
    enemyData.velocity = std::sqrt(velX * velX + velY * velY + velZ * velZ);

    // Collect enemy animation state (you'll need to find the correct offset)
    enemyData.animationState = mem.Read<float>(entityBase + /* ANIMATION_STATE_OFFSET */ 0);

    // Collect enemy health
    int currentHealth = mem.Read<int>(entityBase + offsets::netvars::m_iHealth);

    // Check if the enemy was hit by comparing the current health to the previous health
    bool wasHitByHealth = false;
    if (!enemyDataBuffer.empty()) {
        const EnemyData& previousEnemyData = enemyDataBuffer.back();
        wasHitByHealth = currentHealth < previousEnemyData.previousHealth;
    }

    // Calculate the change in yaw
    float currentYaw = mem.Read<float>(entityBase + offsets::netvars::
    );
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

    // Store the collected data in the buffer
    enemyDataBuffers[entityBase].push_back(enemyData);

    // Optionally, limit the buffer size to a certain length
    if (enemyDataBuffers[entityBase].size() > 100) {
        enemyDataBuffers[entityBase].erase(enemyDataBuffers[entityBase].begin());
    }
}

float CalculateWeightBasedOnVelocityAndAimAndMovement(float enemyVelX, float enemyVelY, float enemyVelZ,
                                           float localPlayerAimPitch, float localPlayerAimYaw) {
    // Calculate the enemy's speed
    float enemySpeed = std::sqrt(enemyVelX * enemyVelX + enemyVelY * enemyVelY + enemyVelZ * enemyVelZ);

    // Convert local player's aim pitch and yaw to a direction vector
    float aimDirX = std::cos(localPlayerAimPitch) * std::cos(localPlayerAimYaw);
    float aimDirY = std::cos(localPlayerAimPitch) * std::sin(localPlayerAimYaw);
    float aimDirZ = std::sin(localPlayerAimPitch);

    // Calculate the cosine of the angle between local player's aim direction and enemy's velocity vector
    float dotProduct = aimDirX * enemyVelX + aimDirY * enemyVelY + aimDirZ * enemyVelZ;
    float aimDirMagnitude = std::sqrt(aimDirX * aimDirX + aimDirY * aimDirY + aimDirZ * aimDirZ);
    float cosAngle = dotProduct / (enemySpeed * aimDirMagnitude);

    // Apply a sigmoid function to transform the angle difference into a value between 0 and 1
    float weight = 1.0f / (1.0f + std::exp(-10.0f * (cosAngle - 0.5f)));
    // Read the weapon's item definition index
    int weaponID = mem.Read<int>(weaponBase + offsets::netvars::m_iItemDefinitionIndex);

    // Get weapon type
    WeaponType weaponType = GetWeaponType(weaponID);

    // Calculate the enemy's speed
    float enemySpeed = std::sqrt(enemyVelX * enemyVelX + enemyVelY * enemyVelY + enemyVelZ * enemyVelZ);

    // Adjust weight based on weapon type and enemy speed
    float weaponTypeWeight = CalculateWeaponTypeWeight(weaponType, enemySpeed);
    weight *= weaponTypeWeight;

    return weight;
}

void DataDrivenResolver(ProcMem& mem, DWORD localPlayerBase, DWORD entityBase) {
    try {
        // Read the enemy's current yaw
        float currentYaw = mem.Read<float>(entityBase + offsets::netvars::m_angEyeAnglesY);
    } catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    // Set the number of yaw steps based on the average velocity and yaw change rate
    int numYawSteps = 16; // Increase the default number of steps

    // Read the enemy's LBY value
    float enemyLBY = mem.Read<float>(entityBase + offsets::netvars::m_flLowerBodyYawTarget);
    float lbyDelta = std::abs(enemyLBY - lastLBY);
    lastLBY = enemyLBY;

    // Check if the enemy is breaking LBY
    bool isBreakingLBY = lbyDelta > 35.0f;

    // Initialize variables for the best yaw and the highest hit rate
    float bestYaw = currentYaw;
    float highestHitRate = 0.0f;

    // Calculate the average enemy velocity and yaw change rate from the enemyDataBuffer
    float avgVelocity = 0.0f;
    float avgYawChangeRate = 0.0f;
    int buffer_size = static_cast<int>(enemyDataBuffers[entityBase].size());
    for (int i = 0; i < buffer_size; ++i) {
        const auto& enemyData = enemyDataBuffers[entityBase][i];
        avgVelocity += enemyData.velocity;
        if (i > 0) {
            float yawChange = std::abs(enemyData.animationState - enemyDataBuffer[i - 1].animationState);
            avgYawChangeRate += yawChange;
        }
    }
    avgVelocity /= buffer_size;
    avgYawChangeRate /= (buffer_size - 1);

    // Set the number of yaw steps based on the average velocity and yaw change rate
    int numYawSteps = 8; // Default value
    if (avgVelocity > 250.0f || avgYawChangeRate > 30.0f) {
        numYawSteps = 16;
    }

    for (int i = 0; i < numYawSteps; ++i) {
        float testYaw = currentYaw + (360.0f / numYawSteps) * i;
        float normalizedTestYaw = NormalizeYaw(testYaw);

        // Read the enemy's velocity
        float enemyVelX = mem.Read<float>(entityBase + offsets::netvars::m_vecVelocity + 0x0);
        float enemyVelY = mem.Read<float>(entityBase + offsets::netvars::m_vecVelocity + 0x4);
        float enemyVelZ = mem.Read<float>(entityBase + offsets::netvars::m_vecVelocity + 0x8);

                // Read local player's aim angles
        float localPlayerAimPitch = mem.Read<float>(localPlayerBase + offsets::netvars::m_angEyeAnglesX);
        float localPlayerAimYaw = mem.Read<float>(localPlayerBase + offsets::netvars::m_angEyeAnglesY);

        float hitCount = 0;
        float totalCount = 0;
        int buffer_size = static_cast<int>(enemyDataBuffers[entityBase].size());
        for (int i = 0; i < buffer_size; ++i) {
            const auto& enemyData = enemyDataBuffers[entityBase][i];

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
                weight *= CalculateWeightBasedOnVelocityAndAimAndMovement(enemyVelX, enemyVelY, enemyVelZ, localPlayerAimPitch, localPlayerAimYaw);

                if (enemyData.wasHit) {
                    hitCount += weight;
                }
                totalCount += weight;
                
                // Adjust the weight based on LBY breaking
                if (isBreakingLBY) {
                    weight *= 1.5f;
                }
            }
        }

        float hitRate = hitCount / totalCount;

        // Use the enemy's animation state to further adjust the hit rate
        if (enemyAnimationState > 0.5f) {
            hitRate *= 1.2f;
        }
        else {
            // enemyAnimationState < -0.5f
            hitRate *= 0.8f;
        }

        // If the hit rate is higher than the previous best, update the best yaw and highest hit rate
        if (hitRate > highestHitRate) {
            highestHitRate = hitRate;
            bestYaw = normalizedTestYaw;
        }
    }

    // Apply smoothing to the best yaw
    float smoothingFactor = 0.2f; // You can adjust this value based on how much smoothing you want (0 = no smoothing, 1 = instant transition)
    float smoothedYaw = currentYaw + smoothingFactor * (bestYaw - currentYaw);
    float normalizedSmoothedYaw = NormalizeYaw(smoothedYaw);

    try {
        mem.Write<float>(entityBase + offsets::netvars::m_angEyeAnglesY, bestYaw);
    } catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }  
}
}
