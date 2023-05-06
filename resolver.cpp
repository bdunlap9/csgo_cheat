#include "ProcMem.h"
#include "csgo.hpp"
#include "offsets.hpp"
#include "resolver.h"
#include "pch.h"

#include <cmath>
#include <unordered_map>
#include <vector>
#include <Windows.h>
#include <iostream>
#include <chrono>
using namespace Weapon::DataDrivenResolver;

namespace resolver {

    const int TICK_RATE = 64;
    const double LAG_COMPENSATION_TIME = 0.1;  // 100ms

    const float DASH_SPEED_MULTIPLIER = 2.0f;
    const float DASH_DURATION = 1.0f;

    void CalculateWeaponTypeWeight(WeaponType weaponType, float enemySpeed); {
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
    };

    // Store enemy data in a buffer
    std::unordered_map<DWORD, std::vector<Weapon::DataDrivenResolver::EnemyData>> enemyDataBuffers;

    // Store enemy data in a buffer
    std::vector<Weapon::DataDrivenResolver::EnemyData> enemyDataBuffer;

    std::vector<Weapon::DataDrivenResolver::PlayerState> players;

    // Add a new variable to store the last LBY value
    float lastLBY = 0.0f;

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

    std::vector<std::pair<double, Weapon::DataDrivenResolver::GameState>> gameStates;

    double get_current_time() {
        return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    }

        Weapon::DataDrivenResolver::GameState get_current_game_state() {
        Weapon::DataDrivenResolver::GameState currentGameState;

        // Update the current game state
        return currentGameState;
    }   

    void update_game_state(const Weapon::DataDrivenResolver::GameState& newGameState) {
        double currentTime = get_current_time();
        gameStates.push_back({currentTime, newGameState});

        // Remove game states older than the lag compensation time
        while (!gameStates.empty() && gameStates.front().first < currentTime - resolver::LAG_COMPENSATION_TIME) {
            gameStates.erase(gameStates.begin());
        }
    }

bool check_collision(const Weapon::DataDrivenResolver::GameState& serverGameState, const Weapon::DataDrivenResolver::PlayerState& player) {
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

void update_velocity(Weapon::DataDrivenResolver::PlayerState& player, const Weapon::DataDrivenResolver::InputCommand& inputCommand) {
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

void compensate_for_lag(Weapon::DataDrivenResolver::GameState& serverGameState, const Weapon::DataDrivenResolver::ClientInput& clientInput) {
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
    } else {
        // Calculate the closest point inside the game boundary
        float clampedX = std::min(std::max(newX, serverGameState.boundary.x), serverGameState.boundary.x + serverGameState.boundary.width - player.width);
        float clampedY = std::min(std::max(newY, serverGameState.boundary.y), serverGameState.boundary.y + serverGameState.boundary.height - player.height);

        // Set the player's position to the closest point inside the game boundary
        player.x = clampedX;
        player.y = clampedY;

        // Stop the player's movement in the colliding direction
        if (clampedX != newX) {
            player.vx = 0;
        }
        if (clampedY != newY) {
            player.vy = 0;
        }
    }
}

void apply_client_input(Weapon::DataDrivenResolver::GameState& lagCompensatedGameState) {
    for (auto& inputBufferPair : inputBuffers) {
        Weapon::DataDrivenResolver::ClientInputBuffer& inputBuffer = inputBufferPair.second;
        Weapon::DataDrivenResolver::PlayerState& player = lagCompensatedGameState.players[inputBufferPair.first];

        while (!inputBuffer.inputs.empty()) {
            const ClientInput& clientInput = inputBuffer.inputs.front();
            update_velocity(player, clientInput.inputCommand);
            inputBuffer.inputs.pop();
        }
    }
}

void apply_ability(Weapon::DataDrivenResolver::PlayerState& player) {
    // Implement the "dash" ability
    if (!player.isDashing) {
        player.isDashing = true;
        player.speed = player.baseSpeed * resolver::DASH_SPEED_MULTIPLIER;
        player.dashDuration = resolver::DASH_DURATION;
    }
}

bool check_collision_3d(const GameState& gameState, const Weapon::DataDrivenResolver::PlayerState& player) {
    // Implement 3D collision detection based on your specific game mechanics and objects
    // This is just a simple AABB collision detection example
    for (const Weapon::DataDrivenResolver::GameObject& obj : gameState.gameObjects) {
        if (AABB_collision(player.boundingBox, obj.boundingBox)) {
            return true;
        }
    }
    return false;
}

void resolve_collision(const Weapon::DataDrivenResolver::GameState& gameState, PlayerState& player) {
    // Implement collision resolution based on your specific game mechanics and objects
    // This example moves the player back to their previous position as a simple resolution method
    player.position = player.previousPosition;
}

void update_game(float deltaTime, Weapon::DataDrivenResolver::GameState& gameState) {
    for (Weapon::DataDrivenResolver::PlayerState& player : gameState.players) {
        // Store the player's previous position
        player.previousPosition = player.position;

        // Update dash duration and reset speed if the dash has ended
        if (player.isDashing) {
            player.dashDuration -= deltaTime;
            if (player.dashDuration <= 0) {
                player.isDashing = false;
                player.speed = player.baseSpeed;
            }
        }

        // Update ability cooldown
        if (player.abilityCooldown > 0) {
            player.abilityCooldown -= deltaTime;
        }

        // Update player position based on velocity and speed
        player.position += player.velocity * player.speed * deltaTime;

        // Check for collisions and resolve them
        if (check_collision_3d(gameState, player)) {
            resolve_collision(gameState, player);
        }
    }

    // Update the rest of the game state, such as non-player objects, physics, etc.
    update_non_player_objects(deltaTime, gameState);
    update_physics(deltaTime, gameState);
}

void buffer_client_input(const Weapon::DataDrivenResolver::ClientInput& clientInput) {
    inputBuffers[clientInput.playerIndex].inputs.push(clientInput);
}

bool is_input_valid(const Weapon::DataDrivenResolver::ClientInput& clientInput) {
    // Check if the input struct contains valid data
    if (clientInput.action == nullptr || clientInput.playerID < 0 || clientInput.timestamp < 0) {
        return false;
    }

    // Implement additional validation logic based on your specific game mechanics
    // Example: Check if the action is one of the allowed actions
    const std::vector<std::string> allowedActions = {"move_forward", "move_backward", "turn_left", "turn_right", "jump", "crouch", "shoot"};
    bool validAction = std::find(allowedActions.begin(), allowedActions.end(), clientInput.action) != allowedActions.end();

    // Example: Check if the input values are within allowed limits
    bool validInputValues = clientInput.moveSpeed >= 0 && clientInput.moveSpeed <= 10 &&
                            clientInput.turnSpeed >= 0 && clientInput.turnSpeed <= 10;

    // Return true if all validation checks pass, otherwise return false
    return validAction && validInputValues;
}

void apply_client_input(Weapon::DataDrivenResolver::GameState& lagCompensatedGameState, const Weapon::DataDrivenResolver::ClientInput& clientInput) {
    Weapon::DataDrivenResolver::PlayerState& player = lagCompensatedGameState.players[clientInput.playerIndex];

    if (clientInput.inputCommand == InputCommand::USE_ABILITY) {
        if (player.abilityCooldown <= 0) {
            apply_ability(player);
            player.abilityCooldown = ABILITY_COOLDOWN_DURATION;
        }
    } else {
        update_velocity(player, clientInput.inputCommand);
    }
}

void process_client_input(double clientTimestamp, const Weapon::DataDrivenResolver::ClientInput& clientInput) {
    Weapon::DataDrivenResolver::GameState* serverGameState = nullptr;

    // Find the server game state at the time of the client input
    for (size_t i = 0; i < gameStates.size() - 1; ++i) {
        if (gameStates[i].first <= clientTimestamp && clientTimestamp <= gameStates[i + 1].first) {
            serverGameState = &gameStates[i].second;
            break;
        }
    }

    // If we found a server game state, perform lag compensation and process the input
    if (serverGameState) {
        Weapon::DataDrivenResolver::GameState lagCompensatedGameState = *serverGameState;
        compensate_for_lag(lagCompensatedGameState, clientInput);
        apply_client_input(lagCompensatedGameState, clientInput);
    } else {
        // Ignore input if we don't have a matching game state
    }
}

void CollectEnemyData(ProcMem& mem, DWORD entityBase) {
    Weapon::DataDrivenResolver::EnemyData enemyData;

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
                                           float localPlayerAimPitch, float localPlayerAimYaw, 
                                           float enemyHealth, float playerAccuracy, float enemyRecentMovements) {
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

    // Factor in enemy health (weight should be higher for lower health)
    float enemyHealthWeight = 1.0f - (enemyHealth / 100.0f);
    weight *= enemyHealthWeight;

    // Factor in player accuracy (weight should be higher for higher accuracy)
    weight *= playerAccuracy;

    // Factor in enemy's recent movements (weight should be higher for less predictable movements)
    weight *= (1.0f - enemyRecentMovements);

    return weight;
}

float AdjustHitRateBasedOnAnimationState(float hitRate, float animationState, float positiveThreshold, float negativeThreshold, float positiveFactor, float negativeFactor) {
    // Adjust hit rate based on enemy's animation state using the provided thresholds and factors
    if (animationState > positiveThreshold) {
        hitRate *= positiveFactor;
    } else if (animationState < negativeThreshold) {
        hitRate *= negativeFactor;
    }
    return hitRate;
}

float CalculateDynamicSmoothingFactor(float enemySpeed, Weapon::WeaponType weaponType) {
    float baseSmoothingFactor = 0.2f;
    float speedFactor = std::min(1.0f, enemySpeed / 300.0f); // Normalize enemy speed (assuming max speed is 300 units/s)
    float weaponTypeFactor = 1.0f;

    switch (weaponType) {
        case WeaponType::SniperRifle:
            weaponTypeFactor = 0.5f;
            break;
        case WeaponType::Rifle:
            weaponTypeFactor = 0.8f;
            break;
        case WeaponType::SMG:
            weaponTypeFactor = 1.0f;
            break;
        case WeaponType::Shotgun:
            weaponTypeFactor = 1.2f;
            break;
        case WeaponType::Pistol:
            weaponTypeFactor = 1.0f;
            break;
        default:
            weaponTypeFactor = 1.0f;
            break;
    }

    float dynamicSmoothingFactor = baseSmoothingFactor * (1.0f - speedFactor) * weaponTypeFactor;
    return dynamicSmoothingFactor;
}

float AdjustWeightForLBYBreaking(bool isBreakingLBY, float currentWeight, float enemySpeed, float distance) {
    if (isBreakingLBY) {
        // Calculate dynamic factors based on enemy speed and distance from the local player
        float speedFactor = 1.0f + 0.5f * std::min(enemySpeed / 300.0f, 1.0f); // You can adjust the factor and threshold values as needed
        float distanceFactor = 1.0f + 0.5f * std::min(distance / 1000.0f, 1.0f); // You can adjust the factor and threshold values as needed
        
        // Add weapon type factor
        float weaponTypeFactor = 1.0f;
        switch (weaponType) {
            case WeaponType::Sniper:
                weaponTypeFactor = 1.2f;
                break;
            case WeaponType::Rifle:
                weaponTypeFactor = 1.1f;
                break;
            case WeaponType::SMG:
                weaponTypeFactor = 0.9f;
                break;
            case WeaponType::Shotgun:
                weaponTypeFactor = 0.8f;
                break;
            // ... (other weapon types)
        }

        // Combine the dynamic factors
        float lbyBreakingFactor = speedFactor * distanceFactor * weaponTypeFactor;

        // Adjust the weight based on the dynamic LBY breaking factor
        currentWeight *= lbyBreakingFactor;
    }

    return currentWeight;
}

float CorrectDesync(float enemyYaw, float localYaw) {
    // Normalize the yaw values to be within the range [-180, 180]
    enemyYaw = NormalizeYaw(enemyYaw);
    localYaw = NormalizeYaw(localYaw);

    // Calculate the difference between the enemy's yaw and the local player's yaw
    float yawDifference = NormalizeYaw(enemyYaw - localYaw);

    // Calculate the direction of the enemy's yaw
    int yawDirection = (yawDifference >= 0.0f) ? 1 : -1;

    // Check if the yaw difference exceeds the desync threshold for CS:GO
    float desyncThreshold = 58.0f; // Desync threshold for CS:GO
    if (std::abs(yawDifference) > desyncThreshold) {
        // Apply correction based on the direction of the enemy's yaw
        float correctionAmount = yawDirection * desyncThreshold;
        localYaw = NormalizeYaw(localYaw + correctionAmount);
    }

    return localYaw;
}

void DataDrivenResolver(ProcMem& mem, DWORD localPlayerBase, DWORD entityBase) {
    try {
        // Read the enemy's current yaw
        float currentYaw = mem.Read<float>(entityBase + offsets::netvars::m_angEyeAnglesY);
    } catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

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
    int numYawSteps = GetNumYawSteps(avgVelocity, avgYawChangeRate);

    for (int i = 0; i < numYawSteps; ++i) {
        float testYaw = currentYaw + (360.0f / numYawSteps) * i;
        float normalizedTestYaw = NormalizeYaw(testYaw);

        // Read the enemy's velocity
        Vector3 enemyVelocity = GetEnemyVelocity(mem, entityBase);

        // Read local player's aim angles
        Vector2 localPlayerAimAngles = GetLocalPlayerAimAngles(mem, localPlayerBase);

        float hitCount = 0;
        float totalCount = 0;
        int buffer_size = static_cast<int>(enemyDataBuffers[entityBase].size());
        for (int i = 0; i < buffer_size; ++i) {
            const auto& enemyData = enemyDataBuffers[entityBase][i];

            // Introduce a weighting system
            float weight = GetWeightBasedOnIndex(i, buffer_size);

            // Check if the test yaw would hit the enemy based on the enemy data and additional factors
            if (enemyData.animationState > 0.5f && std::abs(normalizedTestYaw - currentYaw) < 45.0f) {
                float distance = GetDistance(mem, localPlayerBase, entityBase);

                // Adjust the weight based on distance
                weight = AdjustWeightBasedOnDistance(distance, weight);

                // Adjust the weight based on enemy's velocity and local player's aim angles
                weight = AdjustWeightBasedOnVelocityAndAim(enemyVelocity, localPlayerAimAngles, weight);

                if (enemyData.wasHit) {
                    hitCount += weight;
                }
                totalCount += weight;

                // Adjust the weight based on LBY breaking
                weight = AdjustWeightForLBYBreaking(enemyData.isBreakingLBY, weight);
            }
        }
        
        // Calculate prediction time based on server tickrate
        float predictionTime = 1.0f / serverTickRate;

        // Predict enemy position
        Vector3 predictedEnemyPosition = PredictEnemyPosition(enemyMovementHistory, predictionTime, map);

        float hitRate = hitCount / totalCount;

        // Use the enemy's animation state to further adjust the hit rate
        hitRate = AdjustHitRateBasedOnAnimationState(hitRate, enemyDataBuffers[entityBase].back().animationState);

        // If the hit rate is higher than the previous best, update the best yaw and highest hit rate
        if (hitRate > highestHitRate) {
            highestHitRate = hitRate;
            bestYaw = normalizedTestYaw;
        }
    }

    // Calculate the dynamic smoothing factor
    float dynamicSmoothingFactor = CalculateDynamicSmoothingFactor(enemySpeed, weaponType);

    // Apply smoothing to the best yaw
    float smoothedYaw = currentYaw + dynamicSmoothingFactor * (bestYaw - currentYaw);
    float normalizedSmoothedYaw = NormalizeYaw(smoothedYaw);


    try {
        mem.Write<float>(entityBase + offsets::netvars::m_angEyeAnglesY, normalizedSmoothedYaw);
    } catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }  
}
}

float Weapon::NormalizeYaw(float yaw)
{
    return 0.0f;
}
