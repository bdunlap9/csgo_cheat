#include "resolver.h"
#include "resolver.h"
#ifndef CSGO_HPP
#define CSGO_HPP

struct Vector2 {
    float x;
    float y;

    Vector2(float xValue = 0.0f, float yValue = 0.0f)
        : x(xValue), y(yValue) {}
};

struct EnemyMovementData {
    Vector3 position;
    Vector3 velocity;
    float timestamp;

    EnemyMovementData(const Vector3& pos = Vector3(0, 0, 0), const Vector3& vel = Vector3(0, 0, 0), float time = 0.0f)
        : position(pos), velocity(vel), timestamp(time) {}
};

struct Vector3 {
    float x, y, z;
	
    Vector3 CalculateMapLayoutAdjustment(const Vector3& predictedPosition, const GameMap& map) {
        // Calculate the distance to the nearest wall
        float wallDistance = map.GetDistanceToNearestWall(predictedPosition);

        // Adjust the predicted position based on the wall distance
        Vector3 mapLayoutAdjustment(0, 0, 0);
        if (wallDistance < 50.0f) {
            mapLayoutAdjustment = -predictedPosition.normalized() * (50.0f - wallDistance);
        }

        return mapLayoutAdjustment;
    }

    Vector3 CalculateMovementPatternAdjustment(const std::deque<EnemyMovementData>& enemyMovementHistory, const GameMap& map) {
        if (enemyMovementHistory.size() < 3) {
            return Vector3(0, 0, 0);
        }

        Vector3 averageVelocity(0, 0, 0);
        Vector3 averageAcceleration(0, 0, 0);
        int velocityCount = 0;
        int accelerationCount = 0;

        for (size_t i = 1; i < enemyMovementHistory.size(); ++i) {
            const EnemyMovementData& currentData = enemyMovementHistory[i];
            const EnemyMovementData& previousData = enemyMovementHistory[i - 1];

            // Calculate the velocity difference between two consecutive data points
            Vector3 velocityDifference = currentData.velocity - previousData.velocity;

            // Accumulate the velocity differences
            averageVelocity += velocityDifference;
            velocityCount++;

            if (i >= 2) {
                const EnemyMovementData& twoStepsBeforeData = enemyMovementHistory[i - 2];

                // Calculate the acceleration difference between two consecutive velocity differences
                Vector3 accelerationDifference = velocityDifference - (previousData.velocity - twoStepsBeforeData.velocity);

                // Accumulate the acceleration differences
                averageAcceleration += accelerationDifference;
                accelerationCount++;
            }
        }

        // Calculate the average velocity difference
        if (velocityCount > 0) {
            averageVelocity /= static_cast<float>(velocityCount);
        }

        // Calculate the average acceleration difference
        if (accelerationCount > 0) {
            averageAcceleration /= static_cast<float>(accelerationCount);
        }

        // Use a weighted combination of average velocity and acceleration differences as the movement pattern adjustment
        float velocityWeight = 0.7f;
        float accelerationWeight = 0.3f;
        Vector3 movementPatternAdjustment = (averageVelocity * velocityWeight) + (averageAcceleration * accelerationWeight);

        return movementPatternAdjustment;
    }


    Vector3 CalculateReactionTimeAdjustment(const std::deque<EnemyMovementData>& enemyMovementHistory) {
        // You can customize the reaction time value based on the enemy's skill or other factors
        float reactionTime = 0.2f; // Example: 200ms

        if (enemyMovementHistory.size() < 2) {
            return Vector3(0, 0, 0);
        }

        const EnemyMovementData& lastData = enemyMovementHistory.back();
        const EnemyMovementData& secondLastData = *(enemyMovementHistory.rbegin() + 1);

        // Calculate the enemy's velocity change during the reaction time
        Vector3 velocityChange = (lastData.velocity - secondLastData.velocity) * reactionTime;

        // Calculate the position adjustment based on the velocity change
        Vector3 reactionTimeAdjustment = 0.5 * velocityChange * reactionTime;

        return reactionTimeAdjustment;
    }


    Vector3 PredictEnemyPosition(const std::deque<EnemyMovementData>& enemyMovementHistory, float predictionTime, const GameMap& map) {
        if (enemyMovementHistory.size() < 2) {
            return Vector3(0, 0, 0);
        }

        const EnemyMovementData& lastData = enemyMovementHistory.back();
        const EnemyMovementData& secondLastData = *(enemyMovementHistory.rbegin() + 1);

        // Calculate the enemy's acceleration
        Vector3 acceleration = (lastData.velocity - secondLastData.velocity) / (lastData.timestamp - secondLastData.timestamp);

        // Linear extrapolation based on the enemy's last known position, velocity, and acceleration
        Vector3 predictedPosition = lastData.position + (lastData.velocity * predictionTime) + (0.5 * acceleration * predictionTime * predictionTime);

        // Factor in the enemy's previous movement patterns
        Vector3 movementPatternAdjustment = CalculateMovementPatternAdjustment(enemyMovementHistory, map);
        predictedPosition += movementPatternAdjustment;

        // Factor in the enemy's reaction times
        Vector3 reactionTimeAdjustment = CalculateReactionTimeAdjustment(enemyMovementHistory);
        predictedPosition += reactionTimeAdjustment;

        // Factor in the map layout
        Vector3 mapLayoutAdjustment = CalculateMapLayoutAdjustment(predictedPosition, map);
        predictedPosition += mapLayoutAdjustment;

        // Store the predicted positions
        static std::deque<Vector3> predictedPositions;
        predictedPositions.push_back(predictedPosition);
        if (predictedPositions.size() > 10) { // Keep a history of the last 10 predictions
            predictedPositions.pop_front();
        }

        // Apply smoothing to the predicted position
        Vector3 smoothedPrediction = ApplySmoothing(predictedPositions, predictedPosition);

        return smoothedPrediction;
    }

    Vector3 ApplySmoothing(const std::deque<Vector3>& predictedPositions, const Vector3& currentPrediction) {
        Vector3 smoothedPrediction(0, 0, 0);
        int count = 0;

        // Average the predicted positions
        for (const auto& position : predictedPositions) {
            smoothedPrediction += position;
            count++;
        }

        // Combine the current prediction with the average of the previous predictions
        if (count > 0) {
            smoothedPrediction = (smoothedPrediction / static_cast<float>(count) + currentPrediction) / 2;
        }
        else {
            smoothedPrediction = currentPrediction;
        }

        return smoothedPrediction;
    }


    Vector3& operator+=(const Vector3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vector3 operator+(const Vector3& other) const {
        return Vector3{x + other.x, y + other.y, z + other.z};
    }

    Vector3 operator-(const Vector3& other) const {
        return Vector3{x - other.x, y - other.y, z - other.z};
    }

    Vector3 operator*(float scalar) const {
        return Vector3{x * scalar, y * scalar, z * scalar};
    }
};

inline Vector3 operator*(float scalar, const Vector3& vec) {
    return vec * scalar;
}

#endif // CSGO_HPP
