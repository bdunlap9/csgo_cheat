#ifndef CSGO_HPP
#define CSGO_HPP

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
