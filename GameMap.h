#include "cbase.h"
#include "func_break.h"

class GameMap {
public:
    GameMap() {
        // Populate the walls vector with data from the map
        // ...
    }

    float GetDistanceToNearestWall(const Vector& position) const {
        float nearestDistance = FLT_MAX;

        for (const CBaseEntity* wall : walls) {
            // Calculate the distance between the position and the wall
            float distance = CalculateDistanceToWall(position, wall);

            // Update the nearest distance if needed
            if (distance < nearestDistance) {
                nearestDistance = distance;
            }
        }

        return nearestDistance;
    }

private:
    CUtlVector<CBaseEntity*> walls;

    float CalculateDistanceToWall(const Vector& position, const CBaseEntity* wall) const {
        // Implement the distance calculation logic here
        // ...
        return distance;
    }
};
