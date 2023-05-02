#ifndef CSGO_HPP
#define CSGO_HPP

struct Vector3 {
    float x, y, z;

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

enum CSGOClassID {
    CBaseAnimating = 1,
    CBaseFlex = 12,
    CBasePlayer = 15,
    C_CSPlayer = 39,
    C_BaseEntity = 11
};

#endif // CSGO_HPP
