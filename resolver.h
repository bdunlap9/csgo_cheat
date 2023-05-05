#include "ProcMem.h"
#include "csgo.hpp"
#include "offsets.hpp"

#include <cmath>
#include <unordered_map>
#include <vector>
#include <Windows.h>

namespace Weapon {
    enum WeaponType {
        PISTOL,
        RIFLE,
        SMG,
        SNIPER,
        SHOTGUN,
        MACHINEGUN,
        KNIFE,
        GRENADE,
        OTHER
    };

    enum WeaponId
    {
	    WEAPON_DEAGLE = 1,
	    WEAPON_ELITE = 2,
	    WEAPON_FIVESEVEN = 3,
	    WEAPON_GLOCK = 4,
	    WEAPON_AK47 = 7,
	    WEAPON_AUG = 8,
	    WEAPON_AWP = 9,
	    WEAPON_FAMAS = 10,
	    WEAPON_G3SG1 = 11,
	    WEAPON_GALILAR = 13,
	    WEAPON_M249 = 14,
	    WEAPON_M4A1 = 16,
	    WEAPON_MAC10 = 17,
	    WEAPON_P90 = 19,
	    WEAPON_MP5SD = 23,
	    WEAPON_UMP45 = 24,
	    WEAPON_XM1014 = 25,
	    WEAPON_BIZON = 26,
	    WEAPON_MAG7 = 27,
	    WEAPON_NEGEV = 28,
	    WEAPON_SAWEDOFF = 29,
	    WEAPON_TEC9 = 30,
	    WEAPON_TASER = 31,
	    WEAPON_HKP2000 = 32,
	    WEAPON_MP7 = 33,
	    WEAPON_MP9 = 34,
	    WEAPON_NOVA = 35,
	    WEAPON_P250 = 36,
	    WEAPON_SHIELD = 37,
	    WEAPON_SCAR20 = 38,
	    WEAPON_SG556 = 39,
	    WEAPON_SSG08 = 40,
	    WEAPON_KNIFEGG = 41,
	    WEAPON_KNIFE = 42,
	    WEAPON_FLASHBANG = 43,
	    WEAPON_HEGRENADE = 44,
	    WEAPON_SMOKEGRENADE = 45,
	    WEAPON_MOLOTOV = 46,
	    WEAPON_DECOY = 47,
	    WEAPON_INCGRENADE = 48,
	    WEAPON_C4 = 49,
	    WEAPON_HEALTHSHOT = 57,
	    WEAPON_KNIFE_T = 59,
	    WEAPON_M4A1_SILENCER = 60,
	    WEAPON_USP_SILENCER = 61,
	    WEAPON_CZ75A = 63,
	    WEAPON_REVOLVER = 64,
	    WEAPON_TAGRENADE = 68,
	    WEAPON_FISTS = 69,
	    WEAPON_BREACHCHARGE = 70,
	    WEAPON_TABLET = 72,
	    WEAPON_MELEE = 74,
	    WEAPON_AXE = 75,
	    WEAPON_HAMMER = 76,
	    WEAPON_SPANNER = 78,
	    WEAPON_KNIFE_GHOST = 80,
	    WEAPON_FIREBOMB = 81,
	    WEAPON_DIVERSION = 82,
	    WEAPON_FRAG_GRENADE = 83,
	    WEAPON_SNOWBALL = 84,
    	WEAPON_BUMPMINE = 85,
	    WEAPON_BAYONET = 500,
	    WEAPON_KNIFE_FLIP = 505,
	    WEAPON_KNIFE_GUT = 506,
	    WEAPON_KNIFE_KARAMBIT = 507,
	    WEAPON_KNIFE_M9_BAYONET = 508,
	    WEAPON_KNIFE_TACTICAL = 509,
	    WEAPON_KNIFE_FALCHION = 512,
	    WEAPON_KNIFE_SURVIVAL_BOWIE = 514,
	    WEAPON_KNIFE_BUTTERFLY = 515,
	    WEAPON_KNIFE_PUSH = 516,
	    WEAPON_KNIFE_URSUS = 519,
	    WEAPON_KNIFE_GYPSY_JACKKNIFE = 520,
	    WEAPON_KNIFE_STILETTO = 522,
	    WEAPON_KNIFE_WIDOWMAKER = 523,
	    STUDDED_BLOODHOUND_GLOVES = 5027,
	    T_GLOVES = 5028,
	    CT_GLOVES = 5029,
	    SPORTY_GLOVES = 5030,
	    SLICK_GLOVES = 5031,
	    LEATHER_HANDWRAPS = 5032,
	    MOTORCYCLE_GLOVES = 5033,
	    SPECIALIST_GLOVES = 5034,
	    STUDDED_HYDRA_GLOVES = 5035
    };

    WeaponType GetWeaponType(ItemDefinitionIndex weaponID) {
    // Implement a function that returns the WeaponType based on the weaponID.
    // You can use the weaponID to determine which type of weapon it is.

    switch (weaponID) {
        // Sniper Rifles
        case WeaponId::WEAPON_AWP: case WeaponId::WEAPON_SCAR20: case WeaponId::WEAPON_G3SG1: case WeaponId::WEAPON_SSG08:
            return WeaponType::Sniper;

        // Rifles
        case WeaponId::WEAPON_AK47: case WeaponId::WEAPON_AUG: case WeaponId::WEAPON_FAMAS: case WeaponId::WEAPON_GALILAR: 
        case WeaponId::WEAPON_M4A1: case WeaponId::WEAPON_SG556: case WeaponId::WEAPON_M4A1_SILENCER:
            return WeaponType::Rifle;

        // SMGs
        case WeaponId::WEAPON_MAC10: case WeaponId::WEAPON_P90: case WeaponId::WEAPON_MP5: case WeaponId::WEAPON_UMP45:
        case WeaponId::WEAPON_MP7: case WeaponId::WEAPON_MP9: case WeaponId::WEAPON_BIZON:
            return WeaponType::SMG;

        // Shotguns
        case WeaponId::WEAPON_XM1014: case WeaponId::WEAPON_NOVA: case WeaponId::WEAPON_MAG7: case WeaponId::WEAPON_SAWEDOFF:
            return WeaponType::Shotgun;

        // Pistols
        case WeaponId::WEAPON_DEAGLE: case WeaponId::WEAPON_ELITE: case WeaponId::WEAPON_FIVESEVEN: case WeaponId::WEAPON_GLOCK:
        case WeaponId::WEAPON_TEC9: case WeaponId::WEAPON_HKP2000: case WeaponId::WEAPON_USP_SILENCER: case WeaponId::WEAPON_P250:
        case WeaponId::WEAPON_CZ75A: case WeaponId::WEAPON_REVOLVER:
            return WeaponType::Pistol;

        // Knives
        case WeaponId::WEAPON_KNIFE: case WeaponId::WEAPON_KNIFE_T: case WeaponId::WEAPON_BAYONET: case WeaponId::WEAPON_KNIFE_FLIP:
        case WeaponId::WEAPON_KNIFE_GUT: case WeaponId::WEAPON_KNIFE_KARAMBIT: case WeaponId::WEAPON_KNIFE_M9_BAYONET: case WeaponId::WEAPON_KNIFE_TACTICAL:
        case WeaponId::WEAPON_KNIFE_FALCHION: case WeaponId::WEAPON_KNIFE_SURVIVAL_BOWIE: case WeaponId::WEAPON_KNIFE_BUTTERFLY: case WeaponId::WEAPON_KNIFE_PUSH:
        case WeaponId::WEAPON_KNIFE_URSUS: case WeaponId::WEAPON_KNIFE_GYPSY_JACKKNIFE: case WeaponId::WEAPON_KNIFE_STILETTO: case WeaponId::WEAPON_KNIFE_WIDOWMAKER:
            return WeaponType::Knife;

        // Grenades
        case WeaponId::WEAPON_FLASHBANG: case WeaponId::WEAPON_HEGRENADE: case WeaponId::WEAPON_SMOKEGRENADE: case WeaponId::WEAPON_MOLOTOV:
        case WeaponId::WEAPON_DECOY: case WeaponId::WEAPON_INCGRENADE: case WeaponId::WEAPON_TAGRENADE: case WeaponId::WEAPON_FIREBOMB:
        case WeaponId::WEAPON_DIVERSION: case WeaponId::WEAPON_FRAG_GRENADE:
            return WeaponType::Grenade;

        default:
            return WeaponType::Unknown;
    }
}

class DataDrivenResolver {
public:
    
    struct Boundary {
        float x, y;         // Position (top-left corner)
        float width, height; // Dimensions
    };

    // Custom data structure to store enemy behavior data
    struct EnemyData {
        float velocity;
        float animationState;
        float yaw;
        bool wasHit;
        int previousHealth;
    };
    
    struct GameState {
        std::vector<PlayerState> players;
        std::vector<GameObject> gameObjects;
        float gameTime;
        // Add more global game state variables (e.g., score, game mode, etc.)
    };

    struct PlayerState {
        float x, y;         // Position
        float vx, vy;       // Velocity
        float speed;        // Maximum speed of the player
        float abilityCooldown; // Time remaining before the ability can be used again
        float baseSpeed;       // Normal speed of the player         // Current speed of the player
        float health;
        bool isDashing;        // Whether the player is currently dashing
        float dashDuration;    // Duration of the dash ability
        glm::vec3 position;
        glm::vec3 velocity;
        glm::vec3 baseSpeed; // Normal speed of the player (per-axis)
        glm::vec3 speed;     // Current speed of the player (per-axis)
        bool isDashing;      // Whether the player is currently dashing
        float dashDuration;  // Duration of the dash ability
    };

    enum class InputCommand {
        MOVE_UP,
        MOVE_DOWN,
        MOVE_LEFT,
        MOVE_RIGHT
    };

    struct GameState {
        double timestamp;
        std::vector<PlayerState> players;
        Boundary boundary;  // Game boundary
    };

    struct GameObject {
        int id;
        glm::vec3 position;
        glm::vec3 velocity;
        BoundingBox boundingBox;
        // Add more game object-specific variables (e.g., type, state, etc.)
    };

    struct ClientInput {
        int playerIndex;          // Index of the player sending the input
        InputCommand inputCommand; // Command from the client
    };

    struct ClientInputBuffer {
        int playerIndex;
        std::queue<ClientInput> inputs;
    };

    // Define an enumeration for the actions
    enum class Action {
        MOVE_FORWARD,
        MOVE_BACKWARD,
        TURN_LEFT,
        TURN_RIGHT,
        JUMP,
        CROUCH,
        SHOOT
    };

    struct ClientInput {
        Action action;
        int playerID;
        double timestamp;
        float moveSpeed;
        float turnSpeed;
    };

}

private:
    float CalculateWeaponTypeWeight(WeaponType weaponType, float enemySpeed);
    WeaponType GetWeaponType(int weaponID);
    float NormalizeYaw(float yaw);
    float GetDistance(ProcMem& mem, DWORD localPlayerBase, DWORD entityBase);
    void CollectEnemyData(ProcMem& mem, DWORD entityBase);
    float CalculateWeightBasedOnVelocityAndAimAndMovement(float enemyVelX, float enemyVelY, float enemyVelZ,
                                                           float localPlayerAimPitch, float localPlayerAimYaw);

    // Add other necessary private methods and member variables here
};
