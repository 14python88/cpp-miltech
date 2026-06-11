#pragma once

#include <cmath>
#include <vector>
#include <string>

// enum DroneState{
//     STOPPED,
//     ACCELERATING,
//     DECELERATING,
//     TURNING,
//     MOVING
// };

struct Coord{
    float x;
    float y;

    Coord operator+(const Coord& other) const {
        Coord result;
        result.x = x + other.x;
        result.y = y + other.y;
        return result;
    }
    Coord operator-(const Coord& other) const {
        Coord result;
        result.x = x - other.x;
        result.y = y - other.y;
        return result;
    }
    Coord operator*(float s) const {
        Coord result;
        result.x = x * s;
        result.y = y * s;
        return result;
    }
    Coord operator/(float s) const {
        Coord result;
        result.x = x / s;
        result.y = y / s;
        return result;
    }
    Coord& operator=(const Coord& other) {
        x = other.x;
        y = other.y;
        return *this;
    }
    bool operator==(const Coord& other) const {
        return ((x == other.x) && (y == other.y));
    }
    float length(Coord c){
        float vector_length;
        vector_length = hypot(c.x, c.y);
        return vector_length;
    }
    Coord normalize(Coord c){
        Coord result;
        float length = sqrt(c.x * c.x + c.y * c.y);
        if (length > 0) {
            result.x = c.x / length;
            result.y = c.y / length;
        }
        return result;
    }
};

struct DroneConfig {
    Coord startPos;
    std::string ammo_name;
    float altitude;
    float initial_dir;
    float attack_speed;
    float acceleration_path;
    float array_time_step;
    float sim_time_step;
    float hit_radius;
    float angular_speed;
    float turn_threshold;
};

struct Params {
    float bearing;
    float delta_angle;
    float drop_dist;
    float total_time;
};

struct DroneContext {
    float current_direction;
    float current_speed;
    float current_time;
    float acceleration;
    float t_acceleration;
    Coord dronePosNow;
    DroneConfig config;
};

struct PrefParameters {
    float total_time_pref;
    int target_pref;
    Coord targetPredictedPos;
    Coord dropPosPref;
    float bearing_pref;
    float delta_angle_pref;
    float drop_dist_pref;
    float dist_target_predicted;
};

struct SimStep {
    Coord pos;
    float direction;
    std::string state;
    int target_idx;
};

struct Velocity {
    float vx;
    float vy;
};

struct Ammo {
    std::string name;
    float mass;
    float drag;
    float lift;
};

struct ResultConst {
    float t;
    float h;
};

struct Targets{
    int time_steps;
    int target_count;
    std::vector<std::vector<Coord>> targets;
};