#pragma once

#include <cmath>
#include <vector>
#include <string>

enum DroneState{
    STOPPED,
    ACCELERATING,
    DECELERATING,
    TURNING,
    MOVING
};

struct Coord{
    float x;
    float y;

    Coord operator+(const Coord& other) const {};
    Coord operator-(const Coord& other) const {};
    Coord operator*(float s) const {};
    Coord operator/(float s) const {};
    Coord& operator=(const Coord& other) {};
    bool operator==(const Coord& other) const {};
    float length(Coord c){};
    Coord normalize(Coord c){};
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
    int state;
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