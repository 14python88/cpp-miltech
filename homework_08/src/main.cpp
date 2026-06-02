#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>
#include <cctype>
#include <vector>
#include <iomanip>
#include "json.hpp"
#include <cfloat>

#define PARAMS_SIZE 12
#define USE_MATH_DEFINES

#define ENABLE_LOG	1
#define ENABLE_DEBUG  0
 
#if ENABLE_LOG
  #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
  #define LOG(msg)
#endif
 
#if ENABLE_DEBUG
  #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
  #define DEBUG(msg)
#endif

using ordered_json = nlohmann::ordered_json;
using json = nlohmann::json;
using namespace std;

const float pi = M_PI, g = 9.81;

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
    string ammo_name;
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

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DroneConfig, startPos, ammo_name, altitude, initial_dir, attack_speed, acceleration_path, array_time_step, sim_time_step, hit_radius, angular_speed, turn_threshold)

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
    string name;
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

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Ammo, name, mass, drag, lift)

DroneState state = STOPPED;

inline int findChar(const string& value){
    for (char letter : value){
        if(isalpha(static_cast<unsigned char>(letter))){
            return 1;
        }
    }
    return 0;
}

ResultConst calculateConst(const DroneConfig config, const float& mass, const float& drag, const float& lift){
    float a = drag * g * mass - 2 * drag * drag * lift * config.attack_speed;
    float b = (-3) * g * mass * mass + 3 * drag * lift * mass * config.attack_speed;
    float c = 6 * mass * mass * config.altitude;
    float p = -(b * b) / (3 * a * a);
    float q = (2 * b * b * b) / (27 * a * a * a) + c / a;
    float acos_arg = ((3 * q) / (2 * p)) * sqrt((-3) / p);

    if(!(acos_arg >= -1) || !(acos_arg <= 1)){
        cout << "acos argument out of range (-1,1)!" << endl;
        exit(EXIT_FAILURE);
    }

    float phi = acos(acos_arg);
    float t = 2 * sqrt((-p)/3) * cos((phi + 4*pi)/3) - b / (3 * a);
    float h = config.attack_speed*t - (t * t * drag * config.attack_speed) / (2 * mass) +
    pow(t,3) * (6 * drag * g * lift * mass - 6 * drag * drag * config.attack_speed * (pow(lift,2)-1)) / (36 * mass * mass) +
    pow(t,4) * ((-6) * drag * drag * g * lift * mass * (1 + pow(lift,2) + pow(lift,4)) + 3 * pow(drag,3) * pow(lift,2) * 
    config.attack_speed * (1 + lift * lift) + 6 * pow(drag,3) * pow(lift,4) * config.attack_speed * (1 + lift * lift)) / (36 * mass * mass * mass * pow((1 + pow(lift,2)),2)) +
    pow(t,5) * (3 * pow(drag,3) * g * mass * pow(lift,3) - 3*pow(drag,4) * lift * lift * config.attack_speed * (1 + lift * lift)) / 
    (36 * (1 + lift * lift) * pow(mass,4));

    if(t <= 0 || h <= 0){
        cout << "Inadequate calculation value(s): t, h!" << endl;
        exit(EXIT_FAILURE);
    }
    return ResultConst(t, h);
}

Coord calculateBallistics(const float& acceleration_path, const Coord& dronePos, const Coord& targetPos, const float& h){
    float D = sqrt(pow((targetPos.x - dronePos.x),2) + pow((targetPos.x - dronePos.y),2));
    if(D <= 0){
        cout << "Inadequate calculation value D!" << endl;
        exit(EXIT_FAILURE);
    }
    float ratio = (D - h) / D;
    Coord firePos;

    if((h + acceleration_path) > D){
        float drone_x_mid = targetPos.x - (targetPos.x - dronePos.x) * ((h + acceleration_path) / D);
        float drone_y_mid = targetPos.y - (targetPos.y - dronePos.y) * ((h + acceleration_path) / D);
        float D_mid = sqrt(pow((targetPos.x - drone_x_mid),2) + pow((targetPos.y - drone_y_mid),2));
        float ratio_mid = (D_mid - h) / D_mid;

        firePos = {
            .x = drone_x_mid + (targetPos.x - drone_x_mid) * ratio_mid,
            .y = drone_y_mid + (targetPos.y - drone_y_mid) * ratio_mid
        };
    } else {
        firePos = {
            .x = dronePos.x + (targetPos.x - dronePos.x) * ratio,
            .y = dronePos.y + (targetPos.y - dronePos.y) * ratio
        };
    }
    return firePos;
}

inline float calculateBearing(const float& drone_x, const float& drone_y, const float& target_x, const float& target_y) {
    float bearing = atan2(target_y - drone_y, target_x - drone_x);
    return bearing;
}

float normalizeAngle(float& angle) {
    while (angle > pi){
        angle -= 2.0f * pi;
    }
    while (angle < -pi){
        angle += 2.0f * pi;
    }
    return angle;
}

float getDeltaAngle(const float& target, const float& current) {
    float diff = target - current;
    return normalizeAngle(diff);
}

float getTotalTime(const DroneConfig& config, const float& current_speed, const float& angle, const float& distance, const float& acceleration, const float& t_acceleration){
    float total_time = 0.0f;
    if((fabs(angle) <= config.turn_threshold) && (current_speed == 0.0)){
        total_time = total_time + t_acceleration + distance / config.attack_speed;
    } else if((fabs(angle) <= config.turn_threshold) && (current_speed == config.attack_speed)){
        total_time = total_time + distance / config.attack_speed;
    } else if((fabs(angle) <= config.turn_threshold) && (0 < current_speed) && (current_speed < config.attack_speed)){
        total_time = total_time + ((config.attack_speed - current_speed) / acceleration) + (distance / config.attack_speed);
    } else if((fabs(angle) > config.turn_threshold) && (current_speed == 0.0)){
        total_time = total_time + (fabs(angle) / config.angular_speed) + t_acceleration + (distance / config.attack_speed);
    } else if((fabs(angle) > config.turn_threshold) && (current_speed == config.attack_speed)){
        total_time = total_time + t_acceleration + (fabs(angle) / config.angular_speed) + t_acceleration + (distance / config.attack_speed);
    } else if((fabs(angle) > config.turn_threshold) && (0 < current_speed) && (current_speed < config.attack_speed)){
        total_time = total_time + (current_speed / acceleration) + (fabs(angle) / config.angular_speed) + t_acceleration + (distance / config.attack_speed);
    }
    return total_time;
}

void dronePosChange(const DroneConfig& config, float& current_speed, const float& angle, float& current_dir, const float& preferred_direction, const float& acceleration, Coord& dronePosNow){
    if((fabs(angle) <= config.turn_threshold) && current_speed == 0.0){
        state = ACCELERATING;
        current_dir = preferred_direction;
        current_speed = current_speed + acceleration * config.sim_time_step;
        current_speed = min(current_speed, config.attack_speed);
        dronePosNow.x += current_speed * config.sim_time_step * cos(current_dir);
        dronePosNow.y += current_speed * config.sim_time_step * sin(current_dir);
    } else if((fabs(angle) <= config.turn_threshold) && (current_speed == config.attack_speed) && (current_speed != 0.0)){
        state = MOVING;
        current_dir = preferred_direction;
        current_speed = config.attack_speed;
        dronePosNow.x += current_speed * config.sim_time_step * cos(current_dir);
        dronePosNow.y += current_speed * config.sim_time_step * sin(current_dir);
    } else if((fabs(angle) <= config.turn_threshold) && (0 < current_speed) && (current_speed < config.attack_speed)){
        state = ACCELERATING;
        current_dir = preferred_direction;
        current_speed = current_speed + acceleration * config.sim_time_step;
        current_speed = min(current_speed, config.attack_speed);
        dronePosNow.x += current_speed * config.sim_time_step * cos(current_dir);
        dronePosNow.y += current_speed * config.sim_time_step * sin(current_dir);
    } else if((fabs(angle) > config.turn_threshold) && current_speed == 0.0){
        state = TURNING;
        if(angle < 0){
            current_dir -= config.angular_speed * config.sim_time_step;
        } else if(angle > 0){
            current_dir += config.angular_speed * config.sim_time_step;
        }
    } else if((fabs(angle) > config.turn_threshold) && (current_speed == config.attack_speed) && (current_speed != 0.0)){
        state = DECELERATING;
        current_speed = current_speed - acceleration * config.sim_time_step;
        current_speed = max(0.0f, current_speed);
        dronePosNow.x += current_speed * config.sim_time_step * cos(current_dir);
        dronePosNow.y += current_speed * config.sim_time_step * sin(current_dir);
    } else if((fabs(angle) > config.turn_threshold) && (0 < current_speed) && (current_speed < config.attack_speed)){
        state = DECELERATING;
        current_speed = current_speed - acceleration * config.sim_time_step;
        current_speed = max(0.0f, current_speed);
        dronePosNow.x += current_speed * config.sim_time_step * cos(current_dir);
        dronePosNow.y += current_speed * config.sim_time_step * sin(current_dir);
    }
}

DroneConfig getConfig(std::string path){
    ifstream config_json(path);
    json data = json::parse(config_json);

    DroneConfig droneConfig = {
        .startPos = {data["drone"]["position"]["x"], data["drone"]["position"]["y"]},
        .ammo_name = data["ammo"],
        .altitude = data["drone"]["altitude"],
        .initial_dir = data["drone"]["initialDirection"],
        .attack_speed = data["drone"]["attackSpeed"],
        .acceleration_path = data["drone"]["accelerationPath"],
        .array_time_step = data["targetArrayTimeStep"],
        .sim_time_step = data["simulation"]["timeStep"],
        .hit_radius = data["simulation"]["hitRadius"],
        .angular_speed = data["drone"]["angularSpeed"],
        .turn_threshold = data["drone"]["turnThreshold"]
    };
    return droneConfig;
}

// Interpolates target positions at the current simulation time
std::vector<Coord> interpolateTargets(
    std::vector<Coord>& targetPosNow,
    float& current_time,
    int& sim_step,
    float& array_time_step,
    const int& target_count,
    const ResultConst& resultConst,
    const std::vector<std::vector<Coord>>& targets)
{
    int index = (int)floor(current_time / array_time_step);
    int idx = index % 120;
    int next = (idx + 1) % 120;
    float frac = (resultConst.t / array_time_step) - floor(resultConst.t / array_time_step);

    for (int j = 0; j < target_count; ++j) {
        if (sim_step == 0) {
            targetPosNow[j] = targets[j][0];
        } else {
            targetPosNow[j] = {
                targets[j][idx].x + (targets[j][next].x - targets[j][idx].x) * frac,
                targets[j][idx].y + (targets[j][next].y - targets[j][idx].y) * frac
            };
        }
    }
    return targetPosNow;
}

// Extrapolates predicted target positions based on current velocity
std::vector<Coord> extrapolateTargets(
    std::vector<Coord>& targetDcoord,
    std::vector<Coord>& targetPosPredicted,
    float& current_time,
    float& array_time_step,
    const int& target_count,
    const std::vector<Coord>& targetPosNow,
    const std::vector<Params>& params,
    const std::vector<std::vector<Coord>>& targets)
{
    int index = (int)floor(current_time / array_time_step);
    int idx = index % 120;
    int next = (idx + 1) % 120;

    for (int j = 0; j < target_count; ++j) {
        targetDcoord[j] = targets[j][next] - targets[j][idx];

        Velocity speed = {
            targetDcoord[j].x / array_time_step,
            targetDcoord[j].y / array_time_step
        };

        targetPosPredicted[j] = {
            targetPosNow[j].x + speed.vx * params[j].total_time,
            targetPosNow[j].y + speed.vy * params[j].total_time
        };
    }
    return targetPosPredicted;
}

Targets getTargets(std::string path){
    ifstream targets_json(path);
    json j_targets = json::parse(targets_json);

    int target_count = j_targets["targetCount"];
    int time_steps = j_targets["timeSteps"];

    std::vector<std::vector<Coord>> targets(target_count, std::vector<Coord>(time_steps));
    for (int i = 0; i < target_count; ++i) {
        const auto& positions = j_targets["targets"][i]["positions"];
        for (int j = 0; j < time_steps; ++j) {
            targets[i][j].x = positions[j]["x"];
            targets[i][j].y = positions[j]["y"];
        }
    }
    return Targets(time_steps, target_count, targets);
}

// Loads ammo arsenal from JSON and returns the matching ammo for the given config
Ammo getAmmo(std::string path, const DroneConfig& config) {
    ifstream ammo_json(path);
    json j_ammo = json::parse(ammo_json);

    std::vector<Ammo> arsenal(5);
    for (int i = 0; i < 5; ++i) {
        arsenal[i].name  = j_ammo[i]["name"];
        arsenal[i].mass  = j_ammo[i]["mass"];
        arsenal[i].drag  = j_ammo[i]["drag"];
        arsenal[i].lift  = j_ammo[i]["lift"];
    }

    for (const Ammo& a : arsenal) {
        if (config.ammo_name == a.name) {
            return a;
        }
    }

    cout << "Unknown ammo type!" << endl;
    exit(EXIT_FAILURE);
}

int main(){

    float current_time = 0.0f, current_speed = 0.0f, current_direction = 0.0f;
    float delta_target_bomb = 0.0f;
    int sim_step = 0;

    // Retrieving drone, ammo, targets parameters
    DroneConfig config = getConfig("homework_08/src/config.json");
    Targets targets = getTargets("homework_08/src/targets.json");
    Ammo ammo = getAmmo("homework_08/src/ammo.json", config);

    PrefParameters prefParameters;
    std::vector<SimStep>  steps(10000);
    std::vector<Coord>    dropPos(targets.target_count);
    std::vector<Coord>    targetPosPredicted(targets.target_count, {0.0f, 0.0f});
    std::vector<Coord>    targetPosNow(targets.target_count);
    std::vector<Coord>    targetDcoord(targets.target_count);
    std::vector<Params>   params(targets.target_count);

    // Calculating additional parameters
    float acceleration = config.attack_speed * config.attack_speed / (2 * config.acceleration_path);
    float t_acceleration = (2 * config.acceleration_path) / config.attack_speed;
    current_direction = config.initial_dir;
    Coord dronePosNow = config.startPos;

    // Calculating constants for ballistics
    ResultConst resultConst = calculateConst(config, ammo.mass, ammo.drag, ammo.lift);

    // Preparing json output
    ofstream fout("homework_08/src/simulation.json");
    json out;
    out["totalSteps"] = 0;
    out["steps"] = json::array();

    LOG("Config loaded: ");
    LOG("altitude = " << config.altitude);
    LOG("initial preferred_direction = " << config.initial_dir);
    LOG("attack speed = " << config.attack_speed);
    LOG("acceleration path = " << config.acceleration_path);
    LOG("angular speed = " << config.angular_speed);
    LOG("turn threshold = " << config.turn_threshold);
    LOG("Ammo found: " << config.ammo_name);
    LOG("ammo mass = " << ammo.mass);
    LOG("ammo drag = " << ammo.drag);
    LOG("ammo lift = " << ammo.lift);
    LOG("ammo flight time = " << resultConst.t);
    LOG("ammo flight distance = " << resultConst.h);

    // Main loop
    while (sim_step < 10000) {

        targetPosNow = interpolateTargets(targetPosNow, current_time, sim_step, config.array_time_step, targets.target_count, resultConst, targets.targets);

        for (int j = 0; j < targets.target_count; ++j) {
            dropPos[j] = calculateBallistics(config.acceleration_path, dronePosNow, targetPosNow[j], resultConst.h);

            if (sim_step == 0) {
                params[j].bearing     = calculateBearing(config.startPos.x, config.startPos.y, dropPos[j].x, dropPos[j].y);
                params[j].delta_angle = getDeltaAngle(params[j].bearing, config.initial_dir);
                params[j].drop_dist   = sqrt(pow((dropPos[j].x - config.startPos.x),2) + pow((dropPos[j].y - config.startPos.y),2));
                params[j].total_time  = getTotalTime(config, current_time, params[j].delta_angle, params[j].drop_dist, acceleration, t_acceleration);
            } else {
                targetPosPredicted = extrapolateTargets(targetDcoord, targetPosPredicted, current_time, config.array_time_step, targets.target_count, targetPosNow, params, targets.targets);

                // Calculating ballistics for predicted target positions
                for (int k = 0; k < targets.target_count; ++k) {
                    dropPos[k] = calculateBallistics(config.acceleration_path, dronePosNow, targetPosPredicted[k], resultConst.h);
                }

                params[j].bearing     = calculateBearing(dronePosNow.x, dronePosNow.y, dropPos[j].x, dropPos[j].y);
                params[j].delta_angle = getDeltaAngle(params[j].bearing, current_direction);
                params[j].drop_dist   = sqrt(pow((dropPos[j].x - dronePosNow.x),2) + pow((dropPos[j].y - dronePosNow.y),2));
                params[j].total_time  = getTotalTime(config, current_time, params[j].delta_angle, params[j].drop_dist, acceleration, t_acceleration);
            }
        }

        // Updating the preferred parameters
        float min_time = FLT_MAX;
        for (int i = 0; i < targets.target_count; ++i) {
            if (params[i].total_time < min_time) {
                min_time = params[i].total_time;
                prefParameters = {
                    params[i].total_time,
                    i,
                    targetPosPredicted[i],
                    dropPos[i],
                    params[i].bearing,
                    params[i].delta_angle,
                    params[i].drop_dist,
                    (float)sqrt(pow((targetPosPredicted[i].x - dronePosNow.x),2) + pow((targetPosPredicted[i].y - dronePosNow.y),2))
                };
            }
        }

        steps[sim_step] = {
            dronePosNow,
            current_direction,
            state,
            prefParameters.target_pref
        };

        // Updating drone position
        dronePosChange(config, current_speed, prefParameters.delta_angle_pref, current_direction, prefParameters.bearing_pref, acceleration, dronePosNow);

        DEBUG("Current time is " << current_time);
        DEBUG("Number of simulation steps = " << sim_step + 1 << " Current drone coordinates are " << dronePosNow.x << ", " << dronePosNow.y);
        DEBUG("Current drone direction is " << current_direction);
        DEBUG("Preferred target bearing is: " << prefParameters.bearing_pref);
        DEBUG("Current drone speed is " << current_speed);
        DEBUG("Prefered drop point coordinates are: " << prefParameters.dropPosPref.x << ", " << prefParameters.dropPosPref.y);
        DEBUG("Preferred target index is " << prefParameters.target_pref);
        DEBUG("Current distance to preferred drop point is " << prefParameters.drop_dist_pref);
        DEBUG("Current distance to predicted target point is " << prefParameters.dist_target_predicted);

        Coord bombLand = {
            dronePosNow.x + resultConst.h * cos(current_direction),
            dronePosNow.y + resultConst.h * sin(current_direction)
        };
        delta_target_bomb = sqrt(pow((bombLand.x - prefParameters.targetPredictedPos.x),2) + pow((bombLand.y - prefParameters.targetPredictedPos.y),2));

        // Outputting simulation.json
        json step;
        out["totalSteps"] = sim_step + 1;
        step["position"]        = {{"x", steps[sim_step].pos.x}, {"y", steps[sim_step].pos.y}};
        step["direction"]       = steps[sim_step].direction;
        step["state"]           = steps[sim_step].state;
        step["targetIndex"]     = steps[sim_step].target_idx;
        step["dropPoint"]       = {{"x", prefParameters.dropPosPref.x}, {"y", prefParameters.dropPosPref.y}};
        step["aimPoint"]        = {{"x", bombLand.x}, {"y", bombLand.y}};
        step["predictedTarget"] = {{"x", prefParameters.targetPredictedPos.x}, {"y", prefParameters.targetPredictedPos.y}};
        out["steps"].push_back(step);
        fout.seekp(0);
        fout << out.dump(2);
        fout.flush();

        // Check for bomb release condition
        if ((fabs(current_direction - prefParameters.bearing_pref) < 1e-3) && (current_speed == config.attack_speed) &&
            (prefParameters.drop_dist_pref <= config.hit_radius) && (prefParameters.dist_target_predicted <= resultConst.h + config.hit_radius) &&
            (delta_target_bomb <= config.hit_radius)) {

            LOG("BOMB AWAY! Simulation complete. Steps: " << sim_step + 1);
            LOG("Bomb flight distance is " << resultConst.h);
            LOG("Bomb flight time is " << resultConst.t);
            LOG("Expected target coordinates are " << prefParameters.targetPredictedPos.x << ", " << prefParameters.targetPredictedPos.y);
            LOG("Expected bomb land coordinates are " << bombLand.x << ", " << bombLand.y);
            LOG("Delta between predicted target position and bomb land position is " << delta_target_bomb);

            // All vectors are automatically cleaned up — no manual delete needed
            exit(EXIT_SUCCESS);
        }

        current_time += config.sim_time_step;
        sim_step += 1;
    }

    return 0;
}