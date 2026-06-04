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

class ITargetProvider {
    public:
        virtual std::vector<std::vector<Coord>> getTargetsCoord(std::string path) = 0;
        virtual int getTimeSteps() = 0;
        virtual int getTargetCount() = 0;
        virtual ~ITargetProvider() {}
};

class JSONTargetProvider : public ITargetProvider {
    int target_count;
    int time_steps;

    public:
        
    JSONTargetProvider(std::string path) {
            ifstream targets_json(path);
            json j_targets = json::parse(targets_json);

            this->target_count = j_targets["targetCount"];
            this->time_steps = j_targets["timeSteps"];
            targets_json.close();
        };

        std::vector<std::vector<Coord>> getTargetsCoord(std::string path) override {
            std::vector<std::vector<Coord>> targets(target_count, std::vector<Coord>(time_steps));
            ifstream targets_json(path);
            json j_targets = json::parse(targets_json);
            for (int i = 0; i < target_count; ++i) {
                const auto& positions = j_targets["targets"][i]["positions"];
                for (int j = 0; j < time_steps; ++j) {
                    targets[i][j].x = positions[j]["x"];
                    targets[i][j].y = positions[j]["y"];
                }
            }
            targets_json.close();
            return targets;
        };

        int getTargetCount() override {
            return this->target_count;
        };

        int getTimeSteps() override {
            return this->time_steps;
        };

};

class IBallisticSolver {
    public:
        virtual ResultConst calculateConst(const DroneConfig config, const Ammo& ammo) = 0;
        virtual Coord calculateBallistics(const float& acceleration_path, const Coord& dronePos, const Coord& targetPos, const float& h) = 0;
        virtual ~IBallisticSolver() {};
};

class AnalyticalSolver : public IBallisticSolver {
    public:


        ResultConst calculateConst(const DroneConfig config, const Ammo& ammo) override {
            ResultConst solverResult;
            float a = ammo.drag * g * ammo.mass - 2 * ammo.drag * ammo.drag * ammo.lift * config.attack_speed;
            float b = (-3) * g * ammo.mass * ammo.mass + 3 * ammo.drag * ammo.lift * ammo.mass * config.attack_speed;
            float c = 6 * ammo.mass * ammo.mass * config.altitude;
            float p = -(b * b) / (3 * a * a);
            float q = (2 * b * b * b) / (27 * a * a * a) + c / a;
            float acos_arg = ((3 * q) / (2 * p)) * sqrt((-3) / p);

            if(!(acos_arg >= -1) || !(acos_arg <= 1)){
                cout << "acos argument out of range (-1,1)!" << endl;
                exit(EXIT_FAILURE);
            }

            float phi = acos(acos_arg);
            float t = 2 * sqrt((-p)/3) * cos((phi + 4*pi)/3) - b / (3 * a);
            float h = config.attack_speed*t - (t * t * ammo.drag * config.attack_speed) / (2 * ammo.mass) +
            pow(t,3) * (6 * ammo.drag * g * ammo.lift * ammo.mass - 6 * ammo.drag * ammo.drag * config.attack_speed * (pow(ammo.lift,2)-1)) / (36 * ammo.mass * ammo.mass) +
            pow(t,4) * ((-6) * ammo.drag * ammo.drag * g * ammo.lift * ammo.mass * (1 + pow(ammo.lift,2) + pow(ammo.lift,4)) + 3 * pow(ammo.drag,3) * pow(ammo.lift,2) * 
            config.attack_speed * (1 + ammo.lift * ammo.lift) + 6 * pow(ammo.drag,3) * pow(ammo.lift,4) * config.attack_speed * (1 + ammo.lift * ammo.lift)) / (36 * ammo.mass * ammo.mass * ammo.mass * pow((1 + pow(ammo.lift,2)),2)) +
            pow(t,5) * (3 * pow(ammo.drag,3) * g * ammo.mass * pow(ammo.lift,3) - 3*pow(ammo.drag,4) * ammo.lift * ammo.lift * config.attack_speed * (1 + ammo.lift * ammo.lift)) / 
            (36 * (1 + ammo.lift * ammo.lift) * pow(ammo.mass,4));

            if(t <= 0 || h <= 0){
                cout << "Inadequate calculation value(s): t, h!" << endl;
                exit(EXIT_FAILURE);
            };

            solverResult.t = t;
            solverResult.h = h;
            return solverResult;

        };

        Coord calculateBallistics(const float& acceleration_path, const Coord& dronePos, const Coord& targetPos, const float& h) override {
            Coord firePos;
            float D = sqrt(pow((targetPos.x - dronePos.x),2) + pow((targetPos.x - dronePos.y),2));
            if(D <= 0){
                cout << "Inadequate calculation value D!" << endl;
                exit(EXIT_FAILURE);
            }
            float ratio = (D - h) / D;

            if((h + acceleration_path) > D){
                float drone_x_mid = targetPos.x - (targetPos.x - dronePos.x) * ((h + acceleration_path) / D);
                float drone_y_mid = targetPos.y - (targetPos.y - dronePos.y) * ((h + acceleration_path) / D);
                float D_mid = sqrt(pow((targetPos.x - drone_x_mid),2) + pow((targetPos.y - drone_y_mid),2));
                float ratio_mid = (D_mid - h) / D_mid;

                firePos = {
                    .x = drone_x_mid + (targetPos.x - drone_x_mid) * ratio_mid,
                    .y = drone_y_mid + (targetPos.y - drone_y_mid) * ratio_mid
                };
            }else{
                firePos = {
                    .x = dronePos.x + (targetPos.x - dronePos.x) * ratio,
                    .y = dronePos.y + (targetPos.y - dronePos.y) * ratio
                };
            }
            return firePos;
        }

};

class IConfigLoader {
    std::string ammo_path;
    std::string config_path;

    public:
        virtual DroneConfig getConfig() = 0;
        virtual Ammo getAmmo(const DroneConfig& config) = 0;
        virtual ~IConfigLoader() {};
};

class JSONConfigLoader : public IConfigLoader {
    std::string ammo_path;
    std::string config_path;

    public:
        JSONConfigLoader(std::string config_path, std::string ammo_path) {
            this->ammo_path = ammo_path;
            this->config_path = config_path;
        };

        DroneConfig getConfig() override {
            ifstream config_json(this->config_path);
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
        };

        Ammo getAmmo(const DroneConfig& config) override {
            ifstream ammo_json(this->ammo_path);
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
        };

};

enum class SourceType { JSON };
enum class SolverType { ANALYTICAL };
enum class LoaderType { JSON };
 
ITargetProvider* createProvider(
    SourceType type, std::string path) {
    switch (type) {
    case SourceType::JSON:
        return new JSONTargetProvider(path);
    default: return nullptr;
    }
}

IBallisticSolver* createSolver(
    SolverType type) {
    switch (type) {
    case SolverType::ANALYTICAL:
        return new AnalyticalSolver;
    default: return nullptr;
    }
}

IConfigLoader* createLoader(
    LoaderType type, std::string config_path, std::string ammo_path) {
    switch (type) {
        case LoaderType::JSON:
            return new JSONConfigLoader(config_path, ammo_path);
        default: return nullptr;
    }
}

class MissionProcessor {
    ITargetProvider*  targets;
    IConfigLoader* config;
    IBallisticSolver* solver;

    inline float calculateBearing(const float& drone_x, const float& drone_y, const float& target_x, const float& target_y) {
        float bearing = atan2(target_y - drone_y, target_x - drone_x);
        return bearing;
    };

    float normalizeAngle(float& angle) {
        while (angle > pi){
            angle -= 2.0f * pi;
        }
        while (angle < -pi){
            angle += 2.0f * pi;
        }
        return angle;
    };

    float getDeltaAngle(const float& target, const float& current) {
        float diff = target - current;
        return normalizeAngle(diff);
    };

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

    void dronePosChange(const DroneConfig& config, float& current_speed, const float& angle, float& current_dir, float& preferred_direction, const float& acceleration, Coord& dronePosNow, DroneState& state){
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

public:

    PrefParameters prefParameters;
    float delta_target_bomb;
    Coord bombLand;

    MissionProcessor(ITargetProvider* t, IConfigLoader* c, IBallisticSolver* s) : targets(t), config(c), solver(s) {}

    std::vector<std::vector<Coord>> getTargetsCoord(std::string path) {
        return targets->getTargetsCoord(path);
    };

    int getTargetCount() {
        return targets->getTargetCount();
    };

    int getTimeSteps() {
        return targets->getTimeSteps();
    };

    DroneConfig getConfig() {
        return config->getConfig();
    };
    
    Ammo getAmmo(const DroneConfig& droneConfig) {
        return config->getAmmo(droneConfig);
    };

    ResultConst calculateConst(const DroneConfig droneConfig, const Ammo& ammo) {
        return solver->calculateConst(droneConfig, ammo);
    };
    
    Coord calculateBallistics(const float& acceleration_path, const Coord& dronePos, const Coord& targetPos, const float& h) {
        return solver->calculateBallistics(acceleration_path, dronePos, targetPos, h);
    };

        // Interpolates target positions at the current simulation time
    std::vector<Coord> interpolateTargets(
        std::vector<Coord>& targetPosNow,
        const float& current_time,
        const int& sim_step,
        const float& array_time_step,
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
        std::vector<Coord>& targetPosPredicted,
        const float& current_time,
        const float& array_time_step,
        const int& target_count,
        const std::vector<Coord>& targetPosNow,
        const std::vector<Params>& params,
        const std::vector<std::vector<Coord>>& targets)
        {
        std::vector<Coord>    targetDcoord(5);
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

    void prepareOutput(const DroneConfig& config, const Ammo& ammo, const ResultConst& resultConst) {
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
    };
    

    void step(
        const std::vector<std::vector<Coord>>& targets,
        std::vector<Coord>& dropPos,
        std::vector<Coord>& targetPosPredicted,
        std::vector<Coord>& targetPosNow,
        std::vector<Params>& params,
        std::vector<SimStep>& steps,
        const float& acceleration,
        const float& t_acceleration,
        const int& target_count,
        float& current_time,
        float& current_direction,
        float& current_speed,
        const DroneConfig& config,
        Coord dronePosNow,
        ResultConst resultConst,
        int& sim_step,
        DroneState state
    ) {   
        for (int j = 0; j < target_count; ++j) {
            dropPos[j] = this->solver->calculateBallistics(config.acceleration_path, dronePosNow, targetPosNow[j], resultConst.h);
            if (sim_step == 0) {
                params[j].bearing     = calculateBearing(config.startPos.x, config.startPos.y, dropPos[j].x, dropPos[j].y);
                params[j].delta_angle = getDeltaAngle(params[j].bearing, config.initial_dir);
                params[j].drop_dist   = sqrt(pow((dropPos[j].x - config.startPos.x),2) + pow((dropPos[j].y - config.startPos.y),2));
                params[j].total_time  = getTotalTime(config, current_time, params[j].delta_angle, params[j].drop_dist, acceleration, t_acceleration);
            }else{
                targetPosPredicted = extrapolateTargets(targetPosPredicted, current_time, config.array_time_step, target_count, targetPosNow, params, targets);

            // Calculating ballistics for predicted target position
            for (int k = 0; k < target_count; ++k) {
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
        for (int i = 0; i < target_count; ++i) {
            if (params[i].total_time < min_time) {
                min_time = params[i].total_time;
                this->prefParameters = {
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
            this->prefParameters.target_pref
        };

                // Updating drone position
        dronePosChange(config, current_speed, this->prefParameters.delta_angle_pref, current_direction, this->prefParameters.bearing_pref, acceleration, dronePosNow, state);

        DEBUG("Current time is " << current_time);
        DEBUG("Number of simulation steps = " << sim_step + 1 << " Current drone coordinates are " << dronePosNow.x << ", " << dronePosNow.y);
        DEBUG("Current drone direction is " << current_direction);
        DEBUG("Preferred target bearing is: " << prefParameters.bearing_pref);
        DEBUG("Current drone speed is " << current_speed);
        DEBUG("Prefered drop point coordinates are: " << prefParameters.dropPosPref.x << ", " << prefParameters.dropPosPref.y);
        DEBUG("Preferred target index is " << prefParameters.target_pref);
        DEBUG("Current distance to preferred drop point is " << prefParameters.drop_dist_pref);
        DEBUG("Current distance to predicted target point is " << prefParameters.dist_target_predicted);

        this->bombLand = {
            dronePosNow.x + resultConst.h * cos(current_direction),
            dronePosNow.y + resultConst.h * sin(current_direction)
        };

        this->delta_target_bomb = sqrt(pow((bombLand.x - prefParameters.targetPredictedPos.x),2) + pow((bombLand.y - prefParameters.targetPredictedPos.y),2));
    
        current_time += config.sim_time_step;
        sim_step += 1;
    };

    void checkSuccess(const int& sim_step, const float& current_direction, const float& current_speed, const ResultConst& resultConst, const DroneConfig& config) {
        if ((fabs(current_direction - this->prefParameters.bearing_pref) < 1e-3) && (current_speed == config.attack_speed) &&
            (this->prefParameters.drop_dist_pref <= config.hit_radius) && (this->prefParameters.dist_target_predicted <= resultConst.h + config.hit_radius) &&
            (this->delta_target_bomb <= config.hit_radius)) {

            LOG("BOMB AWAY! Simulation complete. Steps: " << sim_step + 1);
            LOG("Bomb flight distance is " << resultConst.h);
            LOG("Bomb flight time is " << resultConst.t);
            LOG("Expected target coordinates are " << this->prefParameters.targetPredictedPos.x << ", " << this->prefParameters.targetPredictedPos.y);
            LOG("Expected bomb land coordinates are " << this->bombLand.x << ", " << this->bombLand.y);
            LOG("Delta between predicted target position and bomb land position is " << this->delta_target_bomb);

            // All vectors are automatically cleaned up — no manual delete needed
            exit(EXIT_SUCCESS);
        }
    };

    void changeSolver(IBallisticSolver* s) { solver = s; }
};


DroneState state = STOPPED;

int main(){

    float current_time = 0.0f, current_speed = 0.0f, current_direction = 0.0f;
    int sim_step = 0;

    JSONTargetProvider provider("homework_08/src/targets.json");
    JSONConfigLoader loader ("homework_08/src/config.json", "homework_08/src/ammo.json");
    AnalyticalSolver asolver;

    MissionProcessor mission(&provider, &loader, &asolver);
    DroneConfig config = mission.getConfig();
    Ammo ammo = mission.getAmmo(config);
    std::vector<vector<Coord>> targets = provider.getTargetsCoord("homework_08/src/targets.json");

    int target_count = mission.getTargetCount();

    std::vector<SimStep>  steps(10000);
    std::vector<Coord>    dropPos(target_count);
    std::vector<Coord>    targetPosPredicted(target_count, {0.0f, 0.0f});
    std::vector<Coord>    targetPosNow(target_count);
    std::vector<Params>   params(target_count);

    // Calculating additional parameters
    float acceleration = config.attack_speed * config.attack_speed / (2 * config.acceleration_path);
    float t_acceleration = (2 * config.acceleration_path) / config.attack_speed;
    current_direction = config.initial_dir;
    Coord dronePosNow = config.startPos;

    ResultConst resultConst = mission.calculateConst(config, ammo);

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

    // mission.prepareOutput(config, ammo, resultConst);

    // Main loop
    while (sim_step < 10000) {
        mission.step(targets, dropPos, targetPosPredicted, targetPosNow, params, steps, acceleration, t_acceleration, target_count, current_time, current_direction, current_speed, config, dronePosNow, resultConst, sim_step, state);

        mission.delta_target_bomb = sqrt(pow((mission.bombLand.x - mission.prefParameters.targetPredictedPos.x),2) + pow((mission.bombLand.y - mission.prefParameters.targetPredictedPos.y),2));

        // Outputting simulation.json
        json step;
        out["totalSteps"] = sim_step + 1;
        step["position"]        = {{"x", steps[sim_step].pos.x}, {"y", steps[sim_step].pos.y}};
        step["direction"]       = steps[sim_step].direction;
        step["state"]           = steps[sim_step].state;
        step["targetIndex"]     = steps[sim_step].target_idx;
        step["dropPoint"]       = {{"x", mission.prefParameters.dropPosPref.x}, {"y", mission.prefParameters.dropPosPref.y}};
        step["aimPoint"]        = {{"x", mission.bombLand.x}, {"y", mission.bombLand.y}};
        step["predictedTarget"] = {{"x", mission.prefParameters.targetPredictedPos.x}, {"y", mission.prefParameters.targetPredictedPos.y}};
        out["steps"].push_back(step);
        fout.seekp(0);
        fout << out.dump(2);
        fout.flush();

        mission.checkSuccess(sim_step, current_direction, current_speed, resultConst, config);
    };
    return 0;
}