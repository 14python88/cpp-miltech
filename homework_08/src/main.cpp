#include "../include/json.hpp"

#include "../include/MissionProcessor.hpp"

#include "../include/Structs.hpp"

#include "../include/providers/JSONTargetProvider.hpp"

#include "../include/config/JSONConfigLoader.hpp"
#include "../include/config/ComponentFactory.hpp"

#include "../include/solvers/AnalyticalSolver.hpp"

#include "../include/interfaces/IBallisticSolver.hpp"
#include "../include/interfaces/IConfigLoader.hpp"
#include "../include/interfaces/ITargetProvider.hpp"


#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>
#include <cctype>
#include <vector>
#include "../include/json.hpp"
#include <cfloat>

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
        virtual std::vector<std::vector<Coord>> getTargetsCoord() = 0;
        virtual int getTimeSteps() = 0;
        virtual int getTargetCount() = 0;
        virtual ~ITargetProvider() {}
};

class JSONTargetProvider : public ITargetProvider {
    int target_count;
    int time_steps;
    std::string path;

    public:
        
        JSONTargetProvider(std::string path) {
            this->path = path;
            ifstream targets_json(path);
            json j_targets = json::parse(targets_json);

            this->target_count = j_targets["targetCount"];
            this->time_steps = j_targets["timeSteps"];
            targets_json.close();
        };

        std::vector<std::vector<Coord>> getTargetsCoord() override {
            std::vector<std::vector<Coord>> targets(target_count, std::vector<Coord>(time_steps));
            ifstream targets_json(this->path);
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
        virtual ResultConst solve(const DroneConfig config, const Ammo& ammo) = 0;
        virtual Coord calculateBallistics(const float& acceleration_path, const Coord& dronePos, const Coord& targetPos, const float& h) = 0;
        virtual ~IBallisticSolver() {};
};

class AnalyticalSolver : public IBallisticSolver {
    public:
        ResultConst solve(const DroneConfig config, const Ammo& ammo) override {
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
};

IBallisticSolver* createSolver(
    SolverType type) {
    switch (type) {
    case SolverType::ANALYTICAL:
        return new AnalyticalSolver;
    default: return nullptr;
    }
};

IConfigLoader* createLoader(
    LoaderType type, std::string config_path, std::string ammo_path) {
    switch (type) {
        case LoaderType::JSON:
            return new JSONConfigLoader(config_path, ammo_path);
        default: return nullptr;
    }
};

class MissionProcessor {
    ITargetProvider*  provider;
    IConfigLoader* loader;
    IBallisticSolver* solver;

    inline float calculateBearing(const Coord& dronePos, const Coord& targetPos) {
        float bearing = atan2(targetPos.y - dronePos.y, targetPos.x - dronePos.x);
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

public:

    int target_count;
    int time_steps;
    DroneConfig config;
    Ammo ammo;
    std::vector<vector<Coord>> targets;
    DroneState state;
    PrefParameters prefParameters;
    float delta_target_bomb;
    float current_direction;
    float current_speed;
    float current_time;
    float acceleration;
    float t_acceleration;
    Coord bombLand;
    Coord dronePosNow;

    MissionProcessor(ITargetProvider* t, IConfigLoader* l, IBallisticSolver* s) : provider(t), loader(l), solver(s) {}

    void init() {
        this->config = loader->getConfig();
        this->ammo = loader->getAmmo(this->config);
        this->targets = provider->getTargetsCoord();
        this->target_count = provider->getTargetCount();
        this->time_steps = provider->getTimeSteps();
    };

    ResultConst solve(const DroneConfig droneConfig, const Ammo& ammo) {
        return solver->solve(droneConfig, ammo);
    };
    
    Coord calculateBallistics(const Coord& targetPos, const float& h) {
        return solver->calculateBallistics(this->config.acceleration_path, this->dronePosNow, targetPos, h);
    };

    Params calculateParameters(const Coord& dropPos) {
        Params params;
        params.bearing     = this->calculateBearing(this->dronePosNow, dropPos);
        params.delta_angle = this->getDeltaAngle(params.bearing, config.initial_dir);
        params.drop_dist   = sqrt(pow((dropPos.x - this->dronePosNow.x),2) + pow((dropPos.y - this->dronePosNow.y),2));
        params.total_time  = this->getTotalTime(this->config, this->current_time, params.delta_angle, params.drop_dist, this->acceleration, this->t_acceleration);
        return params;
    };

        // Interpolates target positions at the current simulation time
    void interpolateTargets(
        std::vector<Coord>& targetPosNow,
        const int& sim_step,
        const ResultConst& resultConst)
    {
        int index = (int)floor(this->current_time / this->config.array_time_step);
        int idx = index % 120;
        int next = (idx + 1) % 120;
        float frac = (resultConst.t / this->config.array_time_step) - floor(resultConst.t / this->config.array_time_step);

        for (int j = 0; j < this->target_count; ++j) {
            if (sim_step == 0) {
                targetPosNow[j] = targets[j][0];
            } else {
                targetPosNow[j] = {
                    this->targets[j][idx].x + (this->targets[j][next].x - this->targets[j][idx].x) * frac,
                    this->targets[j][idx].y + (this->targets[j][next].y - this->targets[j][idx].y) * frac
                };
            }
        }
    }

    // Extrapolates predicted target positions based on current velocity
    void extrapolateTargets(
        std::vector<Coord>& targetPosPredicted,
        const std::vector<Coord>& targetPosNow,
        const std::vector<Params>& params)
        {
        std::vector<Coord>    targetDcoord(5);
        int index = (int)floor(this->current_time / this->config.array_time_step);
        int idx = index % 120;
        int next = (idx + 1) % 120;

        for (int j = 0; j < this->target_count; ++j) {
            targetDcoord[j] = this->targets[j][next] - this->targets[j][idx];

            Velocity speed = {
                targetDcoord[j].x / this->config.array_time_step,
                targetDcoord[j].y / this->config.array_time_step
            };

            targetPosPredicted[j] = {
                targetPosNow[j].x + speed.vx * params[j].total_time,
                targetPosNow[j].y + speed.vy * params[j].total_time
            };
        };
    };
    
    void updatePrefParams (
        const std::vector<Params>& params,
        const std::vector<Coord>& targetPosPredicted,
        const std::vector<Coord>& dropPos
        ) {
        float min_time = FLT_MAX;
        for (int i = 0; i < this->target_count; ++i) {
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
                    (float)sqrt(pow((targetPosPredicted[i].x - this->dronePosNow.x),2) + pow((targetPosPredicted[i].y - this->dronePosNow.y),2))
                };
            }
        }
    };

    SimStep updateSteps () {
        SimStep steps = {
            this->dronePosNow,
            this->current_direction,
            this->state,
            this->prefParameters.target_pref
        };
        return steps;
    };

    void dronePosChange(){
        if((fabs(this->prefParameters.delta_angle_pref) <= this->config.turn_threshold) && this->current_speed == 0.0){
            this->state = ACCELERATING;
            this->current_direction = this->prefParameters.bearing_pref;
            this->current_speed += this->acceleration * this->config.sim_time_step;
            this->current_speed = min(this->current_speed, this->config.attack_speed);
            this->dronePosNow.x += this->current_speed * this->config.sim_time_step * cos(this->current_direction);
            this->dronePosNow.y += this->current_speed * this->config.sim_time_step * sin(this->current_direction);
        } else if((fabs(this->prefParameters.delta_angle_pref) <= this->config.turn_threshold) && (this->current_speed == this->config.attack_speed) && (this->current_speed != 0.0)){
            this->state = MOVING;
            this->current_direction = this->prefParameters.bearing_pref;
            this->current_speed = this->config.attack_speed;
            this->dronePosNow.x += this->current_speed * this->config.sim_time_step * cos(this->current_direction);
            this->dronePosNow.y += this->current_speed * this->config.sim_time_step * sin(this->current_direction);
        } else if((fabs(this->prefParameters.delta_angle_pref) <= config.turn_threshold) && (0 < current_speed) && (current_speed < config.attack_speed)){
            this->state = ACCELERATING;
            this->current_direction = this->prefParameters.bearing_pref;
            this->current_speed = this->current_speed + this->acceleration * this->config.sim_time_step;
            this->current_speed = min(this->current_speed, config.attack_speed);
            this->dronePosNow.x += this->current_speed * this->config.sim_time_step * cos(this->current_direction);
            this->dronePosNow.y += this->current_speed * this->config.sim_time_step * sin(this->current_direction);
        } else if((fabs(this->prefParameters.delta_angle_pref) > this->config.turn_threshold) && this->current_speed == 0.0){
            this->state = TURNING;
            if(this->prefParameters.delta_angle_pref < 0){
                this->current_direction -= this->config.angular_speed * this->config.sim_time_step;
            } else if(this->prefParameters.delta_angle_pref > 0){
                this->current_direction += this->config.angular_speed * this->config.sim_time_step;
            }
        } else if((fabs(this->prefParameters.delta_angle_pref) > this->config.turn_threshold) && (this->current_speed == this->config.attack_speed) && (this->current_speed != 0.0)){
            this->state = DECELERATING;
            this->current_speed = this->current_speed - this->acceleration * this->config.sim_time_step;
            this->current_speed = max(0.0f, this->current_speed);
            this->dronePosNow.x += this->current_speed * this->config.sim_time_step * cos(this->current_direction);
            this->dronePosNow.y += this->current_speed * this->config.sim_time_step * sin(this->current_direction);
        } else if((fabs(this->prefParameters.delta_angle_pref) > this->config.turn_threshold) && (0 < this->current_speed) && (this->current_speed < this->config.attack_speed)){
            this->state = DECELERATING;
            this->current_speed = this->current_speed - this->acceleration * this->config.sim_time_step;
            this->current_speed = max(0.0f, this->current_speed);
            this->dronePosNow.x += this->current_speed * this->config.sim_time_step * cos(this->current_direction);
            this->dronePosNow.y += this->current_speed * this->config.sim_time_step * sin(this->current_direction);
        }
    };

    void getDropParameters(const float& h) {
        this->bombLand = {
            this->dronePosNow.x + h * cos(this->current_direction),
            this->dronePosNow.y + h * sin(this->current_direction)
        };
        this->delta_target_bomb = sqrt(pow((this->bombLand.x - this->prefParameters.targetPredictedPos.x),2) + pow((this->bombLand.y - this->prefParameters.targetPredictedPos.y),2));
    };

    void checkSuccess(const int& sim_step, const ResultConst& resultConst) {
        if ((fabs(this->current_direction - this->prefParameters.bearing_pref) < 1e-3) && (this->current_speed == config.attack_speed) &&
            (this->prefParameters.drop_dist_pref <= this->config.hit_radius) && (this->prefParameters.dist_target_predicted <= resultConst.h + this->config.hit_radius) &&
            (this->delta_target_bomb <= this->config.hit_radius)) {

            LOG("BOMB AWAY! Simulation complete. Steps: " << sim_step + 1);
            LOG("Bomb flight distance is " << resultConst.h);
            LOG("Bomb flight time is " << resultConst.t);
            LOG("Expected target coordinates are " << this->prefParameters.targetPredictedPos.x << ", " << this->prefParameters.targetPredictedPos.y);
            LOG("Expected bomb land coordinates are " << this->bombLand.x << ", " << this->bombLand.y);
            LOG("Delta between predicted target position and bomb land position is " << this->delta_target_bomb);

            exit(EXIT_SUCCESS);
        }
    };

    void changeSolver(IBallisticSolver* s) { solver = s; }
};

int main(){

    JSONTargetProvider provider("homework_08/src/targets.json");
    JSONConfigLoader loader ("homework_08/src/config.json", "homework_08/src/ammo.json");
    AnalyticalSolver solver;

    MissionProcessor mission(&provider, &loader, &solver);
    mission.init();

    std::vector<SimStep>  steps(10000);
    std::vector<Coord>    dropPos(mission.target_count);
    std::vector<Coord>    targetPosPredicted(mission.target_count, {0.0f, 0.0f});
    std::vector<Coord>    targetPosNow(mission.target_count);
    std::vector<Params>   params(mission.target_count);

    // Calculating additional parameters
    mission.acceleration = mission.config.attack_speed * mission.config.attack_speed / (2 * mission.config.acceleration_path);
    mission.t_acceleration = (2 * mission.config.acceleration_path) / mission.config.attack_speed;
    mission.current_direction = mission.config.initial_dir;
    mission.current_time = 0.0f;
    mission.current_speed = 0.0f;
    mission.dronePosNow = mission.config.startPos;
    int sim_step = 0;
    mission.state = STOPPED;

    ResultConst resultConst = mission.solve(mission.config, mission.ammo);

    // Preparing json output
    ofstream fout("homework_08/src/simulation.json");
    json out;
    out["totalSteps"] = 0;
    out["steps"] = json::array();

    LOG("Config loaded: ");
    LOG("altitude = " << mission.config.altitude);
    LOG("initial preferred_direction = " << mission.config.initial_dir);
    LOG("attack speed = " << mission.config.attack_speed);
    LOG("acceleration path = " << mission.config.acceleration_path);
    LOG("angular speed = " << mission.config.angular_speed);
    LOG("turn threshold = " << mission.config.turn_threshold);
    LOG("Ammo found: " << mission.config.ammo_name);
    LOG("ammo mass = " << mission.ammo.mass);
    LOG("ammo drag = " << mission.ammo.drag);
    LOG("ammo lift = " << mission.ammo.lift);
    LOG("ammo flight time = " << resultConst.t);
    LOG("ammo flight distance = " << resultConst.h);

    // Main loop
    while (sim_step < 10000) {

        mission.interpolateTargets(targetPosNow, sim_step, resultConst);
        for (int j = 0; j < mission.target_count; ++j) {
            dropPos[j] = mission.calculateBallistics(targetPosNow[j], resultConst.h);
            params[j] = mission.calculateParameters(dropPos[j]);

            if (sim_step > 0) {
                mission.extrapolateTargets(targetPosPredicted, targetPosNow, params);
                dropPos[j] = mission.calculateBallistics(targetPosNow[j], resultConst.h);
                params[j] = mission.calculateParameters(dropPos[j]);
            }
        };

        mission.updatePrefParams(params, targetPosPredicted, dropPos);
        steps[sim_step] = mission.updateSteps();

        // Updating drone position
        mission.dronePosChange();

        DEBUG("Current time is " << current_time);
        DEBUG("Number of simulation steps = " << sim_step + 1 << " Current drone coordinates are " << dronePosNow.x << ", " << dronePosNow.y);
        DEBUG("Current drone direction is " << current_direction);
        DEBUG("Preferred target bearing is: " << mission.prefParameters.bearing_pref);
        DEBUG("Current drone speed is " << current_speed);
        DEBUG("Prefered drop point coordinates are: " << mission.prefParameters.dropPosPref.x << ", " << mission.prefParameters.dropPosPref.y);
        DEBUG("Preferred target index is " << mission.prefParameters.target_pref);
        DEBUG("Current distance to preferred drop point is " << mission.prefParameters.drop_dist_pref);
        DEBUG("Current distance to predicted target point is " << mission.prefParameters.dist_target_predicted);

        mission.getDropParameters(resultConst.h);

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

        mission.checkSuccess(sim_step, resultConst);
        mission.current_time += mission.config.sim_time_step;
        sim_step += 1;
        };
    return 0;
}