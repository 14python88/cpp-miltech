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

float t = 0.0f, current_time = 0.0f, current_speed = 0.0f, current_direction = 0.0f;
float delta_target_bomb = 0.0f;
int sim_step = 0;

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
        };
        return result;
    };
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

struct SolveResult {
    float t;
    float h;
};

struct Ammo {
    string name;
    float mass;
    float drag;
    float lift;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Ammo, name, mass, drag, lift)

class ITargetProvider {
    public:
        virtual int getTargetCount() = 0;
        virtual int getTimeSteps() = 0;
        virtual Coord getTargetPos(int index, int time) = 0;
        virtual ~ITargetProvider() {}
};

class IConfigLoader{
    public:
        virtual DroneConfig getConfig() = 0;
        virtual Ammo getAmmoParams() = 0;
        virtual ~IConfigLoader() {}
};

class IBallisticSolver {
    public:
        SolveResult THSolveResult;
        virtual void solve(float altitude, Ammo ammo, float att_speed) = 0;
        virtual void ballistics(Coord dronePos, Coord targetPos, float h, float acc_path) = 0;
        virtual ~IBallisticSolver() {}
};

class IPreferredSelector {
    public:
        Coord** targets;
        Coord dronePosNow;
        Coord* targetPosNow;
        Coord* targetPosPredicted;
        DroneConfig config;
        Params* params;
        PrefParameters prefParameters;
        Coord* dropPos;
        DroneState state;

        int target_count;
        int time_steps;
        float current_time;
        float total_time;
        float current_dir;
        float acceleration;
        float t_acceleration;
        float current_speed;

        virtual void interpolateTargets() = 0;
        virtual void extrapolateTargets() = 0;
        virtual void calculatePrefParams() = 0;
        virtual void getDronePosNow(const float& angle, const float& preferred_direction) = 0;
        virtual ~IPreferredSelector() {}
};

class JsonTargetProvider : public ITargetProvider {
    public:
        int target_count;
        int time_steps;
        Coord** targets;

        JsonTargetProvider(const char* path) : target_count(0), time_steps(0), targets(nullptr) {
            json j_targets;
            ifstream targets_json(path);
            targets_json >> j_targets;
            target_count = j_targets["targetCount"];
            time_steps = j_targets["timeSteps"];
            // Retrieving coordinates from json file
            targets = new Coord*[target_count]{};
            for (int i = 0; i < target_count; ++i){
                targets[i] = new Coord[time_steps]{};
            };
            // Moving targets coordinates from json to 2d array
            for (int i = 0; i < target_count; ++i) {
                const auto& positions = j_targets["targets"][i]["positions"];
                for (int j = 0; j < time_steps; ++j) {
                    targets[i][j].x = positions[j]["x"];
                    targets[i][j].y = positions[j]["y"];
                };
            };
            targets_json.close();
        };

        int getTargetCount() override {
            return target_count;
        };

        int getTimeSteps() override {
            return time_steps;
        };

        Coord getTargetPos(int index, int time) override {
            return targets[index][time];
        };

        ~JsonTargetProvider() {
            for (int i = 0; i < target_count; i++){
                delete[] (targets[i]);
                targets[i] = nullptr;
            };
            delete[] targets;
            targets = nullptr;
        };
};

class FileConfigLoader : public IConfigLoader {
    public:
        json j_config;
        json j_ammo;
        DroneConfig config;
        Ammo ammo;

        FileConfigLoader(const char* configPath, const char* ammoPath) : j_config(""), j_ammo(""), config({}), ammo({}) {
            ifstream config_json(configPath);
            j_config = json::parse(config_json);
            config_json.close();
            ifstream ammo_json(ammoPath);
            j_ammo = json::parse(ammo_json);
            ammo_json.close();
        };

        Ammo getAmmoParams() override {
            Ammo* arsenal = new Ammo[5]{};
                for(int i = 0; i < 5; ++i){
                arsenal[i].name =  j_ammo[i]["name"];
                arsenal[i].mass = j_ammo[i]["mass"];
                arsenal[i].drag = j_ammo[i]["drag"];
                arsenal[i].lift = j_ammo[i]["lift"];
            };
            // Checking ammo name
            bool found = 0;
            for(int n = 0; n < 5; ++n){
                if(config.ammo_name == arsenal[n].name){
                    found = 1;
                    ammo.mass = arsenal[n].mass;
                    ammo.drag = arsenal[n].drag;
                    ammo.lift = arsenal[n].lift;
                };
            };
            if(found == 0){
                cout << "Unknown ammo type!\n";
                exit(EXIT_FAILURE);
            };
            return this->ammo;
        };

        DroneConfig getConfig() override {

            config = {
                .startPos = {j_config["drone"]["position"]["x"], j_config["drone"]["position"]["y"]},
                .ammo_name = j_config["ammo"],
                .altitude = j_config["drone"]["altitude"],
                .initial_dir = j_config["drone"]["initialDirection"],
                .attack_speed = j_config["drone"]["attackSpeed"],
                .acceleration_path = j_config["drone"]["accelerationPath"],
                .array_time_step = j_config["targetArrayTimeStep"],
                .sim_time_step = j_config["simulation"]["timeStep"],
                .hit_radius = j_config["simulation"]["hitRadius"],
                .angular_speed = j_config["drone"]["angularSpeed"],
                .turn_threshold = j_config["drone"]["turnThreshold"]
            };
            return this->config;
        };
        
        ~FileConfigLoader() override {};
};

class AnalyticalSolver : public IBallisticSolver {
    private:
        const float pi = M_PI, g = 9.81;
    public:
        SolveResult THSolveResult;
        Coord firePos;

        AnalyticalSolver() {
            THSolveResult = {0.0f, 0.0f};
            firePos = {0,0};
        };

        void solve(float altitude, Ammo ammo, float att_speed) override {
            // CALCULATING CONSTANTS
            float a = ammo.drag * g * ammo.mass - 2 * ammo.drag * ammo.drag * ammo.lift * att_speed;
            float b = (-3) * g * ammo.mass * ammo.mass + 3 * ammo.drag * ammo.lift * ammo.mass * att_speed;
            float c = 6 * ammo.mass * ammo.mass * altitude;
            float p = -(b * b) / (3 * a * a);
            float q = (2 * b * b * b) / (27 * a * a * a) + c / a;
            float acos_arg = ((3 * q) / (2 * p)) * sqrt((-3) / p);

            if(!(acos_arg >= -1) || !(acos_arg <= 1)){
                cout << "acos argument out of range (-1,1)!\n";
                exit(EXIT_FAILURE);
            };

            float phi = acos(acos_arg);

            this->THSolveResult.t = 2 * sqrt((-p)/3) * cos((phi + 4*pi)/3) - b / (3 * a);
            this->THSolveResult.h = att_speed * this->THSolveResult.t - (this->THSolveResult.t * this->THSolveResult.t * ammo.drag * att_speed) / (2 * ammo.mass) +
            pow(this->THSolveResult.t,3) * (6 * ammo.drag * g * ammo.lift * ammo.mass - 6 * ammo.drag * ammo.drag * att_speed * (pow(ammo.lift,2)-1)) / (36 * ammo.mass * ammo.mass) +
            pow(this->THSolveResult.t,4) * ((-6) * ammo.drag * ammo.drag * g * ammo.lift * ammo.mass * (1 + pow(ammo.lift,2) + pow(ammo.lift,4)) + 3 * pow(ammo.drag,3) * pow(ammo.lift,2) * 
            att_speed * (1 + ammo.lift * ammo.lift) + 6 * pow(ammo.drag,3) * pow(ammo.lift,4) * att_speed * (1 + ammo.lift * ammo.lift)) / (36 * ammo.mass * ammo.mass * ammo.mass * pow((1 + pow(ammo.lift,2)),2)) +
            pow(this->THSolveResult.t,5) * (3 * pow(ammo.drag,3) * g * ammo.mass * pow(ammo.lift,3) - 3*pow(ammo.drag,4) * ammo.lift * ammo.lift * att_speed * (1 + ammo.lift * ammo.lift)) / 
            (36 * (1 + ammo.lift * ammo.lift) * pow(ammo.mass,4));

            if(this->THSolveResult.t <= 0 || this->THSolveResult.h <= 0){
                cout << "Inadequate calculation value(s): t, h!\n";
                exit(EXIT_FAILURE);
            };

        };

        void ballistics(Coord dronePos, Coord targetPos, float h, float acc_path) override {

            // CALCULATING BALLISTICS
            float D = sqrt(pow((targetPos.x - dronePos.x),2) + pow((targetPos.y - dronePos.y),2));
            if(D <= 0){
                cout << "Inadequate calculation value D!\n";
                exit(EXIT_FAILURE);
            };
            float ratio = (D - h) / D;

            if((h + acc_path) > D){
                Coord dronePosMid;
                dronePosMid = {
                    .x = targetPos.x - (targetPos.x - dronePos.x) * ((h + acc_path) / D),
                    .y = targetPos.y - (targetPos.y - dronePos.y) * ((h + acc_path) / D)

                };
                float D_mid = sqrt(pow((targetPos.x - dronePosMid.x),2) + pow((targetPos.y - dronePosMid.y),2));
                float ratio_mid = (D_mid - h) / D_mid;
                firePos = {
                    .x = dronePosMid.x + (targetPos.x - dronePosMid.x) * ratio_mid,
                    .y = dronePosMid.y + (targetPos.y - dronePosMid.y) * ratio_mid
                };
            }else{
                this->firePos = {
                    .x = dronePos.x + (targetPos.x - dronePos.x) * ratio,
                    .y = dronePos.y + (targetPos.y - dronePos.y) * ratio
                };
            };
        };

        ~AnalyticalSolver() override {}
};

class PreferredSelector : public IPreferredSelector {
    float calculateBearing(Coord dronePosNow, Coord targetPosPredicted) {
        float bearing = atan2(targetPosPredicted.y - dronePosNow.y, targetPosPredicted.x - dronePosNow.x);
        return bearing;
    };

    float normalizeAngle(float& angle) {
        while (angle > M_PI){
            angle -= 2.0f * M_PI;
        };
        while (angle < -M_PI){
            angle += 2.0f * M_PI;
        };
        return angle;
    };

    float getDeltaAngle(const float& target, const float& current) {
        float diff = target - current;
        return normalizeAngle(diff);
    };

    float getTotalTime(const float& angle, const float& distance, const float& acceleration, const float& t_acceleration){
        float total_time = 0.0f;
            if((fabs(angle) <= config.turn_threshold) && (current_speed == 0.0)){
                total_time = total_time + t_acceleration + distance / config.attack_speed;

            }else if((fabs(angle) <= config.turn_threshold) && (current_speed == config.attack_speed)){
                total_time = total_time + distance / config.attack_speed;

            }else if((fabs(angle) <= config.turn_threshold) && (0 < current_speed) && (current_speed < config.attack_speed)){
                total_time = total_time + ((config.attack_speed - current_speed) / acceleration) + (distance / config.attack_speed);

            }else if((fabs(angle) > config.turn_threshold) && (current_speed == 0.0)){
                total_time = total_time + (fabs(angle) / config.angular_speed) + t_acceleration + (distance / config.attack_speed);

            }else if((fabs(angle) > config.turn_threshold) && (current_speed == config.attack_speed)){
                total_time = total_time + t_acceleration + (fabs(angle) / config.angular_speed) + t_acceleration + (distance / config.attack_speed);

            }else if((fabs(angle) > config.turn_threshold) && (0 < current_speed) && (current_speed < config.attack_speed)){
                total_time = total_time + (current_speed / acceleration) + (fabs(angle) / config.angular_speed) + t_acceleration + (distance / config.attack_speed);
            };
            return total_time;
    };

    public:
        Coord** targets;
        Coord dronePosNow;
        Coord* targetPosNow;
        Coord* targetPosPredicted;
        DroneConfig config;
        Params* params;
        PrefParameters prefParameters;
        Coord* dropPos;
        DroneState state;

        int target_count;
        int time_steps;
        float current_time;
        float total_time;
        float current_dir;
        float acceleration;
        float t_acceleration;
        float current_speed;

        PreferredSelector(int targets_count, const DroneConfig& config, Coord** targets, Coord* dropPos, int time_steps, float current_time) {
            this->target_count = targets_count;
            this->time_steps = time_steps;
            this->config = config;
            this->targets = targets;
            this->dropPos = dropPos;
            this->current_time = current_time;
            this->targetPosNow = new Coord[target_count]{};
            this->targetPosPredicted = new Coord[target_count]{};
            this->params = new Params[target_count]{};
            this->state = STOPPED;
            this->current_dir = config.initial_dir;
            this->current_speed = 0.0f;

        };

        void interpolateTargets() override {
        int index = (int)floor(current_time / config.array_time_step);
        int idx = index % time_steps;
        int next = (idx + 1) % time_steps;
        float frac = (current_time / config.array_time_step) - floor(current_time / config.array_time_step);
        for(int j = 0; j < target_count; ++j){
            if (sim_step == 0){
                targetPosNow[j] = targets[j][0];
            }else{
                this->targetPosNow[j] =          
                    {
                    .x = targets[j][idx].x + (targets[j][next].x - targets[j][idx].x) * frac,
                    .y = targets[j][idx].y + (targets[j][next].y - targets[j][idx].y) * frac
                    };
                };
            };
        };

        void extrapolateTargets() override {
            int index = (int)floor(current_time / config.array_time_step);
            int idx = index % time_steps;
            int next = (idx + 1) % time_steps;
            Coord* targetDcoord = new Coord[time_steps]{};
            targetDcoord = new Coord[time_steps]{};

            for(int j = 0; j < target_count; ++j){
                targetDcoord[j] = targets[j][next] - targets[j][idx];
                float targetSpeedX[5]{}, targetSpeedY[5]{};
                targetSpeedX[j] = targetDcoord[j].x / config.array_time_step;
                targetSpeedY[j] = targetDcoord[j].y / config.array_time_step;

                this->targetPosPredicted[j] = {
                    targetPosNow[j].x + targetSpeedX[j] * (this->total_time),
                    targetPosNow[j].y + targetSpeedY[j] * (this->total_time)
                };
            };
        };
        
    void getDronePosNow(const float& angle, const float& preferred_direction) override {
            this->acceleration = this->config.attack_speed * this->config.attack_speed / (2 * this->config.acceleration_path);
            this->t_acceleration = (2 * this->config.acceleration_path) / this->config.attack_speed;

            if((fabs(angle) <= config.turn_threshold) && current_speed == 0.0){
                this->state = ACCELERATING;
                //  Drone turns instantly
                this->current_dir = preferred_direction;
                this->current_speed += this->acceleration * this->config.sim_time_step;
                this->current_speed = min(this->current_speed, this->config.attack_speed);
                this->dronePosNow.x += this->current_speed * this->config.sim_time_step * cos(this->current_dir);
                this->dronePosNow.y += this->current_speed * this->config.sim_time_step * sin(this->current_dir);

            }else if((fabs(angle) <= config.turn_threshold) && (current_speed == config.attack_speed) && (current_speed != 0.0)){
                this->state = MOVING;
                // Drone turns instantly
                this->current_dir = preferred_direction;
                // Drone moves to drop point
                this->current_speed = this->config.attack_speed;
                this->dronePosNow.x += this->current_speed * this->config.sim_time_step * cos(this->current_dir);
                this->dronePosNow.y += this->current_speed * this->config.sim_time_step* sin(this->current_dir);

            }else if((fabs(angle) <= config.turn_threshold) && (0 < current_speed) && (current_speed < config.attack_speed)){
                this->state = ACCELERATING;
                // Drone turns instantly
                this->current_dir = preferred_direction;
                // Drone accelerates
                this->current_speed = this->current_speed + this->acceleration * this->config.sim_time_step;
                this->current_speed = min(this->current_speed, this->config.attack_speed);
                this->dronePosNow.x += this->current_speed * this->config.sim_time_step * cos(this->current_dir);
                this->dronePosNow.y += this->current_speed * this->config.sim_time_step * sin(this->current_dir);

            }else if((fabs(angle) > config.turn_threshold) && current_speed == 0.0){
                this->state = TURNING;
                // Drone turns
                if(angle < 0){
                    this->current_dir -= this->config.angular_speed * this->config.sim_time_step;
                }else if(angle > 0){
                    this->current_dir += this->config.angular_speed * this->config.sim_time_step;
                };

            }else if((fabs(angle) > config.turn_threshold) && (current_speed == config.attack_speed) && (current_speed != 0.0)){
                this->state = DECELERATING;
                // Drone decelerates
                this->current_speed = this->current_speed - this->acceleration * this->config.sim_time_step;
                this->current_speed = max(0.0f, this->current_speed);
                this->dronePosNow.x += this->current_speed * this->config.sim_time_step * cos(this->current_dir);
                this->dronePosNow.y += this->current_speed * this->config.sim_time_step * sin(this->current_dir);

            }else if((fabs(angle) > config.turn_threshold) && (0 < current_speed) && (current_speed < config.attack_speed)){
                this->state = DECELERATING;
                // Drone decelerates
                this->current_speed = this->current_speed - this->acceleration * this->config.sim_time_step;
                this->current_speed = max(0.0f, this->current_speed);
                this->dronePosNow.x += this->current_speed * this->config.sim_time_step * cos(this->current_dir);
                this->dronePosNow.y += this->current_speed * this->config.sim_time_step* sin(this->current_dir);
            };
        };

    void calculatePrefParams() override {

            for(int j = 0; j < target_count; ++j){
                params[j].bearing = calculateBearing(dronePosNow, targetPosPredicted[j]);
                params[j].delta_angle = getDeltaAngle(params[j].bearing, this->current_dir);
                params[j].drop_dist = sqrt(pow((dropPos[j].x - dronePosNow.x),2) + pow((dropPos[j].y - dronePosNow.y),2));
                params[j].total_time = getTotalTime(params[j].delta_angle, params[j].drop_dist, acceleration, t_acceleration);
            };
            // Updating the Preferred Parameters array
            float min_time = FLT_MAX;
            for (int i = 0; i < target_count; ++i){
                if(params[i].total_time  < min_time){
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
                };
            };
        };

        ~PreferredSelector() override {
            for (int i = 0; i < target_count; i++){
                delete[] (targets[i]);
                targets[i] = nullptr;
            };
            delete [] targets;
            delete [] targetPosNow;
            delete [] targetPosPredicted;
            delete [] params;
            delete [] dropPos;
                targets = nullptr;
                targetPosNow = nullptr;
                targetPosPredicted = nullptr;
                params = nullptr;
                dropPos = nullptr;
        }
};

enum class SolverType   { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType   { FILE };
 
ITargetProvider*  createProvider(ProviderType type, const char* path) {
    switch(type) {
        case ProviderType::JSON:
            return new JsonTargetProvider(path);
        default: return nullptr;
    }
};
// REMEMBER DELETE !!!

IConfigLoader*    createLoader(LoaderType type, const char* configPath, const char* ammoPath) {
    switch(type) {
        case LoaderType::FILE:
            return new FileConfigLoader(configPath, ammoPath);
        default: return nullptr;
    }
};
// REMEMBER DELETE !!!

IBallisticSolver* createSolver(SolverType type) {
    switch(type) {
        case SolverType::ANALYTICAL:
            return new AnalyticalSolver();
        default: return nullptr;
    }
};
// REMEMBER DELETE !!!

IPreferredSelector* createPreferredSelector() {
    return new PreferredSelector(0, {}, nullptr, nullptr, 0, 0);
};

class MissionProcessor {

    public:

        ITargetProvider*  targets;
        IConfigLoader* loader;
        IBallisticSolver* solver;
        IPreferredSelector* preferredSelector;
        int idx;
        int target_count;
        int time_steps;
        DroneConfig config;
        Ammo ammo;
        Coord targetPos;

        MissionProcessor(ITargetProvider* t, IConfigLoader* l, IBallisticSolver* s, IPreferredSelector* p) : targets(t), loader(l), solver(s), preferredSelector(p) {}

        void init() {
            this->target_count = targets->getTargetCount();
            this->config = loader->getConfig();
            this->ammo = loader->getAmmoParams();
        };

        void solve(float altitude, Ammo ammo, float att_speed) {
            solver->solve(altitude, ammo, att_speed);
        };

        void ballistics(Coord dronePos, Coord targetPos, float h, float acc_path) {
            solver->ballistics(dronePos, targetPos, h, acc_path);
        };

        // void step(int index, int time) {
        //     this->targetPos = targets->getTargetPos(index, time);
        //     this->idx++;
        // };

        bool hasNext(int targets_counter) {
            if(idx < targets_counter) {
                return true;
            }else{
                return false;
            };
        };

        bool reset() {
            if(idx == target_count){
                return true;
            }else{
                return false;
            }
        };

        void changeSolver(IBallisticSolver* s) {
            solver = s;
        };

        ~MissionProcessor () {}
};

int main() {

    auto* prov = createProvider(ProviderType::JSON, "homework_07/src/targets.json");
    auto* loader = createLoader(LoaderType::FILE, "homework_07/src/config.json", "homework_07/src/ammo.json");
    auto* solver = createSolver(SolverType::ANALYTICAL);
    auto* preferredSelector = createPreferredSelector();

    MissionProcessor mission(prov, loader, solver, preferredSelector);

    int sim_steps = 0;
    SimStep* steps = new SimStep[10000]{};

    // Inintializing congfg parameters, ammo parameters, target count.
    mission.init();

    // Calculating t, h parameters for ballistics
    mission.solve(mission.config.altitude, mission.ammo, mission.config.attack_speed);

    // Preparing json output
    ofstream fout("homework_07/src/simulation.json");
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
    LOG("Ammo found: " << mission.ammo.name);
    LOG("ammo mass = " << mission.ammo.mass);
    LOG("ammo drag = " << mission.ammo.drag);
    LOG("ammo lift = " << mission.ammo.lift);
    LOG("ammo flight time = " << solver->THSolveResult.t);
    LOG("ammo flight distance = " << solver->THSolveResult.h);

    // Main loop commence.
    while (sim_steps < 10000){
        // while (mission.hasNext(mission.target_count)) {
        //     mission.step(mission.idx, mission.idx);
        // };
        // mission.reset();

        preferredSelector->interpolateTargets();
        mission.ballistics(preferredSelector->dronePosNow, preferredSelector->prefParameters.targetPredictedPos, solver->THSolveResult.h, mission.config.acceleration_path);
        preferredSelector->extrapolateTargets();
        mission.ballistics(preferredSelector->dronePosNow, preferredSelector->prefParameters.targetPredictedPos, solver->THSolveResult.h, mission.config.acceleration_path);
        preferredSelector->calculatePrefParams();
        preferredSelector->getDronePosNow(preferredSelector->prefParameters.delta_angle_pref, preferredSelector->prefParameters.bearing_pref);
        steps[sim_steps] = {
            preferredSelector->dronePosNow,
            preferredSelector->prefParameters.bearing_pref,
            preferredSelector->state,
            preferredSelector->prefParameters.target_pref
            };

        DEBUG("Current time is " << current_time);
        DEBUG("Number of simulation steps = " << sim_steps + 1 << "Current drone coordinates are " << preferredSelector->dronePosNow.x << ", " << preferredSelector->dronePosNow.y);
        DEBUG("Current drone direction is " << preferredSelector->current_dir);
        DEBUG("Preferred target bearing is: " << preferredSelector->prefParameters.bearing_pref);
        DEBUG("Current drone speed is " << preferredSelector->current_speed);
        DEBUG("Prefered drop point coordinates are: " << preferredSelector->prefParameters.dropPosPref.x << ", " << preferredSelector->prefParameters.dropPosPref.y);
        DEBUG("Preferred target index is " << preferredSelector->prefParameters.target_pref);
        DEBUG("Current distance to preferred drop point is " << preferredSelector->prefParameters.drop_dist_pref);
        DEBUG("Current distance to predicted target point is " << preferredSelector->prefParameters.dist_target_predicted);

        Coord bombLand = {
            preferredSelector->dronePosNow.x + solver->THSolveResult.h * cos(preferredSelector->current_dir),
            preferredSelector->dronePosNow.y + solver->THSolveResult.h * sin(preferredSelector->current_dir)
        };
        delta_target_bomb = sqrt(pow((bombLand.x - preferredSelector->prefParameters.targetPredictedPos.x),2)+pow((bombLand.y - preferredSelector->prefParameters.targetPredictedPos.y),2));
        
        // Outputting simulation.json
        json step;
        out["totalSteps"] = sim_step + 1;
        step["position"] = {{"x", steps[sim_step].pos.x}, {"y", steps[sim_step].pos.y}};
        step["direction"] = steps[sim_step].direction;
        step["state"] = steps[sim_step].state;
        step["targetIndex"] = steps[sim_step].target_idx;
        step["dropPoint"] = {{"x", preferredSelector->prefParameters.dropPosPref.x}, {"y", preferredSelector->prefParameters.dropPosPref.y}};
        step["aimPoint"] = {{"x", bombLand.x}, {"y", bombLand.y}};
        step["predictedTarget"] = {{"x", preferredSelector->prefParameters.targetPredictedPos.x}, {"y", preferredSelector->prefParameters.targetPredictedPos.y}};  // â† moved down
        out["steps"].push_back(step);
        fout.seekp(0);
        fout << out.dump(2);
        fout.flush();

        // Check for possibility of bomb release
        if((fabs(preferredSelector->current_dir - preferredSelector->prefParameters.bearing_pref) < 1e-3) && (preferredSelector->current_speed == preferredSelector->config.attack_speed) && (preferredSelector->prefParameters.drop_dist_pref <= preferredSelector->config.hit_radius) && (preferredSelector->prefParameters.dist_target_predicted <= solver->THSolveResult.h + preferredSelector->config.hit_radius) && (delta_target_bomb <= preferredSelector->config.hit_radius)){
            
            LOG("BOMB AWAY! Simulation complete. Steps: " << sim_step + 1);
            LOG("Bomb flight distance is " << solver->THSolveResult.h);
            LOG("Bomb flight time is " << t);
            LOG("Expected target coordinates are " << preferredSelector->prefParameters.targetPredictedPos.x << ", " << preferredSelector->prefParameters.targetPredictedPos.y);
            LOG("Expected bomb land coordinates are " << bombLand.x << ", " << bombLand.y);
            LOG("Delta between predicted target position and bomb land position is " << delta_target_bomb);

            delete [] steps;
            delete [] prov;
            delete [] loader;
            delete [] solver;
            delete [] preferredSelector;
        };
        preferredSelector->current_time += preferredSelector->config.sim_time_step;
        sim_steps++;
        };
    return 0;
    }