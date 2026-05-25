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
        virtual Coord solve(Coord dronePos, Coord targetPos, float altitude, Ammo ammo_parameters, float att_speed, float acc_path) = 0;
        virtual ~IBallisticSolver() {}
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
        DroneConfig config;
        Ammo ammo;

        FileConfigLoader(const char* config_path, const char* ammo_path) : config({}), ammo({}) {
            ifstream config_json(config_path);
            json data = json::parse(config_json);
            DroneConfig config = {
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
            config_json.close();

            ifstream ammo_json(ammo_path);
            json j_ammo;
            ammo_json >> j_ammo;
            Ammo* arsenal = new Ammo[5]{};
                for(int i = 0; i < 5; ++i){
                arsenal[i].name =  j_ammo[i]["name"];
                arsenal[i].mass = j_ammo[i]["mass"];
                arsenal[i].drag = j_ammo[i]["drag"];
                arsenal[i].lift = j_ammo[i]["lift"];
            };
            ammo_json.close();
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
        };

        DroneConfig getConfig() override {
            return config;
        };

        Ammo getAmmoParams() override {
            return ammo;
        };
        
        ~FileConfigLoader() {};
};

class AnalyticalSolver : public IBallisticSolver {
    private:
        const float pi = M_PI, g = 9.81;
    public:
        float t;
        float h;
        Coord firePos;

        AnalyticalSolver() {
            t = 0;
            h = 0;
            firePos = {0,0};
        };

        Coord solve(Coord dronePos, Coord targetPos, float altitude, Ammo ammo, float att_speed, float acc_path) override {
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
            t = 2 * sqrt((-p)/3) * cos((phi + 4*pi)/3) - b / (3 * a);
            h = att_speed * t - (t * t * ammo.drag * att_speed) / (2 * ammo.mass) +
            pow(t,3) * (6 * ammo.drag * g * ammo.lift * ammo.mass - 6 * ammo.drag * ammo.drag * att_speed * (pow(ammo.lift,2)-1)) / (36 * ammo.mass * ammo.mass) +
            pow(t,4) * ((-6) * ammo.drag * ammo.drag * g * ammo.lift * ammo.mass * (1 + pow(ammo.lift,2) + pow(ammo.lift,4)) + 3 * pow(ammo.drag,3) * pow(ammo.lift,2) * 
            att_speed * (1 + ammo.lift * ammo.lift) + 6 * pow(ammo.drag,3) * pow(ammo.lift,4) * att_speed * (1 + ammo.lift * ammo.lift)) / (36 * ammo.mass * ammo.mass * ammo.mass * pow((1 + pow(ammo.lift,2)),2)) +
            pow(t,5) * (3 * pow(ammo.drag,3) * g * ammo.mass * pow(ammo.lift,3) - 3*pow(ammo.drag,4) * ammo.lift * ammo.lift * att_speed * (1 + ammo.lift * ammo.lift)) / 
            (36 * (1 + ammo.lift * ammo.lift) * pow(ammo.mass,4));

            if(t <= 0 || h <= 0){
                cout << "Inadequate calculation value(s): t, h!\n";
                exit(EXIT_FAILURE);
            };

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
                firePos = {
                    .x = dronePos.x + (targetPos.x - dronePos.x) * ratio,
                    .y = fire_pos_y = dronePos.y + (targetPos.y - dronePos.y) * ratio
                };
            };
            return firePos;
        };

        ~AnalyticalSolver() {}
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

IConfigLoader*    createLoader(LoaderType type, const char* config_path, const char* ammo_path) {
    switch(type) {
        case LoaderType::FILE:
            return new FileConfigLoader(config_path, ammo_path);
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

class MissionProcessor {
    public:
        init(configSource)
        hasNext()
        step()
        reset()
        changeSolver(s)



};



//             // Calculating additional parameters
//             float acceleration = config.attack_speed * config.attack_speed / (2 * config.acceleration_path);
//             float t_acceleration = (2 * config.acceleration_path) / config.attack_speed;
//             current_direction = config.initial_dir;
//             dronePosNow = config.startPos;
//             }
// }

DroneState state = STOPPED;
Coord dronePosNow;
Coord dropPosPref;
DroneConfig config;

// inline int findChar(const string& value){
//     for (char letter : value){
//         if(isalpha(static_cast<unsigned char>(letter))){
//             return 1;
//         };
//     };
//     return 0;
// }

// int calculateConst(const float& mass, const float& drag, const float&  lift){
//     float a = drag * g * mass - 2 * drag * drag * lift * config.attack_speed;
//     float b = (-3) * g * mass * mass + 3 * drag * lift * mass * config.attack_speed;
//     float c = 6 * mass * mass * config.altitude;
//     float p = -(b * b) / (3 * a * a);
//     float q = (2 * b * b * b) / (27 * a * a * a) + c / a;
//     float acos_arg = ((3 * q) / (2 * p)) * sqrt((-3) / p);

//     if(!(acos_arg >= -1) || !(acos_arg <= 1)){
//         cout << "acos argument out of range (-1,1)!" << endl;
//         return 1;
//     };

//     float phi = acos(acos_arg);
//     t = 2 * sqrt((-p)/3) * cos((phi + 4*pi)/3) - b / (3 * a);
//     h = config.attack_speed*t - (t * t * drag * config.attack_speed) / (2 * mass) +
//     pow(t,3) * (6 * drag * g * lift * mass - 6 * drag * drag * config.attack_speed * (pow(lift,2)-1)) / (36 * mass * mass) +
//     pow(t,4) * ((-6) * drag * drag * g * lift * mass * (1 + pow(lift,2) + pow(lift,4)) + 3 * pow(drag,3) * pow(lift,2) * 
//     config.attack_speed * (1 + lift * lift) + 6 * pow(drag,3) * pow(lift,4) * config.attack_speed * (1 + lift * lift)) / (36 * mass * mass * mass * pow((1 + pow(lift,2)),2)) +
//     pow(t,5) * (3 * pow(drag,3) * g * mass * pow(lift,3) - 3*pow(drag,4) * lift * lift * config.attack_speed * (1 + lift * lift)) / 
//     (36 * (1 + lift * lift) * pow(mass,4));

//     if(t <= 0 || h <= 0){
//         cout << "Inadequate calculation value(s): t, h!" << endl;
//         return 1;
//     };
//     return 0;
// }

// int calculateBallistics(const float& drone_x, const float& drone_y, const float& target_x, const float& target_y){

//     float D = sqrt(pow((target_x - drone_x),2) + pow((target_y - drone_y),2));
//     if(D <= 0){
//         cout << "Inadequate calculation value D!" << endl;
//         return 1;
//     };
//     float ratio = (D - h) / D;

//     if((h + config.acceleration_path) > D){
//         float drone_x_mid = target_x - (target_x - drone_x) * ((h + config.acceleration_path) / D);
//         float drone_y_mid = target_y - (target_y - drone_y) * ((h + config.acceleration_path) / D);
//         float D_mid = sqrt(pow((target_x - drone_x_mid),2) + pow((target_y - drone_y_mid),2));
//         float ratio_mid = (D_mid - h) / D_mid;
//         float fire_pos_x = drone_x_mid + (target_x - drone_x_mid) * ratio_mid;
//         float fire_pos_y = drone_y_mid + (target_y - drone_y_mid) * ratio_mid;
//     }else{
//         float fire_pos_x = drone_x + (target_x - drone_x) * ratio;
//         float fire_pos_y = drone_y + (target_y - drone_y) * ratio;
//     };
//     return 0;

// }

inline float calculateBearing(const float& drone_x, const float& drone_y, const float& target_x, const float& target_y) {
    float bearing = atan2(target_y - drone_y, target_x - drone_x);
    return bearing;
}

float normalizeAngle(float& angle) {
    while (angle > pi){
        angle -= 2.0f * pi;
    };
    while (angle < -pi){
        angle += 2.0f * pi;
    };
    return angle;
}

float getDeltaAngle(const float& target, const float& current) {
    float diff = target - current;
    return normalizeAngle(diff);
}

float getTotalTime(const float& angle, const float& distance, const float& acceleration, const float& t_acceleration){
    float total_time = 0.0f;
        if((fabs(angle) <= config.turn_threshold) && (current_speed == 0.0)){
            // Drone turns instantly
            // Drone accelerates
            // Drone travels to drop point
            total_time = total_time + t_acceleration + distance / config.attack_speed;

        }else if((fabs(angle) <= config.turn_threshold) && (current_speed == config.attack_speed)){
            // Drone turns instantly
            // Drone travels to drop point
            total_time = total_time + distance / config.attack_speed;

        }else if((fabs(angle) <= config.turn_threshold) && (0 < current_speed) && (current_speed < config.attack_speed)){
            // Drone turns instantly
            // Drone accelerates
            // Drone travels to drop point
            total_time = total_time + ((config.attack_speed - current_speed) / acceleration) + (distance / config.attack_speed);

        }else if((fabs(angle) > config.turn_threshold) && (current_speed == 0.0)){
            // drone turns
            // Drone accelerates
            // Drone travels to drop point
            total_time = total_time + (fabs(angle) / config.angular_speed) + t_acceleration + (distance / config.attack_speed);

        }else if((fabs(angle) > config.turn_threshold) && (current_speed == config.attack_speed)){
            // Drone decelerates
            // Drone turns
            // Drone accelerates
            // Drone travels to drop point
            total_time = total_time + t_acceleration + (fabs(angle) / config.angular_speed) + t_acceleration + (distance / config.attack_speed);

        }else if((fabs(angle) > config.turn_threshold) && (0 < current_speed) && (current_speed < config.attack_speed)){
            // Drone decelerates
            // Drone turns
            // Drone accelerates
            // Drone travels to drop point
            total_time = total_time + (current_speed / acceleration) + (fabs(angle) / config.angular_speed) + t_acceleration + (distance / config.attack_speed);
        };
        return total_time;
}

void dronePosChange(const float& angle, float& current_dir, const float& preferred_direction, const float& acceleration){
        if((fabs(angle) <= config.turn_threshold) && current_speed == 0.0){
            state = ACCELERATING;
            //  Drone turns instantly
            current_dir = preferred_direction;
            current_speed = current_speed + acceleration * config.sim_time_step;
            current_speed = min(current_speed, config.attack_speed);
            dronePosNow.x += current_speed * config.sim_time_step * cos(current_dir);
            dronePosNow.y += current_speed * config.sim_time_step * sin(current_dir);

        }else if((fabs(angle) <= config.turn_threshold) && (current_speed == config.attack_speed) && (current_speed != 0.0)){
            state = MOVING;
            // Drone turns instantly
            current_dir = preferred_direction;
            // Drone moves to drop point
            current_speed = config.attack_speed;
            dronePosNow.x += current_speed * config.sim_time_step * cos(current_dir);
            dronePosNow.y += current_speed * config.sim_time_step* sin(current_dir);

        }else if((fabs(angle) <= config.turn_threshold) && (0 < current_speed) && (current_speed < config.attack_speed)){
            state = ACCELERATING;
            // Drone turns instantly
            current_dir = preferred_direction;
            // Drone accelerates
            current_speed = current_speed + acceleration * config.sim_time_step;
            current_speed = min(current_speed, config.attack_speed);
            dronePosNow.x += current_speed * config.sim_time_step * cos(current_dir);
            dronePosNow.y += current_speed * config.sim_time_step * sin(current_dir);

        }else if((fabs(angle) > config.turn_threshold) && current_speed == 0.0){
            state = TURNING;
            // Drone turns
            if(angle < 0){
                current_dir -= config.angular_speed * config.sim_time_step;
            }else if(angle > 0){
                current_dir += config.angular_speed * config.sim_time_step;
            };

        }else if((fabs(angle) > config.turn_threshold) && (current_speed == config.attack_speed) && (current_speed != 0.0)){
            state = DECELERATING;
            // Drone decelerates
            current_speed = current_speed - acceleration * config.sim_time_step;
            current_speed = max(0.0f, current_speed);
            dronePosNow.x += current_speed * config.sim_time_step * cos(current_dir);
            dronePosNow.y += current_speed * config.sim_time_step * sin(current_dir);

        }else if((fabs(angle) > config.turn_threshold) && (0 < current_speed) && (current_speed < config.attack_speed)){
            state = DECELERATING;
            // Drone decelerates
            current_speed = current_speed - acceleration * config.sim_time_step;
            current_speed = max(0.0f, current_speed);
            dronePosNow.x += current_speed * config.sim_time_step * cos(current_dir);
            dronePosNow.y += current_speed * config.sim_time_step* sin(current_dir);
        };
}

// DroneConfig getConfig(){
//     ifstream config_json("config.json");
//     json data = json::parse(config_json);

//     DroneConfig droneConfig = {
//         .startPos = {data["drone"]["position"]["x"], data["drone"]["position"]["y"]},
//         .ammo_name = data["ammo"],
//         .altitude = data["drone"]["altitude"],
//         .initial_dir = data["drone"]["initialDirection"],
//         .attack_speed = data["drone"]["attackSpeed"],
//         .acceleration_path = data["drone"]["accelerationPath"],
//         .array_time_step = data["targetArrayTimeStep"],
//         .sim_time_step = data["simulation"]["timeStep"],
//         .hit_radius = data["simulation"]["hitRadius"],
//         .angular_speed = data["drone"]["angularSpeed"],
//         .turn_threshold = data["drone"]["turnThreshold"]
//     };
//     return droneConfig;
// }

int main(){

    // Retrieving drone parameters
    FileConfigLoader configLoader;
    DroneConfig config = configLoader.getConfig();
    Ammo* arsenal = configLoader.getAmmoParams();

    JsonTargetProvider targetProvider;
    int target_count = targetProvider.getTargetCount();

    // // Retrieving ammo parameters
    // ifstream config_json("ammo.json");
    // json j_ammo;
    // config_json >> j_ammo;
    // Ammo* arsenal = new Ammo[N]{};
    // for(int i = 0; i < N; ++i){
    //     arsenal[i].name =  j_ammo[i]["name"];
    //     arsenal[i].mass = j_ammo[i]["mass"];
    //     arsenal[i].drag = j_ammo[i]["drag"];
    //     arsenal[i].lift = j_ammo[i]["lift"];
    // };

    // // Checking ammo name
    // bool found = 0;
    // for(int n = 0; n < N; ++n){
    //     if(config.ammo_name == arsenal[n].name){
    //         found = 1;
    //         m = arsenal[n].mass;
    //         d = arsenal[n].drag;
    //         l = arsenal[n].lift;
    //     };
    // };
    // if(found == 0){
    //     cout << "Unknown ammo type!" << endl;
    //     return 1;
    // };

    // // Retrieving targets info
    // ifstream targets_json("targets.json");
    // json j_targets;
    // targets_json >> j_targets;
    // int target_count = j_targets["targetCount"];
    // int time_steps = j_targets["timeSteps"];
    // Coord** targets = new Coord*[target_count]{};
    // for (int i = 0; i < target_count; ++i){
    //     targets[i] = new Coord[time_steps]{};
    // };

    // // Moving targets coordinates from json to 2d array
    // for (int i = 0; i < target_count; ++i) {
    //     const auto& positions = j_targets["targets"][i]["positions"];
    //     for (int j = 0; j < time_steps; ++j) {
    //         targets[i][j].x = positions[j]["x"];
    //         targets[i][j].y = positions[j]["y"];
    //     };
    // };

    // Calculating additional parameters
    float acceleration = config.attack_speed * config.attack_speed / (2 * config.acceleration_path);
    float t_acceleration = (2 * config.acceleration_path) / config.attack_speed;
    current_direction = config.initial_dir;
    dronePosNow = config.startPos;
    // Calculating constants for Ballistics
    

    AnalyticalSolver solver;
    Coord firePos = solver.solve() 
    calculateConst(m, d, l);

    // Initializing arrays from structs
    PrefParameters prefParameters;
    SimStep* steps = new SimStep[10000]{};
    Coord* targetPosNow = new Coord[time_steps]{};
    Coord* dropPos = new Coord[time_steps]{};
    Coord* targetDcoord = new Coord[time_steps]{};
    Coord* targetPosPredicted = new Coord[time_steps]{};
    Params* params = new Params[time_steps]{};

    for(int j = 0; j < target_count; ++j){
        targetPosPredicted[j] = {0.0f, 0.0f};
    };

    // Preparing json output
    ofstream fout("simulation.json");
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
    LOG("ammo mass = " << m);
    LOG("ammo drag = " << d);
    LOG("ammo lift = " << l);
    LOG("ammo flight time = " << t);
    LOG("ammo flight distance = " << h);
 
    // Main loop start
    while (sim_step < 10000){
        
        int index = (int)floor(current_time / config.array_time_step);
        int idx = index % time_steps;
        int next = (idx + 1) % time_steps;
        float frac = (t / config.array_time_step) - floor(t / config.array_time_step);
        // (current_time - index * config.array_time_step) / config.array_time_step;
        for(int j = 0; j < target_count; ++j){
            if (sim_step == 0){
                targetPosNow[j] = targets[j][0];
            }else{
                targetPosNow[j] =          
                {
                    targets[j][idx].x + (targets[j][next].x - targets[j][idx].x) * frac,
                    targets[j][idx].y + (targets[j][next].y - targets[j][idx].y) * frac
                };
            };
            // Calculating balistics for current targets positions
            calculateBallistics(dronePosNow.x, dronePosNow.y, targetPosNow[j].x, targetPosNow[j].y);

            dropPos[j] = {
                fire_pos_x,
                fire_pos_y
            };
            
            if(sim_step == 0){
                params[j].bearing = calculateBearing(config.startPos.x, config.startPos.y, dropPos[j].x, dropPos[j].y);
                params[j].delta_angle = getDeltaAngle(params[j].bearing, config.initial_dir);
                params[j].drop_dist = sqrt(pow((dropPos[j].x - config.startPos.x),2) + pow((dropPos[j].y - config.startPos.y),2));
                params[j].total_time = getTotalTime(params[j].delta_angle, params[j].drop_dist, acceleration, t_acceleration);
            }else{
                int index1 = (int)floor(current_time / config.array_time_step);
                int idx1 = index1 % time_steps;
                int next1 = (idx1 + 1) % time_steps;
                targetDcoord[j] = targets[j][next1] - targets[j][idx1];
                float targetSpeedX[5]{}, targetSpeedY[5]{};
                targetSpeedX[j] = targetDcoord[j].x / config.array_time_step;
                targetSpeedY[j] = targetDcoord[j].y / config.array_time_step;
                targetPosPredicted[j] = {
                    targetPosNow[j].x + targetSpeedX[j] * (params[j].total_time),
                    targetPosNow[j].y + targetSpeedY[j] * (params[j].total_time)
                };
                // Calculating Ballistics for predicted targets positions
                calculateBallistics(dronePosNow.x, dronePosNow.y, targetPosPredicted[j].x, targetPosPredicted[j].y);

                dropPos[j] = {
                    fire_pos_x,
                    fire_pos_y
                };

                params[j].bearing = calculateBearing(dronePosNow.x, dronePosNow.y, dropPos[j].x, dropPos[j].y);
                params[j].delta_angle = getDeltaAngle(params[j].bearing, current_direction);
                params[j].drop_dist = sqrt(pow((dropPos[j].x - dronePosNow.x),2) + pow((dropPos[j].y - dronePosNow.y),2));
                params[j].total_time = getTotalTime(params[j].delta_angle, params[j].drop_dist, acceleration, t_acceleration);
            };
        };
        // Updating the Preferred Parameters array
        float min_time = FLT_MAX;
        for (int i = 0; i < target_count; ++i){
            if(params[i].total_time  < min_time){
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
            };
        };
    
        steps[sim_step] = {
            dronePosNow,
            current_direction,
            state,
            prefParameters.target_pref
            };

        // Initializing Drone position change
        dronePosChange(prefParameters.delta_angle_pref, current_direction, prefParameters.bearing_pref, acceleration);

        DEBUG("Current time is " << current_time);
        DEBUG("Number of simulation steps = " << sim_step +1 << "Current drone coordinates are " << dronePosNow.x << ", " << dronePosNow.y);
        DEBUG("Current drone direction is " << current_direction);
        DEBUG("Preferred target bearing is: " << prefParameters.bearing_pref);
        DEBUG("Current drone speed is " << current_speed);
        DEBUG("Prefered drop point coordinates are: " << prefParameters.dropPosPref.x << ", " << prefParameters.dropPosPref.y);
        DEBUG("Preferred target index is " << prefParameters.target_pref);
        DEBUG("Current distance to preferred drop point is " << prefParameters.drop_dist_pref);
        DEBUG("Current distance to predicted target point is " << prefParameters.dist_target_predicted);

        Coord bombLand = {
            dronePosNow.x + h * cos(current_direction),
            dronePosNow.y + h * sin(current_direction)
        };
        delta_target_bomb = sqrt(pow((bombLand.x - prefParameters.targetPredictedPos.x),2)+pow((bombLand.y - prefParameters.targetPredictedPos.y),2));
        
        // Outputting simulation.json
        json step;
        out["totalSteps"] = sim_step + 1;
        step["position"] = {{"x", steps[sim_step].pos.x}, {"y", steps[sim_step].pos.y}};
        step["direction"] = steps[sim_step].direction;
        step["state"] = steps[sim_step].state;
        step["targetIndex"] = steps[sim_step].target_idx;
        step["dropPoint"] = {{"x", prefParameters.dropPosPref.x}, {"y", prefParameters.dropPosPref.y}};
        step["aimPoint"] = {{"x", bombLand.x}, {"y", bombLand.y}};
        step["predictedTarget"] = {{"x", prefParameters.targetPredictedPos.x}, {"y", prefParameters.targetPredictedPos.y}};  // â† moved down
        out["steps"].push_back(step);
        fout.seekp(0);
        fout << out.dump(2);
        fout.flush();

        // Check for possibility of bomb release
        if((fabs(current_direction - prefParameters.bearing_pref) < 1e-3) && (current_speed == config.attack_speed) && (prefParameters.drop_dist_pref <= config.hit_radius) && (prefParameters.dist_target_predicted <= h + config.hit_radius) && (delta_target_bomb <= config.hit_radius)){
            
            LOG("BOMB AWAY! Simulation complete. Steps: " << sim_step + 1);
            LOG("Bomb flight distance is " << h);
            LOG("Bomb flight time is " << t);
            LOG("Expected target coordinates are " << prefParameters.targetPredictedPos.x << ", " << prefParameters.targetPredictedPos.y);
            LOG("Expected bomb land coordinates are " << bombLand.x << ", " << bombLand.y);
            LOG("Delta between predicted target position and bomb land position is " << delta_target_bomb);

            // CLEANUP memory
            for (int i = 0; i < target_count; i++){
                delete[] (targets[i]);
                targets[i] = nullptr;
            };
            delete[] targets;
            targets = nullptr;
            delete[] targetPosNow;
            targetPosNow = nullptr;
            delete[] dropPos;
            dropPos = nullptr;
            delete[] targetDcoord;
            targetDcoord = nullptr;
            delete[] targetPosPredicted;
            targetPosPredicted = nullptr;
            delete[] params;
            params = nullptr;
            delete[] arsenal;
            arsenal = nullptr;
            exit(EXIT_SUCCESS);
        };
    current_time += config.sim_time_step;
    sim_step += 1;
    };
return 0;
}