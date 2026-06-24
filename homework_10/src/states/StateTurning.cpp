#include <interfaces/IDroneState.h>
#include <states/DroneStates.h>
#include <Structs.h>

#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cctype>
#include <cfloat>
#include <memory>

    std::unique_ptr<IDroneState> StateTurning::execute(DroneConfig& config, DroneTelemetry ctx, PrefParameters& prefParams, float& acceleration, float& t_acceleration) {
        bool angle_large = fabs(prefParams.delta_angle_pref) > config.turn_threshold;
        bool speed_zero = ctx.current_speed == 0.0f;
        bool angle_small = fabs(prefParams.delta_angle_pref) <= config.turn_threshold;
        bool speed_between = ctx.current_speed > 0.0f && ctx.current_speed < config.attack_speed;

        if(angle_large && speed_zero && prefParams.delta_angle_pref < 0){
            ctx.current_direction -= config.angular_speed * config.sim_time_step;
            return nullptr;
        }else if(angle_large && speed_zero && prefParams.delta_angle_pref > 0){ 
            ctx.current_direction += config.angular_speed * config.sim_time_step;
            return nullptr;
        };
        
        if(angle_small && speed_between){
            return std::make_unique<StateAccelerating>();
        }else if(angle_small && speed_zero){
            return std::make_unique<StateAccelerating>();
        };
    };

    std::string StateTurning::name() const {
        return "Turning";
    };