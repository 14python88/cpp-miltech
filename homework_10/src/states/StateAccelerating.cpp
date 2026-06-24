#include <interfaces/IDroneState.h>
#include <states/DroneStates.h>
#include <Structs.h>

#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cctype>
#include <cfloat>
#include <memory>


    std::unique_ptr<IDroneState> StateAccelerating::execute(DroneConfig& config, DroneTelemetry ctx, PrefParameters& prefParams, float& acceleration, float& t_acceleration) {
        bool angle_small = fabs(prefParams.delta_angle_pref) <= config.turn_threshold;
        bool speed_between = ctx.current_speed > 0.0f && ctx.current_speed < config.attack_speed;
        bool speed_zero = ctx.current_speed == 0.0f;

        ctx.current_direction = prefParams.bearing_pref;
        ctx.current_speed += acceleration * config.sim_time_step;
        ctx.current_speed = std::min(ctx.current_speed, config.attack_speed);
        ctx.current_pos.x += ctx.current_speed * config.sim_time_step * cos(ctx.current_direction);
        ctx.current_pos.y += ctx.current_speed * config.sim_time_step * sin(ctx.current_direction);
        if(angle_small && speed_between){
            return nullptr;
        }else if(angle_small && speed_zero){
            return nullptr;
        };
        return std::make_unique<StateMoving>();
    };

    std::string StateAccelerating::name() const {
        return "Accelerating";
    };
