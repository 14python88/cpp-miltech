#include <interfaces/IDroneState.h>
#include <states/DroneStates.h>
#include <Structs.h>

#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cctype>
#include <cfloat>
#include <memory>


    std::unique_ptr<IDroneState> StateMoving::execute(DroneConfig& config, DroneTelemetry ctx, PrefParameters& prefParams, float& acceleration, float& t_acceleration) {
        bool angle_small = fabs(prefParams.delta_angle_pref) <= config.turn_threshold;
        // bool speed_max = ctx.current_speed >= ctx.config.attack_speed;

        ctx.current_direction = prefParams.bearing_pref;
        ctx.current_speed = config.attack_speed;
        ctx.current_pos.x += ctx.current_speed * config.sim_time_step * cos(ctx.current_direction);
        ctx.current_pos.y += ctx.current_speed * config.sim_time_step * sin(ctx.current_direction);

        if(angle_small){
            return nullptr;
        };
        return std::make_unique<StateDecelerating>();
    };

    std::string StateMoving::name() const {
        return "Moving";
    };
