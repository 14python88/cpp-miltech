#include <interfaces/IDroneState.h>
#include <states/DroneStates.h>
#include <Structs.h>

#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cctype>
#include <cfloat>
#include <memory>

    std::unique_ptr<IDroneState> StateStopped::execute(DroneContext& ctx, PrefParameters& prefParams) {
        bool angle_small = fabs(prefParams.delta_angle_pref) <= ctx.config.turn_threshold;
        bool angle_large = fabs(prefParams.delta_angle_pref) > ctx.config.turn_threshold;
        bool speed_zero = ctx.current_speed == 0.0f;
        if(angle_small && speed_zero){
            ctx.current_direction = prefParams.bearing_pref;
            ctx.current_speed += ctx.acceleration * ctx.config.sim_time_step;
            ctx.current_speed = std::min(ctx.current_speed, ctx.config.attack_speed);
            ctx.dronePosNow.x += ctx.current_speed * ctx.config.sim_time_step * cos(ctx.current_direction);
            ctx.dronePosNow.y += ctx.current_speed * ctx.config.sim_time_step * sin(ctx.current_direction);
            return std::make_unique<StateAccelerating>();
        }else if(angle_large && speed_zero){
            return std::make_unique<StateTurning>();
        };
    };

    std::string StateStopped::name() const {
        return "Stopped";
    };