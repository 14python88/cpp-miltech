#include <interfaces/IDroneState.h>
#include <states/DroneStates.h>
#include <Structs.h>

#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cctype>
#include <cfloat>
#include <memory>

    std::unique_ptr<IDroneState> StateDecelerating::execute(DroneContext& ctx, PrefParameters& prefParams) {
        bool angle_large = fabs(prefParams.delta_angle_pref) > ctx.config.turn_threshold;
        bool speed_max = ctx.current_speed >= ctx.config.attack_speed;\
        bool speed_between = ctx.current_speed > 0.0f && ctx.current_speed < ctx.config.attack_speed;

        ctx.current_speed -= ctx.acceleration * ctx.config.sim_time_step;
        ctx.current_speed = std::max(0.0f, ctx.current_speed);
        ctx.dronePosNow.x += ctx.current_speed * ctx.config.sim_time_step * std::cos(ctx.current_direction);
        ctx.dronePosNow.y += ctx.current_speed * ctx.config.sim_time_step * std::sin(ctx.current_direction);

        if(angle_large && speed_max){
            return nullptr;
        }else if(angle_large && speed_between){
            return nullptr;
        };
        return std::make_unique<StateStopped>();
    };

    std::string StateDecelerating::name() const {
        return "Decelerating";
    };
