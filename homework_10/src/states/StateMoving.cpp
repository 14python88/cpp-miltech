#include <interfaces/IDroneState.h>
#include <states/DroneStates.h>
#include <Structs.h>

#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cctype>
#include <cfloat>
#include <memory>


    std::unique_ptr<IDroneState> StateMoving::execute(DroneContext& ctx, PrefParameters& prefParams) {
        bool angle_small = fabs(prefParams.delta_angle_pref) <= ctx.config.turn_threshold;
        // bool speed_max = ctx.current_speed >= ctx.config.attack_speed;

        ctx.current_direction = prefParams.bearing_pref;
        ctx.current_speed = ctx.config.attack_speed;
        ctx.dronePosNow.x += ctx.current_speed * ctx.config.sim_time_step * cos(ctx.current_direction);
        ctx.dronePosNow.y += ctx.current_speed * ctx.config.sim_time_step * sin(ctx.current_direction);

        if(angle_small){
            return nullptr;
        };
        return std::make_unique<StateDecelerating>();
    };

    std::string StateMoving::name() const {
        return "Moving";
    };
