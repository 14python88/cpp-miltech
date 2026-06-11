#include <interfaces/IDroneState.h>
#include <states/DroneStates.h>
#include <Structs.h>

#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cctype>
#include <cfloat>
#include <memory>

    std::unique_ptr<IDroneState> StateTurning::execute(DroneContext& ctx, PrefParameters& prefParams) {
        bool angle_large = fabs(prefParams.delta_angle_pref) > ctx.config.turn_threshold;
        bool speed_zero = ctx.current_speed == 0.0f;

        if(angle_large && speed_zero && prefParams.delta_angle_pref < 0){
            ctx.current_direction -= ctx.config.angular_speed * ctx.config.sim_time_step;
            return nullptr;
        }else if(angle_large && speed_zero && prefParams.delta_angle_pref > 0){
            ctx.current_direction += ctx.config.angular_speed * ctx.config.sim_time_step;
            return nullptr;
        };
        
        // if(angle_large && speed_zero){
        //     return nullptr;
        // };
        
        return std::make_unique<StateAccelerating>();
    };

    std::string StateTurning::name() const {
        return "Turning";
    };