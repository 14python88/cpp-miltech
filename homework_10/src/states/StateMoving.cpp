#include <interfaces/IDroneState.h>
#include <states/DroneStates.h>
#include <Structs.h>
#include <cmath>
#include <memory>

IDroneState::ExecuteResult StateMoving::execute(
    DroneConfig& config, DroneTelemetry ctx,
    PrefParameters& prefParams,
    float& acceleration,
    float& t_acceleration)
{
    bool angle_small = fabs(prefParams.delta_angle_pref) <= config.turn_threshold;

    ctx.current_direction = prefParams.bearing_pref;
    ctx.current_speed = config.attack_speed;
    ctx.current_pos.x += ctx.current_speed * config.sim_time_step * cos(ctx.current_direction);
    ctx.current_pos.y += ctx.current_speed * config.sim_time_step * sin(ctx.current_direction);

    if (angle_small) {
        return IDroneState::ExecuteResult{ nullptr, ctx };
    }

    return IDroneState::ExecuteResult{ std::make_unique<StateDecelerating>(), ctx };
}

std::string StateMoving::name() const { return "Moving"; }
