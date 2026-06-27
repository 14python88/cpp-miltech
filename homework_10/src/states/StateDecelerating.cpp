#include <interfaces/IDroneState.h>
#include <states/DroneStates.h>
#include <Structs.h>
#include <cmath>
#include <memory>

IDroneState::ExecuteResult StateDecelerating::execute(
    DroneConfig& config, DroneTelemetry ctx,
    PrefParameters& prefParams,
    float& acceleration,
    float& t_acceleration)
{
    bool angle_large = fabs(prefParams.delta_angle_pref) >  config.turn_threshold;
    bool angle_small = fabs(prefParams.delta_angle_pref) <= config.turn_threshold;
    bool speed_max = ctx.current_speed >= config.attack_speed;
    bool speed_between = ctx.current_speed > 0.0f && ctx.current_speed < config.attack_speed;

    ctx.current_speed -= acceleration * config.sim_time_step;
    ctx.current_speed = std::max(0.0f, ctx.current_speed);
    ctx.current_pos.x += ctx.current_speed * config.sim_time_step * cos(ctx.current_direction);
    ctx.current_pos.y += ctx.current_speed * config.sim_time_step * sin(ctx.current_direction);

    if (angle_large && (speed_max || speed_between)) {
        return IDroneState::ExecuteResult{ nullptr, ctx };
    } else if (angle_small && speed_between) {
        return IDroneState::ExecuteResult{ std::make_unique<StateAccelerating>(), ctx };
    }

    return IDroneState::ExecuteResult{ std::make_unique<StateStopped>(), ctx };
}

std::string StateDecelerating::name() const { return "Decelerating"; }
