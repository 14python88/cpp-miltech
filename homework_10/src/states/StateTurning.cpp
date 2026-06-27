#include <interfaces/IDroneState.h>
#include <states/DroneStates.h>
#include <Structs.h>
#include <cmath>
#include <memory>

IDroneState::ExecuteResult StateTurning::execute(
    DroneConfig& config, DroneTelemetry ctx,
    PrefParameters& prefParams,
    float& acceleration,
    float& t_acceleration)
{
    bool angle_large = fabs(prefParams.delta_angle_pref) >  config.turn_threshold;
    bool angle_small = fabs(prefParams.delta_angle_pref) <= config.turn_threshold;
    bool speed_zero = ctx.current_speed == 0.0f;
    bool speed_between = ctx.current_speed > 0.0f && ctx.current_speed < config.attack_speed;

    if (angle_large && speed_zero && prefParams.delta_angle_pref < 0) {
        ctx.current_direction -= config.angular_speed * config.sim_time_step;
        return IDroneState::ExecuteResult{ nullptr, ctx };
    } else if (angle_large && speed_zero && prefParams.delta_angle_pref > 0) {
        ctx.current_direction += config.angular_speed * config.sim_time_step;
        return IDroneState::ExecuteResult{ nullptr, ctx };
    }

    if (angle_small && (speed_zero || speed_between)) {
        return IDroneState::ExecuteResult{ std::make_unique<StateAccelerating>(), ctx };
    }

    return IDroneState::ExecuteResult{ nullptr, ctx };
}

std::string StateTurning::name() const { return "Turning"; }