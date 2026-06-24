#include "interfaces/IDroneState.h"
#include "Structs.h"
#include <DronePhysics.h>

DronePhysics::DronePhysics(DroneConfig& config, Coord pos, float speed, float direction, float time, float acceleration, float t_acceleration){
    this->config = config;
    this->telemetry.current_pos = pos;
    this->telemetry.current_speed = speed;
    this->telemetry.current_direction = direction;
    this->telemetry.timeSecSinceStart = time;
    this->acceleration = acceleration;
    this->t_acceleration = t_acceleration;
};

void DronePhysics::changeState(PrefParameters& prefParameters) {
    auto next = this->state->execute(this->config, this->telemetry, prefParameters, this->acceleration, this->t_acceleration);
    if (next) {
        this->state = std::move(next);
    }
}
