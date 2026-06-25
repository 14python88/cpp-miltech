#include "interfaces/IDroneState.h"
#include "Structs.h"
#include <DronePhysics.h>

DronePhysics::DronePhysics(DroneConfig config, Coord pos, float speed, float direction, float time){
    this->telemetry.current_pos = pos;
    this->telemetry.current_speed = speed;
    this->telemetry.current_direction = direction;
    this->telemetry.timeSecSinceStart = time;
    this->acceleration = config.attack_speed * config.attack_speed / (2 * config.acceleration_path);
    this->t_acceleration = (2 * config.acceleration_path) / config.attack_speed;
};

void DronePhysics::changeState(DroneConfig config, PrefParameters prefParameters) {
    auto next = this->state->execute(config, this->telemetry, prefParameters, this->acceleration, this->t_acceleration);
    if (next) {
        this->state = std::move(next);
    }
}
