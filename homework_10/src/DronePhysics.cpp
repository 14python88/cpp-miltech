#include "interfaces/IDroneState.h"
#include "Structs.h"
#include <DronePhysics.h>


void DronePhysics::changeState(PrefParameters& prefParameters) {
    auto next = this->state->execute(this->ctx, prefParameters);
    if (next) {
        this->state = std::move(next);
    }
}
