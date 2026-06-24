    #pragma once

    #include "Structs.h"
    
    struct DroneTelemetry {
        Coord current_pos;
        float current_speed;
        float current_direction;
        float timeSecSinceStart;
    };
