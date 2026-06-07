#include <interfaces/IBallisticSolver.h>
#include <AnalyticalSolver.h>

#include <iostream>

#define USE_MATH_DEFINES

        ResultConst AnalyticalSolver::solve(const DroneConfig config, const Ammo& ammo) {
            ResultConst solverResult;
            float a = ammo.drag *9.81* ammo.mass - 2 * ammo.drag * ammo.drag * ammo.lift * config.attack_speed;
            float b = (-3) *9.81* ammo.mass * ammo.mass + 3 * ammo.drag * ammo.lift * ammo.mass * config.attack_speed;
            float c = 6 * ammo.mass * ammo.mass * config.altitude;
            float p = -(b * b) / (3 * a * a);
            float q = (2 * b * b * b) / (27 * a * a * a) + c / a;
            float acos_arg = ((3 * q) / (2 * p)) * sqrt((-3) / p);

            if(!(acos_arg >= -1) || !(acos_arg <= 1)){
                std::cout << "acos argument out of range (-1,1)!" << '\n';
                exit(EXIT_FAILURE);
            }

            float phi = acos(acos_arg);
            float t = 2 * sqrt((-p)/3) * cos((phi + 4*M_PI)/3) - b / (3 * a);
            float h = config.attack_speed*t - (t * t * ammo.drag * config.attack_speed) / (2 * ammo.mass) +
            pow(t,3) * (6 * ammo.drag *9.81* ammo.lift * ammo.mass - 6 * ammo.drag * ammo.drag * config.attack_speed * (pow(ammo.lift,2)-1)) / (36 * ammo.mass * ammo.mass) +
            pow(t,4) * ((-6) * ammo.drag * ammo.drag *9.81* ammo.lift * ammo.mass * (1 + pow(ammo.lift,2) + pow(ammo.lift,4)) + 3 * pow(ammo.drag,3) * pow(ammo.lift,2) * 
            config.attack_speed * (1 + ammo.lift * ammo.lift) + 6 * pow(ammo.drag,3) * pow(ammo.lift,4) * config.attack_speed * (1 + ammo.lift * ammo.lift)) / (36 * ammo.mass * ammo.mass * ammo.mass * pow((1 + pow(ammo.lift,2)),2)) +
            pow(t,5) * (3 * pow(ammo.drag,3) *9.81* ammo.mass * pow(ammo.lift,3) - 3*pow(ammo.drag,4) * ammo.lift * ammo.lift * config.attack_speed * (1 + ammo.lift * ammo.lift)) / 
            (36 * (1 + ammo.lift * ammo.lift) * pow(ammo.mass,4));

            if(t <= 0 || h <= 0){
                std::cout << "Inadequate calculation value(s): t, h!" << '\n';
                exit(EXIT_FAILURE);
            };

            solverResult.t = t;
            solverResult.h = h;
            return solverResult;

        };

        Coord AnalyticalSolver::calculateBallistics(const float& acceleration_path, const Coord& dronePos, const Coord& targetPos, const float& h) {
            Coord firePos;
            float D = sqrt(pow((targetPos.x - dronePos.x),2) + pow((targetPos.x - dronePos.y),2));
            if(D <= 0){
                std::cout << "Inadequate calculation value D!" << '\n';
                exit(EXIT_FAILURE);
            }
            float ratio = (D - h) / D;

            if((h + acceleration_path) > D){
                float drone_x_mid = targetPos.x - (targetPos.x - dronePos.x) * ((h + acceleration_path) / D);
                float drone_y_mid = targetPos.y - (targetPos.y - dronePos.y) * ((h + acceleration_path) / D);
                float D_mid = sqrt(pow((targetPos.x - drone_x_mid),2) + pow((targetPos.y - drone_y_mid),2));
                float ratio_mid = (D_mid - h) / D_mid;

                firePos = {
                    .x = drone_x_mid + (targetPos.x - drone_x_mid) * ratio_mid,
                    .y = drone_y_mid + (targetPos.y - drone_y_mid) * ratio_mid
                };
            }else{
                firePos = {
                    .x = dronePos.x + (targetPos.x - dronePos.x) * ratio,
                    .y = dronePos.y + (targetPos.y - dronePos.y) * ratio
                };
            }
            return firePos;
        }