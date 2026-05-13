#include "ballistics.hpp"

#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>
#include <string>
#include <cctype>
#include <vector>
#include <cstdlib>
#include <cfloat>

#define USE_MATH_DEFINES

BallisticsInput readInput(const char* path) {

    std::ifstream file{path};
    if (!file) {
        std::cerr << "error: failed to open input file: " << path << std::endl;
        exit(EXIT_FAILURE);
    };

    std::string input_file;
    getline(file, input_file);
    std::vector<std::string> params_list = {"xd", "yd", "zd", "target_x", "target_y", "attack_speed", "acceleration_path", "ammo_name"};
    std::vector<std::string> params;
    size_t pos = 0;
    std::string parameter;
    while ((pos = input_file.find(" ")) != std::string::npos) {
        parameter = input_file.substr(0, pos);
        params.push_back(parameter);
        input_file.erase(0, pos+1);
    };
    params.push_back(input_file);

    BallisticsInput input{
        .xd = stof(params[0]),
        .yd = stof(params[1]),
        .zd = stof(params[2]),
        .target_x = stof(params[3]),
        .target_y = stof(params[4]),
        .attack_speed = stof(params[5]),
        .acceleration_path = stof(params[6]),
        .ammo_name = params[7],
    };
    file.close();
    return input;
}

AmmoInput getAmmoInput(const char* ammo_name){
    Ammo arsenal[5] = {
        {"VOG-17", 0.35f, 0.07f, 0.0f},
        {"M67", 0.6f, 0.10f, 0.0f},
        {"RKG-3", 1.2f, 0.10f, 0.0f},
        {"GLIDING-VOG", 0.45f, 0.10f, 1.0f},
        {"GLIDING-RKG", 1.4f, 0.10f, 1.0f} 
    };

    float m = 0.0f, d = 0.0f, l = 0.0f;

    char ammo_name_cpy[15] = "";
    strcpy(ammo_name_cpy,ammo_name);
    bool found = 0;
    for(int n = 0; n < 5; ++n){
        if(strcmp(ammo_name_cpy, arsenal[n].name) == 0){
            found = 1;
            m = arsenal[n].mass;
            d = arsenal[n].drag;
            l = arsenal[n].lift;
        };
    };
    
    if(found == 0){
        std::cerr << "Unknown ammo type!" << std::endl;
        exit(EXIT_FAILURE);
    };

    AmmoInput ammo_input{
        .mass = m,
        .drag = d,
        .lift = l
    };

    return ammo_input;
}

Coord calculateBallistics(const float& mass, const float& drag, const float&  lift, const float& zd, const float& attack_speed, const float& xd, const float& yd, const float& target_x, const float& target_y, const float& acceleration_path){

    double pi = M_PI, g = 9.81;

    float a = drag * g * mass - 2 * drag * drag * lift * attack_speed;
    float b = (-3) * g * mass * mass + 3 * drag * lift * mass * attack_speed;
    float c = 6 * mass * mass * zd;
    float p = -(b * b) / (3 * a * a);
    float q = (2 * b * b * b) / (27 * a * a * a) + c / a;
    float acos_arg = ((3 * q) / (2 * p)) * sqrt((-3) / p);

    if(!(acos_arg >= -1) || !(acos_arg <= 1)){
        std::cerr << "acos argument out of range (-1,1)!" << std::endl;
        exit(EXIT_FAILURE);
    };

    float phi = acos(acos_arg);

    float t = 2 * sqrt((-p)/3) * cos((phi + 4*pi)/3) - b / (3 * a);
    float h = attack_speed*t - (pow(t,2) * drag * attack_speed) / (2 * mass) +
    pow(t,3) * (6 * drag * g * lift * mass - 6 * drag * drag * attack_speed * (pow(lift,2)-1)) / (36 * mass * mass) +
    pow(t,4) * ((-6) * drag * drag * g * lift * mass * (1 + pow(lift,2) + pow(lift,4)) + 3 * pow(drag,3) * pow(lift,2) * 
    attack_speed * (1 + lift * lift) + 6 * pow(drag,3) * pow(lift,4) * attack_speed * (1 + lift * lift)) / (36 * mass * mass * mass * pow((1 + pow(lift,2)),2)) +
    pow(t,5) * (3 * pow(drag,3) * g * mass * pow(lift,3) - 3*pow(drag,4) * lift * lift * attack_speed * (1 + lift * lift)) / 
    (36 * (1 + lift * lift) * pow(mass,4));

    if(t <= 0 || h <= 0){
        std::cerr << "Inadequate calculation value(s): t, h!" << std::endl;
        exit(EXIT_FAILURE);
    };

    float D = sqrt(pow((target_x - xd),2) + pow((target_y - yd),2));
    if(D <= 0){
        std::cerr << "Inadequate value D!" << std::endl;
        exit(EXIT_FAILURE);
    };

    float ratio = (D - h) / D;
    Coord fire_coords{};

    if((h + acceleration_path) > D){
        float xd_mid = target_x - (target_x - xd) * ((h + acceleration_path) / D);
        float yd_mid = target_y - (target_y - yd) * ((h + acceleration_path) / D);
        float D_mid = sqrt(pow((target_x - xd_mid),2) + pow((target_y - yd_mid),2));
        float ratio_mid = (D_mid - h) / D_mid;
        fire_coords = {
            .x = xd_mid + (target_x - xd_mid) * ratio_mid,
            .y = yd_mid + (target_y - yd_mid) * ratio_mid
            };
        }else{
            fire_coords = {
                .x = xd + (target_x - xd) * ratio,
                .y = yd + (target_y - yd) * ratio
            };
        };
    return fire_coords;
}

int printResult(const float& fireX, const float& fireY){
    std::cout << "Bomb drop coordinates are: x = " << fireX << ", y = " << fireY << std::endl;
    return 0;
}