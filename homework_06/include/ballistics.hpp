#pragma once

#include <string>

// Organizes input parameters
struct BallisticsInput {
    float xd;
    float yd;
    float zd;
    float target_x;
    float target_y;
    float attackSpeed;
    float accelerationPath;
    std::string ammo_name;
};

// Organizes coordinates
struct Coord {
    float x;
    float y;
};


// Reads input from data file and returns in as a BallisticsInput{} struct
BallisticsInput readInput(const char* path);

// Calculates bomb drop coordinates and returns them as a Coord{} struct
Coord calculateBalistics(const float& mass, const float& drag, const float&  lift, const float& zd, const float& attackSpeed, const float& xd, const float& yd, const float& target_x, const float& target_y, const float& accelerationPath);

// Outputs the result into the console
void PrintResult(const float& fireX, const float& fireY);