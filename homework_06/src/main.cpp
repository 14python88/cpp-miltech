#include "ballistics.hpp"

#include <iostream>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: ballistics <input_path>\n";
        return 1;
    };

    BallisticsInput input = readInput(argv[1]);

    Coord drop_point = calculateBallistics(const float& mass, const float& drag, const float& lift, input.zd, input.attackSpeed, input.xd, input.yd, input.target_x, input.target_y, input.accelerationPath);

    const Summary summary = summarize(frames, frame_count);
    print_summary(summary);

    return 0;
}