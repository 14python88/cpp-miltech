#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <cstring>

#define ARGS_COUNT 5

using namespace std;

const int ticks_per_revolution = 1024;
const float wheel_radius_m = 0.3;
const float wheelbase_m = 1.0;
const float pi = M_PI;

int main(int argc, char** argv) {

    int d_fl = 0, d_fr = 0, d_bl = 0, d_br = 0;
    float d_left = 0.0f, d_right = 0.0f, distance_per_tick = 0.0f, dL = 0.0f, dR = 0.0f, d = 0.0f, dtheta = 0.0f, x = 0.0f, y = 0.0f, theta = 0.0f;

    if (argc != 2) {
        std::cerr << "usage: ugv_odometry <input_path>\n";
        return 1;
    };

    ifstream file(argv[1]);
    if(!file.is_open()){
        cout << "Cannot access input file" << endl;
        return 1;
    };

    int line_count = 0;
    string line = "";
    while (getline(file, line)) {
        line_count++;
    };
    file.clear();
    file.seekg(0);

    int rows = line_count, cols = ARGS_COUNT;

    int** data_array = new int*[rows];
    for(int i = 0; i < rows; ++i){
       data_array[i] = new int[cols];
    };
 
    for(int i = 0; i < rows; ++i) {
        for(int j = 0; j < cols; ++j) {
            file >> data_array[i][j];
        };
    };

    int* timestamp_ms = new int[line_count];
    int* fl_ticks = new int[line_count];
    int* fr_ticks = new int[line_count];
    int* bl_ticks = new int[line_count];
    int* br_ticks = new int[line_count];

    for(int j = 0; j < line_count; ++j){
        timestamp_ms[j] = data_array[j][0];
        fl_ticks[j] = data_array[j][1];
        fr_ticks[j] = data_array[j][2];
        bl_ticks[j] = data_array[j][3];
        br_ticks[j] = data_array[j][4];
        cout << timestamp_ms[j] << " " << fl_ticks[j] << " " << fr_ticks[j] << " " << bl_ticks[j] << " " << br_ticks[j] << endl;
    };

    file.close();

    for(int i = 0; i < line_count; ++i){
        if(i == 0){
            continue;
        };
        // Calculating delta pos for each wheel
        d_fl = fl_ticks[i] - fl_ticks[i-1];
        d_fr = fr_ticks[i] - fr_ticks[i-1];
        d_bl = bl_ticks[i] - bl_ticks[i-1];
        d_br = br_ticks[i] - br_ticks[i-1];
        // Synchronizing sides
        d_left  = static_cast<float>(d_fl + d_bl) / 2;
        d_right = static_cast<float>(d_fr + d_br) / 2;
        // Dist to meters
        distance_per_tick = 2 * pi * wheel_radius_m / ticks_per_revolution;
        dL = d_left  * distance_per_tick;
        dR = d_right * distance_per_tick;
        // Calculating distance and turn mag
        d = static_cast<float>(dL + dR) / 2;
        dtheta = (dR - dL) / wheelbase_m;
        // Midpoint integration
        x += d * cos(theta + dtheta / 2);
        y += d * sin(theta + dtheta / 2);
        theta += dtheta;

        cout << timestamp_ms[i] << " " << x << " " << y << " " << theta << endl;
    };

    for(int i = 0; i < rows; ++i){
        delete[] data_array[i];
    };
    delete[] data_array;
    delete[] timestamp_ms;
    delete[] fl_ticks;
    delete[] fr_ticks;
    delete[] bl_ticks;
    delete[] br_ticks;
    return 0;
}
