#pragma once
#include "Structs.h"
#include "interfaces/IDroneState.h"
#include "PhysicsStructs.h"

#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <queue>
#include <string>

class DronePhysics {
    void workerLoop();

    DroneConfig config;
    float physicsTimeStep;

    std::unique_ptr<IDroneState> state;
    DroneTelemetry telemetry {
        {0, 0},
        0.0f,
        0.0f,
        0.0f
    };
    float acceleration;
    float t_acceleration;
    PrefParameters current_pref {};   // Last received parameters

    // Incoming command queue
    struct Command {
        std::unique_ptr<IDroneState> newState;
        PrefParameters prefParameters;
    };
    mutable std::mutex queue_mutex;
    std::queue<Command> state_queue;

    // Telemetry lock 
    mutable std::mutex telemetry_mutex;

    // Start gate 
    std::mutex start_mutex;
    std::condition_variable start_cv;
    bool start_signaled = false;

    // Thread state
    std::atomic<bool> stop_flag {false};
    std::atomic<bool> thread_ready {false};
    std::thread worker_thread;

    public:
        DronePhysics(DroneConfig config, Coord pos, float speed, float direction, float time, float physicsTimeStep);
        ~DronePhysics();

        bool isThreadReady() const;
        void start();
        void stop();

        void pushCommand(std::unique_ptr<IDroneState> newState, PrefParameters prefParameters);

        DroneTelemetry getDroneTelemetry() const;
        std::string getStateName() const;
};