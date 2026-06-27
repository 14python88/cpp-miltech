#pragma once
#include <interfaces/ITargetProvider.h>
#include <Structs.h>

#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <string>

class ThreadSafeTargetProvider : public ITargetProvider {
    void workerLoop();
    float array_time_step;

    // Private target data
    int target_count;
    int time_steps;
    std::vector<std::vector<Coord>> targets;  // [target_i][node_j]
    std::vector<int> node_index; // current node for every target

    // Public snapshot under mutex
    mutable std::mutex  targetNow_mutex;
    std::vector<Target> targetNow;
    std::vector<Coord>  targetPosPredicted;

    // Start gate
    std::mutex start_mutex;
    std::condition_variable start_cv;
    bool start_signaled = false;

    // Thread state
    std::atomic<bool> stop_flag    {false};
    std::atomic<bool> thread_ready {false};
    std::thread worker_thread;

public:
    explicit ThreadSafeTargetProvider(const std::string& path, float array_time_step);
    ~ThreadSafeTargetProvider() override;

    bool isThreadReady() const override;
    void start() override;
    void stop() override;

    std::vector<Target> getTargetNow() const override;
    std::vector<Coord> getTargetPredicted() const override;
    void extrapolateTargets(const std::vector<Params>& params) override;

    int getTargetCount() const override;

};