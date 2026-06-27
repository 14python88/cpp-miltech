#include <ThreadSafeTargetProvider.h>
#include <interfaces/ITargetProvider.h>
#include <json.hpp>
#include <Structs.h>

#include <fstream>
#include <stdexcept>
#include <chrono>

using json = nlohmann::json;

ThreadSafeTargetProvider::ThreadSafeTargetProvider(const std::string& path, float array_time_step) : array_time_step(array_time_step)
{
    // Parsing target data
    std::ifstream targets_json(path);
    if (!targets_json.is_open())
        throw std::runtime_error("ThreadSafeTargetProvider: cannot open " + path);

    json j_targets = json::parse(targets_json);
    targets_json.close();

    this->target_count = j_targets["targetCount"];
    this->time_steps = j_targets["timeSteps"];

    this->targets = std::vector<std::vector<Coord>>(target_count, std::vector<Coord>(time_steps));

    for (int i = 0; i < target_count; ++i) {
        const auto& positions = j_targets["targets"][i]["positions"];
        for (int j = 0; j < time_steps; ++j) {
            this->targets[i][j].x = positions[j]["x"];
            this->targets[i][j].y = positions[j]["y"];
        }
    }

    // Initializing node 0 snapshot
    node_index.assign(target_count, 0);
    targetNow.resize(target_count);

    for (int i = 0; i < target_count; ++i) {
        int next = 1 % time_steps;

        targetNow[i].pos = targets[i][0];

        Coord delta = targets[i][next] - targets[i][0];
        targetNow[i].velocity = { delta.x / array_time_step,
                                  delta.y / array_time_step };
    }

    // Thread starts immediately but is blocked at start_cv until start() is invoked
    worker_thread = std::thread(&ThreadSafeTargetProvider::workerLoop, this);
}

ThreadSafeTargetProvider::~ThreadSafeTargetProvider() {
    stop();
}

// thread lifecycle

bool ThreadSafeTargetProvider::isThreadReady() const
{
    return thread_ready.load(std::memory_order_acquire);
}

void ThreadSafeTargetProvider::start()
{
    {
        std::lock_guard<std::mutex> lk(start_mutex);
        start_signaled = true;
    }
    start_cv.notify_one();
}

void ThreadSafeTargetProvider::stop()
{
    stop_flag.store(true, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lk(start_mutex);
        start_signaled = true;
    }
    start_cv.notify_one();

    if (worker_thread.joinable())
        worker_thread.join();
}

// Data access

std::vector<Target> ThreadSafeTargetProvider::getTargetNow() const
{
    std::lock_guard<std::mutex> lk(targetNow_mutex);
    return targetNow;
}

std::vector<Coord> ThreadSafeTargetProvider::getTargetPredicted() const
{
    std::lock_guard<std::mutex> lk(targetNow_mutex);
    return targetPosPredicted;
}

void ThreadSafeTargetProvider::extrapolateTargets(const std::vector<Params>& params)
{
    // Getting current targets under mutex
    std::vector<Target> snapshot;
    {
        std::lock_guard<std::mutex> lk(targetNow_mutex);
        snapshot = targetNow;
    }

    std::vector<Coord> predicted(target_count);
    for (int j = 0; j < target_count; ++j) {
        predicted[j] = {
            snapshot[j].pos.x + snapshot[j].velocity.vx * params[j].total_time,
            snapshot[j].pos.y + snapshot[j].velocity.vy * params[j].total_time
        };
    }

    {
        std::lock_guard<std::mutex> lk(targetNow_mutex);
        targetPosPredicted = std::move(predicted);
    }
}

int ThreadSafeTargetProvider::getTargetCount() const
{
    return target_count;
}

// Worker thread

void ThreadSafeTargetProvider::workerLoop()
{
    // Thread is ready and waiting for start()
    thread_ready.store(true, std::memory_order_release);

    // Block until start()
    {
        std::unique_lock<std::mutex> lk(start_mutex);
        start_cv.wait(lk, [this] { return start_signaled; });
    }

    // Exit if stop() comes before start()
    if (stop_flag.load(std::memory_order_acquire)){
        return;
    }

    // Main body
    using Clock   = std::chrono::steady_clock;
    using Seconds = std::chrono::duration<double>;

    auto next_tick = Clock::now() + Seconds(array_time_step);

    while (!stop_flag.load(std::memory_order_acquire)) {

        std::this_thread::sleep_until(next_tick);
        next_tick += Seconds(array_time_step);

        if (stop_flag.load(std::memory_order_acquire))
            break;

        // Updating snapshot under mutex
        std::lock_guard<std::mutex> lk(targetNow_mutex);

        for (int i = 0; i < target_count; ++i) {

            node_index[i] = (node_index[i] + 1) % time_steps;
            int idx = node_index[i];
            int next = (idx + 1) % time_steps;

            targetNow[i].pos = targets[i][idx];

            Coord delta = targets[i][next] - targets[i][idx];
            targetNow[i].velocity = 
                { delta.x / array_time_step,
                  delta.y / array_time_step
                };
        }
    }
}
