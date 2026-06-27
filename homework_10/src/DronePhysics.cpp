#include "interfaces/IDroneState.h"
#include "Structs.h"
#include <DronePhysics.h>
#include "states/DroneStates.h"

#include <chrono>

DronePhysics::DronePhysics(DroneConfig config, Coord pos, float speed, float direction, float time, float physicsTimeStep) 
: config(config), physicsTimeStep(physicsTimeStep), state(std::make_unique<StateStopped>())
{
    this->telemetry.current_pos = pos;
    this->telemetry.current_speed = speed;
    this->telemetry.current_direction = direction;
    this->telemetry.timeSecSinceStart = time;

    this->acceleration = config.attack_speed * config.attack_speed / (2.0f * config.acceleration_path);
    this->t_acceleration = (2.0f * config.acceleration_path) / config.attack_speed;

    worker_thread = std::thread(&DronePhysics::workerLoop, this);
}

DronePhysics::~DronePhysics()
{
    stop();
}

// Thread lifecycle

bool DronePhysics::isThreadReady() const
{
    return thread_ready.load(std::memory_order_acquire);
}

void DronePhysics::start()
{
    {
        std::lock_guard<std::mutex> lk(start_mutex);
        start_signaled = true;
    }
    start_cv.notify_one();
}

void DronePhysics::stop()
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

// Commands
void DronePhysics::pushCommand(std::unique_ptr<IDroneState> newState, PrefParameters prefParameters)
{
    std::lock_guard<std::mutex> lk(queue_mutex);
    state_queue.push({ std::move(newState), std::move(prefParameters) });
}

// Getters
DroneTelemetry DronePhysics::getDroneTelemetry() const
{
    std::lock_guard<std::mutex> lk(telemetry_mutex);
    return telemetry;
}

std::string DronePhysics::getStateName() const
{
    std::lock_guard<std::mutex> lk(telemetry_mutex);
    return state->name();
}

// Private: worker thread

void DronePhysics::workerLoop()
{
    thread_ready.store(true, std::memory_order_release);

    // Waiting for start()
    {
        std::unique_lock<std::mutex> lk(start_mutex);
        start_cv.wait(lk, [this] { return start_signaled; });
    }

    if (stop_flag.load(std::memory_order_acquire)) {
        return;
    }

    // Main body
    using Clock   = std::chrono::steady_clock;
    using Seconds = std::chrono::duration<double>;

    auto next_tick = Clock::now() + Seconds(physicsTimeStep);

    while (!stop_flag.load(std::memory_order_acquire)) {

        std::this_thread::sleep_until(next_tick);
        next_tick += Seconds(physicsTimeStep);

        if (stop_flag.load(std::memory_order_acquire)) {
            break;
        }

        std::lock_guard<std::mutex> lk(telemetry_mutex);

        // Intefrating last command from MissionProcessor
        {
            std::lock_guard<std::mutex> qlk(queue_mutex);
            while (!state_queue.empty()) {
                Command cmd = std::move(state_queue.front());
                state_queue.pop();
                if (cmd.newState){
                    state = std::move(cmd.newState);
                }
                current_pref = std::move(cmd.prefParameters);
            }
        }

        // Return updated drone telemetry and state
        IDroneState::ExecuteResult result = state->execute(config, telemetry, current_pref, acceleration, t_acceleration);
        telemetry = std::move(result.telemetry);
        
        if (result.nextState)
            state = std::move(result.nextState);
    }
}
