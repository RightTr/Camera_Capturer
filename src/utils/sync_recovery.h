#pragma once

#include <chrono>
#include <cstdint>

// Caller serializes this state with the synchronization/publication mutex.
struct SyncRecovery {
    using Clock = std::chrono::steady_clock;
    enum class State { preroll, running, recovering };
    State state = State::preroll;
    std::uint64_t count = 0;
    Clock::time_point started{};
    Clock::time_point next_warning{};
    bool initialized = false;

    bool active() const { return state == State::running; }
    void restart(Clock::time_point now)
    {
        if (!initialized || active()) {
            started = now;
            next_warning = now + std::chrono::seconds(10);
        }
        if (active()) { ++count; state = State::recovering; }
        initialized = true;
    }
    void resume() { state = State::running; }
    bool warning_due(Clock::time_point now)
    {
        if (!initialized || active() || now < next_warning) return false;
        next_warning = now + std::chrono::seconds(5);
        return true;
    }
    std::int64_t elapsed_ms(Clock::time_point now) const
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(now - started).count();
    }
};
