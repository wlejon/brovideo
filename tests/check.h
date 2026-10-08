#pragma once
// Minimal test harness. Checks never abort and are never compiled out: every
// failure is printed with its location and counted, and main() returns
// non-zero when any failed, so Release builds test exactly like Debug ones.
// A watchdog turns a hang into a reported failure naming the phase.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>

namespace check {

inline std::atomic<int> g_failures{0};
inline std::atomic<int> g_checks{0};
inline std::mutex g_phase_m;
inline std::string g_phase = "start";

template <class T>
std::string show(const T& v) {
    if constexpr (std::is_same_v<T, bool>) return v ? "true" : "false";
    else if constexpr (std::is_enum_v<T>) return std::to_string(static_cast<long long>(v));
    else if constexpr (std::is_arithmetic_v<T>) return std::to_string(v);
    else if constexpr (std::is_convertible_v<T, std::string>) return "\"" + std::string(v) + "\"";
    else return "<value>";
}

inline void fail(const char* file, int line, const std::string& what) {
    ++g_failures;
    std::lock_guard<std::mutex> lk(g_phase_m);
    std::printf("FAIL %s:%d [%s]: %s\n", file, line, g_phase.c_str(), what.c_str());
    std::fflush(stdout);
}

template <class A, class B>
void eq(const A& a, const B& b, const char* ea, const char* eb, const char* file, int line) {
    ++g_checks;
    if (!(a == b)) fail(file, line, std::string(ea) + " == " + eb + "\n     got  " + show(a) + "\n     want " + show(b));
}

inline void phase(const std::string& name) {
    {
        std::lock_guard<std::mutex> lk(g_phase_m);
        g_phase = name;
    }
    std::printf("-- %s\n", name.c_str());
    std::fflush(stdout);
}

// Kill the process with a failure if it is still running after `seconds`.
inline void watchdog(int seconds) {
    std::thread([seconds] {
        std::this_thread::sleep_for(std::chrono::seconds(seconds));
        {
            std::lock_guard<std::mutex> lk(g_phase_m);
            std::printf("FAIL watchdog: still running after %d s in phase [%s]\n", seconds, g_phase.c_str());
        }
        std::fflush(stdout);
        std::_Exit(3);
    }).detach();
}

// Poll `pred` until it holds or `timeout_ms` passes.
template <class P>
bool wait_for(P pred, int timeout_ms = 5000) {
    const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (!pred()) {
        if (std::chrono::steady_clock::now() >= until) return pred();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return true;
}

inline int finish() {
    std::printf("%d checks, %d failures\n", g_checks.load(), g_failures.load());
    return g_failures.load() ? 1 : 0;
}

}  // namespace check

#define CHECK(c)                                                       \
    do {                                                               \
        ++check::g_checks;                                             \
        if (!(c)) check::fail(__FILE__, __LINE__, "CHECK(" #c ")");    \
    } while (0)
#define CHECK_EQ(a, b) check::eq((a), (b), #a, #b, __FILE__, __LINE__)
#define WAIT(c, ms) CHECK(check::wait_for([&] { return bool(c); }, (ms)))
