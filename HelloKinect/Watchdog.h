#pragma once
// Watchdog.h - Internal watchdog for auto-restart on crashes/hangs

#include <Windows.h>
#include <atomic>
#include <chrono>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>
#include <functional>

 // Forward declaration for thread function
DWORD WINAPI WatchdogThreadProc(LPVOID lpParam);

class Watchdog {
    // Allow thread function to call private monitorLoop()
    friend DWORD WINAPI WatchdogThreadProc(LPVOID lpParam);

public:
    // Initialize with CLI arguments for restart preservation
    Watchdog(int argc, char* argv[],
             const std::string& crashLogPath = "C:\\Temp\\tempCG\\WatchdogCrash.log",
             int timeoutSeconds = 20,
             int cooldownSeconds = 60,
             int maxRestartsBeforeCooldown = 3);

    ~Watchdog();

    // Register a worker thread for monitoring (call before starting thread)
    void registerWorker(size_t workerIndex);

    // Worker threads call this periodically to signal they are alive
    static void heartbeat(size_t workerIndex);

    // Register a callback that will be invoked when watchdog detects workerIndex hung.
    // The callback is executed on the watchdog thread (keep it short / thread-safe).
    void registerRestartHandler(size_t workerIndex, std::function<void(size_t)> handler);

    // Start the watchdog monitoring thread
    void start();

    // Signal watchdog to stop (for graceful shutdown)
    void stop();

    // Check if restart was requested for full-process restart (legacy behavior)
    bool shouldRestart() const;

private:
    // Monitoring thread function
    void monitorLoop();

    // Restart the application or notify per-worker handler
    void triggerRestart(const std::string& reason, size_t workerIndex);

    // Log crash event to file
    void logCrashEvent(const std::string& reason);

    // Stored CLI arguments for restart
    std::vector<std::string> m_cliArgs;
    std::string m_executablePath;

    // Crash log path
    std::string m_crashLogPath;
    std::mutex m_logMutex;

    // Timeout and cooldown settings
    int m_timeoutSeconds;
    int m_cooldownSeconds;
    int m_maxRestartsBeforeCooldown;

    // Restart tracking
    std::chrono::steady_clock::time_point m_lastRestartTime;
    int m_recentRestartCount;

    // Worker heartbeat tracking (static for easy access from worker threads)
    // Using raw array + atomic pointer to avoid std::vector<atomic> copy issues
    static constexpr size_t MAX_WORKERS = 16;
    static std::atomic<int64_t> s_heartbeats[MAX_WORKERS];
    static std::atomic<size_t> s_workerCount;

    // Per-worker restart handlers
    std::mutex m_handlerMutex;
    std::function<void(size_t)> m_restartHandlers[MAX_WORKERS];

    // Control flags
    std::atomic<bool> m_running;
    std::atomic<bool> m_restartRequested;

    // Monitoring thread handle
    HANDLE m_monitorThread;
};
