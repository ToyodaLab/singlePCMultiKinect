#pragma once
// WorkerController.h - Supervises a single device worker and restarts it on request.

#include <thread>
#include <atomic>
#include <memory>
#include <functional>
#include <chrono>
#include <fstream>
#include <mutex>
#include <condition_variable>
#include <string>

#include "ProcessDevice.h"
#include "Watchdog.h"

class WorkerController {
public:
    WorkerController(std::shared_ptr<DeviceContext> ctx,
                     int deviceIndex,
                     std::ofstream& outputFile,
                     std::mutex& outputFileMutex,
                     bool recordTimestamps,
                     int stressHangAfterFrames,
                     Watchdog* watchdog);

    ~WorkerController();

    // Start supervising (creates supervisor thread)
    void start();

    // Request supervisor to stop and join threads
    void stopAndJoin();

private:
    void superviseLoop();
    void spawnWorker();

    // Attempt to reopen the physical device that matches the context serial.
    // Returns true if reopened and installed into device context.
    bool performReopenBySerial(const std::string& serial, bool forceReplace = false);

    // Helper logging (timestamp + thread id)
    std::string nowStamp();

    std::shared_ptr<DeviceContext> m_deviceCtx;
    int m_deviceIndex;
    std::ofstream& m_outputFile;
    std::mutex& m_outputFileMutex;
    bool m_recordTimestamps;
    int m_stressHangAfterFrames;
    Watchdog* m_watchdog;

    std::thread m_supervisorThread;
    std::thread m_workerThread;

    // Supervisor control
    std::atomic<bool> m_stop;
    std::atomic<bool> m_requestRestart;

    // Worker lifecycle flag so supervisor can detect worker exit without blocking join()
    std::atomic<bool> m_workerRunning;

    // Condition variable to wake supervisor quickly (used by watchdog handler)
    std::mutex m_cvMutex;
    std::condition_variable m_cv;

    // Restart throttling / cooldown
    int m_maxRestartsBeforeCooldown;
    int m_cooldownMs;
    std::atomic<int> m_recentRestarts;
};