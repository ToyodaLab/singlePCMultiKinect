#pragma once
// WorkerController.h - Supervises a single device worker and restarts it on request.

#include <thread>
#include <atomic>
#include <memory>
#include <functional>
#include <chrono>
#include <fstream>
#include <mutex>
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

    std::shared_ptr<DeviceContext> m_deviceCtx;
    int m_deviceIndex;
    std::ofstream& m_outputFile;
    std::mutex& m_outputFileMutex;
    bool m_recordTimestamps;
    int m_stressHangAfterFrames;
    Watchdog* m_watchdog;

    std::thread m_supervisorThread;
    std::thread m_workerThread;

    std::atomic<bool> m_stop;
    std::atomic<bool> m_requestRestart;
    int m_maxRestartsBeforeCooldown;
    int m_cooldownMs;
};