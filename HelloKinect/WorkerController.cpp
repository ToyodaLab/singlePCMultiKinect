#include "WorkerController.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <thread>

#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define RESET "\x1B[0m"

WorkerController::WorkerController(std::shared_ptr<DeviceContext> ctx,
                                   int deviceIndex,
                                   std::ofstream& outputFile,
                                   std::mutex& outputFileMutex,
                                   bool recordTimestamps,
                                   int stressHangAfterFrames,
                                   Watchdog* watchdog)
    : m_deviceCtx(ctx)
    , m_deviceIndex(deviceIndex)
    , m_outputFile(outputFile)
    , m_outputFileMutex(outputFileMutex)
    , m_recordTimestamps(recordTimestamps)
    , m_stressHangAfterFrames(stressHangAfterFrames)
    , m_watchdog(watchdog)
    , m_stop(false)
    , m_requestRestart(false)
    , m_maxRestartsBeforeCooldown(5)
    , m_cooldownMs(5000)
{
}

WorkerController::~WorkerController() {
    stopAndJoin();
}

void WorkerController::start() {
    // Register worker with watchdog
    if (m_watchdog) {
        m_watchdog->registerWorker(static_cast<size_t>(m_deviceIndex));
        // Register restart handler - set a restart flag when watchdog detects hang
        m_watchdog->registerRestartHandler(static_cast<size_t>(m_deviceIndex),
            [this](size_t idx) {
                // set restart flag (must be lightweight)
                m_requestRestart.store(true);
            });
    }

    m_supervisorThread = std::thread(&WorkerController::superviseLoop, this);
}

void WorkerController::stopAndJoin() {
    m_stop.store(true);
    // Request worker to stop by resetting device handle (worker will see no device)
    if (m_deviceCtx) {
        std::lock_guard<std::mutex> lk(m_deviceCtx->mutex);
        m_deviceCtx->device.reset();
    }

    if (m_supervisorThread.joinable()) {
        m_supervisorThread.join();
    }
    if (m_workerThread.joinable()) {
        m_workerThread.join();
    }
}

void WorkerController::spawnWorker() {
    // Launch the worker thread that runs DetectJoints
    m_workerThread = std::thread([this]() {
        try {
            JointFinder finder;
            finder.DetectJoints(m_deviceIndex, m_deviceCtx, m_outputFile, m_outputFileMutex,
                                m_recordTimestamps, m_stressHangAfterFrames);
        } catch (const std::exception& ex) {
            std::ostringstream oss;
            oss << "Worker " << m_deviceIndex << " caught exception: " << ex.what();
            std::cerr << RED << oss.str() << RESET << std::endl;
        } catch (...) {
            std::cerr << RED << "Worker " << m_deviceIndex << " caught unknown exception" << RESET << std::endl;
        }
    });
}

void WorkerController::superviseLoop() {
    int consecutiveRestarts = 0;

    // Initially spawn one worker
    spawnWorker();

    while (!m_stop.load()) {
        // Poll loop: check worker thread liveness and restart requests
        bool workerAlive = m_workerThread.joinable();
        if (workerAlive) {
            // If joinable but finished, joinable remains true only until joined; use try_join pattern
            if (m_workerThread.joinable()) {
                // Attempt to see if worker has finished via try to join with zero wait (C++ doesn't have try_join).
                // Use timed sleep and check joinable later.
            }
        }

        // If worker thread has finished (not joinable means it hasn't been started), handle it:
        // We'll check native handle using std::future would be nicer; keep simple: check joinable + implement periodic join when done.
        if (m_workerThread.joinable()) {
            // check if thread finished by trying to join with timeout pattern:
            // we cannot join with timeout; instead check via sleep and then join if not joinable? Simpler:
            // Use std::this_thread::sleep_for and then check m_requestRestart flag or if worker thread terminated (by checking joinable).
        }

        // If watchdog requested restart, or worker thread has terminated, restart worker.
        if (m_requestRestart.load() || (m_workerThread.joinable() == false)) {
            // If worker thread is joinable, join it (clean up)
            if (m_workerThread.joinable()) {
                m_workerThread.join();
            }

            if (m_stop.load()) break;

            ++consecutiveRestarts;
            std::cerr << YEL << "WorkerController: restarting device " << m_deviceIndex << " (count=" << consecutiveRestarts << ")" << RESET << std::endl;

            if (consecutiveRestarts >= m_maxRestartsBeforeCooldown) {
                std::cerr << RED << "WorkerController: cooldown for device " << m_deviceIndex << " (" << m_cooldownMs << "ms)" << RESET << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(m_cooldownMs));
                consecutiveRestarts = 0;
            } else {
                int backoff = 200 * (1 << (std::min)(consecutiveRestarts, 6));
                backoff = (std::min)(backoff, 5000);
                std::this_thread::sleep_for(std::chrono::milliseconds(backoff));
            }

            // reset request flag and spawn worker
            m_requestRestart.store(false);
            spawnWorker();
        } else {
            // idle sleep small interval
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    } // end while

    // Ensure worker thread is joined before exit
    if (m_workerThread.joinable()) {
        m_workerThread.join();
    }
}