#include "WorkerController.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <thread>
#include <chrono>
#include <iomanip>

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
    , m_workerRunning(false)
    , m_maxRestartsBeforeCooldown(5)
    , m_cooldownMs(5000)
    , m_recentRestarts(0)
{
}

WorkerController::~WorkerController() {
    stopAndJoin();
}

void WorkerController::start() {
    // Register worker with watchdog (lightweight)
    if (m_watchdog) {
        m_watchdog->registerWorker(static_cast<size_t>(m_deviceIndex));
        // Register restart handler - keep it tiny: set flag + notify supervisor
        m_watchdog->registerRestartHandler(static_cast<size_t>(m_deviceIndex),
            [this](size_t /*idx*/) {
                // Keep handler minimal and non-blocking
                m_requestRestart.store(true);
                m_cv.notify_one();
            });
    }

    // Launch supervisor thread
    m_supervisorThread = std::thread(&WorkerController::superviseLoop, this);
}

void WorkerController::stopAndJoin() {
    // Signal stop
    m_stop.store(true);

    // Wake supervisor so it can exit promptly
    m_cv.notify_one();

    // Also request worker stop by clearing device handle (worker observes this)
    if (m_deviceCtx) {
        std::lock_guard<std::mutex> lk(m_deviceCtx->mutex);
        m_deviceCtx->device.reset();
    }

    // Join supervisor thread
    if (m_supervisorThread.joinable()) {
        m_supervisorThread.join();
    }

    // Ensure worker thread joined
    if (m_workerThread.joinable()) {
        // If worker hasn't indicated exit yet, give short grace period
        const int JOIN_GRACE_MS = 2000;
        int waited = 0;
        while (m_workerRunning.load() && waited < JOIN_GRACE_MS) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            waited += 100;
        }

        if (m_workerThread.joinable()) {
            try {
                m_workerThread.join();
            } catch (...) {
                std::cerr << RED << "WorkerController: exception joining worker thread on shutdown" << RESET << std::endl;
            }
        }
    }
}

void WorkerController::spawnWorker() {
    // If an existing worker thread object is joinable but finished, join first.
    if (m_workerThread.joinable() && !m_workerRunning.load()) {
        try {
            m_workerThread.join();
        } catch (...) {
            std::cerr << YEL << "WorkerController: exception joining previous worker" << RESET << std::endl;
        }
    }

    // Build device label (e.g. "<serial>#<index>")
    std::string deviceLabel = m_deviceCtx && !m_deviceCtx->serial.empty()
        ? m_deviceCtx->serial + "#" + std::to_string(m_deviceIndex)
        : "device#" + std::to_string(m_deviceIndex);

    // Launch the worker thread that runs DetectJoints. Wrap to set m_workerRunning.
    m_workerThread = std::thread([this, deviceLabel]() {
        m_workerRunning.store(true);
        try {
            JointFinder finder;
            finder.DetectJoints(m_deviceIndex, m_deviceCtx, deviceLabel, m_outputFile, m_outputFileMutex,
                                m_recordTimestamps, m_stressHangAfterFrames);
        } catch (const std::exception& ex) {
            std::ostringstream oss;
            oss << "Worker " << m_deviceIndex << " caught exception: " << ex.what();
            std::cerr << RED << oss.str() << RESET << std::endl;
        } catch (...) {
            std::cerr << RED << "Worker " << m_deviceIndex << " caught unknown exception" << RESET << std::endl;
        }
        m_workerRunning.store(false);
        // Notify supervisor quickly that worker has exited (so it may reopen/respawn)
        m_requestRestart.store(true);
        m_cv.notify_one();
    });
}

std::string WorkerController::nowStamp() {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    std::time_t t = system_clock::to_time_t(now);
    std::tm tm;
    localtime_s(&tm, &t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0') << std::setw(3) << ms.count()
        << " [tid=" << std::this_thread::get_id() << "]";
    return oss.str();
}

bool WorkerController::performReopenBySerial(const std::string& serial, bool forceReplace) {
    // Deterministic reopen on supervisor thread. This function only opens a new k4a device
    // matching the serial and places the DeviceHandle into the DeviceContext under lock.
    // It intentionally does not start cameras or create trackers; the worker will do that on spawn.
    const int MAX_REOPEN_ATTEMPTS = 8;
    const int POLL_MS = 200;
    std::ostringstream baseLog;
    baseLog << nowStamp() << " WorkerController[" << m_deviceIndex << "]: performReopenBySerial(serial=" << serial << ", forceReplace=" << (forceReplace?1:0) << ")";
    std::cerr << baseLog.str() << std::endl;

    // Prevent concurrent reopen attempts at ProcessDevice level (DeviceContext has reconnecting flag)
    bool expected = false;
    if (!m_deviceCtx->reconnecting.compare_exchange_strong(expected, true)) {
        std::cerr << nowStamp() << " WorkerController[" << m_deviceIndex << "]: another reopen already in progress\n";
        return false;
    }
    // RAII clear
    struct Guard { std::atomic<bool>& f; Guard(std::atomic<bool>& f_) : f(f_) {} ~Guard(){ f.store(false); } } guard(m_deviceCtx->reconnecting);

    // Optionally force release old handle so OS can re-enumerate
    std::shared_ptr<k4a_device_t> oldHandleForRelease;
    if (forceReplace) {
        std::cerr << nowStamp() << " WorkerController[" << m_deviceIndex << "]: forceReplace: releasing old handle\n";
        {
            std::lock_guard<std::mutex> lk(m_deviceCtx->mutex);
            oldHandleForRelease = m_deviceCtx->device;
            m_deviceCtx->device.reset();
            // keep serial to match reopened device
        }
        // Release outside lock
        try {
            if (oldHandleForRelease) oldHandleForRelease.reset();
        } catch (...) {
            std::cerr << nowStamp() << " WorkerController[" << m_deviceIndex << "]: exception releasing old handle\n";
        }

        // Give OS time to re-enumerate
        const int WAIT_MS = 8000;
        int waited = 0;
        while (waited < WAIT_MS) {
            int installed = k4a_device_get_installed_count();
            if (installed > 0) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(POLL_MS));
            waited += POLL_MS;
        }
        std::cerr << nowStamp() << " WorkerController[" << m_deviceIndex << "]: waited " << waited << " ms for re-enumeration\n";
    }

    for (int attempt = 0; attempt < MAX_REOPEN_ATTEMPTS; ++attempt) {
        int installed = k4a_device_get_installed_count();
        std::cerr << nowStamp() << " WorkerController[" << m_deviceIndex << "]: reopen attempt " << (attempt+1)
                  << " scanning " << installed << " installed devices\n";

        for (int i = 0; i < installed; ++i) {
            k4a_device_t cand = NULL;
            k4a_result_t openRes = k4a_device_open(i, &cand);
            if (openRes != K4A_RESULT_SUCCEEDED || cand == NULL) {
                std::cerr << nowStamp() << " WorkerController[" << m_deviceIndex << "]: k4a_device_open index=" << i << " failed (res=" << (int)openRes << ")\n";
                continue;
            }

            // small pause to let driver populate serial if needed
            std::this_thread::sleep_for(std::chrono::milliseconds(150));

            std::string candSN;
            // Try to read serial a few times
            for (int snTry = 0; snTry < 3 && candSN.empty(); ++snTry) {
                candSN = get_device_serial(cand);
                if (candSN.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            if (candSN.empty()) {
                std::cerr << nowStamp() << " WorkerController[" << m_deviceIndex << "]: index=" << i << " has empty serial; closing\n";
                k4a_device_close(cand);
                continue;
            }

            std::cerr << nowStamp() << " WorkerController[" << m_deviceIndex << "]: index=" << i << " serial=" << candSN << "\n";

            if (candSN != serial) {
                k4a_device_close(cand);
                continue;
            }

            // We matched serial: install handle into context (do NOT start cameras here).
            DeviceHandle newHandle = makeDeviceHandle(cand);

            {
                std::lock_guard<std::mutex> lk(m_deviceCtx->mutex);
                m_deviceCtx->device = newHandle;
                m_deviceCtx->serial = serial;
            }

            std::cerr << nowStamp() << " WorkerController[" << m_deviceIndex << "]: Successfully reopened device (index=" << i << ")\n";
            return true;
        }

        // exponential backoff for next scan
        int backoff = 500 * (1 << attempt);
        std::this_thread::sleep_for(std::chrono::milliseconds(backoff));
    }

    std::cerr << nowStamp() << " WorkerController[" << m_deviceIndex << "]: failed to reopen device after attempts\n";
    return false;
}

void WorkerController::superviseLoop() {
    // Initially spawn one worker
    spawnWorker();

    // Supervisor main loop: waits for restart requests or stop signal.
    while (!m_stop.load()) {
        // Wait efficiently until a restart is requested or stop is signaled.
        std::unique_lock<std::mutex> lk(m_cvMutex);
        m_cv.wait_for(lk, std::chrono::milliseconds(500), [this]() {
            return m_stop.load() || m_requestRestart.load();
        });

        if (m_stop.load()) break;

        if (m_requestRestart.load()) {
            // Reset the request flag quickly
            m_requestRestart.store(false);

            // Throttle frequent restarts with cooldown
            m_recentRestarts.fetch_add(1);
            if (m_recentRestarts.load() >= m_maxRestartsBeforeCooldown) {
                std::cerr << nowStamp() << " WorkerController[" << m_deviceIndex << "]: cooldown triggered ("
                          << m_cooldownMs << "ms)\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(m_cooldownMs));
                m_recentRestarts.store(0);
            }

            std::cerr << nowStamp() << " WorkerController[" << m_deviceIndex << "]: restart requested\n";

            // Wait for worker to exit in bounded time (worker sets m_workerRunning=false and signals supervisor)
            const int JOIN_TIMEOUT_MS = 5000;
            int waited = 0;
            const int POLL_MS = 100;
            while (m_workerRunning.load() && waited < JOIN_TIMEOUT_MS && !m_stop.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(POLL_MS));
                waited += POLL_MS;
            }

            // If thread object still joinable, join it
            if (m_workerThread.joinable()) {
                try {
                    m_workerThread.join();
                } catch (...) {
                    std::cerr << nowStamp() << " WorkerController[" << m_deviceIndex << "]: exception joining worker thread\n";
                }
            }

            // Attempt deterministic reopen using context serial
            std::string serialToOpen;
            {
                std::lock_guard<std::mutex> lkctx(m_deviceCtx->mutex);
                serialToOpen = m_deviceCtx->serial;
            }

            if (!serialToOpen.empty()) {
                bool reopened = performReopenBySerial(serialToOpen, true);
                if (!reopened) {
                    std::cerr << nowStamp() << " WorkerController[" << m_deviceIndex << "]: Reopen by serial failed\n";
                    // Decide: either continue to spawn worker (it will exit quickly) or keep trying later.
                    // For now, we will sleep a short time and allow next loop to try again if restart requested.
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                } else {
                    // Reset recent restart counter on success
                    m_recentRestarts.store(0);
                }
            } else {
                std::cerr << nowStamp() << " WorkerController[" << m_deviceIndex << "]: no serial available to reopen\n";
            }

            // Spawn replacement worker (worker will try to start cameras itself)
            spawnWorker();
        }
    } // end while

    // Final cleanup: ensure worker joined
    if (m_workerThread.joinable()) {
        if (m_workerRunning.load()) {
            // Give a short grace period
            const int JOIN_GRACE_MS = 2000;
            int waited = 0;
            while (m_workerRunning.load() && waited < JOIN_GRACE_MS) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                waited += 100;
            }
        }
        if (m_workerThread.joinable()) {
            try {
                m_workerThread.join();
            } catch (...) {
                std::cerr << YEL << "WorkerController: exception joining worker during supervisor shutdown" << RESET << std::endl;
            }
        }
    }
}