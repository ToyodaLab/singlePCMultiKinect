// Watchdog.cpp - Internal watchdog implementation for auto-restart

#include "Watchdog.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <ctime>

// Static member initialization
std::atomic<int64_t> Watchdog::s_heartbeats[Watchdog::MAX_WORKERS] = {};
std::atomic<size_t> Watchdog::s_workerCount{ 0 };

Watchdog::Watchdog(int argc, char* argv[],
                   const std::string& crashLogPath,
                   int timeoutSeconds,
                   int cooldownSeconds,
                   int maxRestartsBeforeCooldown)
    : m_crashLogPath(crashLogPath)
    , m_timeoutSeconds(timeoutSeconds)
    , m_cooldownSeconds(cooldownSeconds)
    , m_maxRestartsBeforeCooldown(maxRestartsBeforeCooldown)
    , m_recentRestartCount(0)
    , m_running(false)
    , m_restartRequested(false)
    , m_monitorThread(NULL)
{
    // Store executable path
    char exePath[MAX_PATH];
    GetModuleFileNameA(NULL, exePath, MAX_PATH);
    m_executablePath = exePath;

    // Store CLI arguments (skip argv[0] which is the executable)
    for (int i = 1; i < argc; ++i) {
        m_cliArgs.push_back(argv[i]);
    }

    // Allow immediate restart on first run
    m_lastRestartTime = std::chrono::steady_clock::now() -
                        std::chrono::seconds(m_cooldownSeconds);

    std::cout << "[Watchdog] Initialized with timeout=" << m_timeoutSeconds
              << "s, cooldown=" << m_cooldownSeconds << "s" << std::endl;
}

Watchdog::~Watchdog() {
    stop();
}

void Watchdog::registerWorker(size_t workerIndex) {
    if (workerIndex >= MAX_WORKERS) {
        std::cerr << "[Watchdog] Warning: Worker index " << workerIndex
                  << " exceeds MAX_WORKERS (" << MAX_WORKERS << "). Not monitored." << std::endl;
        return;
    }

    // Initialize with current time
    auto now = std::chrono::steady_clock::now();
    s_heartbeats[workerIndex].store(now.time_since_epoch().count());
    s_workerCount.fetch_add(1);

    std::cout << "[Watchdog] Registered worker " << workerIndex << std::endl;
}

void Watchdog::heartbeat(size_t workerIndex) {
    if (workerIndex < MAX_WORKERS) {
        auto now = std::chrono::steady_clock::now();
        s_heartbeats[workerIndex].store(now.time_since_epoch().count());
    }
}

// Windows thread function wrapper (friend of Watchdog class)
DWORD WINAPI WatchdogThreadProc(LPVOID lpParam) {
    Watchdog* watchdog = static_cast<Watchdog*>(lpParam);
    watchdog->monitorLoop();
    return 0;
}

void Watchdog::start() {
    m_running = true;
    m_monitorThread = CreateThread(NULL, 0, WatchdogThreadProc, this, 0, NULL);
    if (m_monitorThread == NULL) {
        std::cerr << "[Watchdog] Failed to create monitoring thread" << std::endl;
    } else {
        std::cout << "[Watchdog] Monitoring started" << std::endl;
    }
}

void Watchdog::stop() {
    m_running = false;
    if (m_monitorThread != NULL) {
        WaitForSingleObject(m_monitorThread, 5000); // Wait up to 5 seconds
        CloseHandle(m_monitorThread);
        m_monitorThread = NULL;
        std::cout << "[Watchdog] Stopped" << std::endl;
    }
}

bool Watchdog::shouldRestart() const {
    return m_restartRequested.load();
}

void Watchdog::monitorLoop() {
    while (m_running) {
        Sleep(2000); // Check every 2 seconds

        if (!m_running) break;

        auto now = std::chrono::steady_clock::now();
        size_t workerCount = s_workerCount.load();

        for (size_t i = 0; i < workerCount && i < MAX_WORKERS; ++i) {
            int64_t lastHeartbeatRaw = s_heartbeats[i].load();
            auto lastHeartbeat = std::chrono::steady_clock::time_point(
                std::chrono::steady_clock::duration(lastHeartbeatRaw));

            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                now - lastHeartbeat).count();

            if (elapsed > m_timeoutSeconds) {
                std::ostringstream oss;
                oss << "Worker thread " << i << " appears hung (no heartbeat for "
                    << elapsed << " seconds)";
                triggerRestart(oss.str());
                return;
            }
        }
    }
}

void Watchdog::triggerRestart(const std::string& reason) {
    logCrashEvent(reason);

    auto now = std::chrono::steady_clock::now();
    auto timeSinceLastRestart = std::chrono::duration_cast<std::chrono::seconds>(
        now - m_lastRestartTime).count();

    // Check cooldown
    if (timeSinceLastRestart < m_cooldownSeconds) {
        m_recentRestartCount++;
        if (m_recentRestartCount >= m_maxRestartsBeforeCooldown) {
            std::ostringstream oss;
            oss << "Too many restarts (" << m_recentRestartCount
                << ") in short period. Waiting " << m_cooldownSeconds << "s cooldown...";
            logCrashEvent(oss.str());
            std::cerr << "[Watchdog] " << oss.str() << std::endl;
            Sleep(m_cooldownSeconds * 1000);
            m_recentRestartCount = 0;
        }
    } else {
        m_recentRestartCount = 1;
    }

    m_lastRestartTime = now;
    m_restartRequested = true;

    // Build command line for restart
    std::string cmdLine = "\"" + m_executablePath + "\"";
    for (const auto& arg : m_cliArgs) {
        // Quote arguments that contain spaces
        if (arg.find(' ') != std::string::npos) {
            cmdLine += " \"" + arg + "\"";
        } else {
            cmdLine += " " + arg;
        }
    }

    logCrashEvent("Initiating restart: " + cmdLine);
    std::cerr << "[Watchdog] Initiating restart: " << cmdLine << std::endl;

    // Create new process
    STARTUPINFOA si = { sizeof(si) };
    PROCESS_INFORMATION pi = { 0 };

    // CreateProcessA requires non-const char*
    std::vector<char> cmdLineBuffer(cmdLine.begin(), cmdLine.end());
    cmdLineBuffer.push_back('\0');

    if (CreateProcessA(
            NULL,                    // Application name (use command line)
            cmdLineBuffer.data(),    // Command line
            NULL,                    // Process security attributes
            NULL,                    // Thread security attributes
            TRUE,                    // Inherit handles (inherit console)
            CREATE_SUSPENDED,        // Start suspended so we can exit first
            NULL,                    // Environment
            NULL,                    // Current directory
            &si,                     // Startup info
            &pi)) {

        logCrashEvent("New process created (suspended). Exiting current process first.");
        std::cerr << "[Watchdog] New process created. Exiting current process..." << std::endl;

        // Resume the new process - it will start after we exit
        ResumeThread(pi.hThread);

        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);

        // Exit immediately - the new process will take over
        ExitProcess(0);
    } else {
        DWORD error = GetLastError();
        std::ostringstream oss;
        oss << "Failed to restart process. Error code: " << error;
        logCrashEvent(oss.str());
        std::cerr << "[Watchdog] " << oss.str() << std::endl;
    }
}

void Watchdog::logCrashEvent(const std::string& reason) {
    std::lock_guard<std::mutex> lock(m_logMutex);

    std::ofstream logFile(m_crashLogPath, std::ios::app);
    if (logFile.is_open()) {
        // Get current time
        auto now = std::chrono::system_clock::now();
        auto timeT = std::chrono::system_clock::to_time_t(now);

        char timeStr[64];
        struct tm localTime;
        localtime_s(&localTime, &timeT);
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &localTime);

        logFile << "[" << timeStr << "] " << reason << std::endl;
        logFile.close();
    }
}
