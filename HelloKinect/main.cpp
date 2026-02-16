// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include <iostream>
#include <vector>
#include <thread>
#include <string>
#include <k4a/k4a.h>                // For Kinect stuff
#include <k4abt.h>                  // For Kinect stuff
#include <stdio.h>                  // For string handling
#include <fstream>                  // For file operations
#include <chrono>                   // For timestamps
#include <mutex> 			        // For thread safe logging         
#include <algorithm>
#include <memory>
#include "Watchdog.h"
#include "ZenohPublisher.h"
#include "ProcessDevice.h"
#include "WorkerController.h"
#include <Windows.h>
#include <atomic>


#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define MAG   "\x1B[35m"
#define RESET "\x1B[0m"

std::mutex outputFileMutex;

// Global watchdog pointer for heartbeat calls from worker threads
Watchdog* g_watchdog = nullptr;

// Global pointer used by worker threads to publish (set in main())
ZenohPublisher* g_zenoh = nullptr;

struct KinectDevice {
    k4a_device_t device;
    std::string serial_number;
    std::string name;
};

// Retrieve device serial number
std::string get_device_serial(k4a_device_t device) {
    char serial_number[256];
    size_t sn_size = sizeof(serial_number);
    if (k4a_device_get_serialnum(device, serial_number, &sn_size) != K4A_RESULT_SUCCEEDED) {
        printf(RED "\nFailed to get serial number for a Kinect device\n" RESET);
        return "";
    }
    return std::string(serial_number);
}

// Write joint + timestamp to a log file
void writeToLog(std::ofstream& outputFile, std::mutex& outputFileMutex, std::string& stringToLog)
{
    // Get the current time
    auto currentTime = std::chrono::system_clock::now();

    // Get time since epoch in microseconds and convert to string
    auto durationMillis = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch());

    std::string duractionMilliAsString = std::to_string(durationMillis.count());

    // Lock the mutex for thread-safe writing
    std::lock_guard<std::mutex> lock(outputFileMutex);

    // Print to the text file / log
    outputFile << duractionMilliAsString + "," + stringToLog + "\n";
}



std::vector<std::string> LoadDesiredOrder(const std::string& filename) {
    std::vector<std::string> desiredOrder;
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        printf(RED "\nFailed to open desired order file for reading: %s\n" RESET, filename.c_str());
        return desiredOrder; // empty
    }

    std::string line;
	bool firstLine = true;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue; // skip blank lines
        if (firstLine) { // skip first line
            firstLine = false;
            continue;
        }
        printf("Parsing room file current line: ");
        printf(line.c_str());
        printf("\n");

        // first CSV field (data is clean; first value is the serial)
        std::string firstField;
        size_t comma = line.find(',');
        if (comma == std::string::npos) firstField = line;
        else firstField = line.substr(0, comma);

        // Validate: must be exactly 11 alphanumeric characters
        if (firstField.size() >= 11 && firstField.size() <= 12) {
            desiredOrder.push_back(firstField);
        }
        else {
            printf(YEL "\nSkipping invalid serial in desired order file: '%s'\n" RESET, firstField.c_str());
        }
    }

    ifs.close();
    return desiredOrder;
}

void printHelp() {
    std::cout << "Usage: HelloKinect [options]\n"
        << "Options:\n"
        << "  --roomName <roomname>   Name of room file to load IDs from\n"
        << "  --logeverything         Log all timestamps and events\n"
        << "  --log <file_path>       Specify log file path (default: C:\\Temp\\tempCG\\KinectLog.txt)\n"
        << "  --stress-hang <frames>  [TEST] Simulate hang after N frames to test watchdog\n"
        << "  -h, --help              Show this help message\n";
}

// Graceful termination flag set by console handler
static std::atomic<bool> g_terminate{ false };

// Fast console control handler — keep it tiny.
BOOL WINAPI ConsoleCtrlHandler(DWORD dwCtrlType)
{
    if (dwCtrlType == CTRL_C_EVENT || dwCtrlType == CTRL_CLOSE_EVENT ||
        dwCtrlType == CTRL_BREAK_EVENT || dwCtrlType == CTRL_SHUTDOWN_EVENT) {
        g_terminate.store(true);
        return TRUE; // handled
    }
    return FALSE; // not handled
}

int main(int argc, char* argv[])
{
    // Brief delay to allow previous process to fully release resources (port, devices)
    // This is needed when restarting via watchdog
    Sleep(1000);

    // Register console handler early so Ctrl+C triggers graceful shutdown
    SetConsoleCtrlHandler(ConsoleCtrlHandler, TRUE);

    printf("HelloDevice Version: 1.3 \n" RESET);
    printf("Use -h to see executable parameters.\n" RESET);

    bool RECORDTIMESTAMPS = false;
    std::string ROOMNAME = "OneCam";
    std::string LOGFILEPATH = "C:\\Temp\\tempCG\\KinectLog.txt";

    // Stress test options (for testing watchdog)
    int STRESS_HANG_AFTER_FRAMES = 0;  // 0 = disabled, >0 = hang after N frames

    // Simple command-line parsing
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        // Support --roomname=<value>
        if (arg.rfind("--roomname=", 0) == 0) {
            ROOMNAME = arg.substr(sizeof("--roomname=") - 1);
        }
        // Support --roomname <value>
        else if (arg == "--roomname") {
            if (i + 1 < argc) {
                ROOMNAME = argv[++i];
            }
            else {
                fprintf(stderr, "Missing value for --roomname\n");
                return 1;
            }
        }
        else if (arg == "--logeverything") {
            RECORDTIMESTAMPS = true;
        }
        else if (arg == "--log") {
            if (i + 1 < argc) {
                LOGFILEPATH = argv[++i];
            }
            else {
                fprintf(stderr, "Missing value for --log\n");
                return 1;
            }
        }
        else if (arg == "-h" || arg == "--help") {
            printHelp();
            return 0;
        }
        else if (arg == "--stress-hang") {
            if (i + 1 < argc) {
                try {
                    STRESS_HANG_AFTER_FRAMES = std::stoi(argv[++i]);
                    printf(YEL "[STRESS TEST] Will simulate hang after %d frames\n" RESET, STRESS_HANG_AFTER_FRAMES);
                }
                catch (const std::exception&) {
                    fprintf(stderr, "Invalid value for --stress-hang: '%s'\n", argv[i]);
                    return 1;
                }
            }
            else {
                fprintf(stderr, "Missing value for --stress-hang\n");
                return 1;
            }
        }
        // unknown args are ignored silently
    }

    printf("\n----------------------\n");
    printf("Start Input Parameters\n");
    printf("----------------------\n\n");
    printf(YEL "Room name = %s\n" RESET, ROOMNAME.c_str());
    printf("Logging timestamps = %s\n", RECORDTIMESTAMPS ? "true" : "false");
    printf("Log file path (if using) = %s\n", LOGFILEPATH.c_str());
    printf("----------------------\n\n");

    // Initialize watchdog for auto-restart on crashes/hangs
    Watchdog watchdog(argc, argv);
    g_watchdog = &watchdog;

    // Zenoh string
    std::string zenohString = "cg/" + ROOMNAME + "/dp/";

    // TODO fix logpath default and directory creation
    std::ofstream outputFile(LOGFILEPATH, std::ios::app);

    if (RECORDTIMESTAMPS) {
        // Check if the file is open
        if (!outputFile.is_open()) {
            std::cerr << "Failed to open the file. Likely KinectLog.txt directory does not exist" << std::endl;
            return 1; // Return an error code
        }

        // Check if file is empty
        std::ifstream ifs(LOGFILEPATH);
        bool fileIsEmpty = ifs.peek() == std::ifstream::traits_type::eof();
        ifs.close();

        if (fileIsEmpty) {

            std::string jointString = "timestamp, eventID, cameraID, cameraSerialID, FrameNumber,";

            for (int jointIdx = 0; jointIdx < 32; ++jointIdx) {
                std::string Idx = "joint" + std::to_string(jointIdx);
                jointString += 
                    Idx + "," +
                    Idx + "_conf," +
                    Idx + "_posx," +
                    Idx + "_posy," +
                    Idx + "_posz," +
                    Idx + "_rotw," +
                    Idx + "_rotx," +
                    Idx + "_roty," +
                    Idx + "_rotz";
                if (jointIdx != 31) jointString += ","; // Add comma except after last joint
            }
            outputFile << jointString << std::endl;
        }
        else {
            printf(YEL "\nFile already exists and is not empty. Will append to file.\n" RESET);
            outputFile << std::endl; // force a new line as unlikely last one closed nicely.
            std::string toLog = "Starting New Recording";
            writeToLog(outputFile, outputFileMutex, toLog);
        }
    }

    uint32_t device_count = k4a_device_get_installed_count();
    printf("Found %d connected devices:\n", device_count);

    // Use DeviceManager so workers can update device handles safely and reorder safely.
    DeviceManager deviceManager;

    std::cout << "Initial order of devices (scanning)...\n";

    for (uint32_t i = 0; i < device_count; i++) {
        k4a_device_t device = nullptr;
        if (k4a_device_open(i, &device) != K4A_RESULT_SUCCEEDED) {
            std::cerr << "Failed to open the Kinect Azure device" << std::endl;
            return 1;
        }
        else {
            std::string serial_number = get_device_serial(device);
            if (!serial_number.empty()) {
                std::cout << " Device index " << i << " SN: " << serial_number << std::endl;

                // Preserve or create DeviceContext for this serial
                auto ctx = deviceManager.getOrCreateBySerial(serial_number);

                // Install the acquired device handle into the context under lock
                {
                    std::lock_guard<std::mutex> lk(ctx->mutex);
                    ctx->device = makeDeviceHandle(device); // take ownership with deleter
                    ctx->serial = serial_number;
                }
                // Note: do not k4a_device_close(device) because makeDeviceHandle takes ownership.
            }
            else {
                k4a_device_close(device);
            }
        }
    }

    // Reorder according to desired file if present
    std::vector<std::string> desiredOrder = LoadDesiredOrder("C:\\CommonGround\\CalibrationFiles\\" + ROOMNAME + ".txt");

    auto snapshot = deviceManager.snapshot();
    if (!desiredOrder.empty()) {
        if (desiredOrder.size() != snapshot.size()) {
            std::cerr << RED "\nDesired order size does not match connected devices size. Check the desired order file and connected devices.\n" RESET;
            return -1;
        }
        deviceManager.reorderBySerial(desiredOrder);
    }

    // Take a fresh snapshot in final order for reporting and worker creation
    snapshot = deviceManager.snapshot();

    // Print sorted order and store real serials (fall back to Device_n only for display)
    std::cout << "Final order of devices:" << std::endl;
    for (size_t i = 0; i < snapshot.size(); i++) {
        // Capture real serial if we have an open handle; otherwise keep the existing label temporarily.
        std::string realSN = "";
        {
            std::lock_guard<std::mutex> lk(snapshot[i]->mutex);
            if (snapshot[i]->device && *snapshot[i]->device) {
                realSN = get_device_serial(*snapshot[i]->device); // real hardware serial
                if (!realSN.empty()) {
                    snapshot[i]->serial = realSN; // store physical serial used for reopen matching
                } else {
                    // fallback to a stable display name if driver didn't provide a serial
                    snapshot[i]->serial = "Device_" + std::to_string(i);
                }
            } else {
                snapshot[i]->serial = "Device_" + std::to_string(i);
            }
        }

        // Keep a separate display name if you want to show a friendly label
        std::string displayName = "Device_" + std::to_string(i);
        std::cout << " Device " << i << " SN: " << snapshot[i]->serial << " Name: " << displayName << std::endl;
    }

    // Initialize Zenoh publisher
    std::unique_ptr<ZenohPublisher> zenohPublisher = std::make_unique<ZenohPublisher>(zenohString + "sk");
    if (!zenohPublisher->init(/* options */ "")) {
        fprintf(stderr, "Zenoh: init() failed — continuing without Zenoh publishing\n");
    }
    else {
        if (!zenohPublisher->declare(zenohString + "sk")) {
            fprintf(stderr, "Zenoh: declare() failed\n");
        }
        g_zenoh = zenohPublisher.get();
        g_zenoh->startHeartbeat(zenohString + "hb");
    }

    // Send a one-time hello message to Zenoh subscribers
    if (g_zenoh && g_zenoh->isActive()) {
        const std::string hello = "HelloKinect: Hello to all Zenoh subscribers";
        std::vector<uint8_t> hello_pkt(hello.begin(), hello.end());
        if (g_zenoh->publish(hello_pkt)) {
            printf(GRN "Zenoh: Sent hello message\n" RESET);
        }
        else {
            fprintf(stderr, "Zenoh: Failed to publish hello message\n");
        }
    }

    // worker supervisors
    std::vector<std::shared_ptr<WorkerController>> supervisors;
    supervisors.reserve(snapshot.size());

    for (size_t i = 0; i < snapshot.size(); i++) {
        std::cerr << "Creating supervisor for Device with serial number: " << snapshot[i]->serial << std::endl;
        auto sup = std::make_shared<WorkerController>(
            snapshot[i],
            static_cast<int>(i),
            outputFile,
            outputFileMutex,
            RECORDTIMESTAMPS,
            STRESS_HANG_AFTER_FRAMES,
            &watchdog);
        supervisors.push_back(sup);
        sup->start();
    }

    // Start watchdog monitoring (it will now invoke per-worker restart handlers rather than process exit)
    watchdog.start();

    // Wait until termination (Ctrl+C) or full-process restart requested by watchdog.
    // This replaces the previous infinite loop that ignored g_terminate.
    while (!g_terminate.load() && !watchdog.shouldRestart()) {
        Sleep(500);
    }

    std::cerr << "Shutdown requested, stopping supervisors..." << std::endl;

    // On shutdown: stop and join supervisors
    for (auto& sup : supervisors) {
        sup->stopAndJoin();
    }

    watchdog.stop();
    g_watchdog = nullptr;

    // DeviceHandles are owned by DeviceContext -> shared_ptr deleter will stop/close devices.
    // Reset contexts to run deleters if worker threads already exited.
    {
        auto finalSnapshot = deviceManager.snapshot();
        for (auto& ctx : finalSnapshot) {
            std::lock_guard<std::mutex> lk(ctx->mutex);
            ctx->device.reset();
        }
    }

    if (RECORDTIMESTAMPS) {
        // Close the file when done
        outputFile.close();
    }

    std::cerr << "Shutdown complete." << std::endl;
    return 0;

}