#pragma once
// ProcessDevice.h
// Separate file for Kinect device processing (JointFinder).

#include <string>
#include <vector>
#include <mutex>
#include <fstream>
#include <memory>
#include <k4a/k4a.h>
#include <k4abt.h>

#include "ZenohPublisher.h"
#include "Watchdog.h"

// main.cpp defines these globals; declare them as extern so this TU can use them.
extern Watchdog* g_watchdog;
extern ZenohPublisher* g_zenoh;

// main.cpp defines these helper functions; declare them so DeviceProcessor can call them.
std::string get_device_serial(k4a_device_t device);
void writeToLog(std::ofstream& outputFile, std::mutex& outputFileMutex, std::string& stringToLog);

// Add this prototype so `floatToHalf` can be used as a free function (matches the definition in ProcessDevice.cpp)
uint16_t floatToHalf(float value);

// Shared device handle and context used to synchronize worker <-> main
using DeviceHandle = std::shared_ptr<k4a_device_t>;

struct DeviceContext {
    std::mutex mutex;       // protects access to `device`
    DeviceHandle device;    // shared pointer owning the device handle
    std::string serial;
    DeviceContext() : device(), serial() {}
};

// Helper to wrap a raw k4a_device_t into a shared pointer with proper deleter.
// The deleter stops cameras and closes the device when the last shared_ptr is destroyed.
inline DeviceHandle makeDeviceHandle(k4a_device_t dev)
{
    if (!dev) return DeviceHandle();
    return DeviceHandle(new k4a_device_t(dev),
        [](k4a_device_t* p) {
            if (p && *p) {
                // best-effort cleanup
                k4a_device_stop_cameras(*p);
                k4a_device_close(*p);
            }
            delete p;
        });
}

class JointFinder {
public:
    // DetectJoints - main processing loop for a single Kinect device.
    // Now takes a shared DeviceContext so worker can atomically swap device handles.
    void DetectJoints(int deviceIndex, std::shared_ptr<DeviceContext> deviceCtx,
        std::ofstream& outputFile, std::mutex& outputFileMutex,
        bool RECORDTIMESTAMPS,
        int stressHangAfterFrames = 0);

private:
    // helper to convert float -> IEEE 16-bit half float
    static uint16_t floatToHalf(float value);
};
