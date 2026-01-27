#pragma once
// ProcessDevice.h
// Separate file for Kinect device processing (JointFinder).

#include <string>
#include <vector>
#include <mutex>
#include <fstream>
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

class JointFinder {
public:
    // DetectJoints - main processing loop for a single Kinect device.
    void DetectJoints(int deviceIndex, k4a_device_t openedDevice,
        std::ofstream& outputFile, std::mutex& outputFileMutex,
        bool RECORDTIMESTAMPS,
        int stressHangAfterFrames = 0);

private:
    // helper to convert float -> IEEE 16-bit half float
    static uint16_t floatToHalf(float value);
};
