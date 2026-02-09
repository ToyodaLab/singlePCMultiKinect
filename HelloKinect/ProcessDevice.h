#pragma once

// ProcessDevice.h - Kinect device processing (JointFinder)
//
// Thread-safety & reconnection guidance (integrated docs)
//
// Overview
// - A `DeviceContext` represents one logical Kinect device. The `device` member
//   is a `DeviceHandle` (a `shared_ptr<k4a_device_t>`). The `DeviceContext`
//   contains a `mutex` that protects access to its internal handle and `serial`.
// - The codebase uses worker threads (one `JointFinder::DetectJoints` per device)
//   that hold `std::shared_ptr<DeviceContext>` to reference the device state.
//
// Key rules for safe multi-threaded reconnect handling
// 1. Preserve `DeviceContext` identity when possible
//    - If the OS disconnects and reconnects a device, prefer to keep the same
//      `shared_ptr<DeviceContext>` instance and update its internal `device`
//      handle under `deviceCtx->mutex`. This preserves pointer identity for all
//      consumers and avoids dangling references to stale `DeviceContext` objects.
// 2. If you must replace the `shared_ptr<DeviceContext>` (rare):
//    - Replace it under a global container lock and ensure all consumers obtain
//      the latest pointer via a thread-safe snapshot (see `DeviceManager` below).
//    - Any threads that still hold the old `shared_ptr` will continue to hold a
//      valid (but possibly disconnected) `DeviceContext` until their copy is released.
// 3. Always protect the container holding `DeviceContext` objects (e.g. a
//    `std::vector<std::shared_ptr<DeviceContext>>`) with a mutex when reading or
//    writing. For iteration in worker threads take a snapshot copy of the vector
//    under the lock and iterate the snapshot outside the lock.
// 4. Detect disconnects early and release OS handles before attempting to re-open.
//    - Close / release the `k4a_device_t` handle (via the `DeviceHandle` deleter)
//      before scanning/attempting to re-open. Releasing should happen outside
//      of any long-held locks when practical.
// 5. Use finite timeouts for blocking operations (e.g. `k4a_device_get_capture`,
//    tracker operations) so threads can detect failures and attempt recovery.
//
// Example safe pattern (high-level):
// - Worker thread:
//   1. Acquire a local `DeviceHandle` snapshot under `deviceCtx->mutex`.
//   2. Use the local handle for SDK calls (no need to hold `deviceCtx->mutex`).
///   3. On SDK errors/timeouts, call a reconnect helper that replaces the handle
//      inside the existing `DeviceContext` under lock.
// - Reconnect helper:
//   1. Optionally destroy tracker / per-thread objects that reference the device.
//   2. Swap out the `deviceCtx->device` under the lock into a local `DeviceHandle`.
//   3. Release the old handle outside the lock (so the deleter can close the device).
//   4. Scan OS devices and open a new `k4a_device_t`; construct a `DeviceHandle` and
//      store it into `deviceCtx->device` while holding the `deviceCtx->mutex`.
//
// Device container helper (recommended): `DeviceManager`
// - Provides a single place to manage the vector of contexts with well-defined
//   thread-safe operations: snapshot, reorder, find-or-create, remove.
// - Use `DeviceManager::snapshot()` to get a stable list of `shared_ptr`s to iterate.
//
// Reordering safely (example):
// - Use `DeviceManager::reorderBySerial(desiredOrder)` which acquires the manager
//   lock and rebuilds the internal vector in the desired order (matching by serial).
// - Worker threads should never hold the manager lock for long; they use the
//   snapshot to perform work without blocking container modifications.

#include <string>
#include <vector>
#include <mutex>
#include <fstream>
#include <memory>
#include <k4a/k4a.h>
#include <k4abt.h>
#include <algorithm>
#include <atomic>

#include "ZenohPublisher.h"
#include "Watchdog.h"

extern Watchdog* g_watchdog;
extern ZenohPublisher* g_zenoh;

std::string get_device_serial(k4a_device_t device);
void writeToLog(std::ofstream& outputFile, std::mutex& outputFileMutex, std::string& stringToLog);
uint16_t floatToHalf(float value);

using DeviceHandle = std::shared_ptr<k4a_device_t>;

struct DeviceContext {
    std::mutex mutex;           // protects `device` and `serial`
    DeviceHandle device;        // shared ownership wrapper that closes device on destruction
    std::string serial;

    // Prevent concurrent reconnect attempts for the same logical device.
    // This guards reopen logic so only one thread attempts to re-open/replace the handle.
    std::atomic<bool> reconnecting{ false };

    DeviceContext() : device(), serial() {}
    explicit DeviceContext(const std::string& sn) : device(), serial(sn) {}
};

inline DeviceHandle makeDeviceHandle(k4a_device_t dev)
{
    if (!dev) return DeviceHandle();
    return DeviceHandle(new k4a_device_t(dev),
        [](k4a_device_t* p) {
            if (p && *p) {
                k4a_device_stop_cameras(*p);
                k4a_device_close(*p);
            }
            delete p;
        });
}

// DeviceManager: thread-safe container for DeviceContext instances.
//
// Usage notes:
// - Call `snapshot()` to iterate devices from worker threads without holding the manager lock.
// - Call `getOrCreateBySerial()` when a new device appears; this preserves context identity.
// - Call `reorderBySerial()` while holding no per-device locks; it only rearranges the vector
//   of `shared_ptr<DeviceContext>` under the manager mutex.
class DeviceManager {
public:
    // Return a snapshot copy of the current device contexts (copy of shared_ptrs).
    std::vector<std::shared_ptr<DeviceContext>> snapshot() {
        std::lock_guard<std::mutex> lk(mutex_);
        return devices_;
    }

    // Find context by serial; returns nullptr if not found.
    std::shared_ptr<DeviceContext> findBySerial(const std::string& serial) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = std::find_if(devices_.begin(), devices_.end(),
            [&](const std::shared_ptr<DeviceContext>& c) { return c && c->serial == serial; });
        return (it != devices_.end()) ? *it : nullptr;
    }

    // Get existing context or create a new one (preserving identity).
    std::shared_ptr<DeviceContext> getOrCreateBySerial(const std::string& serial) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = std::find_if(devices_.begin(), devices_.end(),
            [&](const std::shared_ptr<DeviceContext>& c) { return c && c->serial == serial; });
        if (it != devices_.end()) return *it;
        auto ctx = std::make_shared<DeviceContext>(serial);
        devices_.push_back(ctx);
        return ctx;
    }

    // Remove context by serial. Returns true if removed.
    bool removeBySerial(const std::string& serial) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = std::remove_if(devices_.begin(), devices_.end(),
            [&](const std::shared_ptr<DeviceContext>& c) { return !c || c->serial == serial; });
        if (it != devices_.end()) {
            devices_.erase(it, devices_.end());
            return true;
        }
        return false;
    }

    // Reorder the internal device vector to match `desiredOrder` (list of serials).
    // Devices not listed in `desiredOrder` keep their relative order after the matched ones.
    void reorderBySerial(const std::vector<std::string>& desiredOrder) {
        std::lock_guard<std::mutex> lk(mutex_);
        std::vector<std::shared_ptr<DeviceContext>> reordered;
        reordered.reserve(devices_.size());

        // First, append in desired order if present
        for (const auto& sn : desiredOrder) {
            auto it = std::find_if(devices_.begin(), devices_.end(),
                [&](const std::shared_ptr<DeviceContext>& ctx) { return ctx && ctx->serial == sn; });
            if (it != devices_.end()) {
                reordered.push_back(*it);
            }
        }

        // Then append any contexts not already included (preserve their relative order)
        for (const auto& ctx : devices_) {
            if (!ctx) continue;
            auto already = std::find(reordered.begin(), reordered.end(), ctx);
            if (already == reordered.end()) reordered.push_back(ctx);
        }

        devices_.swap(reordered);
    }

private:
    std::mutex mutex_;
    std::vector<std::shared_ptr<DeviceContext>> devices_;
};

class JointFinder {
public:
    // Main processing loop for a single Kinect device.
    // Added `deviceLabel` parameter so the caller can provide a prebuilt label
    // (e.g. "<serial>1", "<serial>2") that is used immediately by the worker.
    void DetectJoints(int deviceIndex,
                      std::shared_ptr<DeviceContext> deviceCtx,
                      const std::string& deviceLabel,
                      std::ofstream& outputFile,
                      std::mutex& outputFileMutex,
                      bool RECORDTIMESTAMPS,
                      int stressHangAfterFrames = 0);

private:
    static uint16_t floatToHalf(float value);
};
