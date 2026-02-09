#include "ProcessDevice.h"

#include <iostream>
#include <vector>
#include <cstring>
#include <cstdint>
#include <thread>
#include <chrono>
#include "winsock.h"
#pragma comment(lib, "Ws2_32.lib")

#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define MAG   "\x1B[35m"
#define RESET "\x1B[0m"

#define BUFFER_SIZE 1024 // kept for internal formatting/log buffers if needed

// Safe release helper: reset a DeviceHandle (calls its deleter) and catch any exceptions.
// Always use this for handles swapped out from deviceCtx so driver destructors don't escape.
static void releaseHandleSafely(DeviceHandle h, int deviceIndex, const char* reason)
{
    try {
        if (h) {
            h.reset(); // deleter runs here
        }
    } catch (const std::exception& e) {
        printf(YEL "Device %d: Exception while releasing handle (%s): %s\n" RESET, deviceIndex, reason, e.what());
    } catch (...) {
        printf(YEL "Device %d: Unknown exception while releasing handle (%s)\n" RESET, deviceIndex, reason);
    }
}

// Each Kinect is a class JointFinder.
// Removed TCP socket param; we only publish via Zenoh now.
void JointFinder::DetectJoints(int deviceIndex, std::shared_ptr<DeviceContext> deviceCtx,
    std::ofstream& outputFile, std::mutex& outputFileMutex,
    bool RECORDTIMESTAMPS,
    int stressHangAfterFrames /* = 0 */)
{
    printf(GRN "Detecting joints in %d\n" RESET, deviceIndex);

    uint32_t deviceID = deviceIndex;
    std::string deviceSerialID;
    {
        std::lock_guard<std::mutex> lk(deviceCtx->mutex);
        deviceSerialID = deviceCtx->serial;
    }

    int captureFrameCount = 0;
    const int32_t TIMEOUT_IN_MS = 1000;
    const int32_t TRACKER_OP_TIMEOUT_MS = 500; // avoid INFINITE waits so thread can recover
    k4a_capture_t capture = NULL;

    // device configuration
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30; // can be 5, 15, 30
    //config.camera_fps = K4A_FRAMES_PER_SECOND_5; // can be 5, 15, 30
    config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
    config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

    // Start capturing from the device
    if (k4a_device_start_cameras(*deviceCtx->device, &config) != K4A_RESULT_SUCCEEDED)
    {
        printf(RED "Failed to start capturing from the device" RESET);
    }

    // setup sensor calibration
    k4a_calibration_t sensor_calibration;
    if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(*deviceCtx->device, config.depth_mode, K4A_COLOR_RESOLUTION_OFF, &sensor_calibration))
    {
        printf(RED "Get depth camera calibration failed!\n" RESET);
    }

    // Set up tracker
    k4abt_tracker_t tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    if (K4A_RESULT_SUCCEEDED != k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker))
    {
        printf(RED "Body tracker initialization failed!\n" RESET);
    }

    printf(GRN "Device %d: Start processing\n" RESET, deviceIndex);

    // resilience parameters (per-thread)
    const int MAX_DEVICE_ERRORS = 10;           // accumulate timeouts/failures before attempting restart
    const int MAX_REOPEN_ATTEMPTS = 5;          // how many times to try re-open by serial before giving up
    int deviceErrorCount = 0;
    int restartAttempts = 0;

    // Helper: attempt to re-open the device by serial and restart cameras/tracker.
    auto reopenDeviceBySerial = [&](const std::string& serial, bool forceReplace = false) -> bool {
        // If context already has the correct device serial and a valid handle, nothing to do.
        {
            std::lock_guard<std::mutex> lk(deviceCtx->mutex);
            if (!forceReplace && deviceCtx->device && *deviceCtx->device && deviceCtx->serial == serial) {
                printf(GRN "Device %d: context already holds device with matching serial '%s'\n" RESET, deviceIndex, serial.c_str());
                return true;
            }
        }

        // If forcing replacement, destroy tracker, swap & release old handle, and wait for OS re-enumeration.
        std::shared_ptr<k4a_device_t> oldHandleForRelease;
        if (forceReplace) {
            printf(YEL "Device %d: Force-replace requested: destroying tracker and releasing old device handle before reopen\n" RESET, deviceIndex);

            // Destroy local tracker so it won't touch device while we release it.
            if (tracker) {
                k4abt_tracker_destroy(tracker);
                tracker = NULL;
            }

            // Swap out old handle under lock; release outside the lock.
            {
                std::lock_guard<std::mutex> lk(deviceCtx->mutex);
                oldHandleForRelease = deviceCtx->device;
                deviceCtx->device.reset();
                deviceCtx->serial.clear();
            }

            // Release outside lock with defensive catch.
            releaseHandleSafely(oldHandleForRelease, deviceIndex, "reopen-old-handle");

            // Wait for OS/driver to re-enumerate the USB device before trying to open.
            // Poll installed device count for a short window.
            const int WAIT_MS = 3000;
            const int POLL_MS = 200;
            int waited = 0;
            while (waited < WAIT_MS) {
                int installed = k4a_device_get_installed_count();
                if (installed > 0) break; // some device(s) present; proceed to scanning
                std::this_thread::sleep_for(std::chrono::milliseconds(POLL_MS));
                waited += POLL_MS;
            }
            printf(YEL "Device %d: waited %d ms for OS re-enumeration before reopen\n" RESET, deviceIndex, waited);
        }

        const int OPEN_RETRIES = 3;
        for (int attempt = 0; attempt < MAX_REOPEN_ATTEMPTS; ++attempt) {
            int installed = k4a_device_get_installed_count();
            printf(YEL "Device %d: Reopen attempt %d scanning %d installed devices\n" RESET, deviceIndex, attempt + 1, installed);

            for (int i = 0; i < installed; ++i) {
                k4a_device_t cand = NULL;
                k4a_result_t openRes = k4a_device_open(i, &cand);
                if (openRes != K4A_RESULT_SUCCEEDED || cand == NULL) {
                    printf(YEL "Device %d: k4a_device_open index=%d failed (res=%d)\n" RESET, deviceIndex, i, (int)openRes);
                    continue;
                }

                // small pause to let driver populate serial if needed
                std::this_thread::sleep_for(std::chrono::milliseconds(50));

                std::string candSN = get_device_serial(cand);
                if (candSN.empty()) {
                    printf(YEL "Device %d: index=%d reported empty serial (will close)\n" RESET, deviceIndex, i);
                    k4a_device_close(cand);
                    continue;
                }

                printf(GRN "Device %d: index=%d has serial '%s'\n" RESET, deviceIndex, i, candSN.c_str());

                if (candSN != serial) {
                    k4a_device_close(cand);
                    continue;
                }

                // Try a few times to start cameras — transient failures happen when USB stack is still settling.
                bool started = false;
                for (int r = 0; r < OPEN_RETRIES; ++r) {
                    if (k4a_device_start_cameras(cand, &config) == K4A_RESULT_SUCCEEDED) {
                        started = true;
                        break;
                    }
                    printf(YEL "Device %d: start_cameras failed on index=%d (retry %d)\n" RESET, deviceIndex, i, r + 1);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100 * (r + 1)));
                }
                if (!started) {
                    printf(RED "Device %d: Failed to start cameras on reopened device index %d\n" RESET, deviceIndex, i);
                    k4a_device_close(cand);
                    continue;
                }

                k4a_calibration_t newCalibration;
                if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(cand, config.depth_mode, K4A_COLOR_RESOLUTION_OFF, &newCalibration)) {
                    printf(RED "Device %d: Get calibration failed on reopened device (index=%d)\n" RESET, deviceIndex, i);
                    k4a_device_stop_cameras(cand);
                    k4a_device_close(cand);
                    continue;
                }

                k4abt_tracker_t newTracker = NULL;
                if (K4A_RESULT_SUCCEEDED != k4abt_tracker_create(&newCalibration, tracker_config, &newTracker)) {
                    printf(RED "Device %d: Tracker create failed on reopened device (index=%d)\n" RESET, deviceIndex, i);
                    k4a_device_stop_cameras(cand);
                    k4a_device_close(cand);
                    continue;
                }

                // Build a new DeviceHandle that will own `cand`. Do not close cand here.
                DeviceHandle newHandle = makeDeviceHandle(cand);

                // Swap the new handle and tracker into the context under lock.
                {
                    std::lock_guard<std::mutex> lk(deviceCtx->mutex);
                    deviceCtx->device = newHandle;
                    deviceCtx->serial = serial;
                }

                // Replace tracker: adopt new tracker
                tracker = newTracker;
                deviceSerialID = serial;
                deviceErrorCount = 0;
                restartAttempts = 0;
                printf(GRN "Device %d: Successfully reopened device and recreated tracker (index=%d)\n" RESET, deviceIndex, i);
                return true;
            }

            // exponential backoff for next scan (give longer time for OS)
            int backoff = 500 * (1 << attempt);
            std::this_thread::sleep_for(std::chrono::milliseconds(backoff));
        }
        return false;
    };

    // run forever
    try {
        while (1)
        {
            // Signal watchdog that this thread is alive
            if (g_watchdog) {
                Watchdog::heartbeat(deviceIndex);
            }

            // Stress test: simulate hang after N frames
            if (stressHangAfterFrames > 0 && captureFrameCount >= stressHangAfterFrames) {
                printf(YEL "[STRESS TEST] Device %d: Simulating hang at frame %d (infinite sleep)\n" RESET, deviceIndex, captureFrameCount);
                while (1) {
                    Sleep(60000);  // Sleep forever - watchdog should detect and restart if configured
                }
            }

            k4a_image_t image;
            //printf("Device: %d, Frame: %d\n", deviceIndex, captureFrameCount);
            captureFrameCount++;
            if (captureFrameCount == 66534) {
                captureFrameCount = 0;
            }

            // Try to get a capture with reasonable timeout so thread can detect problems
            // Acquire a local shared handle for the current iteration (protected by mutex).
            DeviceHandle localHandle;
            {
                std::lock_guard<std::mutex> lk(deviceCtx->mutex);
                localHandle = deviceCtx->device;
            }
            if (!localHandle || !(*localHandle)) {
                printf(YEL "Device %d: no valid device handle (will attempt reopen)\n" RESET, deviceIndex);
                ++deviceErrorCount;
                if (deviceErrorCount >= MAX_DEVICE_ERRORS) {
                    if (!reopenDeviceBySerial(deviceSerialID, true)) {
                        printf(RED "Device %d: Reopen by serial failed - exiting thread\n" RESET, deviceIndex);
                        break;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            k4a_wait_result_t captureResult = k4a_device_get_capture(*localHandle, &capture, TIMEOUT_IN_MS);
            if (captureResult == K4A_WAIT_RESULT_SUCCEEDED) {
                // Good - reset error counter
                deviceErrorCount = 0;
            } else if (captureResult == K4A_WAIT_RESULT_TIMEOUT) {
                printf(YEL "Device %d: get_capture timed out\n" RESET, deviceIndex);
                ++deviceErrorCount;
                if (capture != NULL) { k4a_capture_release(capture); capture = NULL; }
                // try to recover if too many errors
                if (deviceErrorCount >= MAX_DEVICE_ERRORS) {
                    ++restartAttempts;
                    printf(YEL "Device %d: Exceeded max device errors, attempting camera restart\n" RESET, deviceIndex);
                    // First try to stop/start cameras and recreate tracker locally
                    if (!reopenDeviceBySerial(deviceSerialID, true)) {
                        printf(RED "Device %d: Reopen by serial failed\n" RESET, deviceIndex);
                        // If reopen failed after attempts, exit thread (cleanly)
                        break;
                    }
                }
                continue; // go to next loop iteration
            } else { // K4A_WAIT_RESULT_FAILED
                printf(RED "Device %d: get_capture failed\n" RESET, deviceIndex);
                ++deviceErrorCount;
                if (capture != NULL) { k4a_capture_release(capture); capture = NULL; }
                if (deviceErrorCount >= MAX_DEVICE_ERRORS) {
                    ++restartAttempts;
                    if (!reopenDeviceBySerial(deviceSerialID, true)) {
                        printf(RED "Device %d: Failed to recover after failures - exiting thread\n" RESET, deviceIndex);
                        break;
                    }
                }
                continue;
            }

            // get capture to tracker - use finite timeout so thread can be responsive to failures
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, capture, TRACKER_OP_TIMEOUT_MS);
            if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
            {
                printf(RED "Device %d: Error! Adding capture to tracker process queue failed!\n" RESET, deviceIndex);
                ++deviceErrorCount;
                if (capture != NULL) { k4a_capture_release(capture); capture = NULL; }
                if (deviceErrorCount >= MAX_DEVICE_ERRORS) {
                    ++restartAttempts;
                    if (!reopenDeviceBySerial(deviceSerialID, true)) {
                        printf(RED "Device %d: Failed to recover after enqueue failures - exiting thread\n" RESET, deviceIndex);
                        break;
                    }
                }
                continue;
            } else if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT) {
                printf(YEL "Device %d: enqueue_capture timed out\n" RESET, deviceIndex);
                ++deviceErrorCount;
                if (capture != NULL) { k4a_capture_release(capture); capture = NULL; }
                if (deviceErrorCount >= MAX_DEVICE_ERRORS) {
                    ++restartAttempts;
                    if (!reopenDeviceBySerial(deviceSerialID, true)) {
                        printf(RED "Device %d: Failed to recover after enqueue timeouts - exiting thread\n" RESET, deviceIndex);
                        break;
                    }
                }
                continue;
            }

            // get body frame from tracker
            k4abt_frame_t body_frame = NULL;


            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, TRACKER_OP_TIMEOUT_MS);
            if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
            {
                // Successfully found a body tracking frame 
                deviceErrorCount = 0; // success - reset counts
                size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);

                // for each found body
                for (size_t bodyCounter = 0; bodyCounter < num_bodies; bodyCounter++)
                {
                    k4abt_skeleton_t skeleton;
                    k4abt_frame_get_body_skeleton(body_frame, bodyCounter, &skeleton);
                    uint32_t id = k4abt_frame_get_body_id(body_frame, bodyCounter);

                    // Create a packet/string for whole skeleton (byte array)
                    std::vector<uint8_t> packet;
                    std::string skeletonString;

                    // Add DEVICEID as a header
                    packet.push_back((deviceID >> 0) & 0xFF); // LSB
                    packet.push_back((deviceID >> 8) & 0xFF); // MSB

                    // Add a second time to match header (does nothing really)
                    packet.push_back((deviceID >> 0) & 0xFF);
                    packet.push_back((deviceID >> 8) & 0xFF);

                    // LKTODO create a variable e.g. DoSendMessage that is assumed true
                    bool DoSendMessage = true;

                    if (RECORDTIMESTAMPS) {
                        if (deviceSerialID == "") {
                            // best-effort: get serial from context
                            std::lock_guard<std::mutex> lk(deviceCtx->mutex);
                            deviceSerialID = deviceCtx->serial;
                        }
                        skeletonString = "RawPoseEstimation," + std::to_string(deviceID) + "," + deviceSerialID + "," + std::to_string(captureFrameCount) + "," + std::to_string(bodyCounter);
                    }

                    // for each joint in the found body
                    for (uint32_t jointCounter = 0; jointCounter < 32; jointCounter++)
                    {
                        //LKTODO if (skeleton.joints[jointCounter].position.xyz.z > 2800) {
                        if (skeleton.joints[jointCounter].position.xyz.z > 4400) {
                            DoSendMessage = false;
                        }

                        int integers[2] = {
                            static_cast<int>(jointCounter),
                            skeleton.joints[jointCounter].confidence_level
                        };

                        // Add integer bytes. Only two bytes per integer -> up to 65,535 
                        for (int i = 0; i < 2; ++i) {
                            packet.push_back((integers[i] >> 0) & 0xFF); // LSB
                            packet.push_back((integers[i] >> 8) & 0xFF); // MSB
                        }

                        float floats[7] = {
                            skeleton.joints[jointCounter].position.xyz.x,
                            skeleton.joints[jointCounter].position.xyz.y,
                            skeleton.joints[jointCounter].position.xyz.z,
                            skeleton.joints[jointCounter].orientation.wxyz.x,
                            skeleton.joints[jointCounter].orientation.wxyz.y,
                            skeleton.joints[jointCounter].orientation.wxyz.z,
                            skeleton.joints[jointCounter].orientation.wxyz.w // TODO, SHOULD BE WXYZ. 
                            //Flip these when demos slow down.
                            //https://microsoft.github.io/Azure-Kinect-Body-Tracking/release/1.1.x/structk4a__quaternion__t_1_1__wxyz.html#details
                        };

                        // Add half-float bytes
                        for (int i = 0; i < 7; ++i) {
                            uint16_t halfFloat = floatToHalf(floats[i]);
                            for (int j = 0; j < sizeof(halfFloat); ++j) {
                                packet.push_back((halfFloat >> (j * 8)) & 0xFF);
                            }
                        }

                        if (RECORDTIMESTAMPS) {
                            skeletonString +=
                                std::to_string(jointCounter) + "," +
                                std::to_string(skeleton.joints[jointCounter].confidence_level) + "," +
                                std::to_string(skeleton.joints[jointCounter].position.xyz.x) + "," +
                                std::to_string(skeleton.joints[jointCounter].position.xyz.y) + "," +
                                std::to_string(skeleton.joints[jointCounter].position.xyz.z) + "," +
                                std::to_string(skeleton.joints[jointCounter].orientation.wxyz.x) + "," +
                                std::to_string(skeleton.joints[jointCounter].orientation.wxyz.y) + "," +
                                std::to_string(skeleton.joints[jointCounter].orientation.wxyz.z) + "," +
                                std::to_string(skeleton.joints[jointCounter].orientation.wxyz.w);
                            if (jointCounter != 31) skeletonString += ","; // Add comma except after last joint
                        }

                        if (jointCounter == 0) {
                            char str[BUFFER_SIZE];
                            snprintf(str, sizeof(str), "%d ", deviceID); // include a space after the deviceID
                            printf(str);
                        }
                    }

                    if (DoSendMessage) {
                        if (g_zenoh && g_zenoh->isActive()) {
                            if (!g_zenoh->publish(packet)) {
                                std::cerr << "[Zenoh] publish failed for skeleton packet\n";
                            }
                        }
                    }

                    if (RECORDTIMESTAMPS) {
                        // Write the skeleton string to the log file
                        writeToLog(outputFile, outputFileMutex, skeletonString);
                        skeletonString = ""; // Reset the string for the next body
                    }
                }

                // release the body frame once you finish using it
                k4abt_frame_release(body_frame);
            }
            else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT) {
                // tracker didn't produce a frame in time; count toward errors but keep running
                printf(YEL "Device %d: tracker pop timed out\n" RESET, deviceIndex);
                ++deviceErrorCount;
                if (deviceErrorCount >= MAX_DEVICE_ERRORS) {
                    ++restartAttempts;
                    if (!reopenDeviceBySerial(deviceSerialID, true)) {
                        printf(RED "Device %d: Failed to recover after tracker timeouts - exiting thread\n" RESET, deviceIndex);
                        break;
                    }
                }
            }
            else { // K4A_WAIT_RESULT_FAILED
                printf(RED "Device %d: tracker pop failed\n" RESET, deviceIndex);
                ++deviceErrorCount;
                if (deviceErrorCount >= MAX_DEVICE_ERRORS) {
                    ++restartAttempts;
                    if (!reopenDeviceBySerial(deviceSerialID, true)) {
                        printf(RED "Device %d: Failed to recover after tracker failures - exiting thread\n" RESET, deviceIndex);
                        break;
                    }
                }
            }

            // release capture
            if (capture != NULL) {
                k4a_capture_release(capture);
                capture = NULL;
            }
            fflush(stdout);
        } // end while
    }
    catch (const std::exception& ex) {
        printf(RED "Device %d: Caught std::exception in worker: %s\n" RESET, deviceIndex, ex.what());
    }
    catch (...) {
        printf(RED "Device %d: Caught unknown exception in worker\n" RESET, deviceIndex);
    }

    // Deterministic shutdown: destroy tracker first, then release device handle.
    // Swap the device handle out under lock and release it outside the lock so the
    // device deleter runs without holding the context mutex (and after the tracker is gone).
    auto safeShutdown = [&](k4abt_tracker_t &trk) {
        if (trk) {
            k4abt_tracker_destroy(trk);
            trk = NULL;
        }

        DeviceHandle oldHandle;
        {
            std::lock_guard<std::mutex> lk(deviceCtx->mutex);
            oldHandle.swap(deviceCtx->device); // deviceCtx->device is now empty
        }

        // Release outside lock; catch exceptions from deleter to avoid unwinding through destructors.
        try {
            if (oldHandle) {
                oldHandle.reset();
            }
        } catch (const std::exception& e) {
            printf(YEL "Device %d: Exception during device cleanup: %s\n" RESET, deviceIndex, e.what());
        } catch (...) {
            printf(YEL "Device %d: Unknown exception during device cleanup\n" RESET, deviceIndex);
        }
    };

    safeShutdown(tracker);

    printf("Exit\n");
}

// Function to pack an int into the byte array
void packInt(std::vector<uint8_t>& packet, int value) {
    uint32_t networkValue = htonl(value); // convert to network byte order (big-endian)
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&networkValue);
    packet.insert(packet.end(), bytes, bytes + sizeof(networkValue));
}

// Function to pack a half-float (16-bit float) into the byte array
void packHalfFloat(std::vector<uint8_t>& packet, float value) {
    uint16_t halfFloat = floatToHalf(value); // Convert float to half-float (16-bit)
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&halfFloat);
    packet.insert(packet.end(), bytes, bytes + sizeof(halfFloat)); // Append to packet
}

uint16_t floatToHalf(float value) {
    uint32_t floatBits = *(uint32_t*)&value; // Get the float bits
    uint32_t sign = (floatBits >> 16) & 0x8000; // Get sign bit
    uint32_t exponent = (floatBits >> 23) & 0xFF; // Get exponent bits
    uint32_t mantissa = floatBits & 0x7FFFFF; // Get mantissa bits

    // Handle special cases
    if (exponent == 255) { // NaN or infinity
        return sign | 0x7FFF; // Return half-float NaN
    }
    if (exponent == 0) { // Zero or denormalized number
        return sign; // Return zero or signed zero
    }

    // Adjust exponent for half-float
    exponent -= 112; // 127 - 15
    if (exponent >= 31) { // Too large for half-float
        return sign | 0x7C00; // Set as infinity
    }
    if (exponent <= 0) { // Too small for half-float
        if (exponent < -10) return sign; // Too small to be represented
        mantissa |= 0x800000; // Implicit leading bit
        int shift = 14 - exponent; // Calculate shift
        mantissa >>= shift; // Shift to fit into half-float
        return sign | (uint16_t)(mantissa);
    }

    mantissa >>= 13; // Scale down mantissa
    return sign | (exponent << 10) | (uint16_t)(mantissa);
}

// Provide definition for the class static helper declared in ProcessDevice.h
uint16_t JointFinder::floatToHalf(float value)
{
    return ::floatToHalf(value);
}