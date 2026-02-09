#include "ProcessDevice.h"
#include "ComInit.h" // RAII wrapper for COM initialization
#pragma comment(lib, "ole32.lib").
#include <iostream>
#include <vector>
#include <cstring>
#include <cstdint>
#include <thread>
#include <chrono>
#include <atomic>
#include "winsock.h"
#pragma comment(lib, "Ws2_32.lib")

#include <objbase.h> // CoInitializeEx / CoUninitialize for COM initialization on worker threads
#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define MAG   "\x1B[35m"
#define RESET "\x1B[0m"

#define BUFFER_SIZE 1024

static void releaseHandleSafely(DeviceHandle h, int deviceIndex, const char* reason)
{
    try {
        if (h) {
            h.reset();
        }
    } catch (...) {
        printf(YEL "Device %d: Exception while releasing handle (%s)\n" RESET, deviceIndex, reason);
    }
}

// Updated signature: added `deviceLabel` (caller-provided label such as "SN1").
void JointFinder::DetectJoints(int deviceIndex, std::shared_ptr<DeviceContext> deviceCtx,
    const std::string& deviceLabel,
    std::ofstream& outputFile, std::mutex& outputFileMutex,
    bool RECORDTIMESTAMPS,
    int stressHangAfterFrames /* = 0 */)
{
    printf(GRN "Detecting joints in %d\n" RESET, deviceIndex);

    uint32_t deviceID = deviceIndex;
    // Use the provided deviceLabel (e.g. "serial1") as the initial serial/label.
    // Fall back to context serial only if deviceLabel is empty.
    std::string deviceSerialID = deviceLabel;
    if (deviceSerialID.empty()) {
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
    config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
    config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

    // Attempt to start capturing from the device (worker expects a valid handle installed by supervisor)
    {
        std::lock_guard<std::mutex> lk(deviceCtx->mutex);
        if (!deviceCtx->device || !(*deviceCtx->device)) {
            printf(YEL "Device %d: no device handle at worker start; exiting\n" RESET, deviceIndex);
            return;
        }
    }

    if (k4a_device_start_cameras(*deviceCtx->device, &config) != K4A_RESULT_SUCCEEDED)
    {
        printf(RED "Failed to start capturing from the device" RESET);
        // Let worker exit and allow supervisor to reopen/resume
        return;
    }

    // setup sensor calibration
    k4a_calibration_t sensor_calibration;
    if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(*deviceCtx->device, config.depth_mode, K4A_COLOR_RESOLUTION_OFF, &sensor_calibration))
    {
        printf(RED "Get depth camera calibration failed!\n" RESET);
        // Let worker exit and allow supervisor to reopen/resume
        k4a_device_stop_cameras(*deviceCtx->device);
        return;
    }

    // Set up tracker
    k4abt_tracker_t tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    if (K4A_RESULT_SUCCEEDED != k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker))
    {
        printf(RED "Body tracker initialization failed!\n" RESET);
        k4a_device_stop_cameras(*deviceCtx->device);
        return;
    }

    printf(GRN "Device %d: Start processing (label='%s')\n" RESET, deviceIndex, deviceSerialID.c_str());

    // resilience parameters (per-thread)
    const int MAX_DEVICE_ERRORS = 10;           // accumulate timeouts/failures before attempting restart
    int deviceErrorCount = 0;

    // Initialize COM for this worker thread using RAII helper.
    ComInit comInit;
    if (!comInit.ok()) {
        printf(YEL "Device %d: ComInit failed: 0x%08x\n" RESET, deviceIndex, (unsigned)comInit.hr);
    } else {
        printf(GRN "Device %d: ComInit OK\n" RESET, deviceIndex);
    }

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
            captureFrameCount++;
            if (captureFrameCount == 66534) {
                captureFrameCount = 0;
            }

            // Acquire a local shared handle for the current iteration (protected by mutex).
            DeviceHandle localHandle;
            {
                std::lock_guard<std::mutex> lk(deviceCtx->mutex);
                localHandle = deviceCtx->device;
            }
            if (!localHandle || !(*localHandle)) {
                printf(YEL "Device %d: no valid device handle (will exit worker; supervisor will reopen)\n" RESET, deviceIndex);
                break; // exit so supervisor can reopen and respawn
            }

            k4a_wait_result_t captureResult = k4a_device_get_capture(*localHandle, &capture, TIMEOUT_IN_MS);
            if (captureResult == K4A_WAIT_RESULT_SUCCEEDED) {
                // Good - reset error counter
                deviceErrorCount = 0;
            } else if (captureResult == K4A_WAIT_RESULT_TIMEOUT) {
                printf(YEL "Device %d: get_capture timed out\n" RESET, deviceIndex);
                ++deviceErrorCount;
                if (capture != NULL) { k4a_capture_release(capture); capture = NULL; }
                // try to recover if too many errors -> exit and let supervisor reopen
                if (deviceErrorCount >= MAX_DEVICE_ERRORS) {
                    printf(YEL "Device %d: Exceeded max device errors, exiting worker to allow supervisor reopen\n" RESET, deviceIndex);
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue; // go to next loop iteration
            } else { // K4A_WAIT_RESULT_FAILED
                printf(RED "Device %d: get_capture failed\n" RESET, deviceIndex);
                ++deviceErrorCount;
                if (capture != NULL) { k4a_capture_release(capture); capture = NULL; }
                if (deviceErrorCount >= MAX_DEVICE_ERRORS) {
                    printf(RED "Device %d: Too many capture failures, exiting worker to allow supervisor reopen\n" RESET, deviceIndex);
                    break;
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
                    printf(RED "Device %d: Too many enqueue failures, exiting worker to allow supervisor reopen\n" RESET, deviceIndex);
                    break;
                }
                continue;
            } else if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT) {
                printf(YEL "Device %d: enqueue_capture timed out\n" RESET, deviceIndex);
                ++deviceErrorCount;
                if (capture != NULL) { k4a_capture_release(capture); capture = NULL; }
                if (deviceErrorCount >= MAX_DEVICE_ERRORS) {
                    printf(RED "Device %d: Too many enqueue timeouts, exiting worker to allow supervisor reopen\n" RESET, deviceIndex);
                    break;
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
                            // Print deviceID safely
                            printf("%d ", deviceID);
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
                    printf(YEL "Device %d: Too many tracker timeouts, exiting worker to allow supervisor reopen\n" RESET, deviceIndex);
                    break;
                }
            }
            else { // K4A_WAIT_RESULT_FAILED
                printf(RED "Device %d: tracker pop failed\n" RESET, deviceIndex);
                ++deviceErrorCount;
                if (deviceErrorCount >= MAX_DEVICE_ERRORS) {
                    printf(RED "Device %d: Too many tracker failures, exiting worker to allow supervisor reopen\n" RESET, deviceIndex);
                    break;
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
        } catch (...) {
            printf(YEL "Device %d: Exception during device cleanup\n" RESET, deviceIndex);
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