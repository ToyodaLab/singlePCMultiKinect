#include "ProcessDevice.h"

#include <iostream>
#include <vector>
#include <cstring>
#include <cstdint>
#include "winsock.h"
#pragma comment(lib, "Ws2_32.lib")

#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define MAG   "\x1B[35m"
#define RESET "\x1B[0m"

#define BUFFER_SIZE 1024 // kept for internal formatting/log buffers if needed

// Each Kinect is a class JointFinder.
// Removed TCP socket param; we only publish via Zenoh now.
void JointFinder::DetectJoints(int deviceIndex, k4a_device_t openedDevice,
    std::ofstream& outputFile, std::mutex& outputFileMutex,
    bool RECORDTIMESTAMPS,
    int stressHangAfterFrames /* = 0 */)
{
    printf(GRN "Detecting joints in %d\n" RESET, deviceIndex);

    uint32_t deviceID = deviceIndex;
    std::string deviceSerialID = get_device_serial(openedDevice);

    int captureFrameCount = 0;
    const int32_t TIMEOUT_IN_MS = 1000;
    k4a_capture_t capture = NULL;

    // device configuration
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_5; // can be 5, 15, 30
    //config.camera_fps = K4A_FRAMES_PER_SECOND_5; // can be 5, 15, 30
    config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
    config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

    // Start capturing from the device
    if (k4a_device_start_cameras(openedDevice, &config) != K4A_RESULT_SUCCEEDED)
    {
        printf(RED "Failed to start capturing from the device" RESET);
    }

    // setup sensor calibration
    k4a_calibration_t sensor_calibration;
    if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(openedDevice, config.depth_mode, K4A_COLOR_RESOLUTION_OFF, &sensor_calibration))
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

    // run forever
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
                Sleep(60000);  // Sleep forever - watchdog should detect and restart
            }
        }

        k4a_image_t image;
        //printf("Device: %d, Frame: %d\n", deviceIndex, captureFrameCount);
        captureFrameCount++;
        if (captureFrameCount == 66534) {
            captureFrameCount = 0;
        }

        switch (k4a_device_get_capture(openedDevice, &capture, TIMEOUT_IN_MS))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read a capture\n");
            k4a_device_stop_cameras(openedDevice);
            if (capture != NULL) { k4a_capture_release(capture); }
            if (k4a_device_start_cameras(openedDevice, &config) != K4A_RESULT_SUCCEEDED) {
                printf("Failed to restart capturing from the device");
            }
            goto Exit;
        }

        // get capture to tracker
        k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, capture, K4A_WAIT_INFINITE);
        //k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, capture, 0); // Was set to 0
        if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
        {
            printf("Error! Adding capture to tracker process queue failed!\n");
            break;
        }

        // get body frame from tracker
        k4abt_frame_t body_frame = NULL;


        k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
        //k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, 0); // Was set to 0
        if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // Successfully found a body tracking frame 
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
                        deviceSerialID = get_device_serial(openedDevice);
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
                        snprintf(str, sizeof(str), "%d, %d, %d, %d, %d, %.2f, %.2f, %.2f",
                            captureFrameCount,
                            deviceID,
                            bodyCounter,
                            jointCounter,
                            skeleton.joints[jointCounter].confidence_level,
                            skeleton.joints[jointCounter].position.xyz.x,
                            skeleton.joints[jointCounter].position.xyz.y,
                            skeleton.joints[jointCounter].position.xyz.z
                        );
                        printf(str);
                        std::cout << std::endl;
                    }
                }

                if (DoSendMessage) {
                    if (g_zenoh && g_zenoh->isActive()) {
                        printf("ZenohPrint: ");

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

        // release capture
        k4a_capture_release(capture);
        fflush(stdout);
    }
Exit:
    printf("Exit\n");
    k4abt_tracker_destroy(tracker);
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