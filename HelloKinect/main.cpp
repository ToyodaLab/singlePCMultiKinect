// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <array>
#include <iostream>
#include <map>
#include <vector>
#include <k4arecord/playback.h>
#include <k4a/k4a.h>
#include <k4abt.h>
#include <thread>
#include <k4arecord/k4arecord_export.h>

#include <k4arecord/record.h>
#include <k4arecord/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// for file operations
#include <fstream> 


// for UDP
#include<winsock2.h>
#include <Ws2tcpip.h>
#include "main.h"
#pragma comment(lib,"ws2_32.lib") //Winsock Library

#define BUFFERLENGTH 512	//Max length of buffer
#define PORT 8888	//The port on which to listen for incoming data

// Toggle functions
//
bool OPENCAPTUREFRAMES = false;         // Open Capture Frames. typically set to false.
bool RECORDCAPTURESTOVIDEO = false;     // Record Captures To Video. records capture, not needed at the moment
bool RECORDTIMESTAMPS = false;          // Record Time Stamps. logs timestamps to outputFile
bool SENDJOINTSVIAUDP = true;           // Send Joints via UDP. Sets up sockets and sends data using UDP
//
//

const char* pkt = "Message to be sent\n";
sockaddr_in dest;

std::string textToWrite = "";

// Open a file for writing
std::ofstream outputFile("C:\\Temp\\output.txt");

const char* srcIP = "127.0.0.1";
const char* destIP = "127.0.0.1";


void writeToLog(LARGE_INTEGER end, LARGE_INTEGER start, LARGE_INTEGER frequency, const int32_t deviceID)
{
    // Stop the timer
    QueryPerformanceCounter(&end);

    // Calculate the elapsed time in microseconds
    double elapsedMicroseconds = static_cast<double>(end.QuadPart - start.QuadPart) / frequency.QuadPart * 1e6;

    // Print the elapsed time
    //printf("%.2lf\n", elapsedMicroseconds);

    // Convert the variable to a string 
    char variableStrForms[20]; // Buffer to hold the string representation of the number
    snprintf(variableStrForms, sizeof(variableStrForms), "%d", deviceID); // Convert the number to a string

    textToWrite += variableStrForms;    // Write the text to the file
    textToWrite += ",";    // Write the text to the file

    snprintf(variableStrForms, sizeof(variableStrForms), "%.2lf", elapsedMicroseconds); // Convert the number to a string

    textToWrite += variableStrForms;    // Write the text to the file
    textToWrite += "\n";    // Write the text to the file

    // Print to textfile
    outputFile << textToWrite;
    textToWrite = "";
}


// Each Kinect is a JointFinder.
class JointFinder {
public:
    void DetectJoints(int deviceIndex, k4a_device_t openedDevice, SOCKET boundSocket) {
        printf("Detecting joints in %d\n", deviceIndex);

        uint32_t deviceID = deviceIndex;

        int captureFrameCount = 25000;
        const int32_t TIMEOUT_IN_MS = 1000;
        printf("ok");

        k4a_capture_t capture = NULL;

        // device configuration
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30; // can be 5, 15, 30
        config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
        config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
        config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;

        // Start capturing from the device
        if (k4a_device_start_cameras(openedDevice, &config) != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Failed to start capturing from the Kinect Azure device" << std::endl;
        }
         

        // Following code sets up recording for this device
        char path[255]; // Define a character array to hold the file path
        k4a_record_t toRecordTo = nullptr;
        if (RECORDCAPTURESTOVIDEO) {

            int variable = deviceIndex; // Your variable

            // Construct the file path with the variable manually
            const char* directory = "C:\\Temp\\device"; // Replace with your directory
            const char* extension = ".mkv";
            strcpy_s(path, directory);

            // Convert the variable to a string 
            char variableStr[2]; // Buffer to hold the string representation of the number
            snprintf(variableStr, sizeof(variableStr), "%d", variable); // Convert the number to a string

            // Concatenate the strings
            strcat_s(path, variableStr);
            strcat_s(path, extension);

            // to create and start the recording
            k4a_record_create(path, openedDevice, config, &toRecordTo);
            k4a_record_write_header(toRecordTo);
        }

        // setup sensor calibration
        k4a_calibration_t sensor_calibration;
        if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(openedDevice, config.depth_mode, K4A_COLOR_RESOLUTION_OFF, &sensor_calibration))
        {
            printf("Get depth camera calibration failed!\n");
        }

        // Set up tracker
        k4abt_tracker_t tracker = NULL;
        k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
        if (K4A_RESULT_SUCCEEDED != k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker))
        {
            printf("Body tracker initialization failed!\n");
        }

        // run for defined number of frames
        while (captureFrameCount-- > 0)
        {
            k4a_image_t image;
            printf("Device: %d, Frame: %d\n", deviceIndex, captureFrameCount);
            // start timer
            LARGE_INTEGER frequency, start, end;
            if (RECORDTIMESTAMPS) {
                // Get the frequency of the performance counter
                QueryPerformanceFrequency(&frequency);

                // Start the timer
                QueryPerformanceCounter(&start);
            }

            // Get a depth frame
            switch (k4a_device_get_capture(openedDevice, &capture, TIMEOUT_IN_MS))
            {
            case K4A_WAIT_RESULT_SUCCEEDED:
                break;
            case K4A_WAIT_RESULT_TIMEOUT:
                printf("Timed out waiting for a capture\n");
                continue;
                break;
            case K4A_WAIT_RESULT_FAILED:
                printf("Failed to read a capture\n");
                goto Exit;
            }

            // get capture to tracker
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, capture, 0);
            if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
            {
                printf("Error! Adding capture to tracker process queue failed!\n");
                break;
            }

            // get body frame from tracker
            k4abt_frame_t body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, 0);
            if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
            {
                // Successfully found a body tracking frame
                printf("Frame found. Scanning for bodies...\n");
                size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);

                // for each found body
                for (size_t bodyCounter = 0; bodyCounter < num_bodies; bodyCounter++)
                {
                    k4abt_skeleton_t skeleton;
                    k4abt_frame_get_body_skeleton(body_frame, bodyCounter, &skeleton);
                    uint32_t id = k4abt_frame_get_body_id(body_frame, bodyCounter);

                    // for each joint in the found skeleton
                    for (uint32_t skeletonCounter = 0; skeletonCounter < 32; skeletonCounter++)
                    {
                        // create simple quaternion
                        k4a_quaternion_t currentJointQuaternion;
                        currentJointQuaternion.wxyz.w = skeleton.joints[skeletonCounter].orientation.wxyz.w;
                        currentJointQuaternion.wxyz.x = skeleton.joints[skeletonCounter].orientation.wxyz.x;
                        currentJointQuaternion.wxyz.y = skeleton.joints[skeletonCounter].orientation.wxyz.y;
                        currentJointQuaternion.wxyz.z = skeleton.joints[skeletonCounter].orientation.wxyz.z;

                        // change coordinate system via rotation around axis
                        // TODO change this transform depending on joint using getInverseQuaternion
                        k4a_quaternion_t transformedQuart = AngleAxis(90, currentJointQuaternion);

                        transformedQuart = multiplyQuaternion(skeleton.joints[skeletonCounter].orientation, currentJointQuaternion);

                        // convert quaternion to rotator
                        float thisPitch = Pitch(currentJointQuaternion);
                        float thisYaw = Yaw(currentJointQuaternion);
                        float thisRoll = Roll(currentJointQuaternion);

                        char str[BUFFERLENGTH];
                        snprintf(str, sizeof(str), "%d, %d, %d, %d, %f, %f, %f, %f, %f, %f",
                            deviceID,
                            bodyCounter,
                            skeletonCounter,
                            skeleton.joints[skeletonCounter].confidence_level,
                            skeleton.joints[skeletonCounter].position.xyz.x,
                            skeleton.joints[skeletonCounter].position.xyz.y,
                            skeleton.joints[skeletonCounter].position.xyz.z,
                            thisRoll, 
                            thisYaw, 
                            thisPitch
                        );

                        printf(str);
                        printf("\n");

                        if (SENDJOINTSVIAUDP) {
                            pkt = str;
                            sendto(boundSocket, pkt, BUFFERLENGTH, 0, (sockaddr*)&dest, sizeof(dest));
                        }
                    }
                }
                // release the body frame once you finish using it
                k4abt_frame_release(body_frame);

                if (RECORDTIMESTAMPS) {
                    // Stop the timer
                    QueryPerformanceCounter(&end);

                    writeToLog(end, start, frequency, deviceID);
                }
            }
            if (OPENCAPTUREFRAMES)
            {
                // Probe for a color image
                image = k4a_capture_get_color_image(capture);
                if (image)
                {
                    printf(" %d | Color res:%4dx%4d stride:%5d ",
                        deviceID,
                        k4a_image_get_height_pixels(image),
                        k4a_image_get_width_pixels(image),
                        k4a_image_get_stride_bytes(image));
                    k4a_image_release(image);

                    if (RECORDCAPTURESTOVIDEO) {
                        k4a_record_write_capture(toRecordTo, capture);
                    }
                }
                else
                {
                    printf(" | Color None                       ");
                }

                // probe for a IR16 image
                image = k4a_capture_get_ir_image(capture);
                if (image != NULL)
                {
                    printf(" | Ir16 res:%4dx%4d stride:%5d ",
                        k4a_image_get_height_pixels(image),
                        k4a_image_get_width_pixels(image),
                        k4a_image_get_stride_bytes(image));
                    k4a_image_release(image);
                }
                else
                {
                    printf(" | Ir16 None                       ");
                }

                // Probe for a depth16 image
                image = k4a_capture_get_depth_image(capture);
                if (image != NULL)
                {
                    printf(" | Depth16 res:%4dx%4d stride:%5d\n",
                        k4a_image_get_height_pixels(image),
                        k4a_image_get_width_pixels(image),
                        k4a_image_get_stride_bytes(image));
                    k4a_image_release(image);
                }
                else
                {
                    printf(" | Depth16 None\n");
                }
            }

            // release capture
            k4a_capture_release(capture);
            fflush(stdout);
        }
    Exit:
        printf("Exit\n");
        k4abt_tracker_destroy(tracker);

        if (RECORDCAPTURESTOVIDEO) {
            k4a_record_flush(toRecordTo);
            k4a_record_close(toRecordTo);
        }
    }

    float Pitch(k4a_quaternion_t quart)
    {
        float value1 = 2.0 * (quart.wxyz.w * quart.wxyz.x + quart.wxyz.y * quart.wxyz.z);
        float value2 = 1.0 - 2.0 * (quart.wxyz.x * quart.wxyz.x + quart.wxyz.y * quart.wxyz.y);

        float roll = atan2(value1, value2);

        return roll * (180.0 / 3.141592653589793116);
    }

    float Yaw(k4a_quaternion_t quart)
    {
        double value = +2.0 * (quart.wxyz.w * quart.wxyz.y - quart.wxyz.z * quart.wxyz.x);
        value = value > 1.0 ? 1.0 : value;
        value = value < -1.0 ? -1.0 : value;

        float pitch = asin(value);

        return pitch * (180.0 / 3.141592653589793116);
    }

    float Roll(k4a_quaternion_t quart)
    {
        float value1 = 2.0 * (quart.wxyz.w * quart.wxyz.z + quart.wxyz.x * quart.wxyz.y);
        float value2 = 1.0 - 2.0 * (quart.wxyz.y * quart.wxyz.y + quart.wxyz.z * quart.wxyz.z);

        float yaw = atan2(value1, value2);

        return yaw * (180.0 / 3.141592653589793116);
    }

    k4a_quaternion_t AngleAxis(float angle, k4a_quaternion_t axis) {

        // First normalise the axis
        float x, y, z;
        x = axis.wxyz.x; y = axis.wxyz.y; z = axis.wxyz.z;
        float magnitude = std::sqrt(x * x + y * y + z * z);

        // Check if the magnitude is not zero to avoid division by zero
        if (magnitude != 0) {
            x /= magnitude;
            y /= magnitude;
            z /= magnitude;
        }

        angle *= 0.0174532925f; // To radians!
        angle *= 0.5f;
        float sinAngle = sin(angle);

        // create and return the quaternion
        k4a_quaternion_t returnQuart;
        returnQuart.wxyz.w = cos(angle);
        returnQuart.wxyz.x = x * sinAngle;
        returnQuart.wxyz.y = y * sinAngle;
        returnQuart.wxyz.z = z * sinAngle;

        return returnQuart;
    }

    k4a_quaternion_t multiplyQuaternion(k4a_quaternion_t first, k4a_quaternion_t second) {
        k4a_quaternion_t returnQuart;
        returnQuart.wxyz.w = first.wxyz.w * second.wxyz.w - first.wxyz.x * second.wxyz.x - first.wxyz.y * second.wxyz.y - first.wxyz.z * second.wxyz.z;
        returnQuart.wxyz.x = first.wxyz.w * second.wxyz.x + first.wxyz.x * second.wxyz.w + first.wxyz.y * second.wxyz.z - first.wxyz.z * second.wxyz.y;
        returnQuart.wxyz.y = first.wxyz.w * second.wxyz.y - first.wxyz.x * second.wxyz.z + first.wxyz.y * second.wxyz.w + first.wxyz.z * second.wxyz.x;
        returnQuart.wxyz.z = first.wxyz.w * second.wxyz.z + first.wxyz.x * second.wxyz.y - first.wxyz.y * second.wxyz.x + first.wxyz.z * second.wxyz.w;
        return returnQuart;
    }

    /*
    k4a_quaternion_t getInverseQuaternion(int jointNumber) {
        switch (jointNumber) {
        case 0:	    //  PELVIS
        case 1:	    //	SPINE_NAVAL
        case 2:	    //	SPINE_CHEST
        case 3:	    //	NECK
        case 26:	//	HEAD
        case 18:	//	HIP_LEFT
        case 19:	//	KNEE_LEFT
        case 20:	//	ANKLE_LEFT
            //quart newAxis(0, 1, 0, 0);
            //quart newAxis2(0, 0, 0, 1);
            //newAxis = AngleAxis(90, newAxis) * AngleAxis(-90, newAxis);

        case 21:	//	FOOT_LEFT

        case 22:	//	HIP_RIGHT
        case 23:	//	KNEE_RIGHT
        case 24:	//	ANKLE_RIGHT

        case 25:	//	FOOT_RIGHT

        case 4:	    //	CLAVICLE_LEFT
        case 5:	    //	SHOULDER_LEFT
        case 6:	    //	ELBOW_LEFT

        case 7:	    //	WRIST_LEFT

        case 11:	//	CLAVICLE_RIGHT
        case 12:	//	SHOULDER_RIGHT
        case 13:	//	ELBOW_RIGHT


        case 14:	//	WRIST_RIGHT


        case 8:	    //	HAND_LEFT
        case 9:	    //	HANDTIP_LEFT
        case 10:	//	THUMB_LEFT
        case 15:	//	HAND_RIGHT
        case 16:	//	HANDTIP_RIGHT
        case 17:	//	THUMB_RIGHT
        case 27:	//	NOSE
        case 28:	//	EYE_LEFT
        case 29:	//	EAR_LEFT
        case 30:	//	EYE_RIGHT
        case 31:	//	EAR_RIGHT
        default:
            //quart newQ(1,0,0,0);
        }
        k4a_quaternion_t quartToReturn;
    }

    */
};


int main()
{
    LARGE_INTEGER start, end, frequency;

    if (RECORDTIMESTAMPS) {
        // Check if the file is open
        if (!outputFile.is_open()) {
            std::cerr << "Failed to open the file." << std::endl;
            return 1; // Return an error code
        }        
        
        // Get the frequency of the performance counter
        QueryPerformanceFrequency(&frequency);

        // Start the timer
        QueryPerformanceCounter(&start);

        writeToLog(start, start, frequency, -1);
    }

    SOCKET socketToTransmit = NULL;

    if (SENDJOINTSVIAUDP) {
        sockaddr_in local;
        WSAData data;
        WSAStartup(MAKEWORD(2, 2), &data);

        local.sin_family = AF_INET;
        inet_pton(AF_INET, srcIP, &local.sin_addr.s_addr);
        local.sin_port = htons(0);

        dest.sin_family = AF_INET;
        inet_pton(AF_INET, destIP, &dest.sin_addr.s_addr);
        dest.sin_port = htons(PORT);

        socketToTransmit = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        bind(socketToTransmit, (sockaddr*)&local, sizeof(local));

    }

    //worker threads
    std::vector<std::thread> workers;

    uint32_t device_count = k4a_device_get_installed_count();

    //Force number of devices For Debugging 
    //device_count = 4;

    printf("Found %d connected devices:\n", device_count);

    // Max devices is 4 (arbritrary)
    k4a_device_t devices[4] = { nullptr,nullptr,nullptr,nullptr };

    for (int devicesFoundCounter = 0; devicesFoundCounter < device_count; devicesFoundCounter++) {
        if (k4a_device_open(devicesFoundCounter, &devices[devicesFoundCounter]) != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Failed to open the Kinect Azure device" << std::endl;
            return 1;
        }
        else {
            JointFinder kinectJointFinder;
            std::cerr << "Succesfully opened a Kinect Azure device" << std::endl;
            // push_back adds to end of vector list
            workers.push_back(std::thread{ &JointFinder::DetectJoints, 
                &kinectJointFinder, 
                devicesFoundCounter,
                devices[devicesFoundCounter],
                socketToTransmit });
        }
    }

    for (int devicesFoundCounter = 0; devicesFoundCounter < device_count; devicesFoundCounter++) {
        try {
            workers[devicesFoundCounter].join();
        }
        catch (std::exception ex) {
            printf("join() error log : %s\n", ex.what());
            while (1);
        }
    }

    if (SENDJOINTSVIAUDP) {
        // Stop and close the socket when done
        closesocket(socketToTransmit);
        WSACleanup();
    }

    // Stop and close the devices when done
    for (int devicesFoundCounter = 0; devicesFoundCounter < device_count; devicesFoundCounter++) {
        
        k4a_device_stop_cameras(devices[devicesFoundCounter]);
        k4a_device_close(devices[devicesFoundCounter]);
    }

    if (RECORDTIMESTAMPS) {
        // End the timer
        QueryPerformanceCounter(&end);

        writeToLog(end, start, frequency, -1);

        // Close the file when done
        outputFile.close();
    }
    return 0;
}

