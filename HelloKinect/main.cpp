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

// will be used to do join tracking
class MyJoint {
public:
    void DetectJoint(int deviceIndex, k4a_device_t openedDevice) {

        printf("Detecting joints in %d\n", deviceIndex);
        uint32_t deviceID = deviceIndex;

        int captureFrameCount = 200;
        const int32_t TIMEOUT_IN_MS = 1000;
        k4a_capture_t capture = NULL;

        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
        config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

        // Start capturing from the  device
        if (k4a_device_start_cameras(openedDevice, &config) != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Failed to start capturing from the first Kinect Azure device" << std::endl;
        }
        
        char path[100]; // Define a character array to hold the file path
        int variable = deviceIndex; // Your variable

        // Construct the file path with the variable manually
        const char* directory = "C:\\Temp\\device"; // Replace with your directory
        const char* extension = ".mkv"; // Replace with your desired file extension

        // Copy the directory to the path
        strcpy_s(path, directory);

        // Convert the variable to a string (or use itoa if available)
        char variableStr[2]; // Buffer to hold the string representation of the number
        snprintf(variableStr, sizeof(variableStr), "%d", variable); // Convert the number to a string

        // Concatenate the variable string to the path
        strcat_s(path, variableStr);

        // Concatenate the file extension
        strcat_s(path, extension);
        
        k4a_record_t toRecordTo = nullptr;

        k4a_record_create(path, openedDevice, config, &toRecordTo);
        k4a_record_write_header(toRecordTo);

        while (captureFrameCount-- > 0)
        {
            k4a_image_t image;

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

            printf("Capture");

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
                
                k4a_record_write_capture(toRecordTo, capture);
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

            // release capture
            k4a_capture_release(capture);
            fflush(stdout);
        }
        Exit:
        if (openedDevice != NULL)
        {
            k4a_record_flush(toRecordTo);
            k4a_record_close(toRecordTo);
            //k4a_device_close(openedDevice);
        }
    }
};

int main()
{
    //worker threads
    std::vector<std::thread> workers;
    uint32_t device1ID = 0;
    uint32_t device2ID = 1;

    uint32_t device_count = k4a_device_get_installed_count();

    printf("Found %d connected devices:\n", device_count);

    // Open the first Kinect Azure device
    k4a_device_t device1 = nullptr;
    if (k4a_device_open(device1ID, &device1) != K4A_RESULT_SUCCEEDED)
    {
        std::cerr << "Failed to open the first Kinect Azure device" << std::endl;
        return 1;
    }
    else {
        MyJoint firstJoint;
        std::cerr << "Succesfully opened the first Kinect Azure device" << std::endl;
        workers.push_back(std::thread{ &MyJoint::DetectJoint, &firstJoint, device1ID, device1});
    }

    // Open the second Kinect Azure device
    k4a_device_t device2 = nullptr;
    if (k4a_device_open(device2ID, &device2) != K4A_RESULT_SUCCEEDED)
    {
        std::cerr << "Failed to open the second Kinect Azure device" << std::endl;
        return 1;
    }
    else {
        MyJoint secondJoint;
        std::cerr << "Succesfully opened the second Kinect Azure device" << std::endl;
        workers.push_back(std::thread{ &MyJoint::DetectJoint, &secondJoint, device2ID, device2 });
    }

    try {
        workers[0].join();
    }
    catch (std::exception ex) {
        printf("join() error log : %s\n", ex.what());
        while (1);
    }
    
    try {
        workers[1].join();
    }
    catch (std::exception ex) {
        printf("join() error log : %s\n", ex.what());
        while (1);
    }

    // Start capturing from the first device
    //if (k4a_device_start_cameras(device1, nullptr) != K4A_RESULT_SUCCEEDED)
    //{
    //    std::cerr << "Failed to start capturing from the first Kinect Azure device" << std::endl;
    //    return 1;
    //}

    //// Start capturing from the second device
    //if (k4a_device_start_cameras(device2, nullptr) != K4A_RESULT_SUCCEEDED)
    //{
    //    std::cerr << "Failed to start capturing from the second Kinect Azure device" << std::endl;
    //    return 1;
    //}

    //// Capture and process frames from both devices
    //while (true)
    //{
    //    k4a_capture_t capture1 = nullptr;
    //    if (k4a_device_get_capture(device1, &capture1, K4A_WAIT_INFINITE) != K4A_WAIT_RESULT_SUCCEEDED)
    //    {
    //        std::cerr << "Failed to get a capture from the first Kinect Azure device" << std::endl;
    //        return 1;
    //    }

    //    k4a_capture_t capture2 = nullptr;
    //    if (k4a_device_get_capture(device2, &capture2, K4A_WAIT_INFINITE) != K4A_WAIT_RESULT_SUCCEEDED)
    //    {
    //        std::cerr << "Failed to get a capture from the second Kinect Azure device" << std::endl;
    //        return 1;
    //    }

    //    // Process the captured frames from both devices here

    //    // Release the captures
    //    k4a_capture_release(capture1);
    //    k4a_capture_release(capture2);
    //}

    // Stop and close the devices when done
    k4a_device_stop_cameras(device1);
    k4a_device_close(device1);

    k4a_device_stop_cameras(device2);
    k4a_device_close(device2);

    return 0;
}

