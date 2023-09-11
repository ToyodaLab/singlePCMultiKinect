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

// for UDP
#include<winsock2.h>
#include <Ws2tcpip.h>
#pragma comment(lib,"ws2_32.lib") //Winsock Library

#define BUFLEN 512	//Max length of buffer
#define PORT 8888	//The port on which to listen for incoming data


// will be used to do join tracking
class JointFinder {
public:
    void DetectJoints(int deviceIndex, k4a_device_t openedDevice) {

        printf("Detecting joints in %d\n", deviceIndex);
        uint32_t deviceID = deviceIndex;

        int captureFrameCount = 2;
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
        
        // Following code to create recording

        char path[100]; // Define a character array to hold the file path
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
        
        // to create the recording
        //k4a_record_t toRecordTo = nullptr;
        //k4a_record_create(path, openedDevice, config, &toRecordTo);
        //k4a_record_write_header(toRecordTo);


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

            // get capture from tracker
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
                // Successfully popped the body tracking result
                printf("Capture body");
                size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
                // for each found body
                for (size_t i = 0; i < num_bodies; i++)
                {
                    k4abt_skeleton_t skeleton;
                    k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
                    uint32_t id = k4abt_frame_get_body_id(body_frame, i);
                    // for each joint in the found skeleton
                    for (uint32_t j = 0; j < 31; j++)
                    {
                        printf(" %d | Joint %d position x:%f y:%f z:%f\n",
                            deviceID,
                            j,
                            skeleton.joints[j].position.xyz.x,
                            skeleton.joints[j].position.xyz.y,
                            skeleton.joints[j].position.xyz.z
                        );
                    }
                }
                // release the body frame once you finish using it
                k4abt_frame_release(body_frame); 
            }

            //// Probe for a color image
            //image = k4a_capture_get_color_image(capture);
            //if (image)
            //{
            //    printf(" %d | Color res:%4dx%4d stride:%5d ",
            //        deviceID,
            //        k4a_image_get_height_pixels(image),
            //        k4a_image_get_width_pixels(image),
            //        k4a_image_get_stride_bytes(image));
            //    k4a_image_release(image);
            //    
            //    k4a_record_write_capture(toRecordTo, capture);
            //}
            //else
            //{
            //    printf(" | Color None                       ");
            //}

            //// probe for a IR16 image
            //image = k4a_capture_get_ir_image(capture);
            //if (image != NULL)
            //{
            //    printf(" | Ir16 res:%4dx%4d stride:%5d ",
            //        k4a_image_get_height_pixels(image),
            //        k4a_image_get_width_pixels(image),
            //        k4a_image_get_stride_bytes(image));
            //    k4a_image_release(image);
            //}
            //else
            //{
            //    printf(" | Ir16 None                       ");
            //}

            //// Probe for a depth16 image
            //image = k4a_capture_get_depth_image(capture);
            //if (image != NULL)
            //{
            //    printf(" | Depth16 res:%4dx%4d stride:%5d\n",
            //        k4a_image_get_height_pixels(image),
            //        k4a_image_get_width_pixels(image),
            //        k4a_image_get_stride_bytes(image));
            //    k4a_image_release(image);
            //}
            //else
            //{
            //    printf(" | Depth16 None\n");
            //}

            // release capture
            k4a_capture_release(capture);
            //fflush(stdout);
        }
    Exit:
        printf("Exit\n");
        k4abt_tracker_destroy(tracker);

        //for record
        //k4a_record_flush(toRecordTo);
        //k4a_record_close(toRecordTo);


        //k4a_device_close(openedDevice);
    }
};

class PacketSender {
    void sendPacket() {
        while (1) {

        }
    };
};

int main()
{

    //// create UDP server
    //SOCKET socketToTransmit;
    //struct sockaddr_in server, si_other;
    //int sendStringLength;
    //char stringBuffer[BUFLEN];
    //WSADATA wsa;

    //char message[BUFLEN];

    //sendStringLength = sizeof(si_other);

    ////Initialise winsock
    //printf("\nInitialising Winsock...");
    //if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    //{
    //    printf("Failed. Error Code : %d", WSAGetLastError());
    //    exit(EXIT_FAILURE);
    //}
    //printf("Initialised.\n");

    ////Create a socket
    //if ((socketToTransmit = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
    //{
    //    printf("Could not create socket : %d", WSAGetLastError());
    //}
    //printf("Socket created.\n");


    ////Prepare the sockaddr_in structure
    //server.sin_family = AF_INET;
    //server.sin_addr.s_addr = INADDR_ANY;
    //server.sin_port = htons(PORT);

    //BOOL bOptVal = FALSE;
    //int bOptLen = sizeof(BOOL);
    //setsockopt(socketToTransmit, SOL_SOCKET, SO_REUSEADDR, (char*) &bOptVal, bOptLen);

    ////Bind
    //if (bind(socketToTransmit, (struct sockaddr*)&server, sizeof(server)) == SOCKET_ERROR)
    //{
    //    printf("Bind failed with error code : %d", WSAGetLastError());
    //    exit(EXIT_FAILURE);
    //}
    //printf("Bind done");

    const char* pkt = "Message to be sent\n\0";
    const char* srcIP = "127.0.0.1";  
    const char* destIP = "127.0.0.1";
    sockaddr_in dest;
    sockaddr_in local;
    WSAData data;
    WSAStartup(MAKEWORD(2, 2), &data);

    local.sin_family = AF_INET;
    inet_pton(AF_INET, srcIP, &local.sin_addr.s_addr);
    local.sin_port = htons(0);

    dest.sin_family = AF_INET;
    inet_pton(AF_INET, destIP, &dest.sin_addr.s_addr);
    dest.sin_port = htons(PORT);

    SOCKET socketToTransmit = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    bind(socketToTransmit, (sockaddr*)&local, sizeof(local));


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
        JointFinder firstJoint;
        std::cerr << "Succesfully opened the first Kinect Azure device" << std::endl;
        workers.push_back(std::thread{ &JointFinder::DetectJoints, &firstJoint, device1ID, device1 });
    }

    // TODO: put the below in a loop rather than hardcoding 1 + 2

    // Open the second Kinect Azure device
    k4a_device_t device2 = nullptr;
    if (k4a_device_open(device2ID, &device2) != K4A_RESULT_SUCCEEDED)
    {
        std::cerr << "Failed to open the second Kinect Azure device" << std::endl;
        return 1;
    }
    else {
        JointFinder secondJoint;
        std::cerr << "Succesfully opened the second Kinect Azure device" << std::endl;
        workers.push_back(std::thread{ &JointFinder::DetectJoints, &secondJoint, device2ID, device2 });
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

    for(int i = 0; i < 100; i++){
        sendto(socketToTransmit, pkt, BUFLEN, 0, (sockaddr*)&dest, sizeof(dest));
        printf(pkt);
    }

    //while (1)
    //{
    //    printf("Waiting for data...");
    //    fflush(stdout);

    //    //clear the buffer by filling null, it might have previously received data
    //    memset(stringBuffer, '\0', BUFLEN);

    //    //try to receive some data, this is a blocking call
    //    //if ((recieveStringLength = recvfrom(socketToTransmit, stringBuffer, BUFLEN, 0, (struct sockaddr*)&si_other, &sendStringLength)) == SOCKET_ERROR)
    //    //{
    //    //    printf("recvfrom() failed with error code : %d", WSAGetLastError());
    //    //    exit(EXIT_FAILURE);
    //    //}

    //    //print details of the client/peer and the data received
    //    //printf("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
    //    printf("Data: %s\n", stringBuffer);

    //    strcpy_s(stringBuffer, "Testmessage\0");

    //    //now reply the client with the same data
    //    if (sendto(socketToTransmit, stringBuffer, strlen(stringBuffer), 0, (struct sockaddr*)&si_other, sendStringLength) == SOCKET_ERROR)
    //    {
    //        printf("sendto() failed with error code : %d", WSAGetLastError());
    //        exit(EXIT_FAILURE);
    //    }
    //}
       
    // Stop and close the sockets when done
    closesocket(socketToTransmit);
    WSACleanup();

    // Stop and close the devices when done
    k4a_device_stop_cameras(device1);
    k4a_device_close(device1);

    k4a_device_stop_cameras(device2);
    k4a_device_close(device2);

    return 0;
}

