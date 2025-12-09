// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include <array>
#include <iostream>
#include <vector>
#include <thread>
#include <string>
#include <k4a/k4a.h>                // For Kinect stuff
#include <k4abt.h>                  // For Kinect stuff
#include <stdio.h>                  // For string handling
#include <string.h>                 // For string handling
#include <fstream>                  // For file operations
#include <chrono>                   // For timestamps
#include<winsock2.h>                // For UDP / TCP
#include <Ws2tcpip.h>               // For UDP / TCP
#include <mutex> 			        // For thread safe logging         
#include <algorithm>
#include "main.h"

#pragma comment(lib,"ws2_32.lib")   //Winsock Library

int PORT = 8844;	//The port on which to listen for incoming data

#define MAX_CLIENTS 10
SOCKET clientSockets[MAX_CLIENTS]; // Array to hold client sockets
int clientCount = 0; // Current number of connected clients
CRITICAL_SECTION cs; // Critical section for thread safety

#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define MAG   "\x1B[35m"
#define RESET "\x1B[0m"

//File to write to
// Make sure directory exists or will error
//std::string logFilePath = "C:\\Temp\\tempCG\\23-07-25\\KinectLog.txt";
std::mutex outputFileMutex;

const char* pkt = "Message to be sent\n";
sockaddr_in dest;

SOCKET serverSocket, clientSocket;
#define BUFFER_SIZE 1024 //Max length of buffer

struct KinectDevice {
    k4a_device_t device;
    std::string serial_number;
    std::string name;
};

// Retrieve device serial number
std::string get_kinect_serial(k4a_device_t device) {
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

// Each Kinect is a class JointFinder.
class JointFinder {
public:
    void DetectJoints(int deviceIndex, k4a_device_t openedDevice, SOCKET boundSocket,
        std::ofstream& outputFile, std::mutex& outputFileMutex,
        bool RECORDTIMESTAMPS, bool OPENCAPTUREFRAMES, bool SENDJOINTSVIATCP)
    {
        printf(GRN "Detecting joints in %d\n" RESET, deviceIndex);

        uint32_t deviceID = deviceIndex;
        std::string deviceSerialID = get_kinect_serial(openedDevice);

        int captureFrameCount = 0;
        const int32_t TIMEOUT_IN_MS = 1000;
        k4a_capture_t capture = NULL;

        // device configuration
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30; // can be 5, 15, 30
        //config.camera_fps = K4A_FRAMES_PER_SECOND_5; // can be 5, 15, 30
        config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
        config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
        //config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
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
                printf("Timed out waiting for a capture. Restarting capture...\n");
                k4a_device_stop_cameras(openedDevice);
                if (capture != NULL)
                {
                    k4a_capture_release(capture);
                }
                if (k4a_device_start_cameras(openedDevice, &config) != K4A_RESULT_SUCCEEDED) {
                    printf("Failed to restart capturing from the device");
                    goto Exit;
                }
                continue;
            case K4A_WAIT_RESULT_FAILED:
                printf("Failed to read a capture\n");
                k4a_device_stop_cameras(openedDevice);
                if (capture != NULL)
                {
                    k4a_capture_release(capture);
                }
                if (k4a_device_start_cameras(openedDevice, &config) != K4A_RESULT_SUCCEEDED) {
                    printf("Failed to restart capturing from the device");
                    goto Exit;
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
                            deviceSerialID = get_kinect_serial(openedDevice);
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
                            jointCounter,
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
                                skeleton.joints[jointCounter].position.xyz.z/*,
                                // rotation is quart so not helpful to print
                                skeleton.joints[jointCounter].orientation.wxyz.w,*/
                            );
                            printf(str);
                            std::cout << std::endl;
                        }

                    }

                    if (SENDJOINTSVIATCP) {
                        if (DoSendMessage) {
                            // Broadcast message to all clients
                            EnterCriticalSection(&cs);
                            for (int i = 0; i < clientCount; i++) {
                                //if (clientSockets[i] != clientSocket) { // Don't send back to the sender
                                //Sends whole body as one packet

                                printf("Sending Packet size %zd: ", packet.size());

                                send(clientSockets[i], reinterpret_cast<const char*>(packet.data()), packet.size(), 0);
                            }
                            LeaveCriticalSection(&cs);
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
                }
                else
                {
                    printf(" | Color None");
                }
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
};

// Handle communication with the client
DWORD WINAPI ClientHandler(LPVOID lpParam) {
    SOCKET clientSocket = (SOCKET)lpParam;
    char buffer[BUFFER_SIZE];

    while (1) {
        memset(buffer, 0, BUFFER_SIZE); // Clear the buffer
        int bytesReceived = recv(clientSocket, buffer, BUFFER_SIZE, 0);
        if (bytesReceived == SOCKET_ERROR) {
            //fprintf(stderr, "\nReceive failed: %d\n", WSAGetLastError());
            printf(RED "\nReceive failed or sudden disconnect : %d\n" RESET, WSAGetLastError());
            break;
        }
        else if (bytesReceived == 0) {
            printf(RED "\nClient disconnected.\n" RESET);
            break;
        }

        // What is the recieved event
        int thisevent = (buffer[1] << 8) | buffer[0];
        // What is the total packet size
        int packetSendSize = (buffer[3] << 8) | buffer[2];

        // Create a new packet to broadcast
        std::vector<uint8_t> packetToTransmit;

        // Broadcast message to all clients
        EnterCriticalSection(&cs);
        for (int i = 0; i < clientCount; i++) {
            if (clientSockets[i] != clientSocket) { // Don't send back to the sender
                //send(clientSockets[i], buffer, bytesReceived, 0);
                send(clientSockets[i], reinterpret_cast<const char*>(buffer), packetSendSize, 0);
            }
        }
        LeaveCriticalSection(&cs);
    }

    // Remove the client socket from the list and close it
    EnterCriticalSection(&cs);
    for (int i = 0; i < clientCount; i++) {
        if (clientSockets[i] == clientSocket) {
            clientSockets[i] = clientSockets[--clientCount]; // Replace with last client
            break;
        }
    }
    LeaveCriticalSection(&cs);

    closesocket(clientSocket);
    return 0;
}

// Accept incoming connections in a separate thread
DWORD WINAPI AcceptConnections(LPVOID lpParam) {
    SOCKET serverSocket = (SOCKET)lpParam;
    SOCKET clientSocket;
    struct sockaddr_in clientAddr;
    int addrLen = sizeof(clientAddr);

    while (1) {
        clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &addrLen);
        if (clientSocket == INVALID_SOCKET) {
            fprintf(stderr, "Accept failed: %d\n", WSAGetLastError());
            continue; // Continue accepting other clients
        }

        printf("\n%sClient connected!%s\n", "\033[32m", "\033[0m");

        // Add the client socket to the list
        EnterCriticalSection(&cs);
        if (clientCount < MAX_CLIENTS) {
            clientSockets[clientCount++] = clientSocket; // Add the new client
        }
        else {
            printf(RED "\nMax clients reached. Connection refused.\n" RESET);
            closesocket(clientSocket); // Reject connection
        }
        LeaveCriticalSection(&cs);

        // Create a thread to handle the client
        HANDLE threadHandle = CreateThread(NULL, 0, ClientHandler, (LPVOID)clientSocket, 0, NULL);
        if (threadHandle == NULL) {
            fprintf(stderr, "Failed to create thread: %d\n", GetLastError());
            closesocket(clientSocket); // Close the socket if thread creation failed
        }
        else {
            CloseHandle(threadHandle); // Close the thread handle in the main thread
        }
    }
    return 0;
}

// Load the desired order of devices from a file
std::vector<std::string> LoadDesiredOrder(const std::string& filename) {
    std::vector<std::string> desiredOrder;
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        printf(RED "\nFailed to open desired order file for reading: %s\n" RESET, filename.c_str());
        return desiredOrder; // empty
    }

    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue; // skip blank lines

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
        << "  --opencaptureframes     Open capture frames for debugging (slows down processing)\n"
        << "  --desiredorder          Override Kinect order based on desiredorderedDevices.txt\n"
        << "  --roomName <roomname>   Name of room file to load IDs from\n"
        << "  --logeverything         Log all timestamps and events\n"
        << "  --log <file_path>       Specify log file path (default: C:\\Temp\\tempCG\\23-07-25\\KinectLog.txt)\n"
        << "  -h, --help              Show this help message\n";
}


int main(int argc, char* argv[])
{
    printf("HelloDevice Version: 1.2 \n" RESET);
    printf("Use -h to see executable parameters.\n" RESET);

    // Default values
    bool OPENCAPTUREFRAMES = false;
    bool SENDJOINTSVIATCP = true;
    bool OVERRIDEDEVICEORDER = true;
    bool RECORDTIMESTAMPS = false;
    std::string ROOMNAME = "LKTest";
    std::string LOGFILEPATH = "C:\\Temp\\tempCG\\KinectLog.txt";

    // Simple command-line parsing
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--opencaptureframes") {
            OPENCAPTUREFRAMES = true;
        }
        else if (arg == "--desiredorder") {
            OVERRIDEDEVICEORDER = true;
        }
        // Support --roomname=<value>
        else if (arg.rfind("--roomname=", 0) == 0) {
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
        else if (arg == "--port") {
            if (i + 1 < argc) {
                try {
                    int p = std::stoi(argv[++i]); // parse next arg as integer
                    if (p > 0 && p <= 65535) {
                        PORT = p;
                    }
                    else {
                        fprintf(stderr, "Invalid port number: %d (must be 1-65535)\n", p);
                        return 1;
                    }
                }
                catch (const std::exception&) {
                    fprintf(stderr, "Invalid port value: '%s'\n", argv[i]);
                    return -1;
                }
            }
            else {
                fprintf(stderr, "Missing value for --port\n");
                return 1;
            }
        }
        // unknown args are ignored silently (preserve existing behavior)
    }

    printf("\n----------------------\n");
    printf("Start Input Parameters\n");
    printf("----------------------\n\n");
	printf(YEL "Room name = %s\n" RESET, ROOMNAME.c_str());
	printf("Port Number: %d\n", PORT);
    printf("Open capture frames = %s\n", OPENCAPTUREFRAMES ? "true" : "false");
    printf("Setting desired order = %s\n", OVERRIDEDEVICEORDER ? "true" : "false");
	printf("Logging timestamps = %s\n", RECORDTIMESTAMPS ? "true" : "false");
	printf("Log file path (if using) = %s\n", LOGFILEPATH.c_str());

    printf("\n----------------------\n");
    printf("End Input Parameters\n");
    printf("----------------------\n\n");

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

    SOCKET socketToTransmit = NULL;

    if (SENDJOINTSVIATCP) {
        WSADATA wsaData;
        SOCKET serverSocket;
        struct sockaddr_in serverAddr;

        // Initialize Winsock
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            fprintf(stderr, "WSAStartup failed: %d\n", WSAGetLastError());
            return 1;
        }

        // Create a critical section for thread safety
        InitializeCriticalSection(&cs);

        // Create a socket
        serverSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (serverSocket == INVALID_SOCKET) {
            fprintf(stderr, "Socket creation failed: %d\n", WSAGetLastError());
            WSACleanup();
            return 1;
        }

        // Define server address
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = INADDR_ANY; // Listen on all interfaces
        serverAddr.sin_port = htons(PORT); // Port number

        // Bind the socket
        if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
            fprintf(stderr, "Bind failed: %d\n", WSAGetLastError());
            closesocket(serverSocket);
            WSACleanup();
            return 1;
        }

        // Start listening for incoming connections
        if (listen(serverSocket, SOMAXCONN) == SOCKET_ERROR) {
            fprintf(stderr, "Listen failed: %d\n", WSAGetLastError());
            closesocket(serverSocket);
            WSACleanup();
            return 1;
        }

        printf("Server is listening on port %d...\n", PORT);

        // Create a thread to accept incoming connections
        HANDLE acceptThread = CreateThread(NULL, 0, AcceptConnections, (LPVOID)serverSocket, 0, NULL);
        if (acceptThread == NULL) {
            fprintf(stderr, "Failed to create accept thread: %d\n", GetLastError());
            closesocket(serverSocket);
            WSACleanup();
            return 1;
        }
    }

    //worker threads
    std::vector<std::thread> workers;

    // Find number of devices and initialise as nullptr
    uint32_t device_count = k4a_device_get_installed_count();
    printf("Found %d connected devices:\n", device_count);

    // Store devices with their serial numbers
    std::vector<KinectDevice> devices;

    // Retrieve and print serial numbers in initial order
    std::cout << "Initial order of devices:" << std::endl;

    for (uint32_t i = 0; i < device_count; i++) {
        k4a_device_t device = nullptr;
        if (k4a_device_open(i, &device) != K4A_RESULT_SUCCEEDED) {
            std::cerr << "Failed to open the Kinect Azure device" << std::endl;
            return 1;
        }
        else {
            std::string serial_number = get_kinect_serial(device);
            if (!serial_number.empty()) {
                std::cout << " Device " << i << " SN: " << serial_number << std::endl;
                devices.push_back({ device, serial_number });
            }
            else {
                k4a_device_close(device);
            }
        }
    }

    // Sort devices in order of SN
    std::sort(devices.begin(), devices.end(), [](const KinectDevice& a, const KinectDevice& b) {
        return a.serial_number < b.serial_number;
        });

    if (OVERRIDEDEVICEORDER) {
        // Override the order of the devices  
        std::vector<std::string> desiredOrder = LoadDesiredOrder("C:\\CommonGround\\CalibrationFiles\\" + ROOMNAME + ".txt"); // Load the desired order from a file

        if(desiredOrder.size() != device_count) {
            std::cerr << RED "\nDesired order size does not match connected devices size. Check the desired order file and connected devices.\n" RESET;
            return -1;
		}   

        // Reorder devices based on the desired order  
        std::vector<KinectDevice> reorderedDevices;
        for (const auto& sn : desiredOrder) {
            auto it = std::find_if(devices.begin(), devices.end(), [&](const KinectDevice& device) {
                return device.serial_number == sn;
                });
            if (it != devices.end()) {
                reorderedDevices.push_back(*it);
            }
        }
        devices = reorderedDevices;
    }

    // Print sorted order
    std::cout << "Sorted order of devices:" << std::endl;
    for (size_t i = 0; i < devices.size(); i++) {
        devices[i].name = "Device_" + std::to_string(i);
        std::cout << " Device " << i << "SN: " << devices[i].serial_number << " Name: " << devices[i].name << std::endl;
    }

    // Start threads for sorted devices
    for (size_t i = 0; i < devices.size(); i++) {
        JointFinder kinectJointFinder;
        std::cerr << "Starting thread for Device with serial number: " << devices[i].serial_number << std::endl;
        workers.push_back(std::thread(&JointFinder::DetectJoints,
            &kinectJointFinder,
            static_cast<int>(i),
            devices[i].device,
            socketToTransmit,
            std::ref(outputFile),
            std::ref(outputFileMutex),
            RECORDTIMESTAMPS,
            OPENCAPTUREFRAMES,
            SENDJOINTSVIATCP
        ));
    }

    // Join threads
    for (size_t i = 0; i < workers.size(); i++) {
        try {
            workers[i].join();
        }
        catch (std::exception& ex) {
            printf("join() error log: %s\n", ex.what());
        }
    }

    if (SENDJOINTSVIATCP) {
        // Close sockets and clean up
        DeleteCriticalSection(&cs);
        closesocket(serverSocket);
        WSACleanup();
    }

    //HS
    for (size_t i = 0; i < devices.size(); i++) {
        k4a_device_stop_cameras(devices[i].device);
        k4a_device_close(devices[i].device);
    }


    if (RECORDTIMESTAMPS) {
        // Close the file when done
        outputFile.close();
    }

}

