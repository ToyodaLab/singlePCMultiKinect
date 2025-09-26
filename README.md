# HelloKinect

HelloKinect is a C++14 application for multi Kinect Azure/Femto Bolt body tracking, joint data logging, and real-time TCP streaming. It supports multiple devices, configurable device order, and flexible logging options. The application is designed to run on Windows.

## Features

- **Multi-Kinect Azure support**: Detects and manages multiple Kinect Azure devices.
- **Body tracking**: Uses Azure Kinect Body Tracking SDK to extract skeleton/joint data.
- **TCP server**: Streams joint data to multiple clients over TCP.
- **Configurable logging**: Logs joint data and timestamps to a file.
- **Device order override**: Supports custom device ordering via a configuration file.
- **Thread-safe logging and networking**.
- **Command-line options** for flexible operation.

## Requirements

- Windows 10/11
- Visual Studio 2022 (C++14)
- Azure Kinect SDK and Body Tracking SDK
- Winsock2

## Building

1. Open the solution in Visual Studio 2022.
2. Ensure the Azure Kinect SDK and Body Tracking SDK are installed and referenced.
3. Build the solution

## Usage

Run the executable with the following options:

### Options

| Option                  | Description                                                                 |
|-------------------------|-----------------------------------------------------------------------------|
| `--onlyGetIDs`          | Only get Kinect serial IDs and save to file                                 |
| `--opencaptureframes`   | Open capture frames for debugging (slows down processing)                   |
| `--notransmission`      | Do not send joint data via TCP                                              |
| `--desiredorder`        | Override Kinect order based on `desiredorderedDevices.txt`                  |
| `--logeverything`       | Log all timestamps and events                                               |
| `--log <file_path>`     | Specify log file path (default: `C:\Temp\tempCG\KinectLog.txt`)             |
| `-h`, `--help`          | Show help message                                                           |

### Example

HelloKinect.exe --opencaptureframes --log "C:\Temp\tempCG\KinectLog.txt"

## Output

- **Log File**: Joint and timestamp data are written to the specified log file.
- **TCP Streaming**: Joint data is sent to all connected TCP clients.
- **Device Order**: Devices can be reordered by editing `desiredorderedDevices.txt`.

## Custom Device Order

To override the default device order:
1. Create a file named `desiredorderedDevices.txt` in `C:\Temp\tempCG\`.
2. List the desired Kinect serial numbers, one per line.
3. Run with `--desiredorder`.
   
