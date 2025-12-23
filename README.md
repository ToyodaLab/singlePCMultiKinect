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

## Watchdog Auto-Restart

HelloKinect includes an internal watchdog that monitors worker threads and automatically restarts the application if a hang or crash is detected.

### How It Works

1. **Heartbeat Monitoring**: Each Kinect worker thread sends a heartbeat signal every frame
2. **Hang Detection**: The watchdog checks heartbeats every 2 seconds. If no heartbeat is received for 10 seconds, it considers the thread hung
3. **Auto-Restart**: When a hang is detected, the watchdog:
   - Logs the crash event to `C:\Temp\tempCG\WatchdogCrash.log`
   - Spawns a new process with the same CLI arguments
   - Terminates the current (hung) process
4. **Cooldown Protection**: If 3 restarts occur within 60 seconds, the watchdog waits for a cooldown period before restarting again

### Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| Timeout | 10 seconds | Time without heartbeat before triggering restart |
| Check Interval | 2 seconds | How often the watchdog checks heartbeats |
| Cooldown | 60 seconds | Cooldown period after multiple rapid restarts |
| Max Rapid Restarts | 3 | Number of restarts allowed before cooldown |

### Crash Log

Crash events are logged to `C:\Temp\tempCG\WatchdogCrash.log` with timestamps:

```
[2025-12-09 14:32:15] Worker thread 0 appears hung (no heartbeat for 12 seconds)
[2025-12-09 14:32:15] Initiating restart: "HelloKinect.exe" --roomname TestRoom --port 8844
[2025-12-09 14:32:15] New process created (suspended). Exiting current process first.
```

### Stress Testing

To test the watchdog functionality, use the `--stress-hang` option:

```
HelloKinect.exe --stress-hang 100
```

This simulates a hang after 100 frames, allowing you to verify the watchdog triggers correctly.

