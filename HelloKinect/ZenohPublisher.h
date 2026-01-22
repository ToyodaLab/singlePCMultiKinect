#pragma once
// ZenohPublisher.h
// Simple C++ wrapper for publishing bytes via Zenoh using the zenoh-c API.

#include <string>
#include <vector>
#include <mutex>
#include <atomic>

#include <zenoh.h>

class ZenohPublisher
{
public:
    // Construct with a default key (can be changed by calling declare).
    explicit ZenohPublisher(const std::string& key = "kinect/skeleton");

    // Non-copyable
    ZenohPublisher(const ZenohPublisher&) = delete;
    ZenohPublisher& operator=(const ZenohPublisher&) = delete;

    // Movable
    ZenohPublisher(ZenohPublisher&&) noexcept;
    ZenohPublisher& operator=(ZenohPublisher&&) noexcept;

    // Initialize the Zenoh session. Returns true on success.
    // options: JSON-like config string for zenoh (optional).
    bool init(const std::string& options = "");

    // Declare/set the key used for publishing. Returns true on success.
    bool declare(const std::string& key);

    // Publish raw bytes (thread-safe). Returns true on success.
    bool publish(const void* data, size_t len);

    // Convenience overload for std::vector<uint8_t>
    bool publish(const std::vector<uint8_t>& data);

    // Shutdown and release resources.
    void shutdown();

    // Is publisher active (initialized)?
    bool isActive() const noexcept { return active_.load(); }

    ~ZenohPublisher();

private:
    // Implementation details hidden here.
    std::string key_;
    std::mutex publish_mtx_;
    std::atomic<bool> active_;

    // Zenoh-owned objects (updated to new ze_ types)
    z_owned_session_t session_;
    bool session_initialized_;

    z_owned_keyexpr_t keyexpr_;
    bool keyexpr_initialized_;

    z_owned_publisher_t publisher_;
    bool publisher_declared_;
};