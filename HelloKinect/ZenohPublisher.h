#pragma once
// ZenohPublisher.h
// Simple C++ wrapper for publishing bytes via Zenoh using the zenoh-c API.

#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <thread> // For heartbeat

#include <zenoh.h>

class ZenohPublisher
{
public:
    // Construct with a default key (can be changed by calling declare).
    explicit ZenohPublisher(const std::string& key = "cg/skeleton");

    // Non-copyable
    ZenohPublisher(const ZenohPublisher&) = delete;
    ZenohPublisher& operator=(const ZenohPublisher&) = delete;

    // Movable
    ZenohPublisher(ZenohPublisher&&) noexcept;
    ZenohPublisher& operator=(ZenohPublisher&&) noexcept;

    // Start/stop periodic heartbeat publishes.
    // startHeartbeat: key = topic to publish heartbeat to, interval_ms = publish interval (ms).
    // Returns true if heartbeat thread started successfully.
    bool startHeartbeat(const std::string& key = "cg/heartbeat", int interval_ms = 1000);
    void stopHeartbeat();

    // Initialize the Zenoh session. Returns true on success.
    // options: JSON-like config string for zenoh (optional).
    bool init(const std::string& options = "");

    // Declare/set the key used for publishing. Returns true on success.
    // This will create and store a publisher for the given key.
    bool declare(const std::string& key);

    // Publish raw bytes to the default key (thread-safe). Returns true on success.
    bool publish(const void* data, size_t len);

    // Publish raw bytes to a specific key (thread-safe). Returns true on success.
    bool publish(const std::string& key, const void* data, size_t len);

    // Convenience overloads for std::vector<uint8_t>
    bool publish(const std::vector<uint8_t>& data);
    bool publish(const std::string& key, const std::vector<uint8_t>& data);

    // Shutdown and release resources.
    void shutdown();

    // Is publisher active (initialized)?
    bool isActive() const noexcept { return active_.load(); }

    ~ZenohPublisher();

private:
    // Internal per-key entry
    struct Entry {
        z_owned_keyexpr_t keyexpr;
        bool keyexpr_initialized{false};

        z_owned_publisher_t publisher;
        bool publisher_declared{false};
    };

    // Helper to declare/create publisher for a key (assumes active_)
    bool declare_internal(const std::string& key, Entry& entry);

    std::string default_key_;

    // Protects publish path for thread-safety when sending bytes.
    std::mutex publish_mtx_;

    // Protects the entries_ map.
    std::mutex entries_mtx_;

    std::atomic<bool> active_;

    // Zenoh-owned objects (updated to new ze_ types)
    z_owned_session_t session_;
    bool session_initialized_;

    // Map of key -> Entry
    std::unordered_map<std::string, Entry> entries_;

    // Heartbeat internals
    std::atomic<bool> heartbeat_active_{ false };
    std::thread heartbeat_thread_;
    std::string heartbeat_key_;
    int heartbeat_interval_ms_{ 1000 };
};