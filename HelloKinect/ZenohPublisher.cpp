// ZenohPublisher.cpp
// See ZenohPublisher.h for interface.

#include "ZenohPublisher.h"
#include <iostream>
#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>

ZenohPublisher::ZenohPublisher(const std::string& key)
    : default_key_(key),
      active_(false),
      session_initialized_(false)
{
}

ZenohPublisher::ZenohPublisher(ZenohPublisher&& other) noexcept
    : default_key_(std::move(other.default_key_)),
      active_(other.active_.load())
{
    // Transfer session ownership if initialized
    session_initialized_ = other.session_initialized_;
    if (session_initialized_) {
        session_ = other.session_;
        other.session_initialized_ = false;
    }
    // Move entries by explicitly transferring the POD handles and clearing flags on the source
    {
        std::lock_guard<std::mutex> lk(other.entries_mtx_);
        for (auto &kv : other.entries_) {
            Entry e;
            if (kv.second.keyexpr_initialized) {
                e.keyexpr = kv.second.keyexpr;
                e.keyexpr_initialized = true;
                kv.second.keyexpr_initialized = false;
            }
            if (kv.second.publisher_declared) {
                e.publisher = kv.second.publisher;
                e.publisher_declared = true;
                kv.second.publisher_declared = false;
            }
            entries_.emplace(kv.first, std::move(e));
        }
        other.entries_.clear();
    }
    other.active_.store(false);

    // Transfer heartbeat state (do not transfer running thread)
    heartbeat_active_.store(false);
    heartbeat_interval_ms_ = other.heartbeat_interval_ms_;
    heartbeat_key_ = std::move(other.heartbeat_key_);
}

ZenohPublisher& ZenohPublisher::operator=(ZenohPublisher&& other) noexcept
{
    if (this != &other)
    {
        shutdown();

        default_key_ = std::move(other.default_key_);
        active_.store(other.active_.load());

        session_initialized_ = other.session_initialized_;
        if (session_initialized_) {
            session_ = other.session_;
            other.session_initialized_ = false;
        }

        // Transfer entries
        {
            std::lock_guard<std::mutex> lk(other.entries_mtx_);
            for (auto &kv : other.entries_) {
                Entry e;
                if (kv.second.keyexpr_initialized) {
                    e.keyexpr = kv.second.keyexpr;
                    e.keyexpr_initialized = true;
                    kv.second.keyexpr_initialized = false;
                }
                if (kv.second.publisher_declared) {
                    e.publisher = kv.second.publisher;
                    e.publisher_declared = true;
                    kv.second.publisher_declared = false;
                }
                entries_.emplace(kv.first, std::move(e));
            }
            other.entries_.clear();
        }

        other.active_.store(false);

        // Transfer heartbeat config but do not move running thread
        heartbeat_active_.store(false);
        heartbeat_interval_ms_ = other.heartbeat_interval_ms_;
        heartbeat_key_ = std::move(other.heartbeat_key_);
    }
    return *this;
}

bool ZenohPublisher::init(const std::string& options)
{
    // Build a default config and open a session.
    // If the user supplied 'options' we will try to parse it into config via z_config_from_str.
    z_moved_config_t cfg;
    // z_config_default constructs default owned_config inside cfg._this
    if (z_config_default(&cfg._this) != Z_OK) {
        std::cerr << "[ZenohPublisher] z_config_default failed\n";
        return false;
    }

    if (!options.empty()) {
        // Try to parse the provided options string into the config.
        // z_config_from_str expects an owned_config_t pointer.
        if (zc_config_from_str(&cfg._this, options.c_str()) != Z_OK) {
            std::cerr << "[ZenohPublisher] z_config_from_str failed parsing options\n";
            // proceed with default cfg (not fatal)
        }
    }

    if (z_open(&session_, &cfg, nullptr) != Z_OK) {
        std::cerr << "[ZenohPublisher] z_open failed\n";
        return false;
    }

    active_.store(true);
    session_initialized_ = true;
    return true;
}

bool ZenohPublisher::declare_internal(const std::string& key, Entry& entry)
{
    // create a key expression
    if (z_keyexpr_from_str(&entry.keyexpr, key.c_str()) != Z_OK) {
        std::cerr << "[ZenohPublisher] z_keyexpr_from_str failed for key: " << key << "\n";
        return false;
    }
    entry.keyexpr_initialized = true;

    const struct z_loaned_session_t *loaned_session = z_session_loan(&session_);
    const struct z_loaned_keyexpr_t *loaned_ke = z_keyexpr_loan(&entry.keyexpr);

    // declare publisher
    if (z_declare_publisher(loaned_session, &entry.publisher, loaned_ke, nullptr) != Z_OK) {
        std::cerr << "[ZenohPublisher] z_declare_publisher failed for key: " << key << "\n";
        // drop keyexpr if declared (cleanup)
        z_moved_keyexpr_t moved_ke; moved_ke._this = entry.keyexpr;
        z_keyexpr_drop(&moved_ke);
        entry.keyexpr_initialized = false;
        return false;
    }

    entry.publisher_declared = true;
    return true;
}

bool ZenohPublisher::declare(const std::string& key)
{
    if (!isActive()) {
        std::cerr << "[ZenohPublisher] declare() called before init()\n";
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(entries_mtx_);
        auto it = entries_.find(key);
        if (it != entries_.end()) {
            if (it->second.publisher_declared) return true; // already declared
            // otherwise we'll declare below using the existing entry
        } else {
            // insert empty entry to fill
            entries_.emplace(key, Entry{});
            it = entries_.find(key);
        }

        // declare for this entry
        return declare_internal(key, it->second);
    }
}

bool ZenohPublisher::publish(const void* data, size_t len)
{
    // publish to default key
    return publish(default_key_, data, len);
}

bool ZenohPublisher::publish(const std::string& key, const void* data, size_t len)
{
    if (!isActive()) {
        std::cerr << "[ZenohPublisher] publish called but publisher not initialized.\n";
        return false;
    }

    if (!data || len == 0) {
        // allow zero-length publish if desired; here we'll log and return true
        std::cout << "[ZenohPublisher][stub] publish called with empty payload on key: " << key << std::endl;
        return true;
    }

    // Ensure publisher exists for the key
    {
        std::lock_guard<std::mutex> lk(entries_mtx_);
        auto it = entries_.find(key);
        if (it == entries_.end() || !it->second.publisher_declared) {
            // release lock while calling declare() because declare takes entries_mtx_ internally
        }
    }

    // If no publisher, attempt to declare (this will lock entries_mtx_)
    {
        std::lock_guard<std::mutex> lk(entries_mtx_);
        auto it = entries_.find(key);
        if (it == entries_.end()) {
            entries_.emplace(key, Entry{});
            it = entries_.find(key);
        }
        if (!it->second.publisher_declared) {
            if (!declare_internal(key, it->second)) {
                std::cerr << "[ZenohPublisher] publish() failed: cannot declare publisher for key: " << key << "\n";
                return false;
            }
        }
    }

    // Now do the actual publish under publish_mtx_ to serialize publishes from this process
    std::lock_guard<std::mutex> lk(publish_mtx_);

    // find entry (safe now)
    Entry entry;
    {
        std::lock_guard<std::mutex> lk2(entries_mtx_);
        auto it = entries_.find(key);
        if (it == entries_.end() || !it->second.publisher_declared) {
            std::cerr << "[ZenohPublisher] publish() internal error: publisher missing for key: " << key << "\n";
            return false;
        }
        entry = it->second; // copy struct (contains POD z_owned_*).
    }

    // Create owned bytes from buffer (copy)
    z_owned_bytes_t owned_bytes;
    z_result_t r = z_bytes_copy_from_buf(&owned_bytes, reinterpret_cast<const uint8_t*>(data), len);
    if (r != Z_OK) {
        std::cerr << "[ZenohPublisher] z_bytes_copy_from_buf failed\n";
        return false;
    }

    // Wrap into moved bytes (to hand ownership to zenoh)
    z_moved_bytes_t moved_bytes;
    moved_bytes._this = owned_bytes;

    // Send via publisher (use loan on the actual publisher stored in the map)
    const struct z_loaned_publisher_t *loaned_pub = z_publisher_loan(&entry.publisher);
    if (!loaned_pub) {
        std::cerr << "[ZenohPublisher] z_publisher_loan returned NULL\n";
        // drop moved bytes to avoid leak
        z_bytes_drop(&moved_bytes);
        return false;
    }

    r = z_publisher_put(loaned_pub, &moved_bytes, nullptr);
    if (r != Z_OK) {
        std::cerr << "[ZenohPublisher] z_publisher_put failed: " << int(r) << "\n";
        // If put failed, drop the moved bytes to avoid leak
        z_bytes_drop(&moved_bytes);
        return false;
    }

    // success: ownership of moved bytes consumed by zenoh
    return true;
}

bool ZenohPublisher::publish(const std::vector<uint8_t>& data)
{
    return publish(data.data(), data.size());
}

bool ZenohPublisher::publish(const std::string& key, const std::vector<uint8_t>& data)
{
    return publish(key, data.data(), data.size());
}

bool ZenohPublisher::startHeartbeat(const std::string& key, int interval_ms)
{
    if (!isActive()) {
        std::cerr << "[ZenohPublisher] startHeartbeat() called before init()\n";
        return false;
    }

    // If a heartbeat is already running, stop it first.
    stopHeartbeat();

    heartbeat_key_ = key;
    heartbeat_interval_ms_ = (interval_ms > 0) ? interval_ms : 1000;
    heartbeat_active_.store(true);

    // Ensure the heartbeat key is declared so publish doesn't race on first send.
    if (!declare(heartbeat_key_)) {
        std::cerr << "[ZenohPublisher] startHeartbeat: failed to declare key: " << heartbeat_key_ << "\n";
        heartbeat_active_.store(false);
        return false;
    }

    // Launch background thread
    heartbeat_thread_ = std::thread([this]() {
        while (heartbeat_active_.load() && isActive()) {
            // Send a single byte with value 1 as the heartbeat payload
            uint8_t one = 1;
            bool ok = publish(heartbeat_key_, &one, sizeof(one));
            if (!ok) {
                std::cerr << "[ZenohPublisher] heartbeat publish failed for key: " << heartbeat_key_ << "\n";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(heartbeat_interval_ms_));
        }
    });

    return true;
}

void ZenohPublisher::stopHeartbeat()
{
    bool expected = true;
    if (!heartbeat_active_.compare_exchange_strong(expected, false)) {
        // not active
    }
    // Join thread if running
    if (heartbeat_thread_.joinable()) {
        try {
            heartbeat_thread_.join();
        }
        catch (const std::exception& ex) {
            std::cerr << "[ZenohPublisher] stopHeartbeat join exception: " << ex.what() << "\n";
        }
    }
}

void ZenohPublisher::shutdown()
{
    bool expected = true;
    if (!active_.compare_exchange_strong(expected, false)) {
        // already inactive
        return;
    }

    // Stop heartbeat before dropping publishers/session
    stopHeartbeat();

    std::lock_guard<std::mutex> lk(publish_mtx_);
    std::lock_guard<std::mutex> lk2(entries_mtx_);

    // Drop all publishers/keyexprs in the map
    for (auto &kv : entries_) {
        Entry &entry = kv.second;
        if (entry.publisher_declared) {
            z_moved_publisher_t moved_pub;
            moved_pub._this = entry.publisher;
            // Try undeclare/drop; ignore return value
            z_publisher_drop(&moved_pub);
            entry.publisher_declared = false;
        }
        if (entry.keyexpr_initialized) {
            z_moved_keyexpr_t moved_ke;
            moved_ke._this = entry.keyexpr;
            z_keyexpr_drop(&moved_ke);
            entry.keyexpr_initialized = false;
        }
    }
    entries_.clear();

    // Close session
    if (session_initialized_) {
        // Try close (may return error), then drop session storage
        z_result_t r = z_close(z_session_loan_mut(&session_), nullptr);
        if (r != Z_OK) {
            std::cerr << "[ZenohPublisher] z_close returned: " << int(r) << "\n";
        }
        z_moved_session_t moved_s;
        moved_s._this = session_;
        z_session_drop(&moved_s);
        session_initialized_ = false;
    }
}

ZenohPublisher::~ZenohPublisher()
{
    shutdown();
}