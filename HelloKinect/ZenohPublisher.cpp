// ZenohPublisher.cpp
// See ZenohPublisher.h for interface.

#include "ZenohPublisher.h"
#include <iostream>
#include <sstream>
#include <cstring>


ZenohPublisher::ZenohPublisher(const std::string& key)
    : key_(key),
      active_(false),
      session_initialized_(false),
      keyexpr_initialized_(false),
      publisher_declared_(false)
{
}

ZenohPublisher::ZenohPublisher(ZenohPublisher&& other) noexcept
    : key_(std::move(other.key_)),
      active_(other.active_.load())
{
    session_initialized_ = other.session_initialized_;
    keyexpr_initialized_ = other.keyexpr_initialized_;
    publisher_declared_ = other.publisher_declared_;
    
    // Move-punning the underlying POD structs (allowed for C types)
    if (session_initialized_) {
        session_ = other.session_;
        other.session_initialized_ = false;
    }
    if (keyexpr_initialized_) {
        keyexpr_ = other.keyexpr_;
        other.keyexpr_initialized_ = false;
    }
    if (publisher_declared_) {
        publisher_ = other.publisher_;
        other.publisher_declared_ = false;
    }
    other.active_.store(false);
}

ZenohPublisher& ZenohPublisher::operator=(ZenohPublisher&& other) noexcept
{
    if (this != &other)
    {
        shutdown();
        key_ = std::move(other.key_);
        active_.store(other.active_.load());

        session_initialized_ = other.session_initialized_;
        keyexpr_initialized_ = other.keyexpr_initialized_;
        publisher_declared_ = other.publisher_declared_;

        if (session_initialized_) {
            session_ = other.session_;
            other.session_initialized_ = false;
        }
        if (keyexpr_initialized_) {
            keyexpr_ = other.keyexpr_;
            other.keyexpr_initialized_ = false;
        }
        if (publisher_declared_) {
            publisher_ = other.publisher_;
            other.publisher_declared_ = false;
        }
        other.active_.store(false);
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

bool ZenohPublisher::declare(const std::string& key)
{
    std::lock_guard<std::mutex> lk(publish_mtx_);
    key_ = key;

    if (!isActive()) {
        std::cerr << "[ZenohPublisher] declare() called before init()\n";
        return false;
    }

    // create a key expression
    if (z_keyexpr_from_str(&keyexpr_, key_.c_str()) != Z_OK) {
        std::cerr << "[ZenohPublisher] z_keyexpr_from_str failed for key: " << key_ << "\n";
        return false;
    }
    keyexpr_initialized_ = true;

    const struct z_loaned_session_t *loaned_session = z_session_loan(&session_);
    const struct z_loaned_keyexpr_t *loaned_ke = z_keyexpr_loan(&keyexpr_);

    // declare publisher
    if (z_declare_publisher(loaned_session, &publisher_, loaned_ke, nullptr) != Z_OK) {
        std::cerr << "[ZenohPublisher] z_declare_publisher failed for key: " << key_ << "\n";
        // drop keyexpr if declared (cleanup)
        z_moved_keyexpr_t moved_ke; moved_ke._this = keyexpr_;
        z_keyexpr_drop(&moved_ke);
        keyexpr_initialized_ = false;
        return false;
    }

    publisher_declared_ = true;
    return true;
}

bool ZenohPublisher::publish(const void* data, size_t len)
{
    if (!isActive()) {
        std::cerr << "[ZenohPublisher] publish called but publisher not initialized.\n";
        return false;
    }

    if (!data || len == 0) {
        // allow zero-length publish if desired; here we'll log and return true
        std::cout << "[ZenohPublisher][stub] publish called with empty payload on key: " << key_ << std::endl;
        return true;
    }

    std::lock_guard<std::mutex> lk(publish_mtx_);

    if (!session_initialized_) {
        std::cerr << "[ZenohPublisher] publish() called but session not initialized\n";
        return false;
    }
    if (!publisher_declared_) {
        // If user didn't call declare explicitly, try to declare now with stored key.
        if (!declare(key_)) {
            std::cerr << "[ZenohPublisher] publish() failed: cannot auto-declare publisher\n";
            return false;
        }
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

    // Send via publisher
    const struct z_loaned_publisher_t *loaned_pub = z_publisher_loan(&publisher_);
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

void ZenohPublisher::shutdown()
{
    bool expected = true;
    if (!active_.compare_exchange_strong(expected, false)) {
        // already inactive
        return;
    }

    std::lock_guard<std::mutex> lk(publish_mtx_);

    // Drop publisher if declared
    if (publisher_declared_) {
        z_moved_publisher_t moved_pub;
        moved_pub._this = publisher_;
        // Try undeclare/drop; ignore return value
        z_publisher_drop(&moved_pub);
        publisher_declared_ = false;
    }

    // Drop keyexpr if created
    if (keyexpr_initialized_) {
        z_moved_keyexpr_t moved_ke;
        moved_ke._this = keyexpr_;
        z_keyexpr_drop(&moved_ke);
        keyexpr_initialized_ = false;
    }

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