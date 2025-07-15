// ZenohPublisher.cpp
#include "ZenohPublisher.h"

ZenohPublisher::ZenohPublisher(const std::string& keyexpr, const std::string& config)
    : open_(false)
{
    z_owned_config_t zconfig = config.empty() ? z_config_default() : z_config_from_str(config.c_str());
    session_ = z_open(z_config_move(&zconfig));
    if (!z_check(session_)) return;

    publisher_ = z_declare_publisher(z_loan(session_), z_keyexpr(keyexpr.c_str()), NULL);
    open_ = z_check(publisher_);
}

ZenohPublisher::~ZenohPublisher() {
    if (open_) {
        z_undeclare_publisher(z_move(publisher_));
        z_close(z_move(session_));
    }
}

bool ZenohPublisher::isOpen() const {
    return open_;
}

void ZenohPublisher::publish(const void* data, size_t size) {
    if (open_) {
        z_publisher_put(z_loan(publisher_), static_cast<const uint8_t*>(data), size, NULL);
    }
}
