// ZenohPublisher.h
#pragma once
#include <zenoh.h>
#include <string>

class ZenohPublisher {
public:
    ZenohPublisher(const std::string& keyexpr, const std::string& config = "");
    ~ZenohPublisher();
    bool isOpen() const;
    void publish(const void* data, size_t size);

private:
    z_owned_session_t session_;
    z_owned_publisher_t publisher_;
    bool open_;
};