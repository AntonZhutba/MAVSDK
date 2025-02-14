#pragma once

#include <atomic>
#include <mutex>
#include <optional>
#include <functional>

#include "plugins/custom/custom.h"
#include "mavlink_include.h"
#include "plugin_impl_base.h"
#include "system.h"
#include "callback_list.h"
#include "timeout_handler.h"

namespace mavsdk {

class System;

class CustomImpl : public PluginImplBase {
public:
    explicit CustomImpl(System& system);
    explicit CustomImpl(std::shared_ptr<System> system);
    ~CustomImpl();

    void init() override;
    void deinit() override;

    void enable() override;
    void disable() override;

    struct Heartbeat {
        uint8_t type;
        uint8_t autopilot;
        uint8_t base_mode;
        uint32_t custom_mode;
        uint8_t system_status;
    };

    Custom::HeartbeatHandle subscribe_heartbeat(const HeartbeatCallback& callback);
    void unsubscribe_heartbeat(HeartbeatHandle handle);
    Heartbeat heartbeat() const;

    Custom::IsConnectedHandle subscribe_is_connected(const Custom::IsConnectedCallback& callback);
    void unsubscribe_is_connected(Custom::IsConnectedHandle handle);
    bool is_connected() const;

private:
    void set_heartbeat(const Heartbeat& heartbeat);
    void process_heartbeat(const mavlink_message_t& message);
    void set_connected();
    void set_disconnected();
    void heartbeats_timed_out();
    void refresh_timeout_handler(TimeoutHandler::Cookie cookie);
    TimeoutHandler::Cookie register_timeout_handler(const std::function<void()>& callback, double duration_s);
    void unregister_timeout_handler(TimeoutHandler::Cookie cookie);

    std::mutex _heartbeat_mutex{};
    Heartbeat _heartbeat{};
    std::mutex _subscription_mutex{};
    CallbackList<Heartbeat> _heartbeat_subscriptions{};

    std::mutex _connection_mutex{};
    std::atomic<bool> _connected{false};
    CallbackList<bool> _is_connected_callbacks{};

    TimeoutHandler _timeout_handler;
    TimeoutHandler::Cookie _heartbeat_timeout_cookie{nullptr};
};

} // namespace mavsdk
