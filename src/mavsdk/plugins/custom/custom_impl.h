#pragma once

#include <atomic>
#include <mutex>
#include <optional>
#include <functional>

#include "plugins/custom/custom.h"
#include "mavlink_include.h"
#include "plugin_impl_base.h"
#include "callback_list.h"
#include "timeout_handler.h"

namespace mavsdk {

class System;

class CustomImpl : public PluginImplBase {
public:
    explicit CustomImpl(System& system);
    explicit CustomImpl(std::shared_ptr<System> system);

    ~CustomImpl() override;

    void init() override;
    void deinit() override;

    void enable() override;
    void disable() override;


    Custom::HeartbeatHandle subscribe_heartbeat(const Custom::HeartbeatCallback& callback);
    void unsubscribe_heartbeat(Custom::HeartbeatHandle handle);
    Custom::Heartbeat CustomImpl::heartbeat() const;

    Custom::ConnectionStatusHandle subscribe_connection_status(const Custom::ConnectionStatusCallback& callback);
    void unsubscribe_connection_status(Custom::ConnectionStatusHandle handle);
    Custom::ConnectionStatus connection_status() const;

private:
    void set_heartbeat(Custom::Heartbeat heartbeat);
    void process_heartbeat(const mavlink_message_t& message);
    void set_connected();
    void set_disconnected();
    void heartbeats_timed_out();
    TimeoutHandler::Cookie 
    register_timeout_handler(const std::function<void()>& callback, double duration_s);
    void refresh_timeout_handler(TimeoutHandler::Cookie cookie);
    void unregister_timeout_handler(TimeoutHandler::Cookie cookie);

    mutable  std::mutex _heartbeat_mutex{};
    Custom::Heartbeat _heartbeat{};
    mutable std::mutex _subscription_mutex{};
    CallbackList<Custom::Heartbeat> _heartbeat_subscriptions{};

    mutable std::mutex _connection_mutex{};
    Custom::ConnectionStatus _connected{false};
    CallbackList<Custom::ConnectionStatus> _is_connected_callbacks{};

    // TimeoutHandler _timeout_handler;
    TimeoutHandler::Cookie _heartbeat_timeout_cookie{};

    static constexpr double HEARTBEAT_TIMEOUT_S = 3.0;
};

} // namespace mavsdk
