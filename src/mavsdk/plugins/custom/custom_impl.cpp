#include "custom_impl.h"
#include "system.h"
#include <mavsdk/system.h>
#include <mavsdk/mavlink_include.h>
#include <mavsdk/plugin_impl_base.h>

namespace mavsdk {

template class CallbackList<Custom::Heartbeat>;
template class CallbackList<bool>;

CustomImpl::CustomImpl(System& system) : PluginImplBase(system)
{
    _system_impl->register_plugin(this);
}

CustomImpl::CustomImpl(std::shared_ptr<System> system) : PluginImplBase(std::move(system))
{
    _system_impl->register_plugin(this);
}

CustomImpl::~CustomImpl()
{
    _system_impl->unregister_plugin(this);
}

void CustomImpl::init()
{
    _system_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_HEARTBEAT,
        [this](const mavlink_message_t& message) { process_heartbeat(message); },
        this);
}

void CustomImpl::deinit()
{
    _system_impl->unregister_all_mavlink_message_handlers(this);
}

void CustomImpl::enable() {}

void CustomImpl::disable() {}

#pragma region Heartbeat

Custom::HeartbeatHandle CustomImpl::subscribe_heartbeat(const Custom::HeartbeatCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    return _heartbeat_subscriptions.subscribe(callback);
}

void CustomImpl::unsubscribe_heartbeat(Custom::HeartbeatHandle handle)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _heartbeat_subscriptions.unsubscribe(handle);
}

Custom::Heartbeat CustomImpl::heartbeat() const
{
    std::lock_guard<std::mutex> lock(_heartbeat_mutex);
    return _heartbeat;
}

void CustomImpl::set_heartbeat(Custom::Heartbeat heartbeat)
{
    std::lock_guard<std::mutex> lock(_heartbeat_mutex);
    _heartbeat = heartbeat;
}

void CustomImpl::process_heartbeat(const mavlink_message_t& message)
{
    if (message.compid != MAV_COMP_ID_AUTOPILOT1) {
        return;
    }

    mavlink_heartbeat_t heartbeat_msg;
    mavlink_msg_heartbeat_decode(&message, &heartbeat_msg);

    Custom::Heartbeat received_heartbeat{};
    received_heartbeat.type = heartbeat_msg.type;
    received_heartbeat.autopilot = heartbeat_msg.autopilot;
    received_heartbeat.base_mode = heartbeat_msg.base_mode;
    received_heartbeat.custom_mode = heartbeat_msg.custom_mode;
    received_heartbeat.system_status = heartbeat_msg.system_status;
    set_heartbeat(received_heartbeat);

    set_connected();

    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _heartbeat_subscriptions.queue(
        heartbeat(), [this](const auto& func) { _system_impl->call_user_callback(func); });
}

#pragma endregion Heartbeat

#pragma region IsConnected

Custom::IsConnectedHandle CustomImpl::subscribe_is_connected(const Custom::IsConnectedCallback& callback)
{
    std::lock_guard<std::mutex> lock(_connection_mutex);
    return _is_connected_callbacks.subscribe(callback);
}

void CustomImpl::unsubscribe_is_connected(Custom::IsConnectedHandle handle)
{
    _is_connected_callbacks.unsubscribe(handle);
}

bool CustomImpl::is_connected() const
{
    return _is_connected;
}

void CustomImpl::set_connected()
{
    std::lock_guard<std::mutex> lock(_connection_mutex);

    if (!_connected) {
        _connected = true;

        if (_heartbeat_timeout_cookie) {
            unregister_timeout_handler(_heartbeat_timeout_cookie);
        }

        _heartbeat_timeout_cookie =
            register_timeout_handler([this] { heartbeats_timed_out(); }, HEARTBEAT_TIMEOUT_S);

        _is_connected_callbacks.queue(
            true, [this](const auto& func) { _mavsdk_impl.call_user_callback(func); });

    } else if (_connected) {
        refresh_timeout_handler(_heartbeat_timeout_cookie);
    }
}

void CustomImpl::set_disconnected()
{
    if (_heartbeat_timeout_cookie) {
        unregister_timeout_handler(_heartbeat_timeout_cookie);
        _heartbeat_timeout_cookie = nullptr;
    }

    _connected = false;
    _is_connected_callbacks.queue(
        false, [this](const auto& func) { _mavsdk_impl.call_user_callback(func); });
}

void CustomImpl::refresh_timeout_handler(TimeoutHandler::Cookie cookie)
{
    if (cookie) {
        _timeout_handler.refresh(cookie);
    }
}

void CustomImpl::heartbeats_timed_out()
{
    LogInfo() << "heartbeats timed out";
    set_disconnected();
}

TimeoutHandler::Cookie CustomImpl::register_timeout_handler(const std::function<void()>& callback, double duration_s)
{
    return _timeout_handler.add(callback, duration_s);
}

void CustomImpl::unregister_timeout_handler(TimeoutHandler::Cookie cookie)
{
    if (cookie) {
        _timeout_handler.remove(cookie);
    }
}

#pragma endregion IsConnected

} // namespace mavsdk