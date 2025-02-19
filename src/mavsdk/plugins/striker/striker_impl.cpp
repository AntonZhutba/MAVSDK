#include "striker_impl.h"
#include "system.h"
#include "mavlink_include.h"
#include "plugin_impl_base.h"
#include "mavsdk_impl.h"
#include "callback_list.tpp"

namespace mavsdk {

template class CallbackList<Striker::Heartbeat>;
template class CallbackList<Striker::SysStatus>;

StrikerImpl::StrikerImpl(System& system) : PluginImplBase(system)
{
    _system_impl->register_plugin(this);
}

StrikerImpl::StrikerImpl(std::shared_ptr<System> system) : PluginImplBase(std::move(system))
{
    _system_impl->register_plugin(this);
}

StrikerImpl::~StrikerImpl()
{
    _system_impl->unregister_plugin(this);
}

void StrikerImpl::init()
{
    _system_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_HEARTBEAT,
        [this](const mavlink_message_t& message) { process_heartbeat(message); },
        this);

    _system_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_SYS_STATUS,
        [this](const mavlink_message_t& message) { process_sys_status(message); },
        this);
}

void StrikerImpl::deinit()
{
    _system_impl->unregister_all_mavlink_message_handlers(this);
}

void StrikerImpl::enable() {}

void StrikerImpl::disable() {}

Striker::HeartbeatHandle
StrikerImpl::subscribe_heartbeat(const Striker::HeartbeatCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_heartbeat_mutex);
    return _heartbeat_subscriptions.subscribe(callback);
}

void StrikerImpl::unsubscribe_heartbeat(Striker::HeartbeatHandle handle)
{
    std::lock_guard<std::mutex> lock(_subscription_heartbeat_mutex);
    _heartbeat_subscriptions.unsubscribe(handle);
}

Striker::Heartbeat StrikerImpl::heartbeat() const
{
    std::lock_guard<std::mutex> lock(_heartbeat_mutex);
    return _heartbeat;
}

void StrikerImpl::set_heartbeat(Striker::Heartbeat heartbeat)
{
    std::lock_guard<std::mutex> lock(_heartbeat_mutex);
    _heartbeat = heartbeat;
}

void StrikerImpl::process_heartbeat(const mavlink_message_t& message)
{
    if (message.compid != MAV_COMP_ID_AUTOPILOT1) {
        return;
    }

    mavlink_heartbeat_t heartbeat_msg;
    mavlink_msg_heartbeat_decode(&message, &heartbeat_msg);

    Striker::Heartbeat received_heartbeat{};
    received_heartbeat.type = heartbeat_msg.type;
    received_heartbeat.autopilot = heartbeat_msg.autopilot;
    received_heartbeat.base_mode = heartbeat_msg.base_mode;
    received_heartbeat.custom_mode = heartbeat_msg.custom_mode;
    received_heartbeat.system_status = heartbeat_msg.system_status;
    set_heartbeat(received_heartbeat);

    std::lock_guard<std::mutex> lock(_subscription_heartbeat_mutex);
    _heartbeat_subscriptions.queue(
        heartbeat(), [this](const auto& func) { _system_impl->call_user_callback(func); });
}

Striker::SysStatusHandle
StrikerImpl::subscribe_sys_status(const Striker::SysStatusCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_sys_status_mutex);
    return _sys_status_subscriptions.subscribe(callback);
}

void StrikerImpl::unsubscribe_sys_status(Striker::SysStatusHandle handle)
{
    std::lock_guard<std::mutex> lock(_subscription_sys_status_mutex);
    _sys_status_subscriptions.unsubscribe(handle);
}

Striker::SysStatus StrikerImpl::sys_status() const
{
    std::lock_guard<std::mutex> lock(_sys_status_mutex);
    return _sys_status;
}

void StrikerImpl::set_sys_status(Striker::SysStatus sys_status)
{
    std::lock_guard<std::mutex> lock(_sys_status_mutex);
    _sys_status = sys_status;
}

void StrikerImpl::process_sys_status(const mavlink_message_t& message)
{
    mavlink_sys_status_t system_status;
    mavlink_msg_sys_status_decode(&message, &system_status);

    Striker::SysStatus received_sys_status{};
    received_sys_status.onboard_control_sensors_present =
        system_status.onboard_control_sensors_present;
    received_sys_status.onboard_control_sensors_enabled =
        system_status.onboard_control_sensors_enabled;
    received_sys_status.onboard_control_sensors_health =
        system_status.onboard_control_sensors_health;
    received_sys_status.load = system_status.load;
    received_sys_status.voltage_battery = system_status.voltage_battery;
    received_sys_status.current_battery = system_status.current_battery;
    received_sys_status.battery_remaining = system_status.battery_remaining;
    received_sys_status.drop_rate_comm = system_status.drop_rate_comm;
    received_sys_status.errors_comm = system_status.errors_comm;
    received_sys_status.errors_count1 = system_status.errors_count1;
    received_sys_status.errors_count2 = system_status.errors_count2;
    received_sys_status.errors_count3 = system_status.errors_count3;
    received_sys_status.errors_count4 = system_status.errors_count4;
    received_sys_status.onboard_control_sensors_present_extended =
        system_status.onboard_control_sensors_present_extended;
    received_sys_status.onboard_control_sensors_enabled_extended =
        system_status.onboard_control_sensors_enabled_extended;
    received_sys_status.onboard_control_sensors_health_extended =
        system_status.onboard_control_sensors_health_extended;
    set_sys_status(received_sys_status);

    std::lock_guard<std::mutex> lock(_subscription_sys_status_mutex);
    _sys_status_subscriptions.queue(
        sys_status(), [this](const auto& func) { _system_impl->call_user_callback(func); });
}

} // namespace mavsdk