#include "striker_impl.h"
#include "system.h"
#include "mavlink_include.h"
#include "plugin_impl_base.h"
#include "mavsdk_impl.h"
#include "callback_list.tpp"

namespace mavsdk {

template class CallbackList<Striker::Heartbeat>;
template class CallbackList<Striker::SysStatus>;
template class CallbackList<Striker::RcChannel>;
template class CallbackList<Striker::Magnitometer>;
template class CallbackList<Striker::BatteryVoltages>;
template class CallbackList<std::vector<Striker::AvailableMode>>;
template class CallbackList<Striker::ActuatorServosStatus>;

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

    _system_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_RC_CHANNELS,
        [this](const mavlink_message_t& message) { process_rc_channel(message); },
        this);

    _system_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_HIGHRES_IMU,
        [this](const mavlink_message_t& message) { process_magnitometer(message); },
        this);

    _system_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_BATTERY_STATUS,
        [this](const mavlink_message_t& message) { process_battery_voltages(message); },
        this);

    _system_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR,
        [this](const mavlink_message_t& message) { process_available_modes_monitor(message); },
        this);

    _system_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_AVAILABLE_MODES,
        [this](const mavlink_message_t& message) { process_available_modes(message); },
        this);

    _system_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_ACTUATOR_SERVOS_STATUS,
        [this](const mavlink_message_t& message) { process_actuator_servos_status(message); },
        this);
}

void StrikerImpl::deinit()
{
    _system_impl->unregister_all_mavlink_message_handlers(this);
}

void StrikerImpl::enable() {}

void StrikerImpl::disable() {}

// -- Heartbeat --

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

// -- SysStatus --

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

// -- Rc channels --
Striker::RcChannelHandle
StrikerImpl::subscribe_rc_channel(const Striker::RcChannelCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_rc_channel_mutex);
    return _rc_channel_subscriptions.subscribe(callback);
}

void StrikerImpl::unsubscribe_rc_channel(Striker::RcChannelHandle handle)
{
    std::lock_guard<std::mutex> lock(_subscription_rc_channel_mutex);
    _rc_channel_subscriptions.unsubscribe(handle);
}

Striker::RcChannel StrikerImpl::rc_channel() const
{
    std::lock_guard<std::mutex> lock(_rc_channel_mutex);
    return _rc_channel;
}

void StrikerImpl::process_rc_channel(const mavlink_message_t& message)
{
    mavlink_rc_channels_t rc_channels;
    mavlink_msg_rc_channels_decode(&message, &rc_channels);

    set_rc_channel(rc_channels);

    std::lock_guard<std::mutex> lock(_subscription_rc_channel_mutex);
    _rc_channel_subscriptions.queue(
        rc_channel(), [this](const auto& func) { _system_impl->call_user_callback(func); });
}

void StrikerImpl::set_rc_channel(const mavlink_rc_channels_t& rc_channel)
{
    std::lock_guard<std::mutex> lock(_rc_channel_mutex);

    _rc_channel.time_boot_ms = rc_channel.time_boot_ms;
    _rc_channel.chancount = rc_channel.chancount;

    _rc_channel.chan1_raw = rc_channel.chan1_raw;
    _rc_channel.chan2_raw = rc_channel.chan2_raw;
    _rc_channel.chan3_raw = rc_channel.chan3_raw;
    _rc_channel.chan4_raw = rc_channel.chan4_raw;
    _rc_channel.chan5_raw = rc_channel.chan5_raw;
    _rc_channel.chan6_raw = rc_channel.chan6_raw;
    _rc_channel.chan7_raw = rc_channel.chan7_raw;
    _rc_channel.chan8_raw = rc_channel.chan8_raw;
    _rc_channel.chan9_raw = rc_channel.chan9_raw;
    _rc_channel.chan10_raw = rc_channel.chan10_raw;
    _rc_channel.chan11_raw = rc_channel.chan11_raw;
    _rc_channel.chan12_raw = rc_channel.chan12_raw;
    _rc_channel.chan13_raw = rc_channel.chan13_raw;
    _rc_channel.chan14_raw = rc_channel.chan14_raw;
    _rc_channel.chan15_raw = rc_channel.chan15_raw;
    _rc_channel.chan16_raw = rc_channel.chan16_raw;
    _rc_channel.chan17_raw = rc_channel.chan17_raw;
    _rc_channel.chan18_raw = rc_channel.chan18_raw;

    _rc_channel.rssi = rc_channel.rssi;
}

// -- Magnitometer --

Striker::MagnitometerHandle
StrikerImpl::subscribe_magnitometer(const Striker::MagnitometerCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_magnitometer_mutex);
    return _magnitometer_subscriptions.subscribe(callback);
}

void StrikerImpl::unsubscribe_magnitometer(Striker::MagnitometerHandle handle)
{
    std::lock_guard<std::mutex> lock(_subscription_magnitometer_mutex);
    _magnitometer_subscriptions.unsubscribe(handle);
}

Striker::Magnitometer StrikerImpl::magnitometer() const
{
    std::lock_guard<std::mutex> lock(_magnitometer_mutex);
    return _magnitometer;
}

void StrikerImpl::process_magnitometer(const mavlink_message_t& message)
{
    mavlink_highres_imu_t imu;
    mavlink_msg_highres_imu_decode(&message, &imu);

    set_magnitometer(imu);

    std::lock_guard<std::mutex> lock(_subscription_magnitometer_mutex);
    _magnitometer_subscriptions.queue(
        magnitometer(), [this](const auto& func) { _system_impl->call_user_callback(func); });
}

void StrikerImpl::set_magnitometer(const mavlink_highres_imu_t& mav_magnitometer)
{
    std::lock_guard<std::mutex> lock(_magnitometer_mutex);

    _magnitometer.x = mav_magnitometer.xmag;
    _magnitometer.y = mav_magnitometer.ymag;
    _magnitometer.z = mav_magnitometer.zmag;
    _magnitometer.magnetic_heading = atan2(_magnitometer.y, _magnitometer.x) * 180 / M_PI;
}

// -- Battery voltages --

Striker::BatteryVoltagesHandle
StrikerImpl::subscribe_battery_voltages(const Striker::BatteryVoltagesCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_battery_voltages_mutex);
    return _battery_voltages_subscriptions.subscribe(callback);
}

void StrikerImpl::unsubscribe_battery_voltages(Striker::BatteryVoltagesHandle handle)
{
    std::lock_guard<std::mutex> lock(_subscription_battery_voltages_mutex);
    _battery_voltages_subscriptions.unsubscribe(handle);
}

Striker::BatteryVoltages StrikerImpl::battery_voltages() const
{
    std::lock_guard<std::mutex> lock(_battery_voltages_mutex);
    return _battery_voltages;
}

void StrikerImpl::process_battery_voltages(const mavlink_message_t& message)
{
    mavlink_battery_status_t battery_status;
    mavlink_msg_battery_status_decode(&message, &battery_status);

    set_battery_voltages(battery_status);

    std::lock_guard<std::mutex> lock(_subscription_battery_voltages_mutex);
    _battery_voltages_subscriptions.queue(
        battery_voltages(), [this](const auto& func) { _system_impl->call_user_callback(func); });
}

void StrikerImpl::set_battery_voltages(const mavlink_battery_status_t& battery_status)
{
    std::lock_guard<std::mutex> lock(_battery_voltages_mutex);

    _battery_voltages.voltages.assign(
        battery_status.voltages,
        battery_status.voltages + MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN);
    _battery_voltages.ext_voltages.assign(
        battery_status.voltages_ext,
        battery_status.voltages_ext + MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_LEN);
}

// -- Available modes monitor --

Striker::AvailableModesHandle
StrikerImpl::subscribe_available_modes(const Striker::AvailableModesCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_available_modes_mutex);
    return _available_modes_subscriptions.subscribe(callback);
}

void StrikerImpl::unsubscribe_available_modes(Striker::AvailableModesHandle handle)
{
    std::lock_guard<std::mutex> lock(_subscription_available_modes_mutex);
    _available_modes_subscriptions.unsubscribe(handle);
}

std::vector<Striker::AvailableMode> StrikerImpl::available_modes() const
{
    std::lock_guard<std::mutex> lock(_available_modes_mutex);
    return _available_modes;
}

void StrikerImpl::process_available_modes_monitor(const mavlink_message_t& message)
{
    mavlink_available_modes_monitor_t availableModesMonitor;
    mavlink_msg_available_modes_monitor_decode(&message, &availableModesMonitor);

    if (_lastSeq != availableModesMonitor.seq) {
        _lastSeq = availableModesMonitor.seq;
        try_request_available_modes();
    }
}

void StrikerImpl::try_request_available_modes()
{
    if (_requestActive) {
        // If we are in the middle of waiting for a request, wait for the response first
        _wantReset = true;
        return;
    }
    _next_modes.clear();

    // Request one at a time. This could be improved by requesting all, but we can't use
    StrikerImpl::request_available_modes(1);
}

void StrikerImpl::request_available_modes(uint32_t modeIndex)
{
    _requestActive = true;

    MavlinkCommandSender::CommandLong command{};
    command.target_component_id = _system_impl->get_autopilot_id();

    command.command = MAV_CMD_REQUEST_MESSAGE;
    command.params.maybe_param1 = static_cast<float>(MAVLINK_MSG_ID_AVAILABLE_MODES);
    command.params.maybe_param2 = static_cast<float>(modeIndex);

    _system_impl->send_command_async(command, [this](MavlinkCommandSender::Result result, float) {
        if (result != MavlinkCommandSender::Result::Success) {
            LogErr() << "Requesting available modes failed: "
                     << static_cast<int>(StrikerImpl::mode_result_from_command_result(result));
        }
    });
}

void StrikerImpl::process_available_modes(const mavlink_message_t& message)
{
    _requestActive = false;
    if (_wantReset) {
        _wantReset = false;
        try_request_available_modes();
        return;
    }

    mavlink_available_modes_t availableModes;
    mavlink_msg_available_modes_decode(&message, &availableModes);
    bool cannotBeSet = availableModes.properties & MAV_MODE_PROPERTY_NOT_USER_SELECTABLE;
    bool advanced = availableModes.properties & MAV_MODE_PROPERTY_ADVANCED;
    availableModes.mode_name[sizeof(availableModes.mode_name) - 1] = '\0';
    std::string name = availableModes.mode_name;
    switch (availableModes.standard_mode) {
        case MAV_STANDARD_MODE_POSITION_HOLD:
            name = "Position";
            break;
        case MAV_STANDARD_MODE_ORBIT:
            name = "Orbit";
            break;
        case MAV_STANDARD_MODE_CRUISE:
            name = "Cruise";
            break;
        case MAV_STANDARD_MODE_ALTITUDE_HOLD:
            name = "Altitude";
            break;
        case MAV_STANDARD_MODE_RETURN_HOME:
            name = "Return";
            break;
        case MAV_STANDARD_MODE_SAFE_RECOVERY:
            name = "Safe Recovery";
            break;
        case MAV_STANDARD_MODE_MISSION:
            name = "Mission";
            break;
        case MAV_STANDARD_MODE_LAND:
            name = "Land";
            break;
        case MAV_STANDARD_MODE_TAKEOFF:
            name = "Takeoff";
            break;
    }
    if (name == "Takeoff" || name == "VTOL Takeoff" || name == "Orbit" || name == "Land" ||
        name == "Return") { // These are exposed in the UI as separate buttons
        cannotBeSet = true;
    }

    _next_modes[availableModes.custom_mode] = Mode{
        name,
        availableModes.standard_mode,
        availableModes.number_modes,
        availableModes.mode_index,
        availableModes.custom_mode,
        availableModes.properties,
        advanced,
        cannotBeSet};

    if (availableModes.mode_index >= availableModes.number_modes) { // We are done

        ensureUniqueModeNames();
        std::vector<Striker::AvailableMode> list_available_modes;
        for (const auto& mode : _modes) {
            list_available_modes.push_back(Striker::AvailableMode{
                mode.second.numberModes,
                mode.second.modeIndex,
                mode.second.standardMode,
                mode.second.customMode,
                mode.second.properties,
                mode.second.nameMode,
            });
        }

        // Pass the vector to set_available_modes
        set_available_modes(std::move(list_available_modes));

        std::lock_guard<std::mutex> lock(_subscription_available_modes_mutex);
        _available_modes_subscriptions.queue(available_modes(), [this](const auto& func) {
            _system_impl->call_user_callback(func);
        });

    } else {
        request_available_modes(availableModes.mode_index + 1);
    }
}

void StrikerImpl::set_available_modes(std::vector<Striker::AvailableMode> available_modes)
{
    std::lock_guard<std::mutex> lock(_available_modes_mutex);
    _available_modes = std::move(available_modes);
}

void StrikerImpl::ensureUniqueModeNames()
{
    // Ensure mode names are unique. This should generally already be the case, but e.g. during
    // development when restarting dynamic modes, it might not be.
    for (auto iter = _modes.begin(); iter != _modes.end(); ++iter) {
        int duplicateIdx = 0;
        for (auto iter2 = std::next(iter); iter2 != _modes.end(); ++iter2) {
            if (iter->second.nameMode == iter2->second.nameMode) {
                ++duplicateIdx;
                iter2->second.nameMode += " (" + std::to_string(duplicateIdx) + ")";
            }
        }
    }
}

void StrikerImpl::request_available_modes_async(const Striker::ResultCallback callback)
{
    // Start the request for available modes. We will receive them in the
    // process_available_modes function. We will only request one mode at a time.
    StrikerImpl::request_available_modes(1);

    if (callback) {
        auto temp_callback = callback;
        _system_impl->call_user_callback(
            [temp_callback]() { temp_callback(Striker::Result::Success); });
    }
}

Striker::Result StrikerImpl::request_available_modes()
{
    auto prom = std::promise<Striker::Result>();
    auto fut = prom.get_future();

    request_available_modes_async([&prom](Striker::Result result) { prom.set_value(result); });

    return fut.get();
}

// -- Set mode --

void StrikerImpl::set_manual_flight_mode_async(
    uint32_t mode,
    uint32_t custom_mode,
    uint32_t custom_sub_mode,
    const Striker::ResultCallback callback)
{
    MavlinkCommandSender::CommandLong command{};
    command.target_component_id = _system_impl->get_autopilot_id();

    command.command = MAV_CMD_DO_SET_MODE;
    command.params.maybe_param1 = static_cast<float>(mode);
    command.params.maybe_param2 = static_cast<float>(custom_mode);
    command.params.maybe_param3 = static_cast<float>(custom_sub_mode);

    _system_impl->send_command_async(
        command, [this, callback](MavlinkCommandSender::Result result, float) {
            command_result_callback(result, callback);
        });
}

void StrikerImpl::command_result_callback(
    MavlinkCommandSender::Result command_result, const Striker::ResultCallback& callback) const
{
    if (command_result == MavlinkCommandSender::Result::InProgress) {
        // We only want to return once, so we can't call the callback on progress updates.
        return;
    }

    Striker::Result action_result = mode_result_from_command_result(command_result);

    if (callback) {
        auto temp_callback = callback;
        _system_impl->call_user_callback(
            [temp_callback, action_result]() { temp_callback(action_result); });
    }
}

void StrikerImpl::command_rate_result_callback(
    MavlinkCommandSender::Result command_result, const Striker::ResultCallback& callback)
{
    Striker::Result action_result = rate_result_from_command_result(command_result);

    callback(action_result);
}

Striker::Result StrikerImpl::mode_result_from_command_result(MavlinkCommandSender::Result result)
{
    switch (result) {
        case MavlinkCommandSender::Result::Success:
            return Striker::Result::Success;
        case MavlinkCommandSender::Result::NoSystem:
            return Striker::Result::NoSystem;
        case MavlinkCommandSender::Result::ConnectionError:
            return Striker::Result::ConnectionError;
        case MavlinkCommandSender::Result::Busy:
            return Striker::Result::Busy;
        case MavlinkCommandSender::Result::Denied:
            // Fallthrough
        case MavlinkCommandSender::Result::TemporarilyRejected:
            return Striker::Result::CommandDenied;
        case MavlinkCommandSender::Result::Failed:
            return Striker::Result::Failed;
        case MavlinkCommandSender::Result::Timeout:
            return Striker::Result::Timeout;
        case MavlinkCommandSender::Result::Unsupported:
            return Striker::Result::Unsupported;
        default:
            return Striker::Result::Unknown;
    }
}

Striker::Result StrikerImpl::rate_result_from_command_result(MavlinkCommandSender::Result result)
{
    switch (result) {
        case MavlinkCommandSender::Result::Success:
            return Striker::Result::Success;
        case MavlinkCommandSender::Result::NoSystem:
            return Striker::Result::NoSystem;
        case MavlinkCommandSender::Result::ConnectionError:
            return Striker::Result::ConnectionError;
        case MavlinkCommandSender::Result::Busy:
            return Striker::Result::Busy;
        case MavlinkCommandSender::Result::Denied:
            // Fallthrough
        case MavlinkCommandSender::Result::Timeout:
            return Striker::Result::Timeout;
        case MavlinkCommandSender::Result::Unsupported:
            return Striker::Result::Unsupported;
        default:
            return Striker::Result::Unknown;
    }
}

Striker::Result
StrikerImpl::set_manual_flight_mode(uint32_t mode, uint32_t custom_mode, uint32_t custom_sub_mode)
{
    if (_system_impl->autopilot() == Autopilot::Px4) {
        auto prom = std::promise<Striker::Result>();
        auto fut = prom.get_future();

        set_manual_flight_mode_async(
            mode, custom_mode, custom_sub_mode, [&prom](Striker::Result result) {
                prom.set_value(result);
            });

        return fut.get();
    }

    return Striker::Result::Unknown;
}

// -- Actuator servos status --
Striker::ActuatorServosStatusHandle
StrikerImpl::subscribe_actuator_servos_status(const Striker::ActuatorServosStatusCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_actuator_servos_status_mutex);
    return _actuator_servos_status_subscriptions.subscribe(callback);
}

void StrikerImpl::unsubscribe_actuator_servos_status(Striker::ActuatorServosStatusHandle handle)
{
    std::lock_guard<std::mutex> lock(_subscription_actuator_servos_status_mutex);
    _actuator_servos_status_subscriptions.unsubscribe(handle);
}

Striker::ActuatorServosStatus StrikerImpl::actuator_servos_status() const
{
    std::lock_guard<std::mutex> lock(_actuator_servos_status_mutex);
    return _actuator_servos_status;
}

void StrikerImpl::set_actuator_servos_status(Striker::ActuatorServosStatus actuator_servos_status)
{
    std::lock_guard<std::mutex> lock(_actuator_servos_status_mutex);
    _actuator_servos_status = actuator_servos_status;
}

void StrikerImpl::process_actuator_servos_status(const mavlink_message_t& message)
{
    mavlink_actuator_servos_status_t actuator_servos_status_msg;
    mavlink_msg_actuator_servos_status_decode(&message, &actuator_servos_status_msg);

    Striker::ActuatorServosStatus received_actuator_servos_status{};
    received_actuator_servos_status.time_usec = actuator_servos_status_msg.time_usec;
    received_actuator_servos_status.control.assign(
        std::begin(actuator_servos_status_msg.control),
        std::end(actuator_servos_status_msg.control));
    set_actuator_servos_status(received_actuator_servos_status);

    std::lock_guard<std::mutex> lock(_subscription_actuator_servos_status_mutex);
    _actuator_servos_status_subscriptions.queue(actuator_servos_status(), [this](const auto& func) {
        _system_impl->call_user_callback(func);
    });
}

Striker::Result StrikerImpl::set_rate_actuator_servos_status(double rate_hz)
{
    return rate_result_from_command_result(
        _system_impl->set_msg_rate(MAVLINK_MSG_ID_ACTUATOR_SERVOS_STATUS, rate_hz));
}

void StrikerImpl::set_rate_actuator_servos_status_async(
    double rate_hz, const Striker::ResultCallback callback)
{
    {
        _system_impl->set_msg_rate_async(
            MAVLINK_MSG_ID_ACTUATOR_SERVOS_STATUS,
            rate_hz,
            [callback](MavlinkCommandSender::Result command_result, float) {
                command_rate_result_callback(command_result, callback);
            });
    }
}
} // namespace mavsdk