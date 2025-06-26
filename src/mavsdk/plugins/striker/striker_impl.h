#pragma once

#include "plugins/striker/striker.h"
#include "plugin_impl_base.h"
#include "callback_list.h"

#include "plugin_impl_base.h"

namespace mavsdk {

class StrikerImpl : public PluginImplBase {
public:
    explicit StrikerImpl(System& system);
    explicit StrikerImpl(std::shared_ptr<System> system);

    ~StrikerImpl() override;

    void init() override;
    void deinit() override;

    void enable() override;
    void disable() override;

    // Heartbeat subscription
    Striker::HeartbeatHandle subscribe_heartbeat(const Striker::HeartbeatCallback& callback);
    void unsubscribe_heartbeat(Striker::HeartbeatHandle handle);
    Striker::Heartbeat heartbeat() const;

    // SysStatus subscription
    Striker::SysStatusHandle subscribe_sys_status(const Striker::SysStatusCallback& callback);
    void unsubscribe_sys_status(Striker::SysStatusHandle handle);
    Striker::SysStatus sys_status() const;

    // RC_Channel subscription
    Striker::RcChannelHandle subscribe_rc_channel(const Striker::RcChannelCallback& callback);
    void unsubscribe_rc_channel(Striker::RcChannelHandle handle);
    Striker::RcChannel rc_channel() const;

    // Magnitometer subscription
    Striker::MagnitometerHandle
    subscribe_magnitometer(const Striker::MagnitometerCallback& callback);
    void unsubscribe_magnitometer(Striker::MagnitometerHandle handle);
    Striker::Magnitometer magnitometer() const;

    // Battery voltages subscription
    Striker::BatteryVoltagesHandle
    subscribe_battery_voltages(const Striker::BatteryVoltagesCallback& callback);
    void unsubscribe_battery_voltages(Striker::BatteryVoltagesHandle handle);
    Striker::BatteryVoltages battery_voltages() const;

    // Available modes subscription
    Striker::AvailableModesHandle
    subscribe_available_modes(const Striker::AvailableModesCallback& callback);
    void unsubscribe_available_modes(Striker::AvailableModesHandle handle);
    std::vector<Striker::AvailableMode> available_modes() const;

    // Request available modes
    void request_available_modes_async(const Striker::ResultCallback callback);
    Striker::Result request_available_modes();

    // Set mode request
    void set_manual_flight_mode_async(
        uint32_t mode,
        uint32_t custom_mode,
        uint32_t custom_sub_mode,
        const Striker::ResultCallback callback);

    Striker::Result
    set_manual_flight_mode(uint32_t mode, uint32_t custom_mode, uint32_t custom_sub_mode);

    // Actuator servos status subscription
    Striker::ActuatorServosStatusHandle
    subscribe_actuator_servos_status(const Striker::ActuatorServosStatusCallback& callback);
    void unsubscribe_actuator_servos_status(Striker::ActuatorServosStatusHandle handle);
    Striker::ActuatorServosStatus actuator_servos_status() const;
    void
    set_rate_actuator_servos_status_async(double rate_hz, const Striker::ResultCallback callback);
    Striker::Result set_rate_actuator_servos_status(double rate_hz);

    // CAA confidence level subscription
    Striker::CaaConfidenceLevelHandle
    subscribe_caa_confidence_level(const Striker::CaaConfidenceLevelCallback& callback);
    void unsubscribe_caa_confidence_level(Striker::CaaConfidenceLevelHandle handle);
    Striker::CaaConfidenceLevel caa_confidence_level() const;
    void 
    set_rate_caa_confidence_level_async(double rate_hz, const Striker::ResultCallback callback);
    Striker::Result set_rate_caa_confidence_level(double rate_hz);

private:
    void process_heartbeat(const mavlink_message_t& message);
    void process_sys_status(const mavlink_message_t& message);
    void process_rc_channel(const mavlink_message_t& message);
    void process_magnitometer(const mavlink_message_t& message);
    void process_battery_voltages(const mavlink_message_t& message);
    void process_available_modes_monitor(const mavlink_message_t& message);
    void process_available_modes(const mavlink_message_t& message);
    void process_actuator_servos_status(const mavlink_message_t& message);
    void process_caa_confidence_level(const mavlink_message_t& message);

    void set_heartbeat(Striker::Heartbeat heartbeat);
    void set_sys_status(Striker::SysStatus sys_status);
    void set_rc_channel(const mavlink_rc_channels_t& rc_channel);
    void set_magnitometer(const mavlink_highres_imu_t& mav_magnitometer);
    void set_battery_voltages(const mavlink_battery_status_t& battery_status);
    void set_available_modes(std::vector<Striker::AvailableMode> available_modes);
    void set_actuator_servos_status(Striker::ActuatorServosStatus actuator_servos_status);
    void set_caa_confidence_level(Striker::CaaConfidenceLevel caa_confidence_level);

    void try_request_available_modes();
    void request_available_modes(uint32_t mode_index);
    void ensureUniqueModeNames();
    static void command_rate_result_callback(
        MavlinkCommandSender::Result command_result, const Striker::ResultCallback& callback);
    void command_result_callback(
        MavlinkCommandSender::Result command_result, const Striker::ResultCallback& callback) const;
    static Striker::Result mode_result_from_command_result(MavlinkCommandSender::Result result);
    static Striker::Result rate_result_from_command_result(MavlinkCommandSender::Result result);

    mutable std::mutex _heartbeat_mutex;
    mutable std::mutex _sys_status_mutex;
    mutable std::mutex _rc_channel_mutex;
    mutable std::mutex _magnitometer_mutex;
    mutable std::mutex _battery_voltages_mutex;
    mutable std::mutex _available_modes_mutex;
    mutable std::mutex _actuator_servos_status_mutex;
    mutable std::mutex _caa_confidence_level_mutex;

    Striker::Heartbeat _heartbeat{};
    Striker::SysStatus _sys_status{};
    Striker::RcChannel _rc_channel{};
    Striker::Magnitometer _magnitometer{};
    Striker::BatteryVoltages _battery_voltages{};
    std::vector<Striker::AvailableMode> _available_modes{};
    struct Mode {
        std::string nameMode;
        uint8_t standardMode;
        uint32_t numberModes{};
        uint32_t modeIndex{};
        uint32_t customMode{};
        uint32_t properties{};
        bool advanced;
        bool cannotBeSet;
    };
    int _lastSeq{-1};
    bool _requestActive{false};
    bool _wantReset{false};
    std::map<uint32_t, Mode> _next_modes; ///< Modes added by current request
    std::map<uint32_t, Mode> _modes;
    Striker::ActuatorServosStatus _actuator_servos_status{};
    Striker::CaaConfidenceLevel _caa_confidence_level{};

    CallbackList<Striker::Heartbeat> _heartbeat_subscriptions;
    CallbackList<Striker::SysStatus> _sys_status_subscriptions;
    CallbackList<Striker::RcChannel> _rc_channel_subscriptions;
    CallbackList<Striker::Magnitometer> _magnitometer_subscriptions;
    CallbackList<Striker::BatteryVoltages> _battery_voltages_subscriptions;
    CallbackList<std::vector<Striker::AvailableMode>> _available_modes_subscriptions;
    CallbackList<Striker::ActuatorServosStatus> _actuator_servos_status_subscriptions;
    CallbackList<Striker::CaaConfidenceLevel> _caa_confidence_level_subscriptions;

    std::mutex _subscription_heartbeat_mutex;
    std::mutex _subscription_sys_status_mutex;
    std::mutex _subscription_rc_channel_mutex;
    std::mutex _subscription_magnitometer_mutex;
    std::mutex _subscription_battery_voltages_mutex;
    std::mutex _subscription_available_modes_mutex;
    std::mutex _subscription_actuator_servos_status_mutex;
    std::mutex _subscription_caa_confidence_level_mutex;
};
} // namespace mavsdk