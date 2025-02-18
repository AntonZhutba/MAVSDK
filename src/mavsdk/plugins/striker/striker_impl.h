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



private:
    void process_heartbeat(const mavlink_message_t& message);
    void process_sys_status(const mavlink_message_t& message);
    
    void set_heartbeat(Striker::Heartbeat heartbeat);
    void set_sys_status(Striker::SysStatus sys_status);
    
    mutable std::mutex _heartbeat_mutex;
    mutable std::mutex _sys_status_mutex;
    
    Striker::Heartbeat _heartbeat{};
    Striker::SysStatus _sys_status{};
    
    CallbackList<Striker::Heartbeat> _heartbeat_subscriptions;
    CallbackList<Striker::SysStatus> _sys_status_subscriptions;
    
    std::mutex _subscription_heartbeat_mutex;
    std::mutex _subscription_sys_status_mutex;

};

} // namespace mavsdk