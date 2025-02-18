#pragma once

#include "plugins/striker/striker.h"

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




        
    Striker::HeartbeatHandle subscribe_heartbeat(const Striker::HeartbeatCallback& callback);

    void unsubscribe_heartbeat(Striker::HeartbeatHandle handle);
        



    Striker::Heartbeat heartbeat() const;



        
    Striker::SysStatusHandle subscribe_sys_status(const Striker::SysStatusCallback& callback);

    void unsubscribe_sys_status(Striker::SysStatusHandle handle);
        



    Striker::SysStatus sys_status() const;



private:
};

} // namespace mavsdk