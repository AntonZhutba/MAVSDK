#include "striker_impl.h"

namespace mavsdk {


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

void StrikerImpl::init() {}

void StrikerImpl::deinit() {}


void StrikerImpl::enable() {}

void StrikerImpl::disable() {}




    
Striker::HeartbeatHandle StrikerImpl::subscribe_heartbeat(const Striker::HeartbeatCallback& callback)
{
    
    UNUSED(callback);
}

void StrikerImpl::unsubscribe_heartbeat(Striker::HeartbeatHandle handle)
{
    UNUSED(handle);
}
    




Striker::Heartbeat
StrikerImpl::heartbeat() const
{
    

    return {};
}



    
Striker::SysStatusHandle StrikerImpl::subscribe_sys_status(const Striker::SysStatusCallback& callback)
{
    
    UNUSED(callback);
}

void StrikerImpl::unsubscribe_sys_status(Striker::SysStatusHandle handle)
{
    UNUSED(handle);
}
    




Striker::SysStatus
StrikerImpl::sys_status() const
{
    

    return {};
}



} // namespace mavsdk