// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see
// https://github.com/mavlink/MAVSDK-Proto/blob/master/protos/action_server/action_server.proto)

#include "action_server/action_server.grpc.pb.h"
#include "plugins/action_server/action_server.h"

#include "mavsdk.h"

#include "lazy_server_plugin.h"

#include "log.h"
#include <atomic>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <vector>

namespace mavsdk {
namespace mavsdk_server {

template<
    typename ActionServer = ActionServer,
    typename LazyServerPlugin = LazyServerPlugin<ActionServer>>

class ActionServerServiceImpl final : public rpc::action_server::ActionServerService::Service {
public:
    ActionServerServiceImpl(LazyServerPlugin& lazy_plugin) : _lazy_plugin(lazy_plugin) {}

    template<typename ResponseType>
    void fillResponseWithResult(ResponseType* response, mavsdk::ActionServer::Result& result) const
    {
        auto rpc_result = translateToRpcResult(result);

        auto* rpc_action_server_result = new rpc::action_server::ActionServerResult();
        rpc_action_server_result->set_result(rpc_result);
        std::stringstream ss;
        ss << result;
        rpc_action_server_result->set_result_str(ss.str());

        response->set_allocated_action_server_result(rpc_action_server_result);
    }

    static rpc::action_server::FlightMode
    translateToRpcFlightMode(const mavsdk::ActionServer::FlightMode& flight_mode)
    {
        switch (flight_mode) {
            default:
                LogErr() << "Unknown flight_mode enum value: " << static_cast<int>(flight_mode);
            // FALLTHROUGH
            case mavsdk::ActionServer::FlightMode::Unknown:
                return rpc::action_server::FLIGHT_MODE_UNKNOWN;
            case mavsdk::ActionServer::FlightMode::Ready:
                return rpc::action_server::FLIGHT_MODE_READY;
            case mavsdk::ActionServer::FlightMode::Takeoff:
                return rpc::action_server::FLIGHT_MODE_TAKEOFF;
            case mavsdk::ActionServer::FlightMode::Hold:
                return rpc::action_server::FLIGHT_MODE_HOLD;
            case mavsdk::ActionServer::FlightMode::Mission:
                return rpc::action_server::FLIGHT_MODE_MISSION;
            case mavsdk::ActionServer::FlightMode::ReturnToLaunch:
                return rpc::action_server::FLIGHT_MODE_RETURN_TO_LAUNCH;
            case mavsdk::ActionServer::FlightMode::Land:
                return rpc::action_server::FLIGHT_MODE_LAND;
            case mavsdk::ActionServer::FlightMode::Offboard:
                return rpc::action_server::FLIGHT_MODE_OFFBOARD;
            case mavsdk::ActionServer::FlightMode::FollowMe:
                return rpc::action_server::FLIGHT_MODE_FOLLOW_ME;
            case mavsdk::ActionServer::FlightMode::Manual:
                return rpc::action_server::FLIGHT_MODE_MANUAL;
            case mavsdk::ActionServer::FlightMode::Altctl:
                return rpc::action_server::FLIGHT_MODE_ALTCTL;
            case mavsdk::ActionServer::FlightMode::Posctl:
                return rpc::action_server::FLIGHT_MODE_POSCTL;
            case mavsdk::ActionServer::FlightMode::Acro:
                return rpc::action_server::FLIGHT_MODE_ACRO;
            case mavsdk::ActionServer::FlightMode::Stabilized:
                return rpc::action_server::FLIGHT_MODE_STABILIZED;
            case mavsdk::ActionServer::FlightMode::RattitudeLegacy:
                return rpc::action_server::FLIGHT_MODE_RATTITUDE_LEGACY;
            case mavsdk::ActionServer::FlightMode::Chase:
                return rpc::action_server::FLIGHT_MODE_CHASE;
        }
    }

    static mavsdk::ActionServer::FlightMode
    translateFromRpcFlightMode(const rpc::action_server::FlightMode flight_mode)
    {
        switch (flight_mode) {
            default:
                LogErr() << "Unknown flight_mode enum value: " << static_cast<int>(flight_mode);
            // FALLTHROUGH
            case rpc::action_server::FLIGHT_MODE_UNKNOWN:
                return mavsdk::ActionServer::FlightMode::Unknown;
            case rpc::action_server::FLIGHT_MODE_READY:
                return mavsdk::ActionServer::FlightMode::Ready;
            case rpc::action_server::FLIGHT_MODE_TAKEOFF:
                return mavsdk::ActionServer::FlightMode::Takeoff;
            case rpc::action_server::FLIGHT_MODE_HOLD:
                return mavsdk::ActionServer::FlightMode::Hold;
            case rpc::action_server::FLIGHT_MODE_MISSION:
                return mavsdk::ActionServer::FlightMode::Mission;
            case rpc::action_server::FLIGHT_MODE_RETURN_TO_LAUNCH:
                return mavsdk::ActionServer::FlightMode::ReturnToLaunch;
            case rpc::action_server::FLIGHT_MODE_LAND:
                return mavsdk::ActionServer::FlightMode::Land;
            case rpc::action_server::FLIGHT_MODE_OFFBOARD:
                return mavsdk::ActionServer::FlightMode::Offboard;
            case rpc::action_server::FLIGHT_MODE_FOLLOW_ME:
                return mavsdk::ActionServer::FlightMode::FollowMe;
            case rpc::action_server::FLIGHT_MODE_MANUAL:
                return mavsdk::ActionServer::FlightMode::Manual;
            case rpc::action_server::FLIGHT_MODE_ALTCTL:
                return mavsdk::ActionServer::FlightMode::Altctl;
            case rpc::action_server::FLIGHT_MODE_POSCTL:
                return mavsdk::ActionServer::FlightMode::Posctl;
            case rpc::action_server::FLIGHT_MODE_ACRO:
                return mavsdk::ActionServer::FlightMode::Acro;
            case rpc::action_server::FLIGHT_MODE_STABILIZED:
                return mavsdk::ActionServer::FlightMode::Stabilized;
            case rpc::action_server::FLIGHT_MODE_RATTITUDE_LEGACY:
                return mavsdk::ActionServer::FlightMode::RattitudeLegacy;
            case rpc::action_server::FLIGHT_MODE_CHASE:
                return mavsdk::ActionServer::FlightMode::Chase;
        }
    }

    static std::unique_ptr<rpc::action_server::AllowableFlightModes>
    translateToRpcAllowableFlightModes(
        const mavsdk::ActionServer::AllowableFlightModes& allowable_flight_modes)
    {
        auto rpc_obj = std::make_unique<rpc::action_server::AllowableFlightModes>();

        rpc_obj->set_can_auto_mode(allowable_flight_modes.can_auto_mode);

        rpc_obj->set_can_guided_mode(allowable_flight_modes.can_guided_mode);

        rpc_obj->set_can_stabilize_mode(allowable_flight_modes.can_stabilize_mode);

        return rpc_obj;
    }

    static mavsdk::ActionServer::AllowableFlightModes translateFromRpcAllowableFlightModes(
        const rpc::action_server::AllowableFlightModes& allowable_flight_modes)
    {
        mavsdk::ActionServer::AllowableFlightModes obj;

        obj.can_auto_mode = allowable_flight_modes.can_auto_mode();

        obj.can_guided_mode = allowable_flight_modes.can_guided_mode();

        obj.can_stabilize_mode = allowable_flight_modes.can_stabilize_mode();

        return obj;
    }

    static std::unique_ptr<rpc::action_server::ArmDisarm>
    translateToRpcArmDisarm(const mavsdk::ActionServer::ArmDisarm& arm_disarm)
    {
        auto rpc_obj = std::make_unique<rpc::action_server::ArmDisarm>();

        rpc_obj->set_arm(arm_disarm.arm);

        rpc_obj->set_force(arm_disarm.force);

        return rpc_obj;
    }

    static mavsdk::ActionServer::ArmDisarm
    translateFromRpcArmDisarm(const rpc::action_server::ArmDisarm& arm_disarm)
    {
        mavsdk::ActionServer::ArmDisarm obj;

        obj.arm = arm_disarm.arm();

        obj.force = arm_disarm.force();

        return obj;
    }

    static rpc::action_server::ActionServerResult::Result
    translateToRpcResult(const mavsdk::ActionServer::Result& result)
    {
        switch (result) {
            default:
                LogErr() << "Unknown result enum value: " << static_cast<int>(result);
            // FALLTHROUGH
            case mavsdk::ActionServer::Result::Unknown:
                return rpc::action_server::ActionServerResult_Result_RESULT_UNKNOWN;
            case mavsdk::ActionServer::Result::Success:
                return rpc::action_server::ActionServerResult_Result_RESULT_SUCCESS;
            case mavsdk::ActionServer::Result::NoSystem:
                return rpc::action_server::ActionServerResult_Result_RESULT_NO_SYSTEM;
            case mavsdk::ActionServer::Result::ConnectionError:
                return rpc::action_server::ActionServerResult_Result_RESULT_CONNECTION_ERROR;
            case mavsdk::ActionServer::Result::Busy:
                return rpc::action_server::ActionServerResult_Result_RESULT_BUSY;
            case mavsdk::ActionServer::Result::CommandDenied:
                return rpc::action_server::ActionServerResult_Result_RESULT_COMMAND_DENIED;
            case mavsdk::ActionServer::Result::CommandDeniedLandedStateUnknown:
                return rpc::action_server::
                    ActionServerResult_Result_RESULT_COMMAND_DENIED_LANDED_STATE_UNKNOWN;
            case mavsdk::ActionServer::Result::CommandDeniedNotLanded:
                return rpc::action_server::
                    ActionServerResult_Result_RESULT_COMMAND_DENIED_NOT_LANDED;
            case mavsdk::ActionServer::Result::Timeout:
                return rpc::action_server::ActionServerResult_Result_RESULT_TIMEOUT;
            case mavsdk::ActionServer::Result::VtolTransitionSupportUnknown:
                return rpc::action_server::
                    ActionServerResult_Result_RESULT_VTOL_TRANSITION_SUPPORT_UNKNOWN;
            case mavsdk::ActionServer::Result::NoVtolTransitionSupport:
                return rpc::action_server::
                    ActionServerResult_Result_RESULT_NO_VTOL_TRANSITION_SUPPORT;
            case mavsdk::ActionServer::Result::ParameterError:
                return rpc::action_server::ActionServerResult_Result_RESULT_PARAMETER_ERROR;
            case mavsdk::ActionServer::Result::Next:
                return rpc::action_server::ActionServerResult_Result_RESULT_NEXT;
        }
    }

    static mavsdk::ActionServer::Result
    translateFromRpcResult(const rpc::action_server::ActionServerResult::Result result)
    {
        switch (result) {
            default:
                LogErr() << "Unknown result enum value: " << static_cast<int>(result);
            // FALLTHROUGH
            case rpc::action_server::ActionServerResult_Result_RESULT_UNKNOWN:
                return mavsdk::ActionServer::Result::Unknown;
            case rpc::action_server::ActionServerResult_Result_RESULT_SUCCESS:
                return mavsdk::ActionServer::Result::Success;
            case rpc::action_server::ActionServerResult_Result_RESULT_NO_SYSTEM:
                return mavsdk::ActionServer::Result::NoSystem;
            case rpc::action_server::ActionServerResult_Result_RESULT_CONNECTION_ERROR:
                return mavsdk::ActionServer::Result::ConnectionError;
            case rpc::action_server::ActionServerResult_Result_RESULT_BUSY:
                return mavsdk::ActionServer::Result::Busy;
            case rpc::action_server::ActionServerResult_Result_RESULT_COMMAND_DENIED:
                return mavsdk::ActionServer::Result::CommandDenied;
            case rpc::action_server::
                ActionServerResult_Result_RESULT_COMMAND_DENIED_LANDED_STATE_UNKNOWN:
                return mavsdk::ActionServer::Result::CommandDeniedLandedStateUnknown;
            case rpc::action_server::ActionServerResult_Result_RESULT_COMMAND_DENIED_NOT_LANDED:
                return mavsdk::ActionServer::Result::CommandDeniedNotLanded;
            case rpc::action_server::ActionServerResult_Result_RESULT_TIMEOUT:
                return mavsdk::ActionServer::Result::Timeout;
            case rpc::action_server::
                ActionServerResult_Result_RESULT_VTOL_TRANSITION_SUPPORT_UNKNOWN:
                return mavsdk::ActionServer::Result::VtolTransitionSupportUnknown;
            case rpc::action_server::ActionServerResult_Result_RESULT_NO_VTOL_TRANSITION_SUPPORT:
                return mavsdk::ActionServer::Result::NoVtolTransitionSupport;
            case rpc::action_server::ActionServerResult_Result_RESULT_PARAMETER_ERROR:
                return mavsdk::ActionServer::Result::ParameterError;
            case rpc::action_server::ActionServerResult_Result_RESULT_NEXT:
                return mavsdk::ActionServer::Result::Next;
        }
    }

    grpc::Status SubscribeArmDisarm(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::action_server::SubscribeArmDisarmRequest* /* request */,
        grpc::ServerWriter<rpc::action_server::ArmDisarmResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            rpc::action_server::ArmDisarmResponse rpc_response;

            // For server plugins, this should never happen, they should always be constructible.
            auto result = mavsdk::ActionServer::Result::Unknown;
            fillResponseWithResult(&rpc_response, result);
            writer->Write(rpc_response);

            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        const mavsdk::ActionServer::ArmDisarmHandle handle =
            _lazy_plugin.maybe_plugin()->subscribe_arm_disarm(
                [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex, &handle](
                    mavsdk::ActionServer::Result result,
                    const mavsdk::ActionServer::ArmDisarm arm_disarm) {
                    rpc::action_server::ArmDisarmResponse rpc_response;

                    rpc_response.set_allocated_arm(translateToRpcArmDisarm(arm_disarm).release());

                    auto rpc_result = translateToRpcResult(result);
                    auto* rpc_action_server_result = new rpc::action_server::ActionServerResult();
                    rpc_action_server_result->set_result(rpc_result);
                    std::stringstream ss;
                    ss << result;
                    rpc_action_server_result->set_result_str(ss.str());
                    rpc_response.set_allocated_action_server_result(rpc_action_server_result);

                    std::unique_lock<std::mutex> lock(*subscribe_mutex);
                    if (!*is_finished && !writer->Write(rpc_response)) {
                        _lazy_plugin.maybe_plugin()->unsubscribe_arm_disarm(handle);

                        *is_finished = true;
                        unregister_stream_stop_promise(stream_closed_promise);
                        stream_closed_promise->set_value();
                    }
                });

        stream_closed_future.wait();
        std::unique_lock<std::mutex> lock(*subscribe_mutex);
        *is_finished = true;

        return grpc::Status::OK;
    }

    grpc::Status SubscribeFlightModeChange(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::action_server::SubscribeFlightModeChangeRequest* /* request */,
        grpc::ServerWriter<rpc::action_server::FlightModeChangeResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            rpc::action_server::FlightModeChangeResponse rpc_response;

            // For server plugins, this should never happen, they should always be constructible.
            auto result = mavsdk::ActionServer::Result::Unknown;
            fillResponseWithResult(&rpc_response, result);
            writer->Write(rpc_response);

            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        const mavsdk::ActionServer::FlightModeChangeHandle handle =
            _lazy_plugin.maybe_plugin()->subscribe_flight_mode_change(
                [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex, &handle](
                    mavsdk::ActionServer::Result result,
                    const mavsdk::ActionServer::FlightMode flight_mode_change) {
                    rpc::action_server::FlightModeChangeResponse rpc_response;

                    rpc_response.set_flight_mode(translateToRpcFlightMode(flight_mode_change));

                    auto rpc_result = translateToRpcResult(result);
                    auto* rpc_action_server_result = new rpc::action_server::ActionServerResult();
                    rpc_action_server_result->set_result(rpc_result);
                    std::stringstream ss;
                    ss << result;
                    rpc_action_server_result->set_result_str(ss.str());
                    rpc_response.set_allocated_action_server_result(rpc_action_server_result);

                    std::unique_lock<std::mutex> lock(*subscribe_mutex);
                    if (!*is_finished && !writer->Write(rpc_response)) {
                        _lazy_plugin.maybe_plugin()->unsubscribe_flight_mode_change(handle);

                        *is_finished = true;
                        unregister_stream_stop_promise(stream_closed_promise);
                        stream_closed_promise->set_value();
                    }
                });

        stream_closed_future.wait();
        std::unique_lock<std::mutex> lock(*subscribe_mutex);
        *is_finished = true;

        return grpc::Status::OK;
    }

    grpc::Status SubscribeTakeoff(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::action_server::SubscribeTakeoffRequest* /* request */,
        grpc::ServerWriter<rpc::action_server::TakeoffResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            rpc::action_server::TakeoffResponse rpc_response;

            // For server plugins, this should never happen, they should always be constructible.
            auto result = mavsdk::ActionServer::Result::Unknown;
            fillResponseWithResult(&rpc_response, result);
            writer->Write(rpc_response);

            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        const mavsdk::ActionServer::TakeoffHandle handle =
            _lazy_plugin.maybe_plugin()->subscribe_takeoff(
                [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex, &handle](
                    mavsdk::ActionServer::Result result, const bool takeoff) {
                    rpc::action_server::TakeoffResponse rpc_response;

                    rpc_response.set_takeoff(takeoff);

                    auto rpc_result = translateToRpcResult(result);
                    auto* rpc_action_server_result = new rpc::action_server::ActionServerResult();
                    rpc_action_server_result->set_result(rpc_result);
                    std::stringstream ss;
                    ss << result;
                    rpc_action_server_result->set_result_str(ss.str());
                    rpc_response.set_allocated_action_server_result(rpc_action_server_result);

                    std::unique_lock<std::mutex> lock(*subscribe_mutex);
                    if (!*is_finished && !writer->Write(rpc_response)) {
                        _lazy_plugin.maybe_plugin()->unsubscribe_takeoff(handle);

                        *is_finished = true;
                        unregister_stream_stop_promise(stream_closed_promise);
                        stream_closed_promise->set_value();
                    }
                });

        stream_closed_future.wait();
        std::unique_lock<std::mutex> lock(*subscribe_mutex);
        *is_finished = true;

        return grpc::Status::OK;
    }

    grpc::Status SubscribeLand(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::action_server::SubscribeLandRequest* /* request */,
        grpc::ServerWriter<rpc::action_server::LandResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            rpc::action_server::LandResponse rpc_response;

            // For server plugins, this should never happen, they should always be constructible.
            auto result = mavsdk::ActionServer::Result::Unknown;
            fillResponseWithResult(&rpc_response, result);
            writer->Write(rpc_response);

            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        const mavsdk::ActionServer::LandHandle handle = _lazy_plugin.maybe_plugin()->subscribe_land(
            [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex, &handle](
                mavsdk::ActionServer::Result result, const bool land) {
                rpc::action_server::LandResponse rpc_response;

                rpc_response.set_land(land);

                auto rpc_result = translateToRpcResult(result);
                auto* rpc_action_server_result = new rpc::action_server::ActionServerResult();
                rpc_action_server_result->set_result(rpc_result);
                std::stringstream ss;
                ss << result;
                rpc_action_server_result->set_result_str(ss.str());
                rpc_response.set_allocated_action_server_result(rpc_action_server_result);

                std::unique_lock<std::mutex> lock(*subscribe_mutex);
                if (!*is_finished && !writer->Write(rpc_response)) {
                    _lazy_plugin.maybe_plugin()->unsubscribe_land(handle);

                    *is_finished = true;
                    unregister_stream_stop_promise(stream_closed_promise);
                    stream_closed_promise->set_value();
                }
            });

        stream_closed_future.wait();
        std::unique_lock<std::mutex> lock(*subscribe_mutex);
        *is_finished = true;

        return grpc::Status::OK;
    }

    grpc::Status SubscribeReboot(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::action_server::SubscribeRebootRequest* /* request */,
        grpc::ServerWriter<rpc::action_server::RebootResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            rpc::action_server::RebootResponse rpc_response;

            // For server plugins, this should never happen, they should always be constructible.
            auto result = mavsdk::ActionServer::Result::Unknown;
            fillResponseWithResult(&rpc_response, result);
            writer->Write(rpc_response);

            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        const mavsdk::ActionServer::RebootHandle handle =
            _lazy_plugin.maybe_plugin()->subscribe_reboot(
                [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex, &handle](
                    mavsdk::ActionServer::Result result, const bool reboot) {
                    rpc::action_server::RebootResponse rpc_response;

                    rpc_response.set_reboot(reboot);

                    auto rpc_result = translateToRpcResult(result);
                    auto* rpc_action_server_result = new rpc::action_server::ActionServerResult();
                    rpc_action_server_result->set_result(rpc_result);
                    std::stringstream ss;
                    ss << result;
                    rpc_action_server_result->set_result_str(ss.str());
                    rpc_response.set_allocated_action_server_result(rpc_action_server_result);

                    std::unique_lock<std::mutex> lock(*subscribe_mutex);
                    if (!*is_finished && !writer->Write(rpc_response)) {
                        _lazy_plugin.maybe_plugin()->unsubscribe_reboot(handle);

                        *is_finished = true;
                        unregister_stream_stop_promise(stream_closed_promise);
                        stream_closed_promise->set_value();
                    }
                });

        stream_closed_future.wait();
        std::unique_lock<std::mutex> lock(*subscribe_mutex);
        *is_finished = true;

        return grpc::Status::OK;
    }

    grpc::Status SubscribeShutdown(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::action_server::SubscribeShutdownRequest* /* request */,
        grpc::ServerWriter<rpc::action_server::ShutdownResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            rpc::action_server::ShutdownResponse rpc_response;

            // For server plugins, this should never happen, they should always be constructible.
            auto result = mavsdk::ActionServer::Result::Unknown;
            fillResponseWithResult(&rpc_response, result);
            writer->Write(rpc_response);

            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        const mavsdk::ActionServer::ShutdownHandle handle =
            _lazy_plugin.maybe_plugin()->subscribe_shutdown(
                [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex, &handle](
                    mavsdk::ActionServer::Result result, const bool shutdown) {
                    rpc::action_server::ShutdownResponse rpc_response;

                    rpc_response.set_shutdown(shutdown);

                    auto rpc_result = translateToRpcResult(result);
                    auto* rpc_action_server_result = new rpc::action_server::ActionServerResult();
                    rpc_action_server_result->set_result(rpc_result);
                    std::stringstream ss;
                    ss << result;
                    rpc_action_server_result->set_result_str(ss.str());
                    rpc_response.set_allocated_action_server_result(rpc_action_server_result);

                    std::unique_lock<std::mutex> lock(*subscribe_mutex);
                    if (!*is_finished && !writer->Write(rpc_response)) {
                        _lazy_plugin.maybe_plugin()->unsubscribe_shutdown(handle);

                        *is_finished = true;
                        unregister_stream_stop_promise(stream_closed_promise);
                        stream_closed_promise->set_value();
                    }
                });

        stream_closed_future.wait();
        std::unique_lock<std::mutex> lock(*subscribe_mutex);
        *is_finished = true;

        return grpc::Status::OK;
    }

    grpc::Status SubscribeTerminate(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::action_server::SubscribeTerminateRequest* /* request */,
        grpc::ServerWriter<rpc::action_server::TerminateResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            rpc::action_server::TerminateResponse rpc_response;

            // For server plugins, this should never happen, they should always be constructible.
            auto result = mavsdk::ActionServer::Result::Unknown;
            fillResponseWithResult(&rpc_response, result);
            writer->Write(rpc_response);

            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        const mavsdk::ActionServer::TerminateHandle handle =
            _lazy_plugin.maybe_plugin()->subscribe_terminate(
                [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex, &handle](
                    mavsdk::ActionServer::Result result, const bool terminate) {
                    rpc::action_server::TerminateResponse rpc_response;

                    rpc_response.set_terminate(terminate);

                    auto rpc_result = translateToRpcResult(result);
                    auto* rpc_action_server_result = new rpc::action_server::ActionServerResult();
                    rpc_action_server_result->set_result(rpc_result);
                    std::stringstream ss;
                    ss << result;
                    rpc_action_server_result->set_result_str(ss.str());
                    rpc_response.set_allocated_action_server_result(rpc_action_server_result);

                    std::unique_lock<std::mutex> lock(*subscribe_mutex);
                    if (!*is_finished && !writer->Write(rpc_response)) {
                        _lazy_plugin.maybe_plugin()->unsubscribe_terminate(handle);

                        *is_finished = true;
                        unregister_stream_stop_promise(stream_closed_promise);
                        stream_closed_promise->set_value();
                    }
                });

        stream_closed_future.wait();
        std::unique_lock<std::mutex> lock(*subscribe_mutex);
        *is_finished = true;

        return grpc::Status::OK;
    }

    grpc::Status SetAllowTakeoff(
        grpc::ServerContext* /* context */,
        const rpc::action_server::SetAllowTakeoffRequest* request,
        rpc::action_server::SetAllowTakeoffResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            if (response != nullptr) {
                // For server plugins, this should never happen, they should always be
                // constructible.
                auto result = mavsdk::ActionServer::Result::Unknown;
                fillResponseWithResult(response, result);
            }

            return grpc::Status::OK;
        }

        if (request == nullptr) {
            LogWarn() << "SetAllowTakeoff sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }

        auto result = _lazy_plugin.maybe_plugin()->set_allow_takeoff(request->allow_takeoff());

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status SetArmable(
        grpc::ServerContext* /* context */,
        const rpc::action_server::SetArmableRequest* request,
        rpc::action_server::SetArmableResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            if (response != nullptr) {
                // For server plugins, this should never happen, they should always be
                // constructible.
                auto result = mavsdk::ActionServer::Result::Unknown;
                fillResponseWithResult(response, result);
            }

            return grpc::Status::OK;
        }

        if (request == nullptr) {
            LogWarn() << "SetArmable sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }

        auto result =
            _lazy_plugin.maybe_plugin()->set_armable(request->armable(), request->force_armable());

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status SetDisarmable(
        grpc::ServerContext* /* context */,
        const rpc::action_server::SetDisarmableRequest* request,
        rpc::action_server::SetDisarmableResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            if (response != nullptr) {
                // For server plugins, this should never happen, they should always be
                // constructible.
                auto result = mavsdk::ActionServer::Result::Unknown;
                fillResponseWithResult(response, result);
            }

            return grpc::Status::OK;
        }

        if (request == nullptr) {
            LogWarn() << "SetDisarmable sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }

        auto result = _lazy_plugin.maybe_plugin()->set_disarmable(
            request->disarmable(), request->force_disarmable());

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status SetAllowableFlightModes(
        grpc::ServerContext* /* context */,
        const rpc::action_server::SetAllowableFlightModesRequest* request,
        rpc::action_server::SetAllowableFlightModesResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            if (response != nullptr) {
                // For server plugins, this should never happen, they should always be
                // constructible.
                auto result = mavsdk::ActionServer::Result::Unknown;
                fillResponseWithResult(response, result);
            }

            return grpc::Status::OK;
        }

        if (request == nullptr) {
            LogWarn() << "SetAllowableFlightModes sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }

        auto result = _lazy_plugin.maybe_plugin()->set_allowable_flight_modes(
            translateFromRpcAllowableFlightModes(request->flight_modes()));

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status GetAllowableFlightModes(
        grpc::ServerContext* /* context */,
        const rpc::action_server::GetAllowableFlightModesRequest* /* request */,
        rpc::action_server::GetAllowableFlightModesResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            return grpc::Status::OK;
        }

        auto result = _lazy_plugin.maybe_plugin()->get_allowable_flight_modes();

        if (response != nullptr) {
            response->set_allocated_flight_modes(
                translateToRpcAllowableFlightModes(result).release());
        }

        return grpc::Status::OK;
    }

    grpc::Status SetArmedState(
        grpc::ServerContext* /* context */,
        const rpc::action_server::SetArmedStateRequest* request,
        rpc::action_server::SetArmedStateResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            if (response != nullptr) {
                // For server plugins, this should never happen, they should always be
                // constructible.
                auto result = mavsdk::ActionServer::Result::Unknown;
                fillResponseWithResult(response, result);
            }

            return grpc::Status::OK;
        }

        if (request == nullptr) {
            LogWarn() << "SetArmedState sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }

        auto result = _lazy_plugin.maybe_plugin()->set_armed_state(request->is_armed());

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status SetFlightMode(
        grpc::ServerContext* /* context */,
        const rpc::action_server::SetFlightModeRequest* request,
        rpc::action_server::SetFlightModeResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            if (response != nullptr) {
                // For server plugins, this should never happen, they should always be
                // constructible.
                auto result = mavsdk::ActionServer::Result::Unknown;
                fillResponseWithResult(response, result);
            }

            return grpc::Status::OK;
        }

        if (request == nullptr) {
            LogWarn() << "SetFlightMode sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }

        auto result = _lazy_plugin.maybe_plugin()->set_flight_mode(
            translateFromRpcFlightMode(request->flight_mode()));

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    void stop()
    {
        _stopped.store(true);
        std::lock_guard<std::mutex> lock(_stream_stop_mutex);
        for (auto& prom : _stream_stop_promises) {
            if (auto handle = prom.lock()) {
                handle->set_value();
            }
        }
    }

private:
    void register_stream_stop_promise(std::weak_ptr<std::promise<void>> prom)
    {
        // If we have already stopped, set promise immediately and don't add it to list.
        if (_stopped.load()) {
            if (auto handle = prom.lock()) {
                handle->set_value();
            }
        } else {
            std::lock_guard<std::mutex> lock(_stream_stop_mutex);
            _stream_stop_promises.push_back(prom);
        }
    }

    void unregister_stream_stop_promise(std::shared_ptr<std::promise<void>> prom)
    {
        std::lock_guard<std::mutex> lock(_stream_stop_mutex);
        for (auto it = _stream_stop_promises.begin(); it != _stream_stop_promises.end();
             /* ++it */) {
            if (it->lock() == prom) {
                it = _stream_stop_promises.erase(it);
            } else {
                ++it;
            }
        }
    }

    LazyServerPlugin& _lazy_plugin;

    std::atomic<bool> _stopped{false};
    std::mutex _stream_stop_mutex{};
    std::vector<std::weak_ptr<std::promise<void>>> _stream_stop_promises{};
};

} // namespace mavsdk_server
} // namespace mavsdk