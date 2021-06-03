// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: core/core.proto
#ifndef GRPC_core_2fcore_2eproto__INCLUDED
#define GRPC_core_2fcore_2eproto__INCLUDED

#include "core/core.pb.h"

#include <functional>
#include <grpc/impl/codegen/port_platform.h>
#include <grpcpp/impl/codegen/async_generic_service.h>
#include <grpcpp/impl/codegen/async_stream.h>
#include <grpcpp/impl/codegen/async_unary_call.h>
#include <grpcpp/impl/codegen/client_callback.h>
#include <grpcpp/impl/codegen/client_context.h>
#include <grpcpp/impl/codegen/completion_queue.h>
#include <grpcpp/impl/codegen/message_allocator.h>
#include <grpcpp/impl/codegen/method_handler.h>
#include <grpcpp/impl/codegen/proto_utils.h>
#include <grpcpp/impl/codegen/rpc_method.h>
#include <grpcpp/impl/codegen/server_callback.h>
#include <grpcpp/impl/codegen/server_callback_handlers.h>
#include <grpcpp/impl/codegen/server_context.h>
#include <grpcpp/impl/codegen/service_type.h>
#include <grpcpp/impl/codegen/status.h>
#include <grpcpp/impl/codegen/stub_options.h>
#include <grpcpp/impl/codegen/sync_stream.h>

namespace mavsdk {
namespace rpc {
namespace core {

// Access to the connection state and running plugins.
class CoreService final {
 public:
  static constexpr char const* service_full_name() {
    return "mavsdk.rpc.core.CoreService";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    // Subscribe to 'connection state' updates.
    std::unique_ptr< ::grpc::ClientReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>> SubscribeConnectionState(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request) {
      return std::unique_ptr< ::grpc::ClientReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>>(SubscribeConnectionStateRaw(context, request));
    }
    std::unique_ptr< ::grpc::ClientAsyncReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>> AsyncSubscribeConnectionState(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request, ::grpc::CompletionQueue* cq, void* tag) {
      return std::unique_ptr< ::grpc::ClientAsyncReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>>(AsyncSubscribeConnectionStateRaw(context, request, cq, tag));
    }
    std::unique_ptr< ::grpc::ClientAsyncReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>> PrepareAsyncSubscribeConnectionState(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>>(PrepareAsyncSubscribeConnectionStateRaw(context, request, cq));
    }
    class experimental_async_interface {
     public:
      virtual ~experimental_async_interface() {}
      // Subscribe to 'connection state' updates.
      #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      virtual void SubscribeConnectionState(::grpc::ClientContext* context, ::mavsdk::rpc::core::SubscribeConnectionStateRequest* request, ::grpc::ClientReadReactor< ::mavsdk::rpc::core::ConnectionStateResponse>* reactor) = 0;
      #else
      virtual void SubscribeConnectionState(::grpc::ClientContext* context, ::mavsdk::rpc::core::SubscribeConnectionStateRequest* request, ::grpc::experimental::ClientReadReactor< ::mavsdk::rpc::core::ConnectionStateResponse>* reactor) = 0;
      #endif
    };
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
    typedef class experimental_async_interface async_interface;
    #endif
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
    async_interface* async() { return experimental_async(); }
    #endif
    virtual class experimental_async_interface* experimental_async() { return nullptr; }
  private:
    virtual ::grpc::ClientReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>* SubscribeConnectionStateRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request) = 0;
    virtual ::grpc::ClientAsyncReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>* AsyncSubscribeConnectionStateRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request, ::grpc::CompletionQueue* cq, void* tag) = 0;
    virtual ::grpc::ClientAsyncReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>* PrepareAsyncSubscribeConnectionStateRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel);
    std::unique_ptr< ::grpc::ClientReader< ::mavsdk::rpc::core::ConnectionStateResponse>> SubscribeConnectionState(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request) {
      return std::unique_ptr< ::grpc::ClientReader< ::mavsdk::rpc::core::ConnectionStateResponse>>(SubscribeConnectionStateRaw(context, request));
    }
    std::unique_ptr< ::grpc::ClientAsyncReader< ::mavsdk::rpc::core::ConnectionStateResponse>> AsyncSubscribeConnectionState(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request, ::grpc::CompletionQueue* cq, void* tag) {
      return std::unique_ptr< ::grpc::ClientAsyncReader< ::mavsdk::rpc::core::ConnectionStateResponse>>(AsyncSubscribeConnectionStateRaw(context, request, cq, tag));
    }
    std::unique_ptr< ::grpc::ClientAsyncReader< ::mavsdk::rpc::core::ConnectionStateResponse>> PrepareAsyncSubscribeConnectionState(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncReader< ::mavsdk::rpc::core::ConnectionStateResponse>>(PrepareAsyncSubscribeConnectionStateRaw(context, request, cq));
    }
    class experimental_async final :
      public StubInterface::experimental_async_interface {
     public:
      #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      void SubscribeConnectionState(::grpc::ClientContext* context, ::mavsdk::rpc::core::SubscribeConnectionStateRequest* request, ::grpc::ClientReadReactor< ::mavsdk::rpc::core::ConnectionStateResponse>* reactor) override;
      #else
      void SubscribeConnectionState(::grpc::ClientContext* context, ::mavsdk::rpc::core::SubscribeConnectionStateRequest* request, ::grpc::experimental::ClientReadReactor< ::mavsdk::rpc::core::ConnectionStateResponse>* reactor) override;
      #endif
     private:
      friend class Stub;
      explicit experimental_async(Stub* stub): stub_(stub) { }
      Stub* stub() { return stub_; }
      Stub* stub_;
    };
    class experimental_async_interface* experimental_async() override { return &async_stub_; }

   private:
    std::shared_ptr< ::grpc::ChannelInterface> channel_;
    class experimental_async async_stub_{this};
    ::grpc::ClientReader< ::mavsdk::rpc::core::ConnectionStateResponse>* SubscribeConnectionStateRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request) override;
    ::grpc::ClientAsyncReader< ::mavsdk::rpc::core::ConnectionStateResponse>* AsyncSubscribeConnectionStateRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request, ::grpc::CompletionQueue* cq, void* tag) override;
    ::grpc::ClientAsyncReader< ::mavsdk::rpc::core::ConnectionStateResponse>* PrepareAsyncSubscribeConnectionStateRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request, ::grpc::CompletionQueue* cq) override;
    const ::grpc::internal::RpcMethod rpcmethod_SubscribeConnectionState_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    // Subscribe to 'connection state' updates.
    virtual ::grpc::Status SubscribeConnectionState(::grpc::ServerContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* request, ::grpc::ServerWriter< ::mavsdk::rpc::core::ConnectionStateResponse>* writer);
  };
  template <class BaseClass>
  class WithAsyncMethod_SubscribeConnectionState : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_SubscribeConnectionState() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_SubscribeConnectionState() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SubscribeConnectionState(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* /*request*/, ::grpc::ServerWriter< ::mavsdk::rpc::core::ConnectionStateResponse>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestSubscribeConnectionState(::grpc::ServerContext* context, ::mavsdk::rpc::core::SubscribeConnectionStateRequest* request, ::grpc::ServerAsyncWriter< ::mavsdk::rpc::core::ConnectionStateResponse>* writer, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncServerStreaming(0, context, request, writer, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_SubscribeConnectionState<Service > AsyncService;
  template <class BaseClass>
  class ExperimentalWithCallbackMethod_SubscribeConnectionState : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    ExperimentalWithCallbackMethod_SubscribeConnectionState() {
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      ::grpc::Service::
    #else
      ::grpc::Service::experimental().
    #endif
        MarkMethodCallback(0,
          new ::grpc::internal::CallbackServerStreamingHandler< ::mavsdk::rpc::core::SubscribeConnectionStateRequest, ::mavsdk::rpc::core::ConnectionStateResponse>(
            [this](
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
                   ::grpc::CallbackServerContext*
    #else
                   ::grpc::experimental::CallbackServerContext*
    #endif
                     context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* request) { return this->SubscribeConnectionState(context, request); }));
    }
    ~ExperimentalWithCallbackMethod_SubscribeConnectionState() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SubscribeConnectionState(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* /*request*/, ::grpc::ServerWriter< ::mavsdk::rpc::core::ConnectionStateResponse>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
    virtual ::grpc::ServerWriteReactor< ::mavsdk::rpc::core::ConnectionStateResponse>* SubscribeConnectionState(
      ::grpc::CallbackServerContext* /*context*/, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* /*request*/)
    #else
    virtual ::grpc::experimental::ServerWriteReactor< ::mavsdk::rpc::core::ConnectionStateResponse>* SubscribeConnectionState(
      ::grpc::experimental::CallbackServerContext* /*context*/, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* /*request*/)
    #endif
      { return nullptr; }
  };
  #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
  typedef ExperimentalWithCallbackMethod_SubscribeConnectionState<Service > CallbackService;
  #endif

  typedef ExperimentalWithCallbackMethod_SubscribeConnectionState<Service > ExperimentalCallbackService;
  template <class BaseClass>
  class WithGenericMethod_SubscribeConnectionState : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_SubscribeConnectionState() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_SubscribeConnectionState() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SubscribeConnectionState(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* /*request*/, ::grpc::ServerWriter< ::mavsdk::rpc::core::ConnectionStateResponse>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithRawMethod_SubscribeConnectionState : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_SubscribeConnectionState() {
      ::grpc::Service::MarkMethodRaw(0);
    }
    ~WithRawMethod_SubscribeConnectionState() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SubscribeConnectionState(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* /*request*/, ::grpc::ServerWriter< ::mavsdk::rpc::core::ConnectionStateResponse>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestSubscribeConnectionState(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncWriter< ::grpc::ByteBuffer>* writer, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncServerStreaming(0, context, request, writer, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class ExperimentalWithRawCallbackMethod_SubscribeConnectionState : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    ExperimentalWithRawCallbackMethod_SubscribeConnectionState() {
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      ::grpc::Service::
    #else
      ::grpc::Service::experimental().
    #endif
        MarkMethodRawCallback(0,
          new ::grpc::internal::CallbackServerStreamingHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
                   ::grpc::CallbackServerContext*
    #else
                   ::grpc::experimental::CallbackServerContext*
    #endif
                     context, const::grpc::ByteBuffer* request) { return this->SubscribeConnectionState(context, request); }));
    }
    ~ExperimentalWithRawCallbackMethod_SubscribeConnectionState() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SubscribeConnectionState(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* /*request*/, ::grpc::ServerWriter< ::mavsdk::rpc::core::ConnectionStateResponse>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
    virtual ::grpc::ServerWriteReactor< ::grpc::ByteBuffer>* SubscribeConnectionState(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/)
    #else
    virtual ::grpc::experimental::ServerWriteReactor< ::grpc::ByteBuffer>* SubscribeConnectionState(
      ::grpc::experimental::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/)
    #endif
      { return nullptr; }
  };
  typedef Service StreamedUnaryService;
  template <class BaseClass>
  class WithSplitStreamingMethod_SubscribeConnectionState : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithSplitStreamingMethod_SubscribeConnectionState() {
      ::grpc::Service::MarkMethodStreamed(0,
        new ::grpc::internal::SplitServerStreamingHandler<
          ::mavsdk::rpc::core::SubscribeConnectionStateRequest, ::mavsdk::rpc::core::ConnectionStateResponse>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerSplitStreamer<
                     ::mavsdk::rpc::core::SubscribeConnectionStateRequest, ::mavsdk::rpc::core::ConnectionStateResponse>* streamer) {
                       return this->StreamedSubscribeConnectionState(context,
                         streamer);
                  }));
    }
    ~WithSplitStreamingMethod_SubscribeConnectionState() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status SubscribeConnectionState(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* /*request*/, ::grpc::ServerWriter< ::mavsdk::rpc::core::ConnectionStateResponse>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with split streamed
    virtual ::grpc::Status StreamedSubscribeConnectionState(::grpc::ServerContext* context, ::grpc::ServerSplitStreamer< ::mavsdk::rpc::core::SubscribeConnectionStateRequest,::mavsdk::rpc::core::ConnectionStateResponse>* server_split_streamer) = 0;
  };
  typedef WithSplitStreamingMethod_SubscribeConnectionState<Service > SplitStreamedService;
  typedef WithSplitStreamingMethod_SubscribeConnectionState<Service > StreamedService;
};

}  // namespace core
}  // namespace rpc
}  // namespace mavsdk


#endif  // GRPC_core_2fcore_2eproto__INCLUDED
