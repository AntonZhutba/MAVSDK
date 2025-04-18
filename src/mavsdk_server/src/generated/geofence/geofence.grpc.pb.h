// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: geofence/geofence.proto
#ifndef GRPC_geofence_2fgeofence_2eproto__INCLUDED
#define GRPC_geofence_2fgeofence_2eproto__INCLUDED

#include "geofence/geofence.pb.h"

#include <functional>
#include <grpcpp/generic/async_generic_service.h>
#include <grpcpp/support/async_stream.h>
#include <grpcpp/support/async_unary_call.h>
#include <grpcpp/support/client_callback.h>
#include <grpcpp/client_context.h>
#include <grpcpp/completion_queue.h>
#include <grpcpp/support/message_allocator.h>
#include <grpcpp/support/method_handler.h>
#include <grpcpp/impl/proto_utils.h>
#include <grpcpp/impl/rpc_method.h>
#include <grpcpp/support/server_callback.h>
#include <grpcpp/impl/server_callback_handlers.h>
#include <grpcpp/server_context.h>
#include <grpcpp/impl/service_type.h>
#include <grpcpp/support/status.h>
#include <grpcpp/support/stub_options.h>
#include <grpcpp/support/sync_stream.h>
#include <grpcpp/ports_def.inc>

namespace mavsdk {
namespace rpc {
namespace geofence {

// Enable setting a geofence.
class GeofenceService final {
 public:
  static constexpr char const* service_full_name() {
    return "mavsdk.rpc.geofence.GeofenceService";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    //
    // Upload geofences.
    //
    // Polygon and Circular geofences are uploaded to a drone. Once uploaded, the geofence will remain
    // on the drone even if a connection is lost.
    virtual ::grpc::Status UploadGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::UploadGeofenceRequest& request, ::mavsdk::rpc::geofence::UploadGeofenceResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::geofence::UploadGeofenceResponse>> AsyncUploadGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::UploadGeofenceRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::geofence::UploadGeofenceResponse>>(AsyncUploadGeofenceRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::geofence::UploadGeofenceResponse>> PrepareAsyncUploadGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::UploadGeofenceRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::geofence::UploadGeofenceResponse>>(PrepareAsyncUploadGeofenceRaw(context, request, cq));
    }
    //
    // Clear all geofences saved on the vehicle.
    virtual ::grpc::Status ClearGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::ClearGeofenceRequest& request, ::mavsdk::rpc::geofence::ClearGeofenceResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::geofence::ClearGeofenceResponse>> AsyncClearGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::ClearGeofenceRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::geofence::ClearGeofenceResponse>>(AsyncClearGeofenceRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::geofence::ClearGeofenceResponse>> PrepareAsyncClearGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::ClearGeofenceRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::geofence::ClearGeofenceResponse>>(PrepareAsyncClearGeofenceRaw(context, request, cq));
    }
    class async_interface {
     public:
      virtual ~async_interface() {}
      //
      // Upload geofences.
      //
      // Polygon and Circular geofences are uploaded to a drone. Once uploaded, the geofence will remain
      // on the drone even if a connection is lost.
      virtual void UploadGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::UploadGeofenceRequest* request, ::mavsdk::rpc::geofence::UploadGeofenceResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void UploadGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::UploadGeofenceRequest* request, ::mavsdk::rpc::geofence::UploadGeofenceResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
      //
      // Clear all geofences saved on the vehicle.
      virtual void ClearGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::ClearGeofenceRequest* request, ::mavsdk::rpc::geofence::ClearGeofenceResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void ClearGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::ClearGeofenceRequest* request, ::mavsdk::rpc::geofence::ClearGeofenceResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
    };
    typedef class async_interface experimental_async_interface;
    virtual class async_interface* async() { return nullptr; }
    class async_interface* experimental_async() { return async(); }
   private:
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::geofence::UploadGeofenceResponse>* AsyncUploadGeofenceRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::UploadGeofenceRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::geofence::UploadGeofenceResponse>* PrepareAsyncUploadGeofenceRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::UploadGeofenceRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::geofence::ClearGeofenceResponse>* AsyncClearGeofenceRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::ClearGeofenceRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::geofence::ClearGeofenceResponse>* PrepareAsyncClearGeofenceRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::ClearGeofenceRequest& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());
    ::grpc::Status UploadGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::UploadGeofenceRequest& request, ::mavsdk::rpc::geofence::UploadGeofenceResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::geofence::UploadGeofenceResponse>> AsyncUploadGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::UploadGeofenceRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::geofence::UploadGeofenceResponse>>(AsyncUploadGeofenceRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::geofence::UploadGeofenceResponse>> PrepareAsyncUploadGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::UploadGeofenceRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::geofence::UploadGeofenceResponse>>(PrepareAsyncUploadGeofenceRaw(context, request, cq));
    }
    ::grpc::Status ClearGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::ClearGeofenceRequest& request, ::mavsdk::rpc::geofence::ClearGeofenceResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::geofence::ClearGeofenceResponse>> AsyncClearGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::ClearGeofenceRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::geofence::ClearGeofenceResponse>>(AsyncClearGeofenceRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::geofence::ClearGeofenceResponse>> PrepareAsyncClearGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::ClearGeofenceRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::geofence::ClearGeofenceResponse>>(PrepareAsyncClearGeofenceRaw(context, request, cq));
    }
    class async final :
      public StubInterface::async_interface {
     public:
      void UploadGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::UploadGeofenceRequest* request, ::mavsdk::rpc::geofence::UploadGeofenceResponse* response, std::function<void(::grpc::Status)>) override;
      void UploadGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::UploadGeofenceRequest* request, ::mavsdk::rpc::geofence::UploadGeofenceResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
      void ClearGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::ClearGeofenceRequest* request, ::mavsdk::rpc::geofence::ClearGeofenceResponse* response, std::function<void(::grpc::Status)>) override;
      void ClearGeofence(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::ClearGeofenceRequest* request, ::mavsdk::rpc::geofence::ClearGeofenceResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
     private:
      friend class Stub;
      explicit async(Stub* stub): stub_(stub) { }
      Stub* stub() { return stub_; }
      Stub* stub_;
    };
    class async* async() override { return &async_stub_; }

   private:
    std::shared_ptr< ::grpc::ChannelInterface> channel_;
    class async async_stub_{this};
    ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::geofence::UploadGeofenceResponse>* AsyncUploadGeofenceRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::UploadGeofenceRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::geofence::UploadGeofenceResponse>* PrepareAsyncUploadGeofenceRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::UploadGeofenceRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::geofence::ClearGeofenceResponse>* AsyncClearGeofenceRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::ClearGeofenceRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::geofence::ClearGeofenceResponse>* PrepareAsyncClearGeofenceRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::geofence::ClearGeofenceRequest& request, ::grpc::CompletionQueue* cq) override;
    const ::grpc::internal::RpcMethod rpcmethod_UploadGeofence_;
    const ::grpc::internal::RpcMethod rpcmethod_ClearGeofence_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    //
    // Upload geofences.
    //
    // Polygon and Circular geofences are uploaded to a drone. Once uploaded, the geofence will remain
    // on the drone even if a connection is lost.
    virtual ::grpc::Status UploadGeofence(::grpc::ServerContext* context, const ::mavsdk::rpc::geofence::UploadGeofenceRequest* request, ::mavsdk::rpc::geofence::UploadGeofenceResponse* response);
    //
    // Clear all geofences saved on the vehicle.
    virtual ::grpc::Status ClearGeofence(::grpc::ServerContext* context, const ::mavsdk::rpc::geofence::ClearGeofenceRequest* request, ::mavsdk::rpc::geofence::ClearGeofenceResponse* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_UploadGeofence : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_UploadGeofence() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_UploadGeofence() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status UploadGeofence(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::geofence::UploadGeofenceRequest* /*request*/, ::mavsdk::rpc::geofence::UploadGeofenceResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestUploadGeofence(::grpc::ServerContext* context, ::mavsdk::rpc::geofence::UploadGeofenceRequest* request, ::grpc::ServerAsyncResponseWriter< ::mavsdk::rpc::geofence::UploadGeofenceResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithAsyncMethod_ClearGeofence : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_ClearGeofence() {
      ::grpc::Service::MarkMethodAsync(1);
    }
    ~WithAsyncMethod_ClearGeofence() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ClearGeofence(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::geofence::ClearGeofenceRequest* /*request*/, ::mavsdk::rpc::geofence::ClearGeofenceResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestClearGeofence(::grpc::ServerContext* context, ::mavsdk::rpc::geofence::ClearGeofenceRequest* request, ::grpc::ServerAsyncResponseWriter< ::mavsdk::rpc::geofence::ClearGeofenceResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(1, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_UploadGeofence<WithAsyncMethod_ClearGeofence<Service > > AsyncService;
  template <class BaseClass>
  class WithCallbackMethod_UploadGeofence : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_UploadGeofence() {
      ::grpc::Service::MarkMethodCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::mavsdk::rpc::geofence::UploadGeofenceRequest, ::mavsdk::rpc::geofence::UploadGeofenceResponse>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::mavsdk::rpc::geofence::UploadGeofenceRequest* request, ::mavsdk::rpc::geofence::UploadGeofenceResponse* response) { return this->UploadGeofence(context, request, response); }));}
    void SetMessageAllocatorFor_UploadGeofence(
        ::grpc::MessageAllocator< ::mavsdk::rpc::geofence::UploadGeofenceRequest, ::mavsdk::rpc::geofence::UploadGeofenceResponse>* allocator) {
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(0);
      static_cast<::grpc::internal::CallbackUnaryHandler< ::mavsdk::rpc::geofence::UploadGeofenceRequest, ::mavsdk::rpc::geofence::UploadGeofenceResponse>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~WithCallbackMethod_UploadGeofence() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status UploadGeofence(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::geofence::UploadGeofenceRequest* /*request*/, ::mavsdk::rpc::geofence::UploadGeofenceResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* UploadGeofence(
      ::grpc::CallbackServerContext* /*context*/, const ::mavsdk::rpc::geofence::UploadGeofenceRequest* /*request*/, ::mavsdk::rpc::geofence::UploadGeofenceResponse* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithCallbackMethod_ClearGeofence : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_ClearGeofence() {
      ::grpc::Service::MarkMethodCallback(1,
          new ::grpc::internal::CallbackUnaryHandler< ::mavsdk::rpc::geofence::ClearGeofenceRequest, ::mavsdk::rpc::geofence::ClearGeofenceResponse>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::mavsdk::rpc::geofence::ClearGeofenceRequest* request, ::mavsdk::rpc::geofence::ClearGeofenceResponse* response) { return this->ClearGeofence(context, request, response); }));}
    void SetMessageAllocatorFor_ClearGeofence(
        ::grpc::MessageAllocator< ::mavsdk::rpc::geofence::ClearGeofenceRequest, ::mavsdk::rpc::geofence::ClearGeofenceResponse>* allocator) {
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(1);
      static_cast<::grpc::internal::CallbackUnaryHandler< ::mavsdk::rpc::geofence::ClearGeofenceRequest, ::mavsdk::rpc::geofence::ClearGeofenceResponse>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~WithCallbackMethod_ClearGeofence() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ClearGeofence(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::geofence::ClearGeofenceRequest* /*request*/, ::mavsdk::rpc::geofence::ClearGeofenceResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* ClearGeofence(
      ::grpc::CallbackServerContext* /*context*/, const ::mavsdk::rpc::geofence::ClearGeofenceRequest* /*request*/, ::mavsdk::rpc::geofence::ClearGeofenceResponse* /*response*/)  { return nullptr; }
  };
  typedef WithCallbackMethod_UploadGeofence<WithCallbackMethod_ClearGeofence<Service > > CallbackService;
  typedef CallbackService ExperimentalCallbackService;
  template <class BaseClass>
  class WithGenericMethod_UploadGeofence : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_UploadGeofence() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_UploadGeofence() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status UploadGeofence(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::geofence::UploadGeofenceRequest* /*request*/, ::mavsdk::rpc::geofence::UploadGeofenceResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithGenericMethod_ClearGeofence : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_ClearGeofence() {
      ::grpc::Service::MarkMethodGeneric(1);
    }
    ~WithGenericMethod_ClearGeofence() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ClearGeofence(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::geofence::ClearGeofenceRequest* /*request*/, ::mavsdk::rpc::geofence::ClearGeofenceResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithRawMethod_UploadGeofence : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_UploadGeofence() {
      ::grpc::Service::MarkMethodRaw(0);
    }
    ~WithRawMethod_UploadGeofence() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status UploadGeofence(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::geofence::UploadGeofenceRequest* /*request*/, ::mavsdk::rpc::geofence::UploadGeofenceResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestUploadGeofence(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawMethod_ClearGeofence : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_ClearGeofence() {
      ::grpc::Service::MarkMethodRaw(1);
    }
    ~WithRawMethod_ClearGeofence() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ClearGeofence(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::geofence::ClearGeofenceRequest* /*request*/, ::mavsdk::rpc::geofence::ClearGeofenceResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestClearGeofence(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(1, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_UploadGeofence : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_UploadGeofence() {
      ::grpc::Service::MarkMethodRawCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->UploadGeofence(context, request, response); }));
    }
    ~WithRawCallbackMethod_UploadGeofence() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status UploadGeofence(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::geofence::UploadGeofenceRequest* /*request*/, ::mavsdk::rpc::geofence::UploadGeofenceResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* UploadGeofence(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_ClearGeofence : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_ClearGeofence() {
      ::grpc::Service::MarkMethodRawCallback(1,
          new ::grpc::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->ClearGeofence(context, request, response); }));
    }
    ~WithRawCallbackMethod_ClearGeofence() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ClearGeofence(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::geofence::ClearGeofenceRequest* /*request*/, ::mavsdk::rpc::geofence::ClearGeofenceResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* ClearGeofence(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_UploadGeofence : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_UploadGeofence() {
      ::grpc::Service::MarkMethodStreamed(0,
        new ::grpc::internal::StreamedUnaryHandler<
          ::mavsdk::rpc::geofence::UploadGeofenceRequest, ::mavsdk::rpc::geofence::UploadGeofenceResponse>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerUnaryStreamer<
                     ::mavsdk::rpc::geofence::UploadGeofenceRequest, ::mavsdk::rpc::geofence::UploadGeofenceResponse>* streamer) {
                       return this->StreamedUploadGeofence(context,
                         streamer);
                  }));
    }
    ~WithStreamedUnaryMethod_UploadGeofence() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status UploadGeofence(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::geofence::UploadGeofenceRequest* /*request*/, ::mavsdk::rpc::geofence::UploadGeofenceResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedUploadGeofence(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::mavsdk::rpc::geofence::UploadGeofenceRequest,::mavsdk::rpc::geofence::UploadGeofenceResponse>* server_unary_streamer) = 0;
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_ClearGeofence : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_ClearGeofence() {
      ::grpc::Service::MarkMethodStreamed(1,
        new ::grpc::internal::StreamedUnaryHandler<
          ::mavsdk::rpc::geofence::ClearGeofenceRequest, ::mavsdk::rpc::geofence::ClearGeofenceResponse>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerUnaryStreamer<
                     ::mavsdk::rpc::geofence::ClearGeofenceRequest, ::mavsdk::rpc::geofence::ClearGeofenceResponse>* streamer) {
                       return this->StreamedClearGeofence(context,
                         streamer);
                  }));
    }
    ~WithStreamedUnaryMethod_ClearGeofence() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status ClearGeofence(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::geofence::ClearGeofenceRequest* /*request*/, ::mavsdk::rpc::geofence::ClearGeofenceResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedClearGeofence(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::mavsdk::rpc::geofence::ClearGeofenceRequest,::mavsdk::rpc::geofence::ClearGeofenceResponse>* server_unary_streamer) = 0;
  };
  typedef WithStreamedUnaryMethod_UploadGeofence<WithStreamedUnaryMethod_ClearGeofence<Service > > StreamedUnaryService;
  typedef Service SplitStreamedService;
  typedef WithStreamedUnaryMethod_UploadGeofence<WithStreamedUnaryMethod_ClearGeofence<Service > > StreamedService;
};

}  // namespace geofence
}  // namespace rpc
}  // namespace mavsdk


#include <grpcpp/ports_undef.inc>
#endif  // GRPC_geofence_2fgeofence_2eproto__INCLUDED
