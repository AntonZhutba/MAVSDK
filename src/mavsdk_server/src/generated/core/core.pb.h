// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: core/core.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_core_2fcore_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_core_2fcore_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3015000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3015003 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_core_2fcore_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_core_2fcore_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[3]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_core_2fcore_2eproto;
::PROTOBUF_NAMESPACE_ID::Metadata descriptor_table_core_2fcore_2eproto_metadata_getter(int index);
namespace mavsdk {
namespace rpc {
namespace core {
class ConnectionState;
struct ConnectionStateDefaultTypeInternal;
extern ConnectionStateDefaultTypeInternal _ConnectionState_default_instance_;
class ConnectionStateResponse;
struct ConnectionStateResponseDefaultTypeInternal;
extern ConnectionStateResponseDefaultTypeInternal _ConnectionStateResponse_default_instance_;
class SubscribeConnectionStateRequest;
struct SubscribeConnectionStateRequestDefaultTypeInternal;
extern SubscribeConnectionStateRequestDefaultTypeInternal _SubscribeConnectionStateRequest_default_instance_;
}  // namespace core
}  // namespace rpc
}  // namespace mavsdk
PROTOBUF_NAMESPACE_OPEN
template<> ::mavsdk::rpc::core::ConnectionState* Arena::CreateMaybeMessage<::mavsdk::rpc::core::ConnectionState>(Arena*);
template<> ::mavsdk::rpc::core::ConnectionStateResponse* Arena::CreateMaybeMessage<::mavsdk::rpc::core::ConnectionStateResponse>(Arena*);
template<> ::mavsdk::rpc::core::SubscribeConnectionStateRequest* Arena::CreateMaybeMessage<::mavsdk::rpc::core::SubscribeConnectionStateRequest>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace mavsdk {
namespace rpc {
namespace core {

// ===================================================================

class SubscribeConnectionStateRequest PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:mavsdk.rpc.core.SubscribeConnectionStateRequest) */ {
 public:
  inline SubscribeConnectionStateRequest() : SubscribeConnectionStateRequest(nullptr) {}
  virtual ~SubscribeConnectionStateRequest();
  explicit constexpr SubscribeConnectionStateRequest(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  SubscribeConnectionStateRequest(const SubscribeConnectionStateRequest& from);
  SubscribeConnectionStateRequest(SubscribeConnectionStateRequest&& from) noexcept
    : SubscribeConnectionStateRequest() {
    *this = ::std::move(from);
  }

  inline SubscribeConnectionStateRequest& operator=(const SubscribeConnectionStateRequest& from) {
    CopyFrom(from);
    return *this;
  }
  inline SubscribeConnectionStateRequest& operator=(SubscribeConnectionStateRequest&& from) noexcept {
    if (GetArena() == from.GetArena()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const SubscribeConnectionStateRequest& default_instance() {
    return *internal_default_instance();
  }
  static inline const SubscribeConnectionStateRequest* internal_default_instance() {
    return reinterpret_cast<const SubscribeConnectionStateRequest*>(
               &_SubscribeConnectionStateRequest_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(SubscribeConnectionStateRequest& a, SubscribeConnectionStateRequest& b) {
    a.Swap(&b);
  }
  inline void Swap(SubscribeConnectionStateRequest* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(SubscribeConnectionStateRequest* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline SubscribeConnectionStateRequest* New() const final {
    return CreateMaybeMessage<SubscribeConnectionStateRequest>(nullptr);
  }

  SubscribeConnectionStateRequest* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<SubscribeConnectionStateRequest>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const SubscribeConnectionStateRequest& from);
  void MergeFrom(const SubscribeConnectionStateRequest& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(SubscribeConnectionStateRequest* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "mavsdk.rpc.core.SubscribeConnectionStateRequest";
  }
  protected:
  explicit SubscribeConnectionStateRequest(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    return ::descriptor_table_core_2fcore_2eproto_metadata_getter(kIndexInFileMessages);
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // @@protoc_insertion_point(class_scope:mavsdk.rpc.core.SubscribeConnectionStateRequest)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_core_2fcore_2eproto;
};
// -------------------------------------------------------------------

class ConnectionStateResponse PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:mavsdk.rpc.core.ConnectionStateResponse) */ {
 public:
  inline ConnectionStateResponse() : ConnectionStateResponse(nullptr) {}
  virtual ~ConnectionStateResponse();
  explicit constexpr ConnectionStateResponse(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  ConnectionStateResponse(const ConnectionStateResponse& from);
  ConnectionStateResponse(ConnectionStateResponse&& from) noexcept
    : ConnectionStateResponse() {
    *this = ::std::move(from);
  }

  inline ConnectionStateResponse& operator=(const ConnectionStateResponse& from) {
    CopyFrom(from);
    return *this;
  }
  inline ConnectionStateResponse& operator=(ConnectionStateResponse&& from) noexcept {
    if (GetArena() == from.GetArena()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const ConnectionStateResponse& default_instance() {
    return *internal_default_instance();
  }
  static inline const ConnectionStateResponse* internal_default_instance() {
    return reinterpret_cast<const ConnectionStateResponse*>(
               &_ConnectionStateResponse_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(ConnectionStateResponse& a, ConnectionStateResponse& b) {
    a.Swap(&b);
  }
  inline void Swap(ConnectionStateResponse* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(ConnectionStateResponse* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ConnectionStateResponse* New() const final {
    return CreateMaybeMessage<ConnectionStateResponse>(nullptr);
  }

  ConnectionStateResponse* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ConnectionStateResponse>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ConnectionStateResponse& from);
  void MergeFrom(const ConnectionStateResponse& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(ConnectionStateResponse* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "mavsdk.rpc.core.ConnectionStateResponse";
  }
  protected:
  explicit ConnectionStateResponse(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    return ::descriptor_table_core_2fcore_2eproto_metadata_getter(kIndexInFileMessages);
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kConnectionStateFieldNumber = 1,
  };
  // .mavsdk.rpc.core.ConnectionState connection_state = 1;
  bool has_connection_state() const;
  private:
  bool _internal_has_connection_state() const;
  public:
  void clear_connection_state();
  const ::mavsdk::rpc::core::ConnectionState& connection_state() const;
  ::mavsdk::rpc::core::ConnectionState* release_connection_state();
  ::mavsdk::rpc::core::ConnectionState* mutable_connection_state();
  void set_allocated_connection_state(::mavsdk::rpc::core::ConnectionState* connection_state);
  private:
  const ::mavsdk::rpc::core::ConnectionState& _internal_connection_state() const;
  ::mavsdk::rpc::core::ConnectionState* _internal_mutable_connection_state();
  public:
  void unsafe_arena_set_allocated_connection_state(
      ::mavsdk::rpc::core::ConnectionState* connection_state);
  ::mavsdk::rpc::core::ConnectionState* unsafe_arena_release_connection_state();

  // @@protoc_insertion_point(class_scope:mavsdk.rpc.core.ConnectionStateResponse)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::mavsdk::rpc::core::ConnectionState* connection_state_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_core_2fcore_2eproto;
};
// -------------------------------------------------------------------

class ConnectionState PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:mavsdk.rpc.core.ConnectionState) */ {
 public:
  inline ConnectionState() : ConnectionState(nullptr) {}
  virtual ~ConnectionState();
  explicit constexpr ConnectionState(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  ConnectionState(const ConnectionState& from);
  ConnectionState(ConnectionState&& from) noexcept
    : ConnectionState() {
    *this = ::std::move(from);
  }

  inline ConnectionState& operator=(const ConnectionState& from) {
    CopyFrom(from);
    return *this;
  }
  inline ConnectionState& operator=(ConnectionState&& from) noexcept {
    if (GetArena() == from.GetArena()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const ConnectionState& default_instance() {
    return *internal_default_instance();
  }
  static inline const ConnectionState* internal_default_instance() {
    return reinterpret_cast<const ConnectionState*>(
               &_ConnectionState_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    2;

  friend void swap(ConnectionState& a, ConnectionState& b) {
    a.Swap(&b);
  }
  inline void Swap(ConnectionState* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(ConnectionState* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ConnectionState* New() const final {
    return CreateMaybeMessage<ConnectionState>(nullptr);
  }

  ConnectionState* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ConnectionState>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ConnectionState& from);
  void MergeFrom(const ConnectionState& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(ConnectionState* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "mavsdk.rpc.core.ConnectionState";
  }
  protected:
  explicit ConnectionState(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    return ::descriptor_table_core_2fcore_2eproto_metadata_getter(kIndexInFileMessages);
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kIsConnectedFieldNumber = 2,
  };
  // bool is_connected = 2;
  void clear_is_connected();
  bool is_connected() const;
  void set_is_connected(bool value);
  private:
  bool _internal_is_connected() const;
  void _internal_set_is_connected(bool value);
  public:

  // @@protoc_insertion_point(class_scope:mavsdk.rpc.core.ConnectionState)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  bool is_connected_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_core_2fcore_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SubscribeConnectionStateRequest

// -------------------------------------------------------------------

// ConnectionStateResponse

// .mavsdk.rpc.core.ConnectionState connection_state = 1;
inline bool ConnectionStateResponse::_internal_has_connection_state() const {
  return this != internal_default_instance() && connection_state_ != nullptr;
}
inline bool ConnectionStateResponse::has_connection_state() const {
  return _internal_has_connection_state();
}
inline void ConnectionStateResponse::clear_connection_state() {
  if (GetArena() == nullptr && connection_state_ != nullptr) {
    delete connection_state_;
  }
  connection_state_ = nullptr;
}
inline const ::mavsdk::rpc::core::ConnectionState& ConnectionStateResponse::_internal_connection_state() const {
  const ::mavsdk::rpc::core::ConnectionState* p = connection_state_;
  return p != nullptr ? *p : reinterpret_cast<const ::mavsdk::rpc::core::ConnectionState&>(
      ::mavsdk::rpc::core::_ConnectionState_default_instance_);
}
inline const ::mavsdk::rpc::core::ConnectionState& ConnectionStateResponse::connection_state() const {
  // @@protoc_insertion_point(field_get:mavsdk.rpc.core.ConnectionStateResponse.connection_state)
  return _internal_connection_state();
}
inline void ConnectionStateResponse::unsafe_arena_set_allocated_connection_state(
    ::mavsdk::rpc::core::ConnectionState* connection_state) {
  if (GetArena() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(connection_state_);
  }
  connection_state_ = connection_state;
  if (connection_state) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:mavsdk.rpc.core.ConnectionStateResponse.connection_state)
}
inline ::mavsdk::rpc::core::ConnectionState* ConnectionStateResponse::release_connection_state() {
  
  ::mavsdk::rpc::core::ConnectionState* temp = connection_state_;
  connection_state_ = nullptr;
  if (GetArena() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
  return temp;
}
inline ::mavsdk::rpc::core::ConnectionState* ConnectionStateResponse::unsafe_arena_release_connection_state() {
  // @@protoc_insertion_point(field_release:mavsdk.rpc.core.ConnectionStateResponse.connection_state)
  
  ::mavsdk::rpc::core::ConnectionState* temp = connection_state_;
  connection_state_ = nullptr;
  return temp;
}
inline ::mavsdk::rpc::core::ConnectionState* ConnectionStateResponse::_internal_mutable_connection_state() {
  
  if (connection_state_ == nullptr) {
    auto* p = CreateMaybeMessage<::mavsdk::rpc::core::ConnectionState>(GetArena());
    connection_state_ = p;
  }
  return connection_state_;
}
inline ::mavsdk::rpc::core::ConnectionState* ConnectionStateResponse::mutable_connection_state() {
  // @@protoc_insertion_point(field_mutable:mavsdk.rpc.core.ConnectionStateResponse.connection_state)
  return _internal_mutable_connection_state();
}
inline void ConnectionStateResponse::set_allocated_connection_state(::mavsdk::rpc::core::ConnectionState* connection_state) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArena();
  if (message_arena == nullptr) {
    delete connection_state_;
  }
  if (connection_state) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
      ::PROTOBUF_NAMESPACE_ID::Arena::GetArena(connection_state);
    if (message_arena != submessage_arena) {
      connection_state = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, connection_state, submessage_arena);
    }
    
  } else {
    
  }
  connection_state_ = connection_state;
  // @@protoc_insertion_point(field_set_allocated:mavsdk.rpc.core.ConnectionStateResponse.connection_state)
}

// -------------------------------------------------------------------

// ConnectionState

// bool is_connected = 2;
inline void ConnectionState::clear_is_connected() {
  is_connected_ = false;
}
inline bool ConnectionState::_internal_is_connected() const {
  return is_connected_;
}
inline bool ConnectionState::is_connected() const {
  // @@protoc_insertion_point(field_get:mavsdk.rpc.core.ConnectionState.is_connected)
  return _internal_is_connected();
}
inline void ConnectionState::_internal_set_is_connected(bool value) {
  
  is_connected_ = value;
}
inline void ConnectionState::set_is_connected(bool value) {
  _internal_set_is_connected(value);
  // @@protoc_insertion_point(field_set:mavsdk.rpc.core.ConnectionState.is_connected)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace core
}  // namespace rpc
}  // namespace mavsdk

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_core_2fcore_2eproto
