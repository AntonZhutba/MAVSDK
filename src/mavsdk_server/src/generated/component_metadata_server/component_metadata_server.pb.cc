// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: component_metadata_server/component_metadata_server.proto

#include "component_metadata_server/component_metadata_server.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG

namespace _pb = ::PROTOBUF_NAMESPACE_ID;
namespace _pbi = _pb::internal;

namespace mavsdk {
namespace rpc {
namespace component_metadata_server {
PROTOBUF_CONSTEXPR SetMetadataRequest::SetMetadataRequest(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.metadata_)*/{}
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct SetMetadataRequestDefaultTypeInternal {
  PROTOBUF_CONSTEXPR SetMetadataRequestDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~SetMetadataRequestDefaultTypeInternal() {}
  union {
    SetMetadataRequest _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 SetMetadataRequestDefaultTypeInternal _SetMetadataRequest_default_instance_;
PROTOBUF_CONSTEXPR SetMetadataResponse::SetMetadataResponse(
    ::_pbi::ConstantInitialized) {}
struct SetMetadataResponseDefaultTypeInternal {
  PROTOBUF_CONSTEXPR SetMetadataResponseDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~SetMetadataResponseDefaultTypeInternal() {}
  union {
    SetMetadataResponse _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 SetMetadataResponseDefaultTypeInternal _SetMetadataResponse_default_instance_;
PROTOBUF_CONSTEXPR Metadata::Metadata(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.json_metadata_)*/{&::_pbi::fixed_address_empty_string, ::_pbi::ConstantInitialized{}}
  , /*decltype(_impl_.type_)*/0
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct MetadataDefaultTypeInternal {
  PROTOBUF_CONSTEXPR MetadataDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~MetadataDefaultTypeInternal() {}
  union {
    Metadata _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 MetadataDefaultTypeInternal _Metadata_default_instance_;
}  // namespace component_metadata_server
}  // namespace rpc
}  // namespace mavsdk
static ::_pb::Metadata file_level_metadata_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto[3];
static const ::_pb::EnumDescriptor* file_level_enum_descriptors_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto[1];
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto = nullptr;

const uint32_t TableStruct_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::component_metadata_server::SetMetadataRequest, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::component_metadata_server::SetMetadataRequest, _impl_.metadata_),
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::component_metadata_server::SetMetadataResponse, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::component_metadata_server::Metadata, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::component_metadata_server::Metadata, _impl_.type_),
  PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::component_metadata_server::Metadata, _impl_.json_metadata_),
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::mavsdk::rpc::component_metadata_server::SetMetadataRequest)},
  { 7, -1, -1, sizeof(::mavsdk::rpc::component_metadata_server::SetMetadataResponse)},
  { 13, -1, -1, sizeof(::mavsdk::rpc::component_metadata_server::Metadata)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::mavsdk::rpc::component_metadata_server::_SetMetadataRequest_default_instance_._instance,
  &::mavsdk::rpc::component_metadata_server::_SetMetadataResponse_default_instance_._instance,
  &::mavsdk::rpc::component_metadata_server::_Metadata_default_instance_._instance,
};

const char descriptor_table_protodef_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n9component_metadata_server/component_me"
  "tadata_server.proto\022$mavsdk.rpc.componen"
  "t_metadata_server\032\024mavsdk_options.proto\""
  "V\n\022SetMetadataRequest\022@\n\010metadata\030\001 \003(\0132"
  "..mavsdk.rpc.component_metadata_server.M"
  "etadata\"\025\n\023SetMetadataResponse\"c\n\010Metada"
  "ta\022@\n\004type\030\001 \001(\01622.mavsdk.rpc.component_"
  "metadata_server.MetadataType\022\025\n\rjson_met"
  "adata\030\002 \001(\t*b\n\014MetadataType\022\033\n\027METADATA_"
  "TYPE_PARAMETER\020\000\022\030\n\024METADATA_TYPE_EVENTS"
  "\020\001\022\033\n\027METADATA_TYPE_ACTUATORS\020\0022\253\001\n\036Comp"
  "onentMetadataServerService\022\210\001\n\013SetMetada"
  "ta\0228.mavsdk.rpc.component_metadata_serve"
  "r.SetMetadataRequest\0329.mavsdk.rpc.compon"
  "ent_metadata_server.SetMetadataResponse\""
  "\004\200\265\030\001BC\n#io.mavsdk.component_metadata_se"
  "rverB\034ComponentMetadataServerProtob\006prot"
  "o3"
  ;
static const ::_pbi::DescriptorTable* const descriptor_table_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto_deps[1] = {
  &::descriptor_table_mavsdk_5foptions_2eproto,
};
static ::_pbi::once_flag descriptor_table_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto = {
    false, false, 682, descriptor_table_protodef_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto,
    "component_metadata_server/component_metadata_server.proto",
    &descriptor_table_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto_once, descriptor_table_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto_deps, 1, 3,
    schemas, file_default_instances, TableStruct_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto::offsets,
    file_level_metadata_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto, file_level_enum_descriptors_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto,
    file_level_service_descriptors_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto_getter() {
  return &descriptor_table_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto(&descriptor_table_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto);
namespace mavsdk {
namespace rpc {
namespace component_metadata_server {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* MetadataType_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto);
  return file_level_enum_descriptors_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto[0];
}
bool MetadataType_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
      return true;
    default:
      return false;
  }
}


// ===================================================================

class SetMetadataRequest::_Internal {
 public:
};

SetMetadataRequest::SetMetadataRequest(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:mavsdk.rpc.component_metadata_server.SetMetadataRequest)
}
SetMetadataRequest::SetMetadataRequest(const SetMetadataRequest& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  SetMetadataRequest* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.metadata_){from._impl_.metadata_}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:mavsdk.rpc.component_metadata_server.SetMetadataRequest)
}

inline void SetMetadataRequest::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.metadata_){arena}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

SetMetadataRequest::~SetMetadataRequest() {
  // @@protoc_insertion_point(destructor:mavsdk.rpc.component_metadata_server.SetMetadataRequest)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void SetMetadataRequest::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  _impl_.metadata_.~RepeatedPtrField();
}

void SetMetadataRequest::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void SetMetadataRequest::Clear() {
// @@protoc_insertion_point(message_clear_start:mavsdk.rpc.component_metadata_server.SetMetadataRequest)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  _impl_.metadata_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* SetMetadataRequest::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // repeated .mavsdk.rpc.component_metadata_server.Metadata metadata = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_metadata(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<10>(ptr));
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* SetMetadataRequest::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:mavsdk.rpc.component_metadata_server.SetMetadataRequest)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .mavsdk.rpc.component_metadata_server.Metadata metadata = 1;
  for (unsigned i = 0,
      n = static_cast<unsigned>(this->_internal_metadata_size()); i < n; i++) {
    const auto& repfield = this->_internal_metadata(i);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
        InternalWriteMessage(1, repfield, repfield.GetCachedSize(), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:mavsdk.rpc.component_metadata_server.SetMetadataRequest)
  return target;
}

size_t SetMetadataRequest::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:mavsdk.rpc.component_metadata_server.SetMetadataRequest)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .mavsdk.rpc.component_metadata_server.Metadata metadata = 1;
  total_size += 1UL * this->_internal_metadata_size();
  for (const auto& msg : this->_impl_.metadata_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData SetMetadataRequest::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    SetMetadataRequest::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*SetMetadataRequest::GetClassData() const { return &_class_data_; }


void SetMetadataRequest::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<SetMetadataRequest*>(&to_msg);
  auto& from = static_cast<const SetMetadataRequest&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:mavsdk.rpc.component_metadata_server.SetMetadataRequest)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  _this->_impl_.metadata_.MergeFrom(from._impl_.metadata_);
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void SetMetadataRequest::CopyFrom(const SetMetadataRequest& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:mavsdk.rpc.component_metadata_server.SetMetadataRequest)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SetMetadataRequest::IsInitialized() const {
  return true;
}

void SetMetadataRequest::InternalSwap(SetMetadataRequest* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  _impl_.metadata_.InternalSwap(&other->_impl_.metadata_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SetMetadataRequest::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto_getter, &descriptor_table_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto_once,
      file_level_metadata_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto[0]);
}

// ===================================================================

class SetMetadataResponse::_Internal {
 public:
};

SetMetadataResponse::SetMetadataResponse(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::internal::ZeroFieldsBase(arena, is_message_owned) {
  // @@protoc_insertion_point(arena_constructor:mavsdk.rpc.component_metadata_server.SetMetadataResponse)
}
SetMetadataResponse::SetMetadataResponse(const SetMetadataResponse& from)
  : ::PROTOBUF_NAMESPACE_ID::internal::ZeroFieldsBase() {
  SetMetadataResponse* const _this = this; (void)_this;
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:mavsdk.rpc.component_metadata_server.SetMetadataResponse)
}





const ::PROTOBUF_NAMESPACE_ID::Message::ClassData SetMetadataResponse::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::internal::ZeroFieldsBase::CopyImpl,
    ::PROTOBUF_NAMESPACE_ID::internal::ZeroFieldsBase::MergeImpl,
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*SetMetadataResponse::GetClassData() const { return &_class_data_; }







::PROTOBUF_NAMESPACE_ID::Metadata SetMetadataResponse::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto_getter, &descriptor_table_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto_once,
      file_level_metadata_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto[1]);
}

// ===================================================================

class Metadata::_Internal {
 public:
};

Metadata::Metadata(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:mavsdk.rpc.component_metadata_server.Metadata)
}
Metadata::Metadata(const Metadata& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  Metadata* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.json_metadata_){}
    , decltype(_impl_.type_){}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  _impl_.json_metadata_.InitDefault();
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    _impl_.json_metadata_.Set("", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (!from._internal_json_metadata().empty()) {
    _this->_impl_.json_metadata_.Set(from._internal_json_metadata(), 
      _this->GetArenaForAllocation());
  }
  _this->_impl_.type_ = from._impl_.type_;
  // @@protoc_insertion_point(copy_constructor:mavsdk.rpc.component_metadata_server.Metadata)
}

inline void Metadata::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.json_metadata_){}
    , decltype(_impl_.type_){0}
    , /*decltype(_impl_._cached_size_)*/{}
  };
  _impl_.json_metadata_.InitDefault();
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    _impl_.json_metadata_.Set("", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
}

Metadata::~Metadata() {
  // @@protoc_insertion_point(destructor:mavsdk.rpc.component_metadata_server.Metadata)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void Metadata::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  _impl_.json_metadata_.Destroy();
}

void Metadata::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void Metadata::Clear() {
// @@protoc_insertion_point(message_clear_start:mavsdk.rpc.component_metadata_server.Metadata)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  _impl_.json_metadata_.ClearToEmpty();
  _impl_.type_ = 0;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Metadata::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .mavsdk.rpc.component_metadata_server.MetadataType type = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 8)) {
          uint64_t val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          _internal_set_type(static_cast<::mavsdk::rpc::component_metadata_server::MetadataType>(val));
        } else
          goto handle_unusual;
        continue;
      // string json_metadata = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          auto str = _internal_mutable_json_metadata();
          ptr = ::_pbi::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(ptr);
          CHK_(::_pbi::VerifyUTF8(str, "mavsdk.rpc.component_metadata_server.Metadata.json_metadata"));
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* Metadata::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:mavsdk.rpc.component_metadata_server.Metadata)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .mavsdk.rpc.component_metadata_server.MetadataType type = 1;
  if (this->_internal_type() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteEnumToArray(
      1, this->_internal_type(), target);
  }

  // string json_metadata = 2;
  if (!this->_internal_json_metadata().empty()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_json_metadata().data(), static_cast<int>(this->_internal_json_metadata().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "mavsdk.rpc.component_metadata_server.Metadata.json_metadata");
    target = stream->WriteStringMaybeAliased(
        2, this->_internal_json_metadata(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:mavsdk.rpc.component_metadata_server.Metadata)
  return target;
}

size_t Metadata::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:mavsdk.rpc.component_metadata_server.Metadata)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // string json_metadata = 2;
  if (!this->_internal_json_metadata().empty()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_json_metadata());
  }

  // .mavsdk.rpc.component_metadata_server.MetadataType type = 1;
  if (this->_internal_type() != 0) {
    total_size += 1 +
      ::_pbi::WireFormatLite::EnumSize(this->_internal_type());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Metadata::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    Metadata::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Metadata::GetClassData() const { return &_class_data_; }


void Metadata::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<Metadata*>(&to_msg);
  auto& from = static_cast<const Metadata&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:mavsdk.rpc.component_metadata_server.Metadata)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (!from._internal_json_metadata().empty()) {
    _this->_internal_set_json_metadata(from._internal_json_metadata());
  }
  if (from._internal_type() != 0) {
    _this->_internal_set_type(from._internal_type());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Metadata::CopyFrom(const Metadata& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:mavsdk.rpc.component_metadata_server.Metadata)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Metadata::IsInitialized() const {
  return true;
}

void Metadata::InternalSwap(Metadata* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &_impl_.json_metadata_, lhs_arena,
      &other->_impl_.json_metadata_, rhs_arena
  );
  swap(_impl_.type_, other->_impl_.type_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Metadata::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto_getter, &descriptor_table_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto_once,
      file_level_metadata_component_5fmetadata_5fserver_2fcomponent_5fmetadata_5fserver_2eproto[2]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace component_metadata_server
}  // namespace rpc
}  // namespace mavsdk
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::mavsdk::rpc::component_metadata_server::SetMetadataRequest*
Arena::CreateMaybeMessage< ::mavsdk::rpc::component_metadata_server::SetMetadataRequest >(Arena* arena) {
  return Arena::CreateMessageInternal< ::mavsdk::rpc::component_metadata_server::SetMetadataRequest >(arena);
}
template<> PROTOBUF_NOINLINE ::mavsdk::rpc::component_metadata_server::SetMetadataResponse*
Arena::CreateMaybeMessage< ::mavsdk::rpc::component_metadata_server::SetMetadataResponse >(Arena* arena) {
  return Arena::CreateMessageInternal< ::mavsdk::rpc::component_metadata_server::SetMetadataResponse >(arena);
}
template<> PROTOBUF_NOINLINE ::mavsdk::rpc::component_metadata_server::Metadata*
Arena::CreateMaybeMessage< ::mavsdk::rpc::component_metadata_server::Metadata >(Arena* arena) {
  return Arena::CreateMessageInternal< ::mavsdk::rpc::component_metadata_server::Metadata >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
