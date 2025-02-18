// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: local_path.proto

#include "local_path.pb.h"

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

namespace autonomous_proto {
PROTOBUF_CONSTEXPR LocalPath_Direction::LocalPath_Direction(
    ::_pbi::ConstantInitialized) {}
struct LocalPath_DirectionDefaultTypeInternal {
  PROTOBUF_CONSTEXPR LocalPath_DirectionDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~LocalPath_DirectionDefaultTypeInternal() {}
  union {
    LocalPath_Direction _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 LocalPath_DirectionDefaultTypeInternal _LocalPath_Direction_default_instance_;
PROTOBUF_CONSTEXPR LocalPath_Point::LocalPath_Point(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.lat_)*/nullptr
  , /*decltype(_impl_.lon_)*/nullptr
  , /*decltype(_impl_.alt_)*/nullptr
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct LocalPath_PointDefaultTypeInternal {
  PROTOBUF_CONSTEXPR LocalPath_PointDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~LocalPath_PointDefaultTypeInternal() {}
  union {
    LocalPath_Point _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 LocalPath_PointDefaultTypeInternal _LocalPath_Point_default_instance_;
PROTOBUF_CONSTEXPR LocalPath::LocalPath(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.points_)*/{}
  , /*decltype(_impl_.navigations_)*/{}
  , /*decltype(_impl_.vehicle_states_)*/{}
  , /*decltype(_impl_.header_)*/nullptr
  , /*decltype(_impl_.direction_)*/0
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct LocalPathDefaultTypeInternal {
  PROTOBUF_CONSTEXPR LocalPathDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~LocalPathDefaultTypeInternal() {}
  union {
    LocalPath _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 LocalPathDefaultTypeInternal _LocalPath_default_instance_;
}  // namespace autonomous_proto
static ::_pb::Metadata file_level_metadata_local_5fpath_2eproto[3];
static const ::_pb::EnumDescriptor* file_level_enum_descriptors_local_5fpath_2eproto[1];
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_local_5fpath_2eproto = nullptr;

const uint32_t TableStruct_local_5fpath_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::LocalPath_Direction, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::LocalPath_Point, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::LocalPath_Point, _impl_.lat_),
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::LocalPath_Point, _impl_.lon_),
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::LocalPath_Point, _impl_.alt_),
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::LocalPath, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::LocalPath, _impl_.header_),
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::LocalPath, _impl_.direction_),
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::LocalPath, _impl_.points_),
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::LocalPath, _impl_.navigations_),
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::LocalPath, _impl_.vehicle_states_),
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::autonomous_proto::LocalPath_Direction)},
  { 6, -1, -1, sizeof(::autonomous_proto::LocalPath_Point)},
  { 15, -1, -1, sizeof(::autonomous_proto::LocalPath)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::autonomous_proto::_LocalPath_Direction_default_instance_._instance,
  &::autonomous_proto::_LocalPath_Point_default_instance_._instance,
  &::autonomous_proto::_LocalPath_default_instance_._instance,
};

const char descriptor_table_protodef_local_5fpath_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\020local_path.proto\022\020autonomous_proto\032\036go"
  "ogle/protobuf/wrappers.proto\032\022message_in"
  "fo.proto\032\020navigation.proto\032\023vehicle_stat"
  "e.proto\"\352\003\n\tLocalPath\022-\n\006header\030\001 \001(\0132\035."
  "autonomous_proto.MessageInfo\022>\n\tdirectio"
  "n\030\002 \001(\0162+.autonomous_proto.LocalPath.Dir"
  "ection.Value\0221\n\006points\030\003 \003(\0132!.autonomou"
  "s_proto.LocalPath.Point\0221\n\013navigations\030\004"
  " \003(\0132\034.autonomous_proto.Navigation\0226\n\016ve"
  "hicle_states\030\005 \003(\0132\036.autonomous_proto.Ve"
  "hicleState\032E\n\tDirection\"8\n\005Value\022\013\n\007unkn"
  "own\020\000\022\013\n\007forward\020\001\022\025\n\010backward\020\377\377\377\377\377\377\377\377\377"
  "\001\032\210\001\n\005Point\022)\n\003lat\030\001 \001(\0132\034.google.protob"
  "uf.DoubleValue\022)\n\003lon\030\002 \001(\0132\034.google.pro"
  "tobuf.DoubleValue\022)\n\003alt\030\003 \001(\0132\034.google."
  "protobuf.DoubleValueb\006proto3"
  ;
static const ::_pbi::DescriptorTable* const descriptor_table_local_5fpath_2eproto_deps[4] = {
  &::descriptor_table_google_2fprotobuf_2fwrappers_2eproto,
  &::descriptor_table_message_5finfo_2eproto,
  &::descriptor_table_navigation_2eproto,
  &::descriptor_table_vehicle_5fstate_2eproto,
};
static ::_pbi::once_flag descriptor_table_local_5fpath_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_local_5fpath_2eproto = {
    false, false, 628, descriptor_table_protodef_local_5fpath_2eproto,
    "local_path.proto",
    &descriptor_table_local_5fpath_2eproto_once, descriptor_table_local_5fpath_2eproto_deps, 4, 3,
    schemas, file_default_instances, TableStruct_local_5fpath_2eproto::offsets,
    file_level_metadata_local_5fpath_2eproto, file_level_enum_descriptors_local_5fpath_2eproto,
    file_level_service_descriptors_local_5fpath_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_local_5fpath_2eproto_getter() {
  return &descriptor_table_local_5fpath_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_local_5fpath_2eproto(&descriptor_table_local_5fpath_2eproto);
namespace autonomous_proto {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* LocalPath_Direction_Value_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_local_5fpath_2eproto);
  return file_level_enum_descriptors_local_5fpath_2eproto[0];
}
bool LocalPath_Direction_Value_IsValid(int value) {
  switch (value) {
    case -1:
    case 0:
    case 1:
      return true;
    default:
      return false;
  }
}

#if (__cplusplus < 201703) && (!defined(_MSC_VER) || (_MSC_VER >= 1900 && _MSC_VER < 1912))
constexpr LocalPath_Direction_Value LocalPath_Direction::unknown;
constexpr LocalPath_Direction_Value LocalPath_Direction::forward;
constexpr LocalPath_Direction_Value LocalPath_Direction::backward;
constexpr LocalPath_Direction_Value LocalPath_Direction::Value_MIN;
constexpr LocalPath_Direction_Value LocalPath_Direction::Value_MAX;
constexpr int LocalPath_Direction::Value_ARRAYSIZE;
#endif  // (__cplusplus < 201703) && (!defined(_MSC_VER) || (_MSC_VER >= 1900 && _MSC_VER < 1912))

// ===================================================================

class LocalPath_Direction::_Internal {
 public:
};

LocalPath_Direction::LocalPath_Direction(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::internal::ZeroFieldsBase(arena, is_message_owned) {
  // @@protoc_insertion_point(arena_constructor:autonomous_proto.LocalPath.Direction)
}
LocalPath_Direction::LocalPath_Direction(const LocalPath_Direction& from)
  : ::PROTOBUF_NAMESPACE_ID::internal::ZeroFieldsBase() {
  LocalPath_Direction* const _this = this; (void)_this;
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:autonomous_proto.LocalPath.Direction)
}





const ::PROTOBUF_NAMESPACE_ID::Message::ClassData LocalPath_Direction::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::internal::ZeroFieldsBase::CopyImpl,
    ::PROTOBUF_NAMESPACE_ID::internal::ZeroFieldsBase::MergeImpl,
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*LocalPath_Direction::GetClassData() const { return &_class_data_; }







::PROTOBUF_NAMESPACE_ID::Metadata LocalPath_Direction::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_local_5fpath_2eproto_getter, &descriptor_table_local_5fpath_2eproto_once,
      file_level_metadata_local_5fpath_2eproto[0]);
}

// ===================================================================

class LocalPath_Point::_Internal {
 public:
  static const ::PROTOBUF_NAMESPACE_ID::DoubleValue& lat(const LocalPath_Point* msg);
  static const ::PROTOBUF_NAMESPACE_ID::DoubleValue& lon(const LocalPath_Point* msg);
  static const ::PROTOBUF_NAMESPACE_ID::DoubleValue& alt(const LocalPath_Point* msg);
};

const ::PROTOBUF_NAMESPACE_ID::DoubleValue&
LocalPath_Point::_Internal::lat(const LocalPath_Point* msg) {
  return *msg->_impl_.lat_;
}
const ::PROTOBUF_NAMESPACE_ID::DoubleValue&
LocalPath_Point::_Internal::lon(const LocalPath_Point* msg) {
  return *msg->_impl_.lon_;
}
const ::PROTOBUF_NAMESPACE_ID::DoubleValue&
LocalPath_Point::_Internal::alt(const LocalPath_Point* msg) {
  return *msg->_impl_.alt_;
}
void LocalPath_Point::clear_lat() {
  if (GetArenaForAllocation() == nullptr && _impl_.lat_ != nullptr) {
    delete _impl_.lat_;
  }
  _impl_.lat_ = nullptr;
}
void LocalPath_Point::clear_lon() {
  if (GetArenaForAllocation() == nullptr && _impl_.lon_ != nullptr) {
    delete _impl_.lon_;
  }
  _impl_.lon_ = nullptr;
}
void LocalPath_Point::clear_alt() {
  if (GetArenaForAllocation() == nullptr && _impl_.alt_ != nullptr) {
    delete _impl_.alt_;
  }
  _impl_.alt_ = nullptr;
}
LocalPath_Point::LocalPath_Point(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:autonomous_proto.LocalPath.Point)
}
LocalPath_Point::LocalPath_Point(const LocalPath_Point& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  LocalPath_Point* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.lat_){nullptr}
    , decltype(_impl_.lon_){nullptr}
    , decltype(_impl_.alt_){nullptr}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_lat()) {
    _this->_impl_.lat_ = new ::PROTOBUF_NAMESPACE_ID::DoubleValue(*from._impl_.lat_);
  }
  if (from._internal_has_lon()) {
    _this->_impl_.lon_ = new ::PROTOBUF_NAMESPACE_ID::DoubleValue(*from._impl_.lon_);
  }
  if (from._internal_has_alt()) {
    _this->_impl_.alt_ = new ::PROTOBUF_NAMESPACE_ID::DoubleValue(*from._impl_.alt_);
  }
  // @@protoc_insertion_point(copy_constructor:autonomous_proto.LocalPath.Point)
}

inline void LocalPath_Point::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.lat_){nullptr}
    , decltype(_impl_.lon_){nullptr}
    , decltype(_impl_.alt_){nullptr}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

LocalPath_Point::~LocalPath_Point() {
  // @@protoc_insertion_point(destructor:autonomous_proto.LocalPath.Point)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void LocalPath_Point::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete _impl_.lat_;
  if (this != internal_default_instance()) delete _impl_.lon_;
  if (this != internal_default_instance()) delete _impl_.alt_;
}

void LocalPath_Point::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void LocalPath_Point::Clear() {
// @@protoc_insertion_point(message_clear_start:autonomous_proto.LocalPath.Point)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaForAllocation() == nullptr && _impl_.lat_ != nullptr) {
    delete _impl_.lat_;
  }
  _impl_.lat_ = nullptr;
  if (GetArenaForAllocation() == nullptr && _impl_.lon_ != nullptr) {
    delete _impl_.lon_;
  }
  _impl_.lon_ = nullptr;
  if (GetArenaForAllocation() == nullptr && _impl_.alt_ != nullptr) {
    delete _impl_.alt_;
  }
  _impl_.alt_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* LocalPath_Point::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .google.protobuf.DoubleValue Expected_path_lat = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_lat(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .google.protobuf.DoubleValue Expected_path_lon = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_lon(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .google.protobuf.DoubleValue Expected_path_alt = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 26)) {
          ptr = ctx->ParseMessage(_internal_mutable_alt(), ptr);
          CHK_(ptr);
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

uint8_t* LocalPath_Point::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:autonomous_proto.LocalPath.Point)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .google.protobuf.DoubleValue Expected_path_lat = 1;
  if (this->_internal_has_lat()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, _Internal::lat(this),
        _Internal::lat(this).GetCachedSize(), target, stream);
  }

  // .google.protobuf.DoubleValue Expected_path_lon = 2;
  if (this->_internal_has_lon()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, _Internal::lon(this),
        _Internal::lon(this).GetCachedSize(), target, stream);
  }

  // .google.protobuf.DoubleValue Expected_path_alt = 3;
  if (this->_internal_has_alt()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3, _Internal::alt(this),
        _Internal::alt(this).GetCachedSize(), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:autonomous_proto.LocalPath.Point)
  return target;
}

size_t LocalPath_Point::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:autonomous_proto.LocalPath.Point)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .google.protobuf.DoubleValue Expected_path_lat = 1;
  if (this->_internal_has_lat()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.lat_);
  }

  // .google.protobuf.DoubleValue Expected_path_lon = 2;
  if (this->_internal_has_lon()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.lon_);
  }

  // .google.protobuf.DoubleValue Expected_path_alt = 3;
  if (this->_internal_has_alt()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.alt_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData LocalPath_Point::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    LocalPath_Point::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*LocalPath_Point::GetClassData() const { return &_class_data_; }


void LocalPath_Point::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<LocalPath_Point*>(&to_msg);
  auto& from = static_cast<const LocalPath_Point&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:autonomous_proto.LocalPath.Point)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_lat()) {
    _this->_internal_mutable_lat()->::PROTOBUF_NAMESPACE_ID::DoubleValue::MergeFrom(
        from._internal_lat());
  }
  if (from._internal_has_lon()) {
    _this->_internal_mutable_lon()->::PROTOBUF_NAMESPACE_ID::DoubleValue::MergeFrom(
        from._internal_lon());
  }
  if (from._internal_has_alt()) {
    _this->_internal_mutable_alt()->::PROTOBUF_NAMESPACE_ID::DoubleValue::MergeFrom(
        from._internal_alt());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void LocalPath_Point::CopyFrom(const LocalPath_Point& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:autonomous_proto.LocalPath.Point)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool LocalPath_Point::IsInitialized() const {
  return true;
}

void LocalPath_Point::InternalSwap(LocalPath_Point* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(LocalPath_Point, _impl_.alt_)
      + sizeof(LocalPath_Point::_impl_.alt_)
      - PROTOBUF_FIELD_OFFSET(LocalPath_Point, _impl_.lat_)>(
          reinterpret_cast<char*>(&_impl_.lat_),
          reinterpret_cast<char*>(&other->_impl_.lat_));
}

::PROTOBUF_NAMESPACE_ID::Metadata LocalPath_Point::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_local_5fpath_2eproto_getter, &descriptor_table_local_5fpath_2eproto_once,
      file_level_metadata_local_5fpath_2eproto[1]);
}

// ===================================================================

class LocalPath::_Internal {
 public:
  static const ::autonomous_proto::MessageInfo& header(const LocalPath* msg);
};

const ::autonomous_proto::MessageInfo&
LocalPath::_Internal::header(const LocalPath* msg) {
  return *msg->_impl_.header_;
}
void LocalPath::clear_header() {
  if (GetArenaForAllocation() == nullptr && _impl_.header_ != nullptr) {
    delete _impl_.header_;
  }
  _impl_.header_ = nullptr;
}
void LocalPath::clear_navigations() {
  _impl_.navigations_.Clear();
}
void LocalPath::clear_vehicle_states() {
  _impl_.vehicle_states_.Clear();
}
LocalPath::LocalPath(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:autonomous_proto.LocalPath)
}
LocalPath::LocalPath(const LocalPath& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  LocalPath* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.points_){from._impl_.points_}
    , decltype(_impl_.navigations_){from._impl_.navigations_}
    , decltype(_impl_.vehicle_states_){from._impl_.vehicle_states_}
    , decltype(_impl_.header_){nullptr}
    , decltype(_impl_.direction_){}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    _this->_impl_.header_ = new ::autonomous_proto::MessageInfo(*from._impl_.header_);
  }
  _this->_impl_.direction_ = from._impl_.direction_;
  // @@protoc_insertion_point(copy_constructor:autonomous_proto.LocalPath)
}

inline void LocalPath::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.points_){arena}
    , decltype(_impl_.navigations_){arena}
    , decltype(_impl_.vehicle_states_){arena}
    , decltype(_impl_.header_){nullptr}
    , decltype(_impl_.direction_){0}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

LocalPath::~LocalPath() {
  // @@protoc_insertion_point(destructor:autonomous_proto.LocalPath)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void LocalPath::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  _impl_.points_.~RepeatedPtrField();
  _impl_.navigations_.~RepeatedPtrField();
  _impl_.vehicle_states_.~RepeatedPtrField();
  if (this != internal_default_instance()) delete _impl_.header_;
}

void LocalPath::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void LocalPath::Clear() {
// @@protoc_insertion_point(message_clear_start:autonomous_proto.LocalPath)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  _impl_.points_.Clear();
  _impl_.navigations_.Clear();
  _impl_.vehicle_states_.Clear();
  if (GetArenaForAllocation() == nullptr && _impl_.header_ != nullptr) {
    delete _impl_.header_;
  }
  _impl_.header_ = nullptr;
  _impl_.direction_ = 0;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* LocalPath::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .autonomous_proto.MessageInfo header = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_header(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .autonomous_proto.LocalPath.Direction.Value direction = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 16)) {
          uint64_t val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          _internal_set_direction(static_cast<::autonomous_proto::LocalPath_Direction_Value>(val));
        } else
          goto handle_unusual;
        continue;
      // repeated .autonomous_proto.LocalPath.Point points = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 26)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_points(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<26>(ptr));
        } else
          goto handle_unusual;
        continue;
      // repeated .autonomous_proto.Navigation navigations = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 34)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_navigations(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<34>(ptr));
        } else
          goto handle_unusual;
        continue;
      // repeated .autonomous_proto.VehicleState vehicle_states = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 42)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_vehicle_states(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<42>(ptr));
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

uint8_t* LocalPath::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:autonomous_proto.LocalPath)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .autonomous_proto.MessageInfo header = 1;
  if (this->_internal_has_header()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, _Internal::header(this),
        _Internal::header(this).GetCachedSize(), target, stream);
  }

  // .autonomous_proto.LocalPath.Direction.Value direction = 2;
  if (this->_internal_direction() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteEnumToArray(
      2, this->_internal_direction(), target);
  }

  // repeated .autonomous_proto.LocalPath.Point points = 3;
  for (unsigned i = 0,
      n = static_cast<unsigned>(this->_internal_points_size()); i < n; i++) {
    const auto& repfield = this->_internal_points(i);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
        InternalWriteMessage(3, repfield, repfield.GetCachedSize(), target, stream);
  }

  // repeated .autonomous_proto.Navigation navigations = 4;
  for (unsigned i = 0,
      n = static_cast<unsigned>(this->_internal_navigations_size()); i < n; i++) {
    const auto& repfield = this->_internal_navigations(i);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
        InternalWriteMessage(4, repfield, repfield.GetCachedSize(), target, stream);
  }

  // repeated .autonomous_proto.VehicleState vehicle_states = 5;
  for (unsigned i = 0,
      n = static_cast<unsigned>(this->_internal_vehicle_states_size()); i < n; i++) {
    const auto& repfield = this->_internal_vehicle_states(i);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
        InternalWriteMessage(5, repfield, repfield.GetCachedSize(), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:autonomous_proto.LocalPath)
  return target;
}

size_t LocalPath::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:autonomous_proto.LocalPath)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .autonomous_proto.LocalPath.Point points = 3;
  total_size += 1UL * this->_internal_points_size();
  for (const auto& msg : this->_impl_.points_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // repeated .autonomous_proto.Navigation navigations = 4;
  total_size += 1UL * this->_internal_navigations_size();
  for (const auto& msg : this->_impl_.navigations_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // repeated .autonomous_proto.VehicleState vehicle_states = 5;
  total_size += 1UL * this->_internal_vehicle_states_size();
  for (const auto& msg : this->_impl_.vehicle_states_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // .autonomous_proto.MessageInfo header = 1;
  if (this->_internal_has_header()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.header_);
  }

  // .autonomous_proto.LocalPath.Direction.Value direction = 2;
  if (this->_internal_direction() != 0) {
    total_size += 1 +
      ::_pbi::WireFormatLite::EnumSize(this->_internal_direction());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData LocalPath::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    LocalPath::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*LocalPath::GetClassData() const { return &_class_data_; }


void LocalPath::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<LocalPath*>(&to_msg);
  auto& from = static_cast<const LocalPath&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:autonomous_proto.LocalPath)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  _this->_impl_.points_.MergeFrom(from._impl_.points_);
  _this->_impl_.navigations_.MergeFrom(from._impl_.navigations_);
  _this->_impl_.vehicle_states_.MergeFrom(from._impl_.vehicle_states_);
  if (from._internal_has_header()) {
    _this->_internal_mutable_header()->::autonomous_proto::MessageInfo::MergeFrom(
        from._internal_header());
  }
  if (from._internal_direction() != 0) {
    _this->_internal_set_direction(from._internal_direction());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void LocalPath::CopyFrom(const LocalPath& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:autonomous_proto.LocalPath)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool LocalPath::IsInitialized() const {
  return true;
}

void LocalPath::InternalSwap(LocalPath* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  _impl_.points_.InternalSwap(&other->_impl_.points_);
  _impl_.navigations_.InternalSwap(&other->_impl_.navigations_);
  _impl_.vehicle_states_.InternalSwap(&other->_impl_.vehicle_states_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(LocalPath, _impl_.direction_)
      + sizeof(LocalPath::_impl_.direction_)
      - PROTOBUF_FIELD_OFFSET(LocalPath, _impl_.header_)>(
          reinterpret_cast<char*>(&_impl_.header_),
          reinterpret_cast<char*>(&other->_impl_.header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata LocalPath::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_local_5fpath_2eproto_getter, &descriptor_table_local_5fpath_2eproto_once,
      file_level_metadata_local_5fpath_2eproto[2]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace autonomous_proto
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::autonomous_proto::LocalPath_Direction*
Arena::CreateMaybeMessage< ::autonomous_proto::LocalPath_Direction >(Arena* arena) {
  return Arena::CreateMessageInternal< ::autonomous_proto::LocalPath_Direction >(arena);
}
template<> PROTOBUF_NOINLINE ::autonomous_proto::LocalPath_Point*
Arena::CreateMaybeMessage< ::autonomous_proto::LocalPath_Point >(Arena* arena) {
  return Arena::CreateMessageInternal< ::autonomous_proto::LocalPath_Point >(arena);
}
template<> PROTOBUF_NOINLINE ::autonomous_proto::LocalPath*
Arena::CreateMaybeMessage< ::autonomous_proto::LocalPath >(Arena* arena) {
  return Arena::CreateMessageInternal< ::autonomous_proto::LocalPath >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
