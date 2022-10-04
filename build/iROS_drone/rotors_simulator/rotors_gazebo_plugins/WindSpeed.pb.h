// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: WindSpeed.proto

#ifndef PROTOBUF_INCLUDED_WindSpeed_2eproto
#define PROTOBUF_INCLUDED_WindSpeed_2eproto

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3006001
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "Header.pb.h"
#include "vector3d.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_WindSpeed_2eproto 

namespace protobuf_WindSpeed_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_WindSpeed_2eproto
namespace gz_mav_msgs {
class WindSpeed;
class WindSpeedDefaultTypeInternal;
extern WindSpeedDefaultTypeInternal _WindSpeed_default_instance_;
}  // namespace gz_mav_msgs
namespace google {
namespace protobuf {
template<> ::gz_mav_msgs::WindSpeed* Arena::CreateMaybeMessage<::gz_mav_msgs::WindSpeed>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace gz_mav_msgs {

// ===================================================================

class WindSpeed : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gz_mav_msgs.WindSpeed) */ {
 public:
  WindSpeed();
  virtual ~WindSpeed();

  WindSpeed(const WindSpeed& from);

  inline WindSpeed& operator=(const WindSpeed& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  WindSpeed(WindSpeed&& from) noexcept
    : WindSpeed() {
    *this = ::std::move(from);
  }

  inline WindSpeed& operator=(WindSpeed&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const WindSpeed& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const WindSpeed* internal_default_instance() {
    return reinterpret_cast<const WindSpeed*>(
               &_WindSpeed_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(WindSpeed* other);
  friend void swap(WindSpeed& a, WindSpeed& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline WindSpeed* New() const final {
    return CreateMaybeMessage<WindSpeed>(NULL);
  }

  WindSpeed* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<WindSpeed>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const WindSpeed& from);
  void MergeFrom(const WindSpeed& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(WindSpeed* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required .gz_std_msgs.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  private:
  const ::gz_std_msgs::Header& _internal_header() const;
  public:
  const ::gz_std_msgs::Header& header() const;
  ::gz_std_msgs::Header* release_header();
  ::gz_std_msgs::Header* mutable_header();
  void set_allocated_header(::gz_std_msgs::Header* header);

  // required .gazebo.msgs.Vector3d velocity = 2;
  bool has_velocity() const;
  void clear_velocity();
  static const int kVelocityFieldNumber = 2;
  private:
  const ::gazebo::msgs::Vector3d& _internal_velocity() const;
  public:
  const ::gazebo::msgs::Vector3d& velocity() const;
  ::gazebo::msgs::Vector3d* release_velocity();
  ::gazebo::msgs::Vector3d* mutable_velocity();
  void set_allocated_velocity(::gazebo::msgs::Vector3d* velocity);

  // @@protoc_insertion_point(class_scope:gz_mav_msgs.WindSpeed)
 private:
  void set_has_header();
  void clear_has_header();
  void set_has_velocity();
  void clear_has_velocity();

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::gz_std_msgs::Header* header_;
  ::gazebo::msgs::Vector3d* velocity_;
  friend struct ::protobuf_WindSpeed_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// WindSpeed

// required .gz_std_msgs.Header header = 1;
inline bool WindSpeed::has_header() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void WindSpeed::set_has_header() {
  _has_bits_[0] |= 0x00000001u;
}
inline void WindSpeed::clear_has_header() {
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::gz_std_msgs::Header& WindSpeed::_internal_header() const {
  return *header_;
}
inline const ::gz_std_msgs::Header& WindSpeed::header() const {
  const ::gz_std_msgs::Header* p = header_;
  // @@protoc_insertion_point(field_get:gz_mav_msgs.WindSpeed.header)
  return p != NULL ? *p : *reinterpret_cast<const ::gz_std_msgs::Header*>(
      &::gz_std_msgs::_Header_default_instance_);
}
inline ::gz_std_msgs::Header* WindSpeed::release_header() {
  // @@protoc_insertion_point(field_release:gz_mav_msgs.WindSpeed.header)
  clear_has_header();
  ::gz_std_msgs::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::gz_std_msgs::Header* WindSpeed::mutable_header() {
  set_has_header();
  if (header_ == NULL) {
    auto* p = CreateMaybeMessage<::gz_std_msgs::Header>(GetArenaNoVirtual());
    header_ = p;
  }
  // @@protoc_insertion_point(field_mutable:gz_mav_msgs.WindSpeed.header)
  return header_;
}
inline void WindSpeed::set_allocated_header(::gz_std_msgs::Header* header) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(header_);
  }
  if (header) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      header = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    set_has_header();
  } else {
    clear_has_header();
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:gz_mav_msgs.WindSpeed.header)
}

// required .gazebo.msgs.Vector3d velocity = 2;
inline bool WindSpeed::has_velocity() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void WindSpeed::set_has_velocity() {
  _has_bits_[0] |= 0x00000002u;
}
inline void WindSpeed::clear_has_velocity() {
  _has_bits_[0] &= ~0x00000002u;
}
inline const ::gazebo::msgs::Vector3d& WindSpeed::_internal_velocity() const {
  return *velocity_;
}
inline const ::gazebo::msgs::Vector3d& WindSpeed::velocity() const {
  const ::gazebo::msgs::Vector3d* p = velocity_;
  // @@protoc_insertion_point(field_get:gz_mav_msgs.WindSpeed.velocity)
  return p != NULL ? *p : *reinterpret_cast<const ::gazebo::msgs::Vector3d*>(
      &::gazebo::msgs::_Vector3d_default_instance_);
}
inline ::gazebo::msgs::Vector3d* WindSpeed::release_velocity() {
  // @@protoc_insertion_point(field_release:gz_mav_msgs.WindSpeed.velocity)
  clear_has_velocity();
  ::gazebo::msgs::Vector3d* temp = velocity_;
  velocity_ = NULL;
  return temp;
}
inline ::gazebo::msgs::Vector3d* WindSpeed::mutable_velocity() {
  set_has_velocity();
  if (velocity_ == NULL) {
    auto* p = CreateMaybeMessage<::gazebo::msgs::Vector3d>(GetArenaNoVirtual());
    velocity_ = p;
  }
  // @@protoc_insertion_point(field_mutable:gz_mav_msgs.WindSpeed.velocity)
  return velocity_;
}
inline void WindSpeed::set_allocated_velocity(::gazebo::msgs::Vector3d* velocity) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(velocity_);
  }
  if (velocity) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      velocity = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, velocity, submessage_arena);
    }
    set_has_velocity();
  } else {
    clear_has_velocity();
  }
  velocity_ = velocity;
  // @@protoc_insertion_point(field_set_allocated:gz_mav_msgs.WindSpeed.velocity)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace gz_mav_msgs

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_WindSpeed_2eproto
