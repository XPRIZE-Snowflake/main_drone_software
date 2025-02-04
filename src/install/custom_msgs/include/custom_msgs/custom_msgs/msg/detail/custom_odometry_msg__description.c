// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from custom_msgs:msg/CustomOdometryMsg.idl
// generated code does not contain a copyright notice

#include "custom_msgs/msg/detail/custom_odometry_msg__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_custom_msgs
const rosidl_type_hash_t *
custom_msgs__msg__CustomOdometryMsg__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x40, 0x88, 0xd3, 0x1a, 0xd5, 0xd9, 0x30, 0x8e,
      0xf1, 0x45, 0x97, 0xbd, 0x57, 0x61, 0x1f, 0x9b,
      0x93, 0x8c, 0x1d, 0xea, 0xd1, 0x9c, 0xbe, 0x2a,
      0xa7, 0x75, 0x8e, 0xe6, 0xb6, 0xf9, 0xff, 0x11,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char custom_msgs__msg__CustomOdometryMsg__TYPE_NAME[] = "custom_msgs/msg/CustomOdometryMsg";

// Define type names, field names, and default values
static char custom_msgs__msg__CustomOdometryMsg__FIELD_NAME__timestamp[] = "timestamp";
static char custom_msgs__msg__CustomOdometryMsg__FIELD_NAME__latitude[] = "latitude";
static char custom_msgs__msg__CustomOdometryMsg__FIELD_NAME__longitude[] = "longitude";
static char custom_msgs__msg__CustomOdometryMsg__FIELD_NAME__altitude[] = "altitude";
static char custom_msgs__msg__CustomOdometryMsg__FIELD_NAME__pitch[] = "pitch";
static char custom_msgs__msg__CustomOdometryMsg__FIELD_NAME__roll[] = "roll";
static char custom_msgs__msg__CustomOdometryMsg__FIELD_NAME__yaw[] = "yaw";

static rosidl_runtime_c__type_description__Field custom_msgs__msg__CustomOdometryMsg__FIELDS[] = {
  {
    {custom_msgs__msg__CustomOdometryMsg__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {custom_msgs__msg__CustomOdometryMsg__FIELD_NAME__latitude, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {custom_msgs__msg__CustomOdometryMsg__FIELD_NAME__longitude, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {custom_msgs__msg__CustomOdometryMsg__FIELD_NAME__altitude, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {custom_msgs__msg__CustomOdometryMsg__FIELD_NAME__pitch, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {custom_msgs__msg__CustomOdometryMsg__FIELD_NAME__roll, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {custom_msgs__msg__CustomOdometryMsg__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
custom_msgs__msg__CustomOdometryMsg__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {custom_msgs__msg__CustomOdometryMsg__TYPE_NAME, 33, 33},
      {custom_msgs__msg__CustomOdometryMsg__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 timestamp\n"
  "float64 latitude\n"
  "float64 longitude\n"
  "float64 altitude\n"
  "float64 pitch\n"
  "float64 roll\n"
  "float64 yaw";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
custom_msgs__msg__CustomOdometryMsg__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {custom_msgs__msg__CustomOdometryMsg__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 109, 109},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
custom_msgs__msg__CustomOdometryMsg__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *custom_msgs__msg__CustomOdometryMsg__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
