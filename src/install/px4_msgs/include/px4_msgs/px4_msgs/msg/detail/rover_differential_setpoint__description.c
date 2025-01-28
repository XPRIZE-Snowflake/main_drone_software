// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/RoverDifferentialSetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/rover_differential_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__RoverDifferentialSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xdd, 0x6f, 0x0b, 0xaa, 0x03, 0x91, 0x87, 0x0d,
      0xa1, 0x07, 0xe2, 0x8b, 0x75, 0x68, 0xf5, 0xdc,
      0x7e, 0xf8, 0xe8, 0x03, 0x23, 0xae, 0xd0, 0xcc,
      0x50, 0x4d, 0x99, 0x0a, 0x97, 0x59, 0x7a, 0xc3,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__RoverDifferentialSetpoint__TYPE_NAME[] = "px4_msgs/msg/RoverDifferentialSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__RoverDifferentialSetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__RoverDifferentialSetpoint__FIELD_NAME__forward_speed_setpoint[] = "forward_speed_setpoint";
static char px4_msgs__msg__RoverDifferentialSetpoint__FIELD_NAME__forward_speed_setpoint_normalized[] = "forward_speed_setpoint_normalized";
static char px4_msgs__msg__RoverDifferentialSetpoint__FIELD_NAME__yaw_rate_setpoint[] = "yaw_rate_setpoint";
static char px4_msgs__msg__RoverDifferentialSetpoint__FIELD_NAME__speed_diff_setpoint_normalized[] = "speed_diff_setpoint_normalized";
static char px4_msgs__msg__RoverDifferentialSetpoint__FIELD_NAME__yaw_setpoint[] = "yaw_setpoint";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__RoverDifferentialSetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__RoverDifferentialSetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverDifferentialSetpoint__FIELD_NAME__forward_speed_setpoint, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverDifferentialSetpoint__FIELD_NAME__forward_speed_setpoint_normalized, 33, 33},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverDifferentialSetpoint__FIELD_NAME__yaw_rate_setpoint, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverDifferentialSetpoint__FIELD_NAME__speed_diff_setpoint_normalized, 30, 30},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverDifferentialSetpoint__FIELD_NAME__yaw_setpoint, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__RoverDifferentialSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__RoverDifferentialSetpoint__TYPE_NAME, 38, 38},
      {px4_msgs__msg__RoverDifferentialSetpoint__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "float32 forward_speed_setpoint \\t\\t  # [m/s] Desired forward speed for the rover\n"
  "float32 forward_speed_setpoint_normalized # [-1, 1] Normalized forward speed for the rover\n"
  "float32 yaw_rate_setpoint      \\t\\t  # [rad/s] Desired yaw rate for the rover (Overriden by yaw controller if yaw_setpoint is used)\n"
  "float32 speed_diff_setpoint_normalized    # [-1, 1] Normalized speed difference between the left and right wheels\n"
  "float32 yaw_setpoint \\t       \\t\\t  # [rad] Desired yaw (heading) for the rover\n"
  "\n"
  "# TOPICS rover_differential_setpoint";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__RoverDifferentialSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__RoverDifferentialSetpoint__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 588, 588},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__RoverDifferentialSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__RoverDifferentialSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
