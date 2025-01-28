// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/RoverMecanumSetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/rover_mecanum_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__RoverMecanumSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xea, 0x32, 0xb1, 0xf1, 0xff, 0xd4, 0x41, 0x23,
      0x69, 0x0c, 0x4a, 0xb6, 0x03, 0x5b, 0x95, 0x24,
      0x65, 0x51, 0x18, 0x23, 0xd2, 0x9f, 0xc5, 0xa0,
      0x0b, 0x2b, 0x00, 0xf5, 0x09, 0x54, 0x67, 0x8b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__RoverMecanumSetpoint__TYPE_NAME[] = "px4_msgs/msg/RoverMecanumSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__RoverMecanumSetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__RoverMecanumSetpoint__FIELD_NAME__forward_speed_setpoint[] = "forward_speed_setpoint";
static char px4_msgs__msg__RoverMecanumSetpoint__FIELD_NAME__forward_speed_setpoint_normalized[] = "forward_speed_setpoint_normalized";
static char px4_msgs__msg__RoverMecanumSetpoint__FIELD_NAME__lateral_speed_setpoint[] = "lateral_speed_setpoint";
static char px4_msgs__msg__RoverMecanumSetpoint__FIELD_NAME__lateral_speed_setpoint_normalized[] = "lateral_speed_setpoint_normalized";
static char px4_msgs__msg__RoverMecanumSetpoint__FIELD_NAME__yaw_rate_setpoint[] = "yaw_rate_setpoint";
static char px4_msgs__msg__RoverMecanumSetpoint__FIELD_NAME__speed_diff_setpoint_normalized[] = "speed_diff_setpoint_normalized";
static char px4_msgs__msg__RoverMecanumSetpoint__FIELD_NAME__yaw_setpoint[] = "yaw_setpoint";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__RoverMecanumSetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__RoverMecanumSetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverMecanumSetpoint__FIELD_NAME__forward_speed_setpoint, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverMecanumSetpoint__FIELD_NAME__forward_speed_setpoint_normalized, 33, 33},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverMecanumSetpoint__FIELD_NAME__lateral_speed_setpoint, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverMecanumSetpoint__FIELD_NAME__lateral_speed_setpoint_normalized, 33, 33},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverMecanumSetpoint__FIELD_NAME__yaw_rate_setpoint, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverMecanumSetpoint__FIELD_NAME__speed_diff_setpoint_normalized, 30, 30},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverMecanumSetpoint__FIELD_NAME__yaw_setpoint, 12, 12},
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
px4_msgs__msg__RoverMecanumSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__RoverMecanumSetpoint__TYPE_NAME, 33, 33},
      {px4_msgs__msg__RoverMecanumSetpoint__FIELDS, 8, 8},
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
  "float32 forward_speed_setpoint            # [m/s] Desired forward speed\n"
  "float32 forward_speed_setpoint_normalized # [-1, 1] Desired normalized forward speed\n"
  "float32 lateral_speed_setpoint            # [m/s] Desired lateral speed\n"
  "float32 lateral_speed_setpoint_normalized # [-1, 1] Desired normalized lateral speed\n"
  "float32 yaw_rate_setpoint                 # [rad/s] Desired yaw rate\n"
  "float32 speed_diff_setpoint_normalized    # [-1, 1] Normalized speed difference between the left and right wheels\n"
  "float32 yaw_setpoint \\t                  # [rad] Desired yaw (heading)\n"
  "\n"
  "# TOPICS rover_mecanum_setpoint";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__RoverMecanumSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__RoverMecanumSetpoint__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 659, 659},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__RoverMecanumSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__RoverMecanumSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
