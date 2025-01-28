// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/RoverDifferentialGuidanceStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/rover_differential_guidance_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__RoverDifferentialGuidanceStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x36, 0x7d, 0xb1, 0xac, 0xc5, 0xdf, 0xa5, 0x9d,
      0xd1, 0xe9, 0xcc, 0xf4, 0x47, 0x4b, 0x15, 0x4b,
      0x09, 0x0e, 0x9c, 0x9d, 0x49, 0x87, 0xec, 0x1c,
      0xd7, 0xdf, 0x68, 0xbd, 0x3b, 0x52, 0x0a, 0x74,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__RoverDifferentialGuidanceStatus__TYPE_NAME[] = "px4_msgs/msg/RoverDifferentialGuidanceStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__RoverDifferentialGuidanceStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__RoverDifferentialGuidanceStatus__FIELD_NAME__lookahead_distance[] = "lookahead_distance";
static char px4_msgs__msg__RoverDifferentialGuidanceStatus__FIELD_NAME__heading_error_deg[] = "heading_error_deg";
static char px4_msgs__msg__RoverDifferentialGuidanceStatus__FIELD_NAME__state_machine[] = "state_machine";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__RoverDifferentialGuidanceStatus__FIELDS[] = {
  {
    {px4_msgs__msg__RoverDifferentialGuidanceStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverDifferentialGuidanceStatus__FIELD_NAME__lookahead_distance, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverDifferentialGuidanceStatus__FIELD_NAME__heading_error_deg, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverDifferentialGuidanceStatus__FIELD_NAME__state_machine, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__RoverDifferentialGuidanceStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__RoverDifferentialGuidanceStatus__TYPE_NAME, 44, 44},
      {px4_msgs__msg__RoverDifferentialGuidanceStatus__FIELDS, 4, 4},
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
  "float32 lookahead_distance    # [m] Lookahead distance of pure the pursuit controller\n"
  "float32 heading_error_deg \\t      # [deg] Heading error of the pure pursuit controller\n"
  "uint8   state_machine         # Driving state of the rover [0: SPOT_TURNING, 1: DRIVING, 2: GOAL_REACHED]\n"
  "\n"
  "# TOPICS rover_differential_guidance_status";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__RoverDifferentialGuidanceStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__RoverDifferentialGuidanceStatus__TYPE_NAME, 44, 44},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 382, 382},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__RoverDifferentialGuidanceStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__RoverDifferentialGuidanceStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
