// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/CustomOdometryMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_msgs/msg/custom_odometry_msg.h"


#ifndef CUSTOM_MSGS__MSG__DETAIL__CUSTOM_ODOMETRY_MSG__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__CUSTOM_ODOMETRY_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/CustomOdometryMsg in the package custom_msgs.
typedef struct custom_msgs__msg__CustomOdometryMsg
{
  double timestamp;
  double latitude;
  double longitude;
  double altitude;
  double pitch;
  double roll;
  double yaw;
} custom_msgs__msg__CustomOdometryMsg;

// Struct for a sequence of custom_msgs__msg__CustomOdometryMsg.
typedef struct custom_msgs__msg__CustomOdometryMsg__Sequence
{
  custom_msgs__msg__CustomOdometryMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__CustomOdometryMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__CUSTOM_ODOMETRY_MSG__STRUCT_H_
