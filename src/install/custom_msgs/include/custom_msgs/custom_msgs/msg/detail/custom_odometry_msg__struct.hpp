// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/CustomOdometryMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_msgs/msg/custom_odometry_msg.hpp"


#ifndef CUSTOM_MSGS__MSG__DETAIL__CUSTOM_ODOMETRY_MSG__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__CUSTOM_ODOMETRY_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__CustomOdometryMsg __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__CustomOdometryMsg __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CustomOdometryMsg_
{
  using Type = CustomOdometryMsg_<ContainerAllocator>;

  explicit CustomOdometryMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0.0;
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->pitch = 0.0;
      this->roll = 0.0;
      this->yaw = 0.0;
    }
  }

  explicit CustomOdometryMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0.0;
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->pitch = 0.0;
      this->roll = 0.0;
      this->yaw = 0.0;
    }
  }

  // field types and members
  using _timestamp_type =
    double;
  _timestamp_type timestamp;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _altitude_type =
    double;
  _altitude_type altitude;
  using _pitch_type =
    double;
  _pitch_type pitch;
  using _roll_type =
    double;
  _roll_type roll;
  using _yaw_type =
    double;
  _yaw_type yaw;

  // setters for named parameter idiom
  Type & set__timestamp(
    const double & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const double & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__pitch(
    const double & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__roll(
    const double & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::CustomOdometryMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::CustomOdometryMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::CustomOdometryMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::CustomOdometryMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::CustomOdometryMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::CustomOdometryMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::CustomOdometryMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::CustomOdometryMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::CustomOdometryMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::CustomOdometryMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__CustomOdometryMsg
    std::shared_ptr<custom_msgs::msg::CustomOdometryMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__CustomOdometryMsg
    std::shared_ptr<custom_msgs::msg::CustomOdometryMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CustomOdometryMsg_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const CustomOdometryMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CustomOdometryMsg_

// alias to use template instance with default allocator
using CustomOdometryMsg =
  custom_msgs::msg::CustomOdometryMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__CUSTOM_ODOMETRY_MSG__STRUCT_HPP_
