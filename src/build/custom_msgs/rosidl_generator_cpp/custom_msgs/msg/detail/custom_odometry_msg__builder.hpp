// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/CustomOdometryMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_msgs/msg/custom_odometry_msg.hpp"


#ifndef CUSTOM_MSGS__MSG__DETAIL__CUSTOM_ODOMETRY_MSG__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__CUSTOM_ODOMETRY_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/custom_odometry_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_CustomOdometryMsg_yaw
{
public:
  explicit Init_CustomOdometryMsg_yaw(::custom_msgs::msg::CustomOdometryMsg & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::CustomOdometryMsg yaw(::custom_msgs::msg::CustomOdometryMsg::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::CustomOdometryMsg msg_;
};

class Init_CustomOdometryMsg_roll
{
public:
  explicit Init_CustomOdometryMsg_roll(::custom_msgs::msg::CustomOdometryMsg & msg)
  : msg_(msg)
  {}
  Init_CustomOdometryMsg_yaw roll(::custom_msgs::msg::CustomOdometryMsg::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_CustomOdometryMsg_yaw(msg_);
  }

private:
  ::custom_msgs::msg::CustomOdometryMsg msg_;
};

class Init_CustomOdometryMsg_pitch
{
public:
  explicit Init_CustomOdometryMsg_pitch(::custom_msgs::msg::CustomOdometryMsg & msg)
  : msg_(msg)
  {}
  Init_CustomOdometryMsg_roll pitch(::custom_msgs::msg::CustomOdometryMsg::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_CustomOdometryMsg_roll(msg_);
  }

private:
  ::custom_msgs::msg::CustomOdometryMsg msg_;
};

class Init_CustomOdometryMsg_altitude
{
public:
  explicit Init_CustomOdometryMsg_altitude(::custom_msgs::msg::CustomOdometryMsg & msg)
  : msg_(msg)
  {}
  Init_CustomOdometryMsg_pitch altitude(::custom_msgs::msg::CustomOdometryMsg::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_CustomOdometryMsg_pitch(msg_);
  }

private:
  ::custom_msgs::msg::CustomOdometryMsg msg_;
};

class Init_CustomOdometryMsg_longitude
{
public:
  explicit Init_CustomOdometryMsg_longitude(::custom_msgs::msg::CustomOdometryMsg & msg)
  : msg_(msg)
  {}
  Init_CustomOdometryMsg_altitude longitude(::custom_msgs::msg::CustomOdometryMsg::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_CustomOdometryMsg_altitude(msg_);
  }

private:
  ::custom_msgs::msg::CustomOdometryMsg msg_;
};

class Init_CustomOdometryMsg_latitude
{
public:
  explicit Init_CustomOdometryMsg_latitude(::custom_msgs::msg::CustomOdometryMsg & msg)
  : msg_(msg)
  {}
  Init_CustomOdometryMsg_longitude latitude(::custom_msgs::msg::CustomOdometryMsg::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_CustomOdometryMsg_longitude(msg_);
  }

private:
  ::custom_msgs::msg::CustomOdometryMsg msg_;
};

class Init_CustomOdometryMsg_timestamp
{
public:
  Init_CustomOdometryMsg_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CustomOdometryMsg_latitude timestamp(::custom_msgs::msg::CustomOdometryMsg::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_CustomOdometryMsg_latitude(msg_);
  }

private:
  ::custom_msgs::msg::CustomOdometryMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::CustomOdometryMsg>()
{
  return custom_msgs::msg::builder::Init_CustomOdometryMsg_timestamp();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__CUSTOM_ODOMETRY_MSG__BUILDER_HPP_
