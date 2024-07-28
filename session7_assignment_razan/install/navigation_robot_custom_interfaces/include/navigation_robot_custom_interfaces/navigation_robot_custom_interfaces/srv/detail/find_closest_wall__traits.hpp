// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from navigation_robot_custom_interfaces:srv/FindClosestWall.idl
// generated code does not contain a copyright notice

#ifndef NAVIGATION_ROBOT_CUSTOM_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__TRAITS_HPP_
#define NAVIGATION_ROBOT_CUSTOM_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "navigation_robot_custom_interfaces/srv/detail/find_closest_wall__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace navigation_robot_custom_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const FindClosestWall_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FindClosestWall_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FindClosestWall_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace navigation_robot_custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use navigation_robot_custom_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const navigation_robot_custom_interfaces::srv::FindClosestWall_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  navigation_robot_custom_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use navigation_robot_custom_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const navigation_robot_custom_interfaces::srv::FindClosestWall_Request & msg)
{
  return navigation_robot_custom_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<navigation_robot_custom_interfaces::srv::FindClosestWall_Request>()
{
  return "navigation_robot_custom_interfaces::srv::FindClosestWall_Request";
}

template<>
inline const char * name<navigation_robot_custom_interfaces::srv::FindClosestWall_Request>()
{
  return "navigation_robot_custom_interfaces/srv/FindClosestWall_Request";
}

template<>
struct has_fixed_size<navigation_robot_custom_interfaces::srv::FindClosestWall_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<navigation_robot_custom_interfaces::srv::FindClosestWall_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<navigation_robot_custom_interfaces::srv::FindClosestWall_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace navigation_robot_custom_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const FindClosestWall_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FindClosestWall_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FindClosestWall_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace navigation_robot_custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use navigation_robot_custom_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const navigation_robot_custom_interfaces::srv::FindClosestWall_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  navigation_robot_custom_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use navigation_robot_custom_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const navigation_robot_custom_interfaces::srv::FindClosestWall_Response & msg)
{
  return navigation_robot_custom_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<navigation_robot_custom_interfaces::srv::FindClosestWall_Response>()
{
  return "navigation_robot_custom_interfaces::srv::FindClosestWall_Response";
}

template<>
inline const char * name<navigation_robot_custom_interfaces::srv::FindClosestWall_Response>()
{
  return "navigation_robot_custom_interfaces/srv/FindClosestWall_Response";
}

template<>
struct has_fixed_size<navigation_robot_custom_interfaces::srv::FindClosestWall_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<navigation_robot_custom_interfaces::srv::FindClosestWall_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<navigation_robot_custom_interfaces::srv::FindClosestWall_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<navigation_robot_custom_interfaces::srv::FindClosestWall>()
{
  return "navigation_robot_custom_interfaces::srv::FindClosestWall";
}

template<>
inline const char * name<navigation_robot_custom_interfaces::srv::FindClosestWall>()
{
  return "navigation_robot_custom_interfaces/srv/FindClosestWall";
}

template<>
struct has_fixed_size<navigation_robot_custom_interfaces::srv::FindClosestWall>
  : std::integral_constant<
    bool,
    has_fixed_size<navigation_robot_custom_interfaces::srv::FindClosestWall_Request>::value &&
    has_fixed_size<navigation_robot_custom_interfaces::srv::FindClosestWall_Response>::value
  >
{
};

template<>
struct has_bounded_size<navigation_robot_custom_interfaces::srv::FindClosestWall>
  : std::integral_constant<
    bool,
    has_bounded_size<navigation_robot_custom_interfaces::srv::FindClosestWall_Request>::value &&
    has_bounded_size<navigation_robot_custom_interfaces::srv::FindClosestWall_Response>::value
  >
{
};

template<>
struct is_service<navigation_robot_custom_interfaces::srv::FindClosestWall>
  : std::true_type
{
};

template<>
struct is_service_request<navigation_robot_custom_interfaces::srv::FindClosestWall_Request>
  : std::true_type
{
};

template<>
struct is_service_response<navigation_robot_custom_interfaces::srv::FindClosestWall_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // NAVIGATION_ROBOT_CUSTOM_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__TRAITS_HPP_
