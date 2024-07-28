// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from navigation_robot_custom_interfaces:srv/FindClosestWall.idl
// generated code does not contain a copyright notice

#ifndef NAVIGATION_ROBOT_CUSTOM_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__STRUCT_HPP_
#define NAVIGATION_ROBOT_CUSTOM_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__navigation_robot_custom_interfaces__srv__FindClosestWall_Request __attribute__((deprecated))
#else
# define DEPRECATED__navigation_robot_custom_interfaces__srv__FindClosestWall_Request __declspec(deprecated)
#endif

namespace navigation_robot_custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FindClosestWall_Request_
{
  using Type = FindClosestWall_Request_<ContainerAllocator>;

  explicit FindClosestWall_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit FindClosestWall_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    navigation_robot_custom_interfaces::srv::FindClosestWall_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const navigation_robot_custom_interfaces::srv::FindClosestWall_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      navigation_robot_custom_interfaces::srv::FindClosestWall_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      navigation_robot_custom_interfaces::srv::FindClosestWall_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__navigation_robot_custom_interfaces__srv__FindClosestWall_Request
    std::shared_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__navigation_robot_custom_interfaces__srv__FindClosestWall_Request
    std::shared_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FindClosestWall_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const FindClosestWall_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FindClosestWall_Request_

// alias to use template instance with default allocator
using FindClosestWall_Request =
  navigation_robot_custom_interfaces::srv::FindClosestWall_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace navigation_robot_custom_interfaces


#ifndef _WIN32
# define DEPRECATED__navigation_robot_custom_interfaces__srv__FindClosestWall_Response __attribute__((deprecated))
#else
# define DEPRECATED__navigation_robot_custom_interfaces__srv__FindClosestWall_Response __declspec(deprecated)
#endif

namespace navigation_robot_custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FindClosestWall_Response_
{
  using Type = FindClosestWall_Response_<ContainerAllocator>;

  explicit FindClosestWall_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
    }
  }

  explicit FindClosestWall_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
    }
  }

  // field types and members
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;

  // setters for named parameter idiom
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    navigation_robot_custom_interfaces::srv::FindClosestWall_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const navigation_robot_custom_interfaces::srv::FindClosestWall_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      navigation_robot_custom_interfaces::srv::FindClosestWall_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      navigation_robot_custom_interfaces::srv::FindClosestWall_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__navigation_robot_custom_interfaces__srv__FindClosestWall_Response
    std::shared_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__navigation_robot_custom_interfaces__srv__FindClosestWall_Response
    std::shared_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FindClosestWall_Response_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    return true;
  }
  bool operator!=(const FindClosestWall_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FindClosestWall_Response_

// alias to use template instance with default allocator
using FindClosestWall_Response =
  navigation_robot_custom_interfaces::srv::FindClosestWall_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace navigation_robot_custom_interfaces

namespace navigation_robot_custom_interfaces
{

namespace srv
{

struct FindClosestWall
{
  using Request = navigation_robot_custom_interfaces::srv::FindClosestWall_Request;
  using Response = navigation_robot_custom_interfaces::srv::FindClosestWall_Response;
};

}  // namespace srv

}  // namespace navigation_robot_custom_interfaces

#endif  // NAVIGATION_ROBOT_CUSTOM_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__STRUCT_HPP_
