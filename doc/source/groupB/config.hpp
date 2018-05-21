//******************************************************************
// file:       platoon/config.hpp
// purpose:    ros-node for the car platoon communication protocol
// license:    ?
// maintainer: Bruno Lemke - lemkebru@informatik.hu-berlin.de
// created:    2017/11/23
//******************************************************************

#ifndef ROS1_PLATOON_CONFIG_HPP
#define ROS1_PLATOON_CONFIG_HPP

#include <mutex>
#include <stdexcept>
#include <string>

//******************************************************************
// Helper classes
//******************************************************************

namespace platoon {
//------ PlatoonException ------
struct PlatoonException : public std::runtime_error {
  PlatoonException(const std::string& nodelet_name, const std::string& descr)
      : std::runtime_error(nodelet_name + ": " + descr) {}
};

//------ Lock ------
template <typename T>
struct Lock : public std::lock_guard<T> {
  Lock(T& lock) : std::lock_guard<T>(lock) {}

  operator bool() const { return true; }
};

}  // end of namespace platoon

// Helper macros
#define ATOMIC(lock) \
  if (::platoon::Lock<std::mutex> __platoon_lock_guard__{lock})

// Logging/Error
#define DBG(...) NODELET_DEBUG(__VA_ARGS__)
#define INFO(...) NODELET_INFO(__VA_ARGS__)
#define WARN(...) NODELET_WARN(__VA_ARGS__)
#define ERROR(...) NODELET_ERROR(__VA_ARGS__)
#define FATAL(...)                       \
  do {                                   \
    NODELET_FATAL(__VA_ARGS__);          \
    throw PlatoonException(__VA_ARGS__); \
  } while (false)

// Logging/Error once
#define DBG1(...) NODELET_DEBUG_ONCE(__VA_ARGS__)
#define INFO1(...) NODELET_INFO_ONCE(__VA_ARGS__)
#define WARN1(...) NODELET_WARN_ONCE(__VA_ARGS__)

// Logging/Error named
#define DBG_OBJ(...) ROS_DEBUG_NAMED(__VA_ARGS__)
#define INFO_OBJ(...) ROS_INFO_NAMED(__VA_ARGS__)
#define WARN_OBJ(...) ROS_WARN_NAMED(__VA_ARGS__)
#define ERROR_OBJ(...) ROS_ERROR_NAMED(__VA_ARGS__)
#define FATAL_OBJ(...)                   \
  do {                                   \
    ROS_FATAL_NAMED(__VA_ARGS__);        \
    throw PlatoonException(__VA_ARGS__); \
  } while (false)

// Logging/Error named once
#define DBG_OBJ1(...) ROS_DEBUG_ONCE_NAMED(__VA_ARGS__)
#define INFO_OBJ1(...) ROS_INFO_ONCE_NAMED(__VA_ARGS__)
#define WARN_OBJ1(...) ROS_WARN_ONCE_NAMED(__VA_ARGS__)

#endif  // ROS1_PLATOON_CONFIG_HPP
