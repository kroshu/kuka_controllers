#ifndef UTILS_ENUM_CONVERTER_HPP_
#define UTILS_ENUM_CONVERTER_HPP_

#include <nanopb/kuka/motion/external/external_control_mode.pb.hh>
#include <string>


// Helper functions to painlessly create enums from various primitives, if needed

namespace utils
{

template<typename T, typename std::enable_if_t<std::is_arithmetic<T>::value, bool> = true>
static inline auto to_control_mode(T && numeric_value)
{
  return _kuka_motion_external_ExternalControlMode((int)numeric_value);
}

template<typename T, typename std::enable_if_t<std::is_same<typename std::decay<T>::type,
  std::string>::value, bool> = true>
static inline auto to_control_mode(T && string_value)
{
  return _kuka_motion_external_ExternalControlMode(std::stoi(string_value));
}

template<typename T, typename std::enable_if_t<std::is_same<typename std::decay<T>::type,
  char *>::value, bool> = true>
static inline auto to_control_mode(T && char_arr_value)
{
  int retval = std::atoi(char_arr_value);
  if (retval < 1) {
    throw std::runtime_error(
            "Error: failed to convert char array to kuka::motion::external::ExternalControlMode");
  }
  return _kuka_motion_external_ExternalControlMode(retval);
}

}

#endif // UTILS_ENUM_CONVERTER_HPP_
