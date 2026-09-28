#pragma once

#include <iomanip>
#include <sstream>
#include <string>

namespace planner_log {

enum class Channel {
  kMission,
  kLocal,
  kGlobal,
  kMap,
  kSafety,
  kDebug,
};

inline const char* color(Channel channel) {
  switch (channel) {
    case Channel::kMission:
      return "\033[1;36m";
    case Channel::kLocal:
      return "\033[1;32m";
    case Channel::kGlobal:
      return "\033[1;94m";
    case Channel::kMap:
      return "\033[1;34m";
    case Channel::kSafety:
      return "\033[1;35m";
    case Channel::kDebug:
    default:
      return "\033[0;37m";
  }
}

inline std::string paint(Channel channel, const std::string& text) {
  return std::string(color(channel)) + text + "\033[0m";
}

inline std::string tag(Channel channel, const std::string& name,
                       const std::string& text) {
  return paint(channel, "[" + name + "] ") + text;
}

inline std::string stage(Channel channel, const std::string& name,
                         const std::string& text) {
  return paint(channel, "[" + name + "] ") + text;
}

inline std::string boolText(bool value) {
  return value ? "enabled" : "disabled";
}

inline std::string num(double value, int precision = 3) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(precision) << value;
  return ss.str();
}

}  // namespace planner_log
