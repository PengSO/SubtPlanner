#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace subt_terminal {

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

inline void printStartupAnimation() {
  const std::vector<std::string> aircraft = {
      "                    .                     .",
      "                 .-'-'.               .-'-'.",
      "             o--'  O  '--o       o--'  O  '--o",
      "                  \\       \\     /       /",
      "                   \\       .---.       /",
      "                    \\_____/ ___ \\_____/",
      "                    /     \\_____/     \\",
      "                   /       '---'       \\",
      "                  /       /     \\       \\",
      "             o--.  O  .--o       o--.  O  .--o",
      "                 '-.-'               '-.-'",
      "                    '       SUBT UAV    '",
  };

  std::cout << paint(Channel::kMission,
                     "\nSubtPlanner autonomous subterranean exploration\n\n");
  for (const auto& line : aircraft) {
    std::cout << paint(Channel::kLocal, line) << '\n';
  }
  std::cout << paint(Channel::kMission,
                     "\nMission console ready\n\n");
  std::cout << tag(Channel::kMap, "Cognitive Map",
                   "voxel map and topological graph online\n");
  std::cout << tag(Channel::kLocal, "Local Exploration",
                   "structure-guided sampling online\n");
  std::cout << tag(Channel::kGlobal, "Global Repositioning",
                   "branch-aware target selection online\n\n");
}

}  // namespace subt_terminal
