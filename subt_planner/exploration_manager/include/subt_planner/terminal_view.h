#pragma once

#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
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
      return "\033[1;33m";
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

inline void printStartupAnimation() {
  const std::vector<std::string> rows = {
      "        /--------------------------\\",
      "       /                            \\",
      "      /      .                .      \\",
      "     /                                \\",
      "    /    .      exploration      .     \\",
      "   /                                    \\",
      "  /       cognitive map online          \\",
      " /                                      \\",
      "/----------------------------------------\\"};

  std::cout << paint(Channel::kMission,
                     "\nSubtPlanner autonomous exploration startup\n");
  for (int frame = 0; frame < 9; ++frame) {
    const int drone_row = 8 - frame;
    std::cout << paint(Channel::kMap, "\n  Tunnel slice " +
                                      std::to_string(frame + 1) + "/9\n");
    for (int r = 0; r < static_cast<int>(rows.size()); ++r) {
      std::string line = rows[r];
      if (r == drone_row) {
        const std::string drone = "<^>";
        const std::size_t mid = line.size() / 2;
        if (mid + drone.size() < line.size()) {
          line.replace(mid, drone.size(), drone);
        }
        std::cout << paint(Channel::kLocal, line) << '\n';
      } else {
        std::cout << paint(Channel::kDebug, line) << '\n';
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
  }
  std::cout << paint(Channel::kMission,
                     "\nMission console ready: local exploration, global repositioning, and cognitive map logs are color-coded.\n\n");
}

}  // namespace subt_terminal
