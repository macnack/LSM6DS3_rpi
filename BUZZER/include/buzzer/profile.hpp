#pragma once

#include "buzzer/patterns.hpp"

#include <string>

namespace buzzer {

[[nodiscard]] SignalProfile make_default_profile();
[[nodiscard]] SignalProfile load_profile_json_text(const std::string& text);
[[nodiscard]] SignalProfile load_profile_json_file(const std::string& path);
[[nodiscard]] std::string profile_to_json(const SignalProfile& profile);

}  // namespace buzzer
