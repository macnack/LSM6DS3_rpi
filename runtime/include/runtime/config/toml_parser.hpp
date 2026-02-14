#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace runtime {

struct TomlValue {
  using Array = std::vector<TomlValue>;
  using Variant = std::variant<bool, int64_t, double, std::string, Array>;

  Variant value;

  [[nodiscard]] bool is_bool() const;
  [[nodiscard]] bool is_int() const;
  [[nodiscard]] bool is_double() const;
  [[nodiscard]] bool is_string() const;
  [[nodiscard]] bool is_array() const;

  [[nodiscard]] bool as_bool() const;
  [[nodiscard]] int64_t as_int() const;
  [[nodiscard]] double as_double() const;
  [[nodiscard]] const std::string& as_string() const;
  [[nodiscard]] const Array& as_array() const;
};

using TomlSection = std::unordered_map<std::string, TomlValue>;
using TomlDocument = std::unordered_map<std::string, TomlSection>;

TomlDocument parse_toml_file(const std::string& path);

}  // namespace runtime
