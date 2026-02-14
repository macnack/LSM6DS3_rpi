#include "runtime/config/toml_parser.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace runtime {

namespace {

std::string trim(std::string s) {
  auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  return s;
}

std::string strip_comment(const std::string& line) {
  bool in_string = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    if (line[i] == '"') {
      in_string = !in_string;
      continue;
    }
    if (!in_string && line[i] == '#') {
      return line.substr(0, i);
    }
  }
  return line;
}

bool starts_with(const std::string& s, const std::string& prefix) {
  return s.size() >= prefix.size() && std::equal(prefix.begin(), prefix.end(), s.begin());
}

TomlValue parse_value(const std::string& raw, int line_no);

TomlValue parse_string(const std::string& raw, int line_no) {
  if (raw.size() < 2 || raw.front() != '"' || raw.back() != '"') {
    throw std::runtime_error("TOML parse error line " + std::to_string(line_no) +
                             ": invalid string literal");
  }

  std::string out;
  out.reserve(raw.size() - 2);
  for (std::size_t i = 1; i + 1 < raw.size(); ++i) {
    const char ch = raw[i];
    if (ch == '\\') {
      if (i + 1 >= raw.size() - 1) {
        throw std::runtime_error("TOML parse error line " + std::to_string(line_no) +
                                 ": trailing escape");
      }
      const char next = raw[++i];
      switch (next) {
        case '"':
          out.push_back('"');
          break;
        case '\\':
          out.push_back('\\');
          break;
        case 'n':
          out.push_back('\n');
          break;
        case 't':
          out.push_back('\t');
          break;
        default:
          throw std::runtime_error("TOML parse error line " + std::to_string(line_no) +
                                   ": unsupported escape sequence");
      }
      continue;
    }
    out.push_back(ch);
  }
  return TomlValue{out};
}

std::vector<std::string> split_array_items(const std::string& body, int line_no) {
  std::vector<std::string> items;
  std::string current;
  int bracket_depth = 0;
  bool in_string = false;

  for (char ch : body) {
    if (ch == '"') {
      in_string = !in_string;
      current.push_back(ch);
      continue;
    }

    if (!in_string) {
      if (ch == '[') {
        ++bracket_depth;
      } else if (ch == ']') {
        --bracket_depth;
        if (bracket_depth < 0) {
          throw std::runtime_error("TOML parse error line " + std::to_string(line_no) +
                                   ": invalid array nesting");
        }
      } else if (ch == ',' && bracket_depth == 0) {
        items.push_back(trim(current));
        current.clear();
        continue;
      }
    }

    current.push_back(ch);
  }

  if (in_string || bracket_depth != 0) {
    throw std::runtime_error("TOML parse error line " + std::to_string(line_no) +
                             ": malformed array");
  }

  current = trim(current);
  if (!current.empty()) {
    items.push_back(current);
  }

  return items;
}

TomlValue parse_array(const std::string& raw, int line_no) {
  if (raw.size() < 2 || raw.front() != '[' || raw.back() != ']') {
    throw std::runtime_error("TOML parse error line " + std::to_string(line_no) +
                             ": invalid array literal");
  }

  const std::string body = trim(raw.substr(1, raw.size() - 2));
  TomlValue::Array arr;
  if (body.empty()) {
    return TomlValue{arr};
  }

  const auto items = split_array_items(body, line_no);
  arr.reserve(items.size());
  for (const auto& item : items) {
    arr.push_back(parse_value(item, line_no));
  }

  return TomlValue{arr};
}

TomlValue parse_integer(const std::string& raw, int line_no) {
  if (starts_with(raw, "0x") || starts_with(raw, "0X")) {
    int64_t value = 0;
    std::stringstream ss;
    ss << std::hex << raw.substr(2);
    ss >> value;
    if (ss.fail()) {
      throw std::runtime_error("TOML parse error line " + std::to_string(line_no) +
                               ": invalid hex integer");
    }
    return TomlValue{value};
  }

  int64_t value = 0;
  const auto* begin = raw.data();
  const auto* end = raw.data() + raw.size();
  const auto result = std::from_chars(begin, end, value);
  if (result.ec != std::errc() || result.ptr != end) {
    throw std::runtime_error("TOML parse error line " + std::to_string(line_no) +
                             ": invalid integer literal");
  }
  return TomlValue{value};
}

TomlValue parse_value(const std::string& raw, int line_no) {
  const std::string value = trim(raw);
  if (value.empty()) {
    throw std::runtime_error("TOML parse error line " + std::to_string(line_no) +
                             ": empty value");
  }

  if (value.front() == '"') {
    return parse_string(value, line_no);
  }
  if (value.front() == '[') {
    return parse_array(value, line_no);
  }
  if (value == "true") {
    return TomlValue{true};
  }
  if (value == "false") {
    return TomlValue{false};
  }

  const bool maybe_float = (value.find('.') != std::string::npos || value.find('e') != std::string::npos ||
                            value.find('E') != std::string::npos);
  if (maybe_float) {
    std::size_t consumed = 0;
    const double parsed = std::stod(value, &consumed);
    if (consumed != value.size()) {
      throw std::runtime_error("TOML parse error line " + std::to_string(line_no) +
                               ": invalid floating-point literal");
    }
    return TomlValue{parsed};
  }

  return parse_integer(value, line_no);
}

}  // namespace

bool TomlValue::is_bool() const { return std::holds_alternative<bool>(value); }
bool TomlValue::is_int() const { return std::holds_alternative<int64_t>(value); }
bool TomlValue::is_double() const { return std::holds_alternative<double>(value); }
bool TomlValue::is_string() const { return std::holds_alternative<std::string>(value); }
bool TomlValue::is_array() const { return std::holds_alternative<Array>(value); }

bool TomlValue::as_bool() const { return std::get<bool>(value); }
int64_t TomlValue::as_int() const { return std::get<int64_t>(value); }
double TomlValue::as_double() const {
  if (is_int()) {
    return static_cast<double>(as_int());
  }
  return std::get<double>(value);
}
const std::string& TomlValue::as_string() const { return std::get<std::string>(value); }
const TomlValue::Array& TomlValue::as_array() const { return std::get<Array>(value); }

TomlDocument parse_toml_file(const std::string& path) {
  std::ifstream in(path);
  if (!in.is_open()) {
    throw std::runtime_error("Failed to open TOML file: " + path);
  }

  TomlDocument doc;
  std::string current_section;

  std::string raw_line;
  int line_no = 0;
  while (std::getline(in, raw_line)) {
    ++line_no;
    const std::string no_comment = trim(strip_comment(raw_line));
    if (no_comment.empty()) {
      continue;
    }

    if (no_comment.front() == '[') {
      if (no_comment.back() != ']') {
        throw std::runtime_error("TOML parse error line " + std::to_string(line_no) +
                                 ": malformed section header");
      }
      current_section = trim(no_comment.substr(1, no_comment.size() - 2));
      if (current_section.empty()) {
        throw std::runtime_error("TOML parse error line " + std::to_string(line_no) +
                                 ": empty section name");
      }
      if (doc.find(current_section) == doc.end()) {
        doc.emplace(current_section, TomlSection{});
      }
      continue;
    }

    if (current_section.empty()) {
      throw std::runtime_error("TOML parse error line " + std::to_string(line_no) +
                               ": key-value outside section");
    }

    const std::size_t eq_pos = no_comment.find('=');
    if (eq_pos == std::string::npos) {
      throw std::runtime_error("TOML parse error line " + std::to_string(line_no) +
                               ": missing '='");
    }

    const std::string key = trim(no_comment.substr(0, eq_pos));
    const std::string value_raw = trim(no_comment.substr(eq_pos + 1));
    if (key.empty()) {
      throw std::runtime_error("TOML parse error line " + std::to_string(line_no) +
                               ": empty key");
    }

    auto& section = doc[current_section];
    if (section.find(key) != section.end()) {
      throw std::runtime_error("TOML parse error line " + std::to_string(line_no) +
                               ": duplicate key '" + key + "' in section [" + current_section + "]");
    }

    section.emplace(key, parse_value(value_raw, line_no));
  }

  return doc;
}

}  // namespace runtime
