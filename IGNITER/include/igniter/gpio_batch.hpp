#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

namespace igniter {

constexpr std::size_t kIgniterChannels = 4;

class GpioError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

class GpioBatchOut {
 public:
  virtual ~GpioBatchOut() = default;
  virtual void open(const std::string& chip_path,
                    const std::array<uint32_t, kIgniterChannels>& lines) = 0;
  virtual void close() noexcept = 0;
  virtual void set_mask(uint8_t mask, const std::array<uint8_t, kIgniterChannels>& values) = 0;
  [[nodiscard]] virtual std::array<uint8_t, kIgniterChannels> values() const = 0;
};

class GpioBatchIn {
 public:
  virtual ~GpioBatchIn() = default;
  virtual void open(const std::string& chip_path,
                    const std::array<uint32_t, kIgniterChannels>& lines) = 0;
  virtual void close() noexcept = 0;
  [[nodiscard]] virtual std::array<uint8_t, kIgniterChannels> read_values() = 0;
};

class SimGpioBatchOut final : public GpioBatchOut {
 public:
  void open(const std::string& chip_path,
            const std::array<uint32_t, kIgniterChannels>& lines) override;
  void close() noexcept override;
  void set_mask(uint8_t mask, const std::array<uint8_t, kIgniterChannels>& values) override;
  [[nodiscard]] std::array<uint8_t, kIgniterChannels> values() const override;
  [[nodiscard]] uint64_t write_count() const noexcept { return write_count_; }

 private:
  bool open_ = false;
  std::string chip_path_;
  std::array<uint32_t, kIgniterChannels> lines_{};
  std::array<uint8_t, kIgniterChannels> values_{};
  uint64_t write_count_ = 0;
};

class SimGpioBatchIn final : public GpioBatchIn {
 public:
  void open(const std::string& chip_path,
            const std::array<uint32_t, kIgniterChannels>& lines) override;
  void close() noexcept override;
  [[nodiscard]] std::array<uint8_t, kIgniterChannels> read_values() override;
  void set_values(const std::array<uint8_t, kIgniterChannels>& values);

 private:
  bool open_ = false;
  std::string chip_path_;
  std::array<uint32_t, kIgniterChannels> lines_{};
  std::array<uint8_t, kIgniterChannels> values_{1, 1, 1, 1};
};

std::unique_ptr<GpioBatchOut> make_hardware_batch_out();
std::unique_ptr<GpioBatchIn> make_hardware_batch_in();

}  // namespace igniter
