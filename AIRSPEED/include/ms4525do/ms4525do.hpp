#pragma once

#include "ms4525do/linux_i2c.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace ms4525do {

class Ms4525do {
 public:
  enum class ReadPolicy : uint8_t {
    kRequireFresh = 0,
    kAllowStale = 1,
  };

  enum class OutputType : uint8_t {
    kTypeA_10To90 = 0,
    kTypeB_5To95 = 1,
  };

  enum class Status : uint8_t {
    kNormal = 0,
    kCommandMode = 1,
    kStaleData = 2,
    kDiagnosticFault = 3,
  };

  struct Calibration {
    double p_min_psi = -1.0;
    double p_max_psi = 1.0;
    OutputType output_type = OutputType::kTypeB_5To95;
  };

  struct Reading {
    double pressure_psi;
    double pressure_pa;
    double temperature_c;
    uint16_t pressure_counts;
    uint16_t temperature_counts;
    Status status;
  };

  explicit Ms4525do(std::string bus_path = "/dev/i2c-1", uint8_t address = 0x28,
                    unsigned int retries = 2, Calibration calibration = {},
                    unsigned int max_stale_retries = 3, unsigned int stale_retry_delay_ms = 2);
  explicit Ms4525do(std::unique_ptr<I2cDevice> i2c_device, Calibration calibration = {},
                    unsigned int max_stale_retries = 3, unsigned int stale_retry_delay_ms = 2);
  ~Ms4525do();

  Ms4525do(const Ms4525do&) = delete;
  Ms4525do& operator=(const Ms4525do&) = delete;

  void begin();
  void close() noexcept;

  void set_calibration(const Calibration& calibration);
  [[nodiscard]] Calibration calibration() const;

  [[nodiscard]] Reading read(ReadPolicy policy = ReadPolicy::kRequireFresh);
  [[nodiscard]] double read_pressure_psi();
  [[nodiscard]] double read_pressure_pa();
  [[nodiscard]] double read_temperature_c();

 private:
  struct RawFrame {
    Status status;
    uint16_t pressure_counts;
    uint16_t temperature_counts;
  };

  static constexpr uint8_t kFrameSize = 4;
  static constexpr double kPsiToPa = 6894.757;

  void ensure_ready_locked() const;
  static void validate_calibration(const Calibration& calibration);
  static RawFrame decode_frame(const std::vector<uint8_t>& frame);
  [[nodiscard]] RawFrame read_raw_with_policy_locked(ReadPolicy policy) const;
  static void validate_status_or_throw(Status status);
  [[nodiscard]] Reading convert_to_reading_locked(const RawFrame& raw) const;
  [[nodiscard]] uint16_t output_min_counts_locked() const;
  [[nodiscard]] uint16_t output_max_counts_locked() const;

  std::unique_ptr<I2cDevice> i2c_;
  bool initialized_;
  Calibration calibration_;
  unsigned int max_stale_retries_;
  unsigned int stale_retry_delay_ms_;
  mutable std::mutex mutex_;
};

}  // namespace ms4525do
