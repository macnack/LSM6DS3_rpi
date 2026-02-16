#pragma once

#include "ads1115/linux_i2c.hpp"

#include <functional>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace ads1115 {

class Ads1115 {
 public:
  enum class Mode : uint16_t {
    kContinuous = 0x0000,
    kSingleShot = 0x0100,
  };

  enum class DataRate : uint16_t {
    k8Sps = 0x0000,
    k16Sps = 0x0020,
    k32Sps = 0x0040,
    k64Sps = 0x0060,
    k128Sps = 0x0080,
    k250Sps = 0x00A0,
    k475Sps = 0x00C0,
    k860Sps = 0x00E0,
  };

  enum class Mux : uint16_t {
    kDiffAin0Ain1 = 0x0000,
    kDiffAin0Ain3 = 0x1000,
    kDiffAin1Ain3 = 0x2000,
    kDiffAin2Ain3 = 0x3000,
    kSingleAin0 = 0x4000,
    kSingleAin1 = 0x5000,
    kSingleAin2 = 0x6000,
    kSingleAin3 = 0x7000,
  };

  using SampleCallback = std::function<bool(int16_t)>;

  struct ComparatorConfig {
    int16_t low_threshold = -32768;
    int16_t high_threshold = 32767;
    bool window_mode = false;  // COMP_MODE bit in CONFIG[4]
    bool active_high = false;  // COMP_POL bit in CONFIG[3]
    bool latching = false;     // COMP_LAT bit in CONFIG[2]
    uint8_t queue = 0x03;      // COMP_QUE bits in CONFIG[1:0], 0x03 disables comparator
  };

  explicit Ads1115(std::string bus_path = "/dev/i2c-1", uint8_t address = 0x48,
                   unsigned int retries = 2);
  explicit Ads1115(std::unique_ptr<I2cDevice> i2c_device);
  ~Ads1115();

  Ads1115(const Ads1115&) = delete;
  Ads1115& operator=(const Ads1115&) = delete;

  void begin();
  void close() noexcept;

  void set_mode(Mode mode);
  void set_data_rate(DataRate rate);
  void set_gain(uint8_t gain_code);
  void set_mux(Mux mux);
  void configure_comparator(const ComparatorConfig& config);

  [[nodiscard]] int16_t read_adc(uint8_t channel);
  [[nodiscard]] double read_voltage(uint8_t channel);
  [[nodiscard]] int16_t read_differential_adc(uint8_t channel);
  [[nodiscard]] double comparator_voltage(uint8_t channel);
  [[nodiscard]] int16_t read_latest();
  [[nodiscard]] double read_latest_voltage();
  void poll_continuous(const SampleCallback& on_sample, int poll_interval_ms = 1,
                       uint32_t max_samples = 0);

 private:
  static constexpr uint8_t kRegConversion = 0x00;  // CONVERSION register (0x00)
  static constexpr uint8_t kRegConfig = 0x01;      // CONFIG register (0x01)
  static constexpr uint8_t kRegLoThresh = 0x02;    // LO_THRESH register (0x02)
  static constexpr uint8_t kRegHiThresh = 0x03;    // HI_THRESH register (0x03)

  static constexpr uint16_t kConfigOsMask = 0x8000;    // OS bit in CONFIG[15]
  static constexpr uint16_t kConfigMuxMask = 0x7000;   // MUX bits in CONFIG[14:12]
  static constexpr uint16_t kConfigPgaMask = 0x0E00;   // PGA bits in CONFIG[11:9]
  static constexpr uint16_t kConfigModeMask = 0x0100;  // MODE bit in CONFIG[8]
  static constexpr uint16_t kConfigDrMask = 0x00E0;    // DR bits in CONFIG[7:5]
  static constexpr uint16_t kConfigCompModeMask = 0x0010;  // COMP_MODE bit in CONFIG[4]
  static constexpr uint16_t kConfigCompPolMask = 0x0008;   // COMP_POL bit in CONFIG[3]
  static constexpr uint16_t kConfigCompLatMask = 0x0004;   // COMP_LAT bit in CONFIG[2]
  static constexpr uint16_t kConfigCompQueMask = 0x0003;   // COMP_QUE bits in CONFIG[1:0]

  void ensure_ready_locked() const;
  [[nodiscard]] uint16_t read_register16_locked(uint8_t reg);
  void write_register16_locked(uint8_t reg, uint16_t value);
  [[nodiscard]] uint16_t make_config_with_mux_locked(uint16_t mux_bits, bool trigger_single) const;
  [[nodiscard]] bool is_single_shot_mode_locked() const;
  [[nodiscard]] int16_t read_adc_with_mux_locked(uint16_t mux_bits);
  [[nodiscard]] int conversion_time_ms_for_data_rate_locked() const;
  [[nodiscard]] double raw_to_volts_locked(int16_t raw) const;
  [[nodiscard]] uint16_t pga_bits_from_legacy_gain_locked(uint8_t gain_code) const;

  std::unique_ptr<I2cDevice> i2c_;
  bool initialized_;
  uint16_t base_config_;
  uint16_t current_mux_bits_;
  ComparatorConfig comparator_config_;
  mutable std::mutex mutex_;
};

}  // namespace ads1115
