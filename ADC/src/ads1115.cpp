#include "ads1115/ads1115.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <thread>

namespace ads1115 {

namespace {

constexpr uint16_t kMuxDiffAin0Ain1 = 0x0000;  // CONFIG.MUX = 000 (AIN0 - AIN1)
constexpr uint16_t kMuxDiffAin0Ain3 = 0x1000;  // CONFIG.MUX = 001 (AIN0 - AIN3)
constexpr uint16_t kMuxDiffAin1Ain3 = 0x2000;  // CONFIG.MUX = 010 (AIN1 - AIN3)
constexpr uint16_t kMuxDiffAin2Ain3 = 0x3000;  // CONFIG.MUX = 011 (AIN2 - AIN3)
constexpr uint16_t kMuxSingleAin0 = 0x4000;    // CONFIG.MUX = 100 (AIN0 vs GND)
constexpr uint16_t kMuxSingleAin1 = 0x5000;    // CONFIG.MUX = 101 (AIN1 vs GND)
constexpr uint16_t kMuxSingleAin2 = 0x6000;    // CONFIG.MUX = 110 (AIN2 vs GND)
constexpr uint16_t kMuxSingleAin3 = 0x7000;    // CONFIG.MUX = 111 (AIN3 vs GND)

constexpr uint16_t kPgaFs6_144 = 0x0000;  // CONFIG.PGA = 000 (+-6.144V full scale)
constexpr uint16_t kPgaFs4_096 = 0x0200;  // CONFIG.PGA = 001 (+-4.096V full scale)
constexpr uint16_t kPgaFs2_048 = 0x0400;  // CONFIG.PGA = 010 (+-2.048V full scale)
constexpr uint16_t kPgaFs1_024 = 0x0600;  // CONFIG.PGA = 011 (+-1.024V full scale)
constexpr uint16_t kPgaFs0_512 = 0x0800;  // CONFIG.PGA = 100 (+-0.512V full scale)
constexpr uint16_t kPgaFs0_256 = 0x0A00;  // CONFIG.PGA = 101 (+-0.256V full scale)

constexpr uint16_t kCompQueDisable = 0x0003;  // CONFIG.COMP_QUE = 11 (disable comparator)

constexpr int kReadyPollSleepMs = 1;
constexpr int kExtraConversionMarginMs = 2;
constexpr int kReadyPollTimeoutPaddingMs = 20;

uint16_t mux_bits_for_single_channel(uint8_t channel) {
  switch (channel) {
    case 0:
      return kMuxSingleAin0;
    case 1:
      return kMuxSingleAin1;
    case 2:
      return kMuxSingleAin2;
    case 3:
      return kMuxSingleAin3;
    default:
      throw std::invalid_argument("channel must be in range 0..3");
  }
}

uint16_t mux_bits_for_differential_channel(uint8_t channel) {
  switch (channel) {
    case 0:
      return kMuxDiffAin0Ain1;
    case 1:
      return kMuxDiffAin0Ain3;
    case 2:
      return kMuxDiffAin1Ain3;
    case 3:
      return kMuxDiffAin2Ain3;
    default:
      throw std::invalid_argument("differential channel must be in range 0..3");
  }
}

int sps_from_dr_bits(uint16_t dr_bits) {
  switch (dr_bits) {
    case 0x0000:
      return 8;
    case 0x0020:
      return 16;
    case 0x0040:
      return 32;
    case 0x0060:
      return 64;
    case 0x0080:
      return 128;
    case 0x00A0:
      return 250;
    case 0x00C0:
      return 475;
    case 0x00E0:
      return 860;
    default:
      throw std::runtime_error("unexpected ADS1115 data-rate bits in config");
  }
}

double full_scale_volts_from_pga_bits(uint16_t pga_bits) {
  switch (pga_bits) {
    case kPgaFs6_144:
      return 6.144;
    case kPgaFs4_096:
      return 4.096;
    case kPgaFs2_048:
      return 2.048;
    case kPgaFs1_024:
      return 1.024;
    case kPgaFs0_512:
      return 0.512;
    case kPgaFs0_256:
      return 0.256;
    default:
      throw std::runtime_error("unexpected ADS1115 PGA bits in config");
  }
}

}  // namespace

Ads1115::Ads1115(std::string bus_path, uint8_t address, unsigned int retries)
    : i2c_(std::make_unique<LinuxI2c>(std::move(bus_path), address, retries)),
      initialized_(false),
      base_config_(static_cast<uint16_t>(Mode::kSingleShot) | kPgaFs2_048 |
                   static_cast<uint16_t>(DataRate::k128Sps) | kCompQueDisable),
      current_mux_bits_(static_cast<uint16_t>(Mux::kSingleAin0)),
      comparator_config_{} {}

Ads1115::Ads1115(std::unique_ptr<I2cDevice> i2c_device)
    : i2c_(std::move(i2c_device)),
      initialized_(false),
      base_config_(static_cast<uint16_t>(Mode::kSingleShot) | kPgaFs2_048 |
                   static_cast<uint16_t>(DataRate::k128Sps) | kCompQueDisable),
      current_mux_bits_(static_cast<uint16_t>(Mux::kSingleAin0)),
      comparator_config_{} {
  if (i2c_ == nullptr) {
    throw std::invalid_argument("i2c_device must not be null");
  }
}

Ads1115::~Ads1115() { close(); }

void Ads1115::begin() {
  std::scoped_lock lock(mutex_);
  i2c_->open();

  // Communication check: read CONFIG register (0x01).
  (void)read_register16_locked(kRegConfig);

  write_register16_locked(kRegLoThresh, static_cast<uint16_t>(comparator_config_.low_threshold));
  write_register16_locked(kRegHiThresh, static_cast<uint16_t>(comparator_config_.high_threshold));

  // Program CONFIG register (0x01) with current settings.
  write_register16_locked(kRegConfig, make_config_with_mux_locked(current_mux_bits_, false));
  initialized_ = true;
}

void Ads1115::close() noexcept {
  std::scoped_lock lock(mutex_);
  initialized_ = false;
  i2c_->close();
}

void Ads1115::set_mode(Mode mode) {
  std::scoped_lock lock(mutex_);
  base_config_ = static_cast<uint16_t>((base_config_ & ~kConfigModeMask) | static_cast<uint16_t>(mode));
  if (initialized_) {
    write_register16_locked(kRegConfig, make_config_with_mux_locked(current_mux_bits_, false));
  }
}

void Ads1115::set_data_rate(DataRate rate) {
  std::scoped_lock lock(mutex_);
  base_config_ = static_cast<uint16_t>((base_config_ & ~kConfigDrMask) | static_cast<uint16_t>(rate));
  if (initialized_) {
    write_register16_locked(kRegConfig, make_config_with_mux_locked(current_mux_bits_, false));
  }
}

void Ads1115::set_gain(uint8_t gain_code) {
  std::scoped_lock lock(mutex_);
  const uint16_t pga_bits = pga_bits_from_legacy_gain_locked(gain_code);

  base_config_ = static_cast<uint16_t>((base_config_ & ~kConfigPgaMask) | pga_bits);

  if (initialized_) {
    write_register16_locked(kRegConfig, make_config_with_mux_locked(current_mux_bits_, false));
  }
}

void Ads1115::set_mux(Mux mux) {
  std::scoped_lock lock(mutex_);
  current_mux_bits_ = static_cast<uint16_t>(mux);
  if (initialized_) {
    write_register16_locked(kRegConfig, make_config_with_mux_locked(current_mux_bits_, false));
  }
}

void Ads1115::configure_comparator(const ComparatorConfig& config) {
  std::scoped_lock lock(mutex_);

  if (config.queue > 0x03) {
    throw std::invalid_argument("comparator queue must be in range 0..3");
  }
  if (config.low_threshold > config.high_threshold) {
    throw std::invalid_argument("comparator low_threshold must be <= high_threshold");
  }

  comparator_config_ = config;

  uint16_t comp_bits = static_cast<uint16_t>(config.queue & 0x03);  // CONFIG.COMP_QUE
  if (config.window_mode) {
    comp_bits = static_cast<uint16_t>(comp_bits | kConfigCompModeMask);
  }
  if (config.active_high) {
    comp_bits = static_cast<uint16_t>(comp_bits | kConfigCompPolMask);
  }
  if (config.latching) {
    comp_bits = static_cast<uint16_t>(comp_bits | kConfigCompLatMask);
  }

  base_config_ = static_cast<uint16_t>(base_config_ &
                                       ~(kConfigCompModeMask | kConfigCompPolMask |
                                         kConfigCompLatMask | kConfigCompQueMask));
  base_config_ = static_cast<uint16_t>(base_config_ | comp_bits);

  if (initialized_) {
    write_register16_locked(kRegLoThresh, static_cast<uint16_t>(comparator_config_.low_threshold));
    write_register16_locked(kRegHiThresh, static_cast<uint16_t>(comparator_config_.high_threshold));
    write_register16_locked(kRegConfig, make_config_with_mux_locked(current_mux_bits_, false));
  }
}

int16_t Ads1115::read_adc(uint8_t channel) {
  std::scoped_lock lock(mutex_);
  ensure_ready_locked();
  const uint16_t mux = mux_bits_for_single_channel(channel);
  current_mux_bits_ = mux;
  return read_adc_with_mux_locked(mux);
}

double Ads1115::read_voltage(uint8_t channel) {
  std::scoped_lock lock(mutex_);
  ensure_ready_locked();
  const uint16_t mux = mux_bits_for_single_channel(channel);
  current_mux_bits_ = mux;
  const int16_t raw = read_adc_with_mux_locked(mux);
  return raw_to_volts_locked(raw);
}

int16_t Ads1115::read_differential_adc(uint8_t channel) {
  std::scoped_lock lock(mutex_);
  ensure_ready_locked();
  const uint16_t mux = mux_bits_for_differential_channel(channel);
  current_mux_bits_ = mux;
  return read_adc_with_mux_locked(mux);
}

double Ads1115::comparator_voltage(uint8_t channel) {
  std::scoped_lock lock(mutex_);
  ensure_ready_locked();
  const uint16_t mux = mux_bits_for_differential_channel(channel);
  current_mux_bits_ = mux;
  const int16_t raw = read_adc_with_mux_locked(mux);
  return raw_to_volts_locked(raw);
}

int16_t Ads1115::read_latest() {
  std::scoped_lock lock(mutex_);
  ensure_ready_locked();
  const uint16_t raw = read_register16_locked(kRegConversion);
  return static_cast<int16_t>(raw);
}

double Ads1115::read_latest_voltage() {
  std::scoped_lock lock(mutex_);
  ensure_ready_locked();
  const uint16_t raw = read_register16_locked(kRegConversion);
  return raw_to_volts_locked(static_cast<int16_t>(raw));
}

void Ads1115::poll_continuous(const SampleCallback& on_sample, int poll_interval_ms,
                              uint32_t max_samples) {
  if (!on_sample) {
    throw std::invalid_argument("on_sample callback must be valid");
  }
  if (poll_interval_ms < 0) {
    throw std::invalid_argument("poll_interval_ms must be >= 0");
  }

  {
    std::scoped_lock lock(mutex_);
    ensure_ready_locked();
    if (is_single_shot_mode_locked()) {
      throw std::runtime_error("poll_continuous requires continuous mode");
    }
  }

  uint32_t count = 0;
  while ((max_samples == 0) || (count < max_samples)) {
    const int16_t sample = read_latest();
    if (!on_sample(sample)) {
      break;
    }
    ++count;
    if (poll_interval_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
    }
  }
}

void Ads1115::ensure_ready_locked() const {
  if (!initialized_ || !i2c_->is_open()) {
    throw std::runtime_error("ADS1115 not initialized; call begin() first");
  }
}

uint16_t Ads1115::read_register16_locked(uint8_t reg) {
  const auto bytes = i2c_->read_registers(reg, 2);
  if (bytes.size() != 2) {
    throw std::runtime_error("ADS1115 read_register16 received unexpected byte count");
  }
  return static_cast<uint16_t>((static_cast<uint16_t>(bytes[0]) << 8) | static_cast<uint16_t>(bytes[1]));
}

void Ads1115::write_register16_locked(uint8_t reg, uint16_t value) {
  std::array<uint8_t, 2> data{{static_cast<uint8_t>((value >> 8) & 0xFF),
                               static_cast<uint8_t>(value & 0xFF)}};
  i2c_->write_register(reg, data.data(), data.size());
}

uint16_t Ads1115::make_config_with_mux_locked(uint16_t mux_bits, bool trigger_single) const {
  const uint16_t config_no_mux = static_cast<uint16_t>(base_config_ & ~kConfigMuxMask);
  const uint16_t config = static_cast<uint16_t>(config_no_mux | mux_bits);
  if (trigger_single && is_single_shot_mode_locked()) {
    return static_cast<uint16_t>(config | kConfigOsMask);
  }
  return config;
}

bool Ads1115::is_single_shot_mode_locked() const {
  return (base_config_ & kConfigModeMask) != 0;
}

int16_t Ads1115::read_adc_with_mux_locked(uint16_t mux_bits) {
  const uint16_t config = make_config_with_mux_locked(mux_bits, true);
  write_register16_locked(kRegConfig, config);

  const int conversion_ms = conversion_time_ms_for_data_rate_locked();
  std::this_thread::sleep_for(std::chrono::milliseconds(conversion_ms));

  if (!is_single_shot_mode_locked()) {
    const uint16_t raw = read_register16_locked(kRegConversion);
    return static_cast<int16_t>(raw);
  }

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(conversion_ms + kReadyPollTimeoutPaddingMs);

  while (true) {
    const uint16_t cfg = read_register16_locked(kRegConfig);
    if ((cfg & kConfigOsMask) != 0) {
      const uint16_t raw = read_register16_locked(kRegConversion);
      return static_cast<int16_t>(raw);
    }

    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("ADS1115 conversion timeout while waiting for OS=1 in CONFIG register");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(kReadyPollSleepMs));
  }
}

int Ads1115::conversion_time_ms_for_data_rate_locked() const {
  const uint16_t dr_bits = static_cast<uint16_t>(base_config_ & kConfigDrMask);
  const int sps = sps_from_dr_bits(dr_bits);

  // One ADS1115 conversion period in ms, rounded up, with a small safety margin.
  const int period_ms = (1000 + sps - 1) / sps;
  return period_ms + kExtraConversionMarginMs;
}

double Ads1115::raw_to_volts_locked(int16_t raw) const {
  const uint16_t pga_bits = static_cast<uint16_t>(base_config_ & kConfigPgaMask);
  const double full_scale = full_scale_volts_from_pga_bits(pga_bits);
  return (static_cast<double>(raw) * full_scale) / 32768.0;
}

uint16_t Ads1115::pga_bits_from_legacy_gain_locked(uint8_t gain_code) const {
  switch (gain_code) {
    case 0x00:  // ADS1115_REG_CONFIG_PGA_6_144V
      return kPgaFs6_144;
    case 0x02:  // ADS1115_REG_CONFIG_PGA_4_096V
      return kPgaFs4_096;
    case 0x04:  // ADS1115_REG_CONFIG_PGA_2_048V
      return kPgaFs2_048;
    case 0x06:  // ADS1115_REG_CONFIG_PGA_1_024V
      return kPgaFs1_024;
    case 0x08:  // ADS1115_REG_CONFIG_PGA_0_512V
      return kPgaFs0_512;
    case 0x0A:  // ADS1115_REG_CONFIG_PGA_0_256V
      return kPgaFs0_256;
    default:
      throw std::invalid_argument(
          "invalid gain code; expected one of 0x00,0x02,0x04,0x06,0x08,0x0A");
  }
}

}  // namespace ads1115
