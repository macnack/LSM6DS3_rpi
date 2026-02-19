#include "ms4525do/ms4525do.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>
#include <vector>

namespace ms4525do {

namespace {

constexpr uint16_t kPressureMask = 0x3FFF;
constexpr uint16_t kTemperatureMask = 0x07FF;
constexpr uint16_t kOutputAMinCounts = 1638;   // 10% of 16383
constexpr uint16_t kOutputAMaxCounts = 14746;  // 90% of 16383
constexpr uint16_t kOutputBMinCounts = 819;    // 5% of 16383
constexpr uint16_t kOutputBMaxCounts = 15563;  // 95% of 16383

}  // namespace

Ms4525do::Ms4525do(std::string bus_path, uint8_t address, unsigned int retries, Calibration calibration,
                   unsigned int max_stale_retries, unsigned int stale_retry_delay_ms)
    : i2c_(std::make_unique<LinuxI2c>(std::move(bus_path), address, retries)),
      initialized_(false),
      calibration_(calibration),
      max_stale_retries_(max_stale_retries),
      stale_retry_delay_ms_(stale_retry_delay_ms) {
  validate_calibration(calibration_);
}

Ms4525do::Ms4525do(std::unique_ptr<I2cDevice> i2c_device, Calibration calibration,
                   unsigned int max_stale_retries, unsigned int stale_retry_delay_ms)
    : i2c_(std::move(i2c_device)),
      initialized_(false),
      calibration_(calibration),
      max_stale_retries_(max_stale_retries),
      stale_retry_delay_ms_(stale_retry_delay_ms) {
  if (i2c_ == nullptr) {
    throw std::invalid_argument("i2c_device must not be null");
  }
  validate_calibration(calibration_);
}

Ms4525do::~Ms4525do() { close(); }

void Ms4525do::begin() {
  std::scoped_lock lock(mutex_);
  i2c_->open();
  try {
    // Validate that device is healthy and producing non-stale samples.
    const RawFrame raw = read_raw_with_policy_locked(ReadPolicy::kRequireFresh);
    validate_status_or_throw(raw.status);
    initialized_ = true;
  } catch (...) {
    i2c_->close();
    initialized_ = false;
    throw;
  }
}

void Ms4525do::close() noexcept {
  std::scoped_lock lock(mutex_);
  initialized_ = false;
  i2c_->close();
}

void Ms4525do::set_calibration(const Calibration& calibration) {
  std::scoped_lock lock(mutex_);
  validate_calibration(calibration);
  calibration_ = calibration;
}

Ms4525do::Calibration Ms4525do::calibration() const {
  std::scoped_lock lock(mutex_);
  return calibration_;
}

Ms4525do::Reading Ms4525do::read(ReadPolicy policy) {
  std::scoped_lock lock(mutex_);
  ensure_ready_locked();

  const RawFrame raw = read_raw_with_policy_locked(policy);
  validate_status_or_throw(raw.status);

  return convert_to_reading_locked(raw);
}

double Ms4525do::read_pressure_psi() { return read().pressure_psi; }

double Ms4525do::read_pressure_pa() { return read().pressure_pa; }

double Ms4525do::read_temperature_c() { return read().temperature_c; }

void Ms4525do::ensure_ready_locked() const {
  if (!initialized_ || !i2c_->is_open()) {
    throw std::runtime_error("MS4525DO not initialized; call begin() first");
  }
}

void Ms4525do::validate_calibration(const Calibration& calibration) {
  if (calibration.p_max_psi <= calibration.p_min_psi) {
    throw std::invalid_argument("p_max_psi must be greater than p_min_psi");
  }
}

Ms4525do::RawFrame Ms4525do::decode_frame(const std::vector<uint8_t>& frame) {
  if (frame.size() != kFrameSize) {
    throw std::runtime_error("MS4525DO frame must contain exactly 4 bytes");
  }

  const auto status = static_cast<Status>((frame[0] >> 6U) & 0x03U);
  const uint16_t pressure_counts =
      static_cast<uint16_t>(((static_cast<uint16_t>(frame[0]) << 8U) | frame[1]) & kPressureMask);
  const uint16_t temperature_counts = static_cast<uint16_t>(
      ((((static_cast<uint16_t>(frame[2]) << 8U) | frame[3]) & 0xFFE0U) >> 5U) & kTemperatureMask);

  RawFrame raw{};
  raw.status = status;
  raw.pressure_counts = pressure_counts;
  raw.temperature_counts = temperature_counts;
  return raw;
}

Ms4525do::RawFrame Ms4525do::read_raw_with_policy_locked(ReadPolicy policy) const {
  for (unsigned int attempt = 0; attempt <= max_stale_retries_; ++attempt) {
    const RawFrame raw = decode_frame(i2c_->read_bytes(kFrameSize));
    if (policy == ReadPolicy::kAllowStale || raw.status != Status::kStaleData) {
      return raw;
    }
    if (attempt < max_stale_retries_ && stale_retry_delay_ms_ > 0U) {
      std::this_thread::sleep_for(std::chrono::milliseconds(stale_retry_delay_ms_));
    }
  }

  throw std::runtime_error("MS4525DO stale-data timeout while waiting for fresh sample");
}

void Ms4525do::validate_status_or_throw(Status status) {
  if (status == Status::kDiagnosticFault) {
    throw std::runtime_error("MS4525DO reported diagnostic fault status");
  }
  if (status == Status::kCommandMode) {
    throw std::runtime_error("MS4525DO is in command mode; not in normal data mode");
  }
}

Ms4525do::Reading Ms4525do::convert_to_reading_locked(const RawFrame& raw) const {
  const double counts_min = static_cast<double>(output_min_counts_locked());
  const double counts_max = static_cast<double>(output_max_counts_locked());
  const double pressure_span_psi = calibration_.p_max_psi - calibration_.p_min_psi;

  const double pressure_psi =
      ((static_cast<double>(raw.pressure_counts) - counts_min) * pressure_span_psi /
       (counts_max - counts_min)) +
      calibration_.p_min_psi;

  const double temperature_c = (200.0 * static_cast<double>(raw.temperature_counts) / 2047.0) - 50.0;

  Reading reading{};
  reading.pressure_psi = pressure_psi;
  reading.pressure_pa = pressure_psi * kPsiToPa;
  reading.temperature_c = temperature_c;
  reading.pressure_counts = raw.pressure_counts;
  reading.temperature_counts = raw.temperature_counts;
  reading.status = raw.status;
  return reading;
}

uint16_t Ms4525do::output_min_counts_locked() const {
  return calibration_.output_type == OutputType::kTypeA_10To90 ? kOutputAMinCounts : kOutputBMinCounts;
}

uint16_t Ms4525do::output_max_counts_locked() const {
  return calibration_.output_type == OutputType::kTypeA_10To90 ? kOutputAMaxCounts : kOutputBMaxCounts;
}

}  // namespace ms4525do
