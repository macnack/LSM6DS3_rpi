#include "igniter/gpio_batch.hpp"

#include <cstring>
#include <utility>

#if defined(IGNITER_HAVE_LIBGPIOD) && (IGNITER_HAVE_LIBGPIOD == 1)
#include <errno.h>
#include <gpiod.h>
#endif

namespace igniter {

namespace {

#if defined(IGNITER_HAVE_LIBGPIOD) && (IGNITER_HAVE_LIBGPIOD == 1)
class LibgpiodBatchOut final : public GpioBatchOut {
 public:
  void open(const std::string& chip_path,
            const std::array<uint32_t, kIgniterChannels>& lines) override {
    close();

    chip_ = ::gpiod_chip_open(chip_path.c_str());
    if (chip_ == nullptr) {
      throw GpioError("gpiod_chip_open failed for output chip '" + chip_path + "'");
    }

    unsigned int offsets[kIgniterChannels] = {};
    for (std::size_t i = 0; i < kIgniterChannels; ++i) {
      offsets[i] = static_cast<unsigned int>(lines[i]);
    }

    if (::gpiod_chip_get_lines(chip_, offsets, kIgniterChannels, &line_bulk_) < 0) {
      const int err = errno;
      close();
      throw GpioError("gpiod_chip_get_lines(output) failed (errno=" + std::to_string(err) + ")");
    }

    int defaults[kIgniterChannels] = {0, 0, 0, 0};
    if (::gpiod_line_request_bulk_output(&line_bulk_, "igniter-batch-out", defaults) < 0) {
      const int err = errno;
      close();
      throw GpioError("gpiod_line_request_bulk_output failed (errno=" + std::to_string(err) + ")");
    }

    values_ = {0, 0, 0, 0};
    open_ = true;
  }

  void close() noexcept override {
    if (open_) {
      ::gpiod_line_release_bulk(&line_bulk_);
      open_ = false;
    }
    if (chip_ != nullptr) {
      ::gpiod_chip_close(chip_);
      chip_ = nullptr;
    }
    std::memset(&line_bulk_, 0, sizeof(line_bulk_));
  }

  void set_mask(uint8_t mask, const std::array<uint8_t, kIgniterChannels>& values) override {
    if (!open_) {
      throw GpioError("output GPIO batch not open");
    }
    for (std::size_t i = 0; i < kIgniterChannels; ++i) {
      if ((mask & (1U << i)) != 0U) {
        values_[i] = values[i] ? 1U : 0U;
      }
    }

    int raw[kIgniterChannels] = {0, 0, 0, 0};
    for (std::size_t i = 0; i < kIgniterChannels; ++i) {
      raw[i] = values_[i] ? 1 : 0;
    }
    if (::gpiod_line_set_value_bulk(&line_bulk_, raw) < 0) {
      const int err = errno;
      throw GpioError("gpiod_line_set_value_bulk failed (errno=" + std::to_string(err) + ")");
    }
  }

  [[nodiscard]] std::array<uint8_t, kIgniterChannels> values() const override { return values_; }

 private:
  bool open_ = false;
  gpiod_chip* chip_ = nullptr;
  gpiod_line_bulk line_bulk_{};
  std::array<uint8_t, kIgniterChannels> values_{0, 0, 0, 0};
};

class LibgpiodBatchIn final : public GpioBatchIn {
 public:
  void open(const std::string& chip_path,
            const std::array<uint32_t, kIgniterChannels>& lines) override {
    close();

    chip_ = ::gpiod_chip_open(chip_path.c_str());
    if (chip_ == nullptr) {
      throw GpioError("gpiod_chip_open failed for status chip '" + chip_path + "'");
    }

    unsigned int offsets[kIgniterChannels] = {};
    for (std::size_t i = 0; i < kIgniterChannels; ++i) {
      offsets[i] = static_cast<unsigned int>(lines[i]);
    }

    if (::gpiod_chip_get_lines(chip_, offsets, kIgniterChannels, &line_bulk_) < 0) {
      const int err = errno;
      close();
      throw GpioError("gpiod_chip_get_lines(status) failed (errno=" + std::to_string(err) + ")");
    }

    if (::gpiod_line_request_bulk_input(&line_bulk_, "igniter-batch-in") < 0) {
      const int err = errno;
      close();
      throw GpioError("gpiod_line_request_bulk_input failed (errno=" + std::to_string(err) + ")");
    }

    open_ = true;
  }

  void close() noexcept override {
    if (open_) {
      ::gpiod_line_release_bulk(&line_bulk_);
      open_ = false;
    }
    if (chip_ != nullptr) {
      ::gpiod_chip_close(chip_);
      chip_ = nullptr;
    }
    std::memset(&line_bulk_, 0, sizeof(line_bulk_));
  }

  [[nodiscard]] std::array<uint8_t, kIgniterChannels> read_values() override {
    if (!open_) {
      throw GpioError("status GPIO batch not open");
    }

    int raw[kIgniterChannels] = {0, 0, 0, 0};
    if (::gpiod_line_get_value_bulk(&line_bulk_, raw) < 0) {
      const int err = errno;
      throw GpioError("gpiod_line_get_value_bulk failed (errno=" + std::to_string(err) + ")");
    }

    std::array<uint8_t, kIgniterChannels> out{};
    for (std::size_t i = 0; i < kIgniterChannels; ++i) {
      out[i] = (raw[i] != 0) ? 1U : 0U;
    }
    return out;
  }

 private:
  bool open_ = false;
  gpiod_chip* chip_ = nullptr;
  gpiod_line_bulk line_bulk_{};
};
#endif

}  // namespace

void SimGpioBatchOut::open(const std::string& chip_path,
                           const std::array<uint32_t, kIgniterChannels>& lines) {
  chip_path_ = chip_path;
  lines_ = lines;
  values_ = {0, 0, 0, 0};
  write_count_ = 0;
  open_ = true;
}

void SimGpioBatchOut::close() noexcept { open_ = false; }

void SimGpioBatchOut::set_mask(uint8_t mask, const std::array<uint8_t, kIgniterChannels>& values) {
  if (!open_) {
    throw GpioError("sim output GPIO batch not open");
  }
  for (std::size_t i = 0; i < kIgniterChannels; ++i) {
    if ((mask & (1U << i)) != 0U) {
      values_[i] = values[i] ? 1U : 0U;
    }
  }
  ++write_count_;
}

std::array<uint8_t, kIgniterChannels> SimGpioBatchOut::values() const { return values_; }

void SimGpioBatchIn::open(const std::string& chip_path,
                          const std::array<uint32_t, kIgniterChannels>& lines) {
  chip_path_ = chip_path;
  lines_ = lines;
  values_ = {1, 1, 1, 1};
  open_ = true;
}

void SimGpioBatchIn::close() noexcept { open_ = false; }

std::array<uint8_t, kIgniterChannels> SimGpioBatchIn::read_values() {
  if (!open_) {
    throw GpioError("sim status GPIO batch not open");
  }
  return values_;
}

void SimGpioBatchIn::set_values(const std::array<uint8_t, kIgniterChannels>& values) { values_ = values; }

std::unique_ptr<GpioBatchOut> make_hardware_batch_out() {
#if defined(IGNITER_HAVE_LIBGPIOD) && (IGNITER_HAVE_LIBGPIOD == 1)
  return std::make_unique<LibgpiodBatchOut>();
#else
  throw GpioError("igniter hardware backend requires libgpiod at build time");
#endif
}

std::unique_ptr<GpioBatchIn> make_hardware_batch_in() {
#if defined(IGNITER_HAVE_LIBGPIOD) && (IGNITER_HAVE_LIBGPIOD == 1)
  return std::make_unique<LibgpiodBatchIn>();
#else
  throw GpioError("igniter hardware backend requires libgpiod at build time");
#endif
}

}  // namespace igniter
