#include "ads1115/ads1115.hpp"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

class FakeI2c final : public ads1115::I2cDevice {
 public:
  void open() override { open_ = true; }
  void close() noexcept override { open_ = false; }
  [[nodiscard]] bool is_open() const noexcept override { return open_; }

  void write_register(uint8_t reg, const uint8_t* data, std::size_t length) override {
    if (data == nullptr) {
      throw std::runtime_error("FakeI2c received null write buffer");
    }
    if (length != 2) {
      throw std::runtime_error("FakeI2c expects 16-bit register writes in this test");
    }
    const uint16_t value = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) |
                                                 static_cast<uint16_t>(data[1]));

    if (reg <= 0x03) {
      registers_[reg] = value;
    }

    if (reg == 0x01) {
      config_writes_.push_back(value);

      // Single-shot conversion start: clear OS while "busy", then set on next config read.
      if (((value & 0x0100) != 0) && ((value & 0x8000) != 0)) {
        registers_[0x01] = static_cast<uint16_t>(value & ~0x8000U);
        pending_single_shot_ = true;
      }

      // Deterministic conversion value for tests based on MUX bits.
      const uint16_t mux = static_cast<uint16_t>(value & 0x7000U);
      registers_[0x00] = static_cast<uint16_t>(0x0100U | (mux >> 4));
    }
  }

  void write_register(uint8_t reg, uint8_t value) override {
    if (reg > 0x03) {
      throw std::runtime_error("invalid register in FakeI2c");
    }
    registers_[reg] = static_cast<uint16_t>((registers_[reg] & 0xFF00U) | value);
  }

  [[nodiscard]] std::vector<uint8_t> read_registers(uint8_t reg, std::size_t length) override {
    if (length != 2) {
      throw std::runtime_error("FakeI2c expects 16-bit register reads");
    }
    if (reg > 0x03) {
      throw std::runtime_error("invalid register in FakeI2c");
    }

    if ((reg == 0x01) && pending_single_shot_) {
      pending_single_shot_ = false;
      registers_[0x01] = static_cast<uint16_t>(registers_[0x01] | 0x8000U);
    }

    const uint16_t value = registers_[reg];
    return {static_cast<uint8_t>((value >> 8) & 0xFF), static_cast<uint8_t>(value & 0xFF)};
  }

  [[nodiscard]] uint16_t last_config_write() const {
    if (config_writes_.empty()) {
      throw std::runtime_error("no config writes captured");
    }
    return config_writes_.back();
  }

  [[nodiscard]] std::size_t config_write_count() const { return config_writes_.size(); }

 private:
  bool open_ = false;
  bool pending_single_shot_ = false;
  std::array<uint16_t, 4> registers_{
      {0x0000, 0x8583, 0x8000, 0x7FFF},
  };
  std::vector<uint16_t> config_writes_;
};

void expect(bool cond, const std::string& msg) {
  if (!cond) {
    std::cerr << "FAILED: " << msg << "\n";
    std::exit(1);
  }
}

void test_config_bits_and_modes() {
  auto fake = std::make_unique<FakeI2c>();
  FakeI2c* fake_ptr = fake.get();
  ads1115::Ads1115 adc(std::move(fake));
  adc.begin();

  // Default config should include MUX=single AIN0, PGA=2.048V, MODE=single-shot, DR=128SPS.
  expect((fake_ptr->last_config_write() & 0x71E3U) == 0x4183U, "default config bits mismatch");

  adc.set_mode(ads1115::Ads1115::Mode::kContinuous);
  expect((fake_ptr->last_config_write() & 0x0100U) == 0x0000U, "continuous mode bit not applied");

  adc.set_data_rate(ads1115::Ads1115::DataRate::k860Sps);
  expect((fake_ptr->last_config_write() & 0x00E0U) == 0x00E0U, "data-rate bits not applied");

  adc.set_mux(ads1115::Ads1115::Mux::kDiffAin1Ain3);
  expect((fake_ptr->last_config_write() & 0x7000U) == 0x2000U, "mux bits not applied");
}

void test_single_shot_and_read_latest() {
  auto fake = std::make_unique<FakeI2c>();
  FakeI2c* fake_ptr = fake.get();
  ads1115::Ads1115 adc(std::move(fake));
  adc.begin();

  const std::size_t writes_before = fake_ptr->config_write_count();
  const int16_t single = adc.read_adc(3);
  expect(single != 0, "single-shot read should return non-zero mock conversion");
  expect((fake_ptr->last_config_write() & 0x8000U) != 0, "single-shot read should set OS bit");
  expect(fake_ptr->config_write_count() > writes_before, "single-shot read should write CONFIG");

  const std::size_t writes_after_read_adc = fake_ptr->config_write_count();
  (void)adc.read_latest();
  expect(fake_ptr->config_write_count() == writes_after_read_adc, "read_latest must not rewrite CONFIG");
}

void test_poll_continuous() {
  auto fake = std::make_unique<FakeI2c>();
  ads1115::Ads1115 adc(std::move(fake));
  adc.begin();
  adc.set_mode(ads1115::Ads1115::Mode::kContinuous);
  adc.set_mux(ads1115::Ads1115::Mux::kSingleAin2);

  int callback_count = 0;
  adc.poll_continuous(
      [&](int16_t /*sample*/) {
        ++callback_count;
        return true;
      },
      0, 4);
  expect(callback_count == 4, "poll_continuous should deliver max_samples callbacks");
}

}  // namespace

int main() {
  test_config_bits_and_modes();
  test_single_shot_and_read_latest();
  test_poll_continuous();
  std::cout << "ads1115_tests passed\n";
  return 0;
}
