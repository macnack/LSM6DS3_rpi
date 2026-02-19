#include "ms4525do/ms4525do.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

class FakeI2c final : public ms4525do::I2cDevice {
 public:
  void open() override { open_ = true; }
  void close() noexcept override { open_ = false; }
  [[nodiscard]] bool is_open() const noexcept override { return open_; }

  void write_bytes(const uint8_t* /*data*/, std::size_t /*length*/) override {}

  [[nodiscard]] std::vector<uint8_t> read_bytes(std::size_t length) override {
    if (length != 4) {
      throw std::runtime_error("FakeI2c expects 4-byte frame reads");
    }
    if (frames_.empty()) {
      throw std::runtime_error("no queued frame");
    }
    const auto frame = frames_.front();
    frames_.erase(frames_.begin());
    return frame;
  }

  void queue_frame(uint8_t status, uint16_t pressure_counts, uint16_t temperature_counts) {
    const uint8_t b0 = static_cast<uint8_t>(((status & 0x03U) << 6U) | ((pressure_counts >> 8U) & 0x3FU));
    const uint8_t b1 = static_cast<uint8_t>(pressure_counts & 0xFFU);
    const uint8_t b2 = static_cast<uint8_t>((temperature_counts >> 3U) & 0xFFU);
    const uint8_t b3 = static_cast<uint8_t>((temperature_counts & 0x07U) << 5U);
    frames_.push_back({b0, b1, b2, b3});
  }

 private:
  bool open_ = false;
  std::vector<std::vector<uint8_t>> frames_;
};

void expect(bool cond, const std::string& msg) {
  if (!cond) {
    std::cerr << "FAILED: " << msg << "\n";
    std::exit(1);
  }
}

void expect_near(double value, double expected, double tol, const std::string& msg) {
  if (std::abs(value - expected) > tol) {
    std::cerr << "FAILED: " << msg << " expected=" << expected << " got=" << value << "\n";
    std::exit(1);
  }
}

template <typename Fn>
void expect_throws_contains(Fn&& fn, const std::string& needle, const std::string& msg) {
  try {
    fn();
  } catch (const std::exception& ex) {
    if (std::string(ex.what()).find(needle) != std::string::npos) {
      return;
    }
    std::cerr << "FAILED: " << msg << " expected message containing '" << needle
              << "' got='" << ex.what() << "'\n";
    std::exit(1);
  }
  std::cerr << "FAILED: " << msg << " expected exception\n";
  std::exit(1);
}

void test_transfer_function_type_b() {
  auto fake = std::make_unique<FakeI2c>();
  FakeI2c* fake_ptr = fake.get();
  fake_ptr->queue_frame(0, 8192, 1023);  // begin() probe frame
  fake_ptr->queue_frame(0, 819, 511);    // pmin
  fake_ptr->queue_frame(0, 15563, 2047); // pmax

  ms4525do::Ms4525do::Calibration cal{};
  cal.p_min_psi = -1.0;
  cal.p_max_psi = 1.0;
  cal.output_type = ms4525do::Ms4525do::OutputType::kTypeB_5To95;

  ms4525do::Ms4525do sensor(std::move(fake), cal);
  sensor.begin();

  const auto pmin = sensor.read();
  expect_near(pmin.pressure_psi, -1.0, 1e-3, "type-b pmin pressure");
  expect_near(pmin.temperature_c, -0.073, 0.02, "temperature conversion");

  const auto pmax = sensor.read();
  expect_near(pmax.pressure_psi, 1.0, 1e-3, "type-b pmax pressure");
  expect(pmax.status == ms4525do::Ms4525do::Status::kNormal, "status must be normal");
}

void test_transfer_function_type_a() {
  auto fake = std::make_unique<FakeI2c>();
  FakeI2c* fake_ptr = fake.get();
  fake_ptr->queue_frame(0, 8192, 1023);   // begin() probe frame
  fake_ptr->queue_frame(0, 1638, 767);    // type-a pmin
  fake_ptr->queue_frame(0, 14746, 1023);  // type-a pmax

  ms4525do::Ms4525do::Calibration cal{};
  cal.p_min_psi = -1.0;
  cal.p_max_psi = 1.0;
  cal.output_type = ms4525do::Ms4525do::OutputType::kTypeA_10To90;

  ms4525do::Ms4525do sensor(std::move(fake), cal);
  sensor.begin();
  expect_near(sensor.read().pressure_psi, -1.0, 1e-3, "type-a pmin pressure");
  expect_near(sensor.read().pressure_psi, 1.0, 1e-3, "type-a pmax pressure");
}

void test_read_allow_stale_returns_stale_status() {
  auto fake = std::make_unique<FakeI2c>();
  FakeI2c* fake_ptr = fake.get();
  fake_ptr->queue_frame(0, 8192, 1023);  // begin() probe frame
  fake_ptr->queue_frame(2, 9000, 1200);  // stale frame

  ms4525do::Ms4525do sensor(std::move(fake));
  sensor.begin();
  const auto reading = sensor.read(ms4525do::Ms4525do::ReadPolicy::kAllowStale);
  expect(reading.status == ms4525do::Ms4525do::Status::kStaleData, "stale status expected");
}

void test_read_require_fresh_retries_stale_then_succeeds() {
  auto fake = std::make_unique<FakeI2c>();
  FakeI2c* fake_ptr = fake.get();
  fake_ptr->queue_frame(0, 8192, 1023);  // begin() probe frame
  fake_ptr->queue_frame(2, 9000, 1200);  // stale
  fake_ptr->queue_frame(2, 9001, 1201);  // stale
  fake_ptr->queue_frame(0, 10000, 1300); // fresh

  ms4525do::Ms4525do sensor(std::move(fake));
  sensor.begin();
  const auto reading = sensor.read();
  expect(reading.status == ms4525do::Ms4525do::Status::kNormal, "expected fresh status after retries");
}

void test_read_require_fresh_stale_exhaustion_throws() {
  auto fake = std::make_unique<FakeI2c>();
  FakeI2c* fake_ptr = fake.get();
  fake_ptr->queue_frame(0, 8192, 1023);  // begin() probe frame
  fake_ptr->queue_frame(2, 9000, 1200);  // stale
  fake_ptr->queue_frame(2, 9001, 1201);  // stale
  fake_ptr->queue_frame(2, 9002, 1202);  // stale
  fake_ptr->queue_frame(2, 9003, 1203);  // stale

  ms4525do::Ms4525do sensor(std::move(fake));
  sensor.begin();
  expect_throws_contains([&]() { (void)sensor.read(); }, "stale-data timeout",
                         "require-fresh stale timeout");
}

void test_begin_command_mode_throws() {
  auto fake = std::make_unique<FakeI2c>();
  FakeI2c* fake_ptr = fake.get();
  fake_ptr->queue_frame(1, 8192, 1023);  // command mode during begin

  ms4525do::Ms4525do sensor(std::move(fake));
  expect_throws_contains([&]() { sensor.begin(); }, "command mode", "begin should fail in command mode");
}

void test_begin_diagnostic_fault_throws() {
  auto fake = std::make_unique<FakeI2c>();
  FakeI2c* fake_ptr = fake.get();
  fake_ptr->queue_frame(3, 8192, 1023);  // fault during begin

  ms4525do::Ms4525do sensor(std::move(fake));
  expect_throws_contains([&]() { sensor.begin(); }, "diagnostic fault", "begin should fail on diagnostic fault");
}

void test_read_fault_status_throws() {
  auto fake = std::make_unique<FakeI2c>();
  FakeI2c* fake_ptr = fake.get();
  fake_ptr->queue_frame(0, 8192, 1023);  // begin() probe frame
  fake_ptr->queue_frame(3, 8192, 1023);  // fault frame

  ms4525do::Ms4525do sensor(std::move(fake));
  sensor.begin();
  expect_throws_contains([&]() { (void)sensor.read(); }, "diagnostic fault", "read should fail on fault");
}

}  // namespace

int main() {
  test_transfer_function_type_b();
  test_transfer_function_type_a();
  test_read_allow_stale_returns_stale_status();
  test_read_require_fresh_retries_stale_then_succeeds();
  test_read_require_fresh_stale_exhaustion_throws();
  test_begin_command_mode_throws();
  test_begin_diagnostic_fault_throws();
  test_read_fault_status_throws();
  std::cout << "ms4525do_tests passed\n";
  return 0;
}
