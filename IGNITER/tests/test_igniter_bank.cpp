#include "igniter/igniter_bank.hpp"

#include <cstdlib>
#include <iostream>

namespace {

#define REQUIRE(cond, msg)      \
  do {                          \
    if (!(cond)) {              \
      std::cerr << msg << "\n"; \
      return false;             \
    }                           \
  } while (0)

class CountingBatchOut final : public igniter::GpioBatchOut {
 public:
  void open(const std::string&, const std::array<uint32_t, igniter::kIgniterChannels>&) override {
    opened_ = true;
  }

  void close() noexcept override { opened_ = false; }

  void set_mask(uint8_t mask, const std::array<uint8_t, igniter::kIgniterChannels>& values) override {
    if (!opened_) {
      throw std::runtime_error("CountingBatchOut used before open");
    }
    for (std::size_t i = 0; i < igniter::kIgniterChannels; ++i) {
      if ((mask & (1U << i)) != 0U) {
        values_[i] = values[i] ? 1U : 0U;
      }
    }
    last_mask_ = mask;
    ++writes_;
  }

  [[nodiscard]] std::array<uint8_t, igniter::kIgniterChannels> values() const override {
    return values_;
  }

  [[nodiscard]] uint64_t writes() const noexcept { return writes_; }
  [[nodiscard]] uint8_t last_mask() const noexcept { return last_mask_; }

 private:
  bool opened_ = false;
  std::array<uint8_t, igniter::kIgniterChannels> values_{0, 0, 0, 0};
  uint64_t writes_ = 0;
  uint8_t last_mask_ = 0;
};

class FixedBatchIn final : public igniter::GpioBatchIn {
 public:
  void open(const std::string&, const std::array<uint32_t, igniter::kIgniterChannels>&) override {
    opened_ = true;
  }

  void close() noexcept override { opened_ = false; }

  [[nodiscard]] std::array<uint8_t, igniter::kIgniterChannels> read_values() override {
    if (!opened_) {
      throw std::runtime_error("FixedBatchIn used before open");
    }
    return values_;
  }

  void set_values(const std::array<uint8_t, igniter::kIgniterChannels>& values) { values_ = values; }

 private:
  bool opened_ = false;
  std::array<uint8_t, igniter::kIgniterChannels> values_{1, 1, 1, 1};
};

igniter::IgniterBank::Config make_cfg() {
  igniter::IgniterBank::Config cfg;
  cfg.faultPolicy = igniter::FaultPolicy::Global;
  for (auto& ch : cfg.channels) {
    ch.enabled = true;
    ch.driver.settleMs = 5;
    ch.driver.latchFaults = true;
    ch.igniter.maxFireMs = 2000;
    ch.igniter.defaultFireMs = 100;
  }
  return cfg;
}

bool test_fire_mask_single_batch_write() {
  auto out = std::make_unique<CountingBatchOut>();
  auto in = std::make_unique<FixedBatchIn>();
  auto* out_ptr = out.get();

  igniter::IgniterBank bank(make_cfg(), std::move(out), std::move(in));
  bank.bind_hardware("sim", "sim", {1, 2, 3, 4}, {5, 6, 7, 8});
  bank.init(0);

  const uint64_t baseline = out_ptr->writes();
  REQUIRE(bank.arm(1), "arm should succeed");

  const std::array<uint32_t, igniter::kIgniterChannels> durations{120, 120, 120, 120};
  REQUIRE(bank.fire_mask(0x0F, durations, 10), "fire_mask should succeed");

  REQUIRE(out_ptr->writes() == baseline + 1, "group fire must emit exactly one batch GPIO write");
  REQUIRE(out_ptr->last_mask() == 0x0F, "group fire must write all 4 channels in one batch mask");

  const auto snap = bank.snapshot(11);
  REQUIRE((snap.activeMask & 0x0F) == 0x0F, "all igniters should be active after group fire");
  return true;
}

bool test_fire_mask_rejected_when_disarmed() {
  auto out = std::make_unique<CountingBatchOut>();
  auto in = std::make_unique<FixedBatchIn>();
  auto* out_ptr = out.get();

  igniter::IgniterBank bank(make_cfg(), std::move(out), std::move(in));
  bank.bind_hardware("sim", "sim", {1, 2, 3, 4}, {5, 6, 7, 8});
  bank.init(0);

  const uint64_t baseline = out_ptr->writes();
  const std::array<uint32_t, igniter::kIgniterChannels> durations{120, 120, 120, 120};
  REQUIRE(!bank.fire_mask(0x0F, durations, 10), "fire_mask must fail while disarmed");
  REQUIRE(out_ptr->writes() == baseline, "rejected fire should not touch GPIO outputs");
  return true;
}

}  // namespace

int main() {
  if (!test_fire_mask_single_batch_write()) {
    return EXIT_FAILURE;
  }
  if (!test_fire_mask_rejected_when_disarmed()) {
    return EXIT_FAILURE;
  }
  std::cout << "igniter_unit_bank: ok\n";
  return EXIT_SUCCESS;
}
