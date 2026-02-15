#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace runtime {

void set_shm_security_policy(bool require_local_permissions);

class ShmRegion {
 public:
  ShmRegion(std::string name, std::size_t bytes, bool owner);
  ~ShmRegion();

  ShmRegion(const ShmRegion&) = delete;
  ShmRegion& operator=(const ShmRegion&) = delete;

  void open_region();
  void close_region() noexcept;
  void unlink_region() noexcept;

  [[nodiscard]] void* data();
  [[nodiscard]] const void* data() const;
  [[nodiscard]] std::size_t size_bytes() const noexcept;
  [[nodiscard]] bool is_open() const noexcept;
  [[nodiscard]] const std::string& name() const noexcept;
  [[nodiscard]] bool using_file_fallback() const noexcept;
  [[nodiscard]] const std::string& fallback_path_name() const noexcept;

 private:
  [[nodiscard]] std::string fallback_path() const;

  std::string name_;
  std::size_t bytes_;
  bool owner_;
  bool using_file_fallback_;
  std::string fallback_path_;
  int fd_;
  void* ptr_;
};

template <typename Payload>
class ShmMailbox {
 public:
  struct Layout {
    uint64_t seq_counter;
    Payload payload;
  };

  static_assert(std::is_trivially_copyable<Payload>::value,
                "Shared memory payload must be trivially copyable");

  explicit ShmMailbox(std::string name, bool owner)
      : region_(std::move(name), sizeof(Layout), owner), layout_(nullptr), owner_(owner) {}

  ~ShmMailbox() {
    close();
    if (owner_) {
      unlink();
    }
  }

  void open(bool clear_if_owner = true) {
    region_.open_region();
    layout_ = static_cast<Layout*>(region_.data());
    if (owner_ && clear_if_owner) {
      std::memset(layout_, 0, sizeof(Layout));
    }
  }

  void close() noexcept {
    layout_ = nullptr;
    region_.close_region();
  }

  void unlink() noexcept { region_.unlink_region(); }

  [[nodiscard]] uint64_t current_sequence() const {
    ensure_open();
    return __atomic_load_n(&layout_->seq_counter, __ATOMIC_ACQUIRE);
  }

  [[nodiscard]] bool try_read(Payload& out, uint64_t* stable_seq = nullptr) const {
    ensure_open();

    const uint64_t seq_a = __atomic_load_n(&layout_->seq_counter, __ATOMIC_ACQUIRE);
    if ((seq_a & 1ULL) != 0ULL) {
      return false;
    }

    std::memcpy(&out, &layout_->payload, sizeof(Payload));
    __atomic_thread_fence(__ATOMIC_ACQUIRE);

    const uint64_t seq_b = __atomic_load_n(&layout_->seq_counter, __ATOMIC_ACQUIRE);
    if (seq_a != seq_b || (seq_b & 1ULL) != 0ULL) {
      return false;
    }

    if (stable_seq != nullptr) {
      *stable_seq = seq_b;
    }
    return true;
  }

  void write(const Payload& payload) {
    ensure_open();

    uint64_t seq = __atomic_load_n(&layout_->seq_counter, __ATOMIC_RELAXED);
    if ((seq & 1ULL) != 0ULL) {
      ++seq;
    }

    __atomic_store_n(&layout_->seq_counter, seq + 1ULL, __ATOMIC_RELAXED);
    std::memcpy(&layout_->payload, &payload, sizeof(Payload));
    __atomic_thread_fence(__ATOMIC_RELEASE);
    __atomic_store_n(&layout_->seq_counter, seq + 2ULL, __ATOMIC_RELEASE);
  }

 private:
  void ensure_open() const {
    if (layout_ == nullptr) {
      throw std::runtime_error("ShmMailbox is not open");
    }
  }

  ShmRegion region_;
  Layout* layout_;
  bool owner_;
};

}  // namespace runtime
