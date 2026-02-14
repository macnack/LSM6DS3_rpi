#include "runtime/ipc/shm_mailbox.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace runtime {

namespace {

std::runtime_error with_errno(const std::string& msg) {
  return std::runtime_error(msg + " (errno=" + std::to_string(errno) + ": " + std::strerror(errno) + ")");
}

}  // namespace

ShmRegion::ShmRegion(std::string name, std::size_t bytes, bool owner)
    : name_(std::move(name)),
      bytes_(bytes),
      owner_(owner),
      using_file_fallback_(false),
      fallback_path_(fallback_path()),
      fd_(-1),
      ptr_(MAP_FAILED) {
  if (name_.empty() || name_[0] != '/') {
    throw std::invalid_argument("shm name must begin with '/'");
  }
  if (bytes_ == 0) {
    throw std::invalid_argument("shm size must be > 0");
  }
}

ShmRegion::~ShmRegion() { close_region(); }

void ShmRegion::open_region() {
  if (fd_ >= 0) {
    return;
  }

  int flags = O_RDWR;
  if (owner_) {
    flags |= O_CREAT;
  }

  fd_ = ::shm_open(name_.c_str(), flags, 0660);
  if (fd_ < 0) {
    const int err = errno;
    const bool can_fallback = (err == EPERM || err == EACCES || err == ENOSYS || err == ENOENT);
    if (!can_fallback) {
      throw with_errno("shm_open failed for " + name_);
    }

    using_file_fallback_ = true;
    const int file_flags = owner_ ? (O_RDWR | O_CREAT) : O_RDWR;
    fd_ = ::open(fallback_path_.c_str(), file_flags, 0660);
    if (fd_ < 0) {
      throw with_errno("file-backed fallback open failed for " + fallback_path_);
    }
  }

  if (owner_) {
    if (::ftruncate(fd_, static_cast<off_t>(bytes_)) != 0) {
      const auto err = with_errno("ftruncate failed for " + name_);
      ::close(fd_);
      fd_ = -1;
      throw err;
    }
  }

  ptr_ = ::mmap(nullptr, bytes_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  if (ptr_ == MAP_FAILED) {
    const auto err = with_errno("mmap failed for " + name_);
    ::close(fd_);
    fd_ = -1;
    throw err;
  }
}

void ShmRegion::close_region() noexcept {
  if (ptr_ != MAP_FAILED) {
    (void)::munmap(ptr_, bytes_);
    ptr_ = MAP_FAILED;
  }
  if (fd_ >= 0) {
    (void)::close(fd_);
    fd_ = -1;
  }
}

void ShmRegion::unlink_region() noexcept {
  if (!owner_) {
    return;
  }
  if (using_file_fallback_) {
    (void)::unlink(fallback_path_.c_str());
  } else {
    (void)::shm_unlink(name_.c_str());
  }
}

void* ShmRegion::data() {
  if (ptr_ == MAP_FAILED) {
    throw std::runtime_error("ShmRegion not open");
  }
  return ptr_;
}

const void* ShmRegion::data() const {
  if (ptr_ == MAP_FAILED) {
    throw std::runtime_error("ShmRegion not open");
  }
  return ptr_;
}

std::size_t ShmRegion::size_bytes() const noexcept { return bytes_; }

bool ShmRegion::is_open() const noexcept { return fd_ >= 0 && ptr_ != MAP_FAILED; }

const std::string& ShmRegion::name() const noexcept { return name_; }

std::string ShmRegion::fallback_path() const {
  std::string base = name_;
  while (!base.empty() && base.front() == '/') {
    base.erase(base.begin());
  }
  for (char& ch : base) {
    if (ch == '/') {
      ch = '_';
    }
  }
  if (base.empty()) {
    base = "runtime_mailbox";
  }
  return (std::filesystem::path("/tmp") / (base + ".mailbox")).string();
}

}  // namespace runtime
