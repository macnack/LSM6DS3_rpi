#include "runtime/ipc/messages.hpp"
#include "runtime/ipc/shm_mailbox.hpp"

#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <string>
#include <unistd.h>

namespace {

#define REQUIRE(cond, msg)      \
  do {                          \
    if (!(cond)) {              \
      std::cerr << msg << "\n"; \
      return false;             \
    }                           \
  } while (0)

using namespace runtime;

class SecurityPolicyGuard {
 public:
  explicit SecurityPolicyGuard(bool enabled) : previous_(false) { set_shm_security_policy(enabled); }
  ~SecurityPolicyGuard() { set_shm_security_policy(previous_); }

 private:
  bool previous_;
};

bool test_read_write_and_crc() {
  const std::string name = "/rt_test_mailbox_" + std::to_string(::getpid());

  ShmMailbox<ExternalControllerCommandMsg> writer(name, true);
  ShmMailbox<ExternalControllerCommandMsg> reader(name, false);
  writer.open(true);
  reader.open(false);

  ExternalControllerCommandMsg msg{};
  fill_message_header(msg, 1, 123456789ULL);
  msg.armed = 1;
  msg.servo_norm = {0.1F, -0.2F, 0.3F, -0.4F};
  finalize_message_crc(msg);

  writer.write(msg);

  ExternalControllerCommandMsg out{};
  uint64_t seq = 0;
  REQUIRE(reader.try_read(out, &seq), "reader.try_read should succeed after writer.write");
  REQUIRE(validate_message_crc(out), "CRC validation should pass for written message");
  REQUIRE(out.seq == 1ULL, "Message payload seq must match written value");
  REQUIRE(seq >= 2ULL, "Mailbox sequence counter should advance to even >= 2");

  ExternalControllerCommandMsg msg2 = msg;
  fill_message_header(msg2, 2, 123456999ULL);
  msg2.servo_norm = {-0.1F, 0.2F, -0.3F, 0.4F};
  finalize_message_crc(msg2);

  const uint64_t before = writer.current_sequence();
  writer.write(msg2);
  const uint64_t after = writer.current_sequence();
  REQUIRE(after > before, "Mailbox sequence counter must be monotonic");

  writer.close();
  reader.close();
  return true;
}

bool test_torn_write_detection() {
  const std::string name = "/rt_test_mailbox_torn_" + std::to_string(::getpid());

  ShmMailbox<ExternalEstimatorStateMsg> writer(name, true);
  ShmMailbox<ExternalEstimatorStateMsg> reader(name, false);
  writer.open(true);
  reader.open(false);

  // Force odd sequence directly to emulate writer mid-update.
  ShmRegion raw(name, sizeof(ShmMailbox<ExternalEstimatorStateMsg>::Layout), false);
  raw.open_region();
  auto* layout = static_cast<ShmMailbox<ExternalEstimatorStateMsg>::Layout*>(raw.data());
  layout->seq_counter = 7ULL;

  ExternalEstimatorStateMsg out{};
  REQUIRE(!reader.try_read(out, nullptr), "reader.try_read must fail when mailbox sequence is odd");

  raw.close_region();
  writer.close();
  reader.close();
  return true;
}

bool test_mailbox_security_allows_secure_path() {
  const std::string name = "/rt_test_mailbox_sec_" + std::to_string(::getpid());
  SecurityPolicyGuard guard(true);

  ShmMailbox<ExternalControllerCommandMsg> writer(name, true);
  writer.open(true);
  ExternalControllerCommandMsg msg{};
  fill_message_header(msg, 1, 42ULL);
  finalize_message_crc(msg);
  writer.write(msg);
  writer.close();
  return true;
}

bool test_mailbox_security_rejects_world_writable_shm() {
  const std::string name = "/rt_test_mailbox_insecure_" + std::to_string(::getpid());
  SecurityPolicyGuard guard(true);

  const std::size_t bytes = sizeof(ShmMailbox<ExternalControllerCommandMsg>::Layout);
  std::string fallback_path;
  int fd = ::shm_open(name.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd >= 0) {
    REQUIRE(::ftruncate(fd, static_cast<off_t>(bytes)) == 0, "ftruncate fixture must succeed");
    REQUIRE(::fchmod(fd, 0666) == 0, "fchmod(0666) fixture must succeed");
    (void)::close(fd);
  } else {
    ShmRegion probe(name, bytes, false);
    fallback_path = probe.fallback_path_name();
    const int file_fd = ::open(fallback_path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0666);
    REQUIRE(file_fd >= 0, "fallback file open for insecure fixture must succeed");
    REQUIRE(::ftruncate(file_fd, static_cast<off_t>(bytes)) == 0, "fallback ftruncate fixture must succeed");
    REQUIRE(::fchmod(file_fd, 0666) == 0, "fallback fchmod(0666) fixture must succeed");
    (void)::close(file_fd);
  }

  bool threw = false;
  try {
    ShmMailbox<ExternalControllerCommandMsg> reader(name, false);
    reader.open(false);
  } catch (const std::exception&) {
    threw = true;
  }
  if (!fallback_path.empty()) {
    (void)::unlink(fallback_path.c_str());
  }
  (void)::shm_unlink(name.c_str());

  REQUIRE(threw, "Reader open should fail for world-writable shm when security policy is enabled");
  return true;
}

}  // namespace

int main() {
  if (!test_read_write_and_crc()) {
    return EXIT_FAILURE;
  }
  if (!test_torn_write_detection()) {
    return EXIT_FAILURE;
  }
  if (!test_mailbox_security_allows_secure_path()) {
    return EXIT_FAILURE;
  }
  if (!test_mailbox_security_rejects_world_writable_shm()) {
    return EXIT_FAILURE;
  }
  std::cout << "runtime_unit_ipc: ok\n";
  return EXIT_SUCCESS;
}
