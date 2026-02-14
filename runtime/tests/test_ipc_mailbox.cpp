#include "runtime/ipc/messages.hpp"
#include "runtime/ipc/shm_mailbox.hpp"

#include <cstdlib>
#include <iostream>
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

bool test_read_write_and_crc() {
  const std::string name = "/rt_test_mailbox_" + std::to_string(::getpid());

  ShmMailbox<PyControllerCommandMsg> writer(name, true);
  ShmMailbox<PyControllerCommandMsg> reader(name, false);
  writer.open(true);
  reader.open(false);

  PyControllerCommandMsg msg{};
  fill_message_header(msg, 1, 123456789ULL);
  msg.armed = 1;
  msg.servo_norm = {0.1F, -0.2F, 0.3F, -0.4F};
  finalize_message_crc(msg);

  writer.write(msg);

  PyControllerCommandMsg out{};
  uint64_t seq = 0;
  REQUIRE(reader.try_read(out, &seq), "reader.try_read should succeed after writer.write");
  REQUIRE(validate_message_crc(out), "CRC validation should pass for written message");
  REQUIRE(out.seq == 1ULL, "Message payload seq must match written value");
  REQUIRE(seq >= 2ULL, "Mailbox sequence counter should advance to even >= 2");

  PyControllerCommandMsg msg2 = msg;
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

  ShmMailbox<PyEstimatorStateMsg> writer(name, true);
  ShmMailbox<PyEstimatorStateMsg> reader(name, false);
  writer.open(true);
  reader.open(false);

  // Force odd sequence directly to emulate writer mid-update.
  ShmRegion raw(name, sizeof(ShmMailbox<PyEstimatorStateMsg>::Layout), false);
  raw.open_region();
  auto* layout = static_cast<ShmMailbox<PyEstimatorStateMsg>::Layout*>(raw.data());
  layout->seq_counter = 7ULL;

  PyEstimatorStateMsg out{};
  REQUIRE(!reader.try_read(out, nullptr), "reader.try_read must fail when mailbox sequence is odd");

  raw.close_region();
  writer.close();
  reader.close();
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
  std::cout << "runtime_unit_ipc: ok\n";
  return EXIT_SUCCESS;
}
