#include "runtime/ipc/messages.hpp"

#include <pybind11/pybind11.h>

#include <cstring>

namespace py = pybind11;

namespace runtime {
namespace {


template <typename Msg>
bool decode_message_bytes(const py::bytes& bytes, Msg& out) {
  const std::string raw = bytes;
  if (raw.size() != sizeof(Msg)) {
    return false;
  }
  std::memcpy(&out, raw.data(), sizeof(Msg));
  if (out.msg_magic != kMessageMagic || out.msg_version != kMessageVersion ||
      out.payload_bytes != payload_size_bytes<Msg>()) {
    return false;
  }
  if (!validate_message_crc(out)) {
    return false;
  }
  return true;
}

py::object decode_sensor_snapshot(const py::bytes& bytes) {
  SensorSnapshotMsg msg{};
  if (!decode_message_bytes(bytes, msg)) {
    return py::none();
  }
  py::dict d;
  d["seq"] = msg.seq;
  d["t_ns"] = msg.t_ns;
  d["imu_valid"] = (msg.imu_valid != 0);
  d["baro_valid"] = (msg.baro_valid != 0);
  d["ax_mps2"] = msg.ax_mps2;
  d["ay_mps2"] = msg.ay_mps2;
  d["az_mps2"] = msg.az_mps2;
  d["gx_rads"] = msg.gx_rads;
  d["gy_rads"] = msg.gy_rads;
  d["gz_rads"] = msg.gz_rads;
  d["pressure_pa"] = msg.pressure_pa;
  d["temperature_c"] = msg.temperature_c;
  return std::move(d);
}

py::bytes encode_sensor_snapshot(uint64_t seq, uint64_t t_ns, bool imu_valid, bool baro_valid, double ax_mps2,
                                 double ay_mps2, double az_mps2, double gx_rads, double gy_rads, double gz_rads,
                                 double pressure_pa, double temperature_c) {
  SensorSnapshotMsg msg{};
  fill_message_header(msg, seq, t_ns);
  msg.imu_valid = imu_valid ? 1U : 0U;
  msg.baro_valid = baro_valid ? 1U : 0U;
  msg.ax_mps2 = ax_mps2;
  msg.ay_mps2 = ay_mps2;
  msg.az_mps2 = az_mps2;
  msg.gx_rads = gx_rads;
  msg.gy_rads = gy_rads;
  msg.gz_rads = gz_rads;
  msg.pressure_pa = pressure_pa;
  msg.temperature_c = temperature_c;
  finalize_message_crc(msg);
  return py::bytes(reinterpret_cast<const char*>(&msg), sizeof(msg));
}

py::object decode_controller_command(const py::bytes& bytes) {
  ExternalControllerCommandMsg msg{};
  if (!decode_message_bytes(bytes, msg)) {
    return py::none();
  }
  py::dict d;
  d["seq"] = msg.seq;
  d["t_ns"] = msg.t_ns;
  d["armed"] = (msg.armed != 0);
  py::tuple servos(kServoCount);
  for (std::size_t i = 0; i < kServoCount; ++i) {
    servos[i] = msg.servo_norm[i];
  }
  d["servo_norm"] = std::move(servos);
  return std::move(d);
}

py::bytes encode_controller_command(uint64_t seq, uint64_t t_ns, bool armed, float s0, float s1, float s2, float s3) {
  ExternalControllerCommandMsg msg{};
  fill_message_header(msg, seq, t_ns);
  msg.armed = armed ? 1U : 0U;
  msg.servo_norm = {s0, s1, s2, s3};
  finalize_message_crc(msg);
  return py::bytes(reinterpret_cast<const char*>(&msg), sizeof(msg));
}

py::object decode_estimator_state(const py::bytes& bytes) {
  ExternalEstimatorStateMsg msg{};
  if (!decode_message_bytes(bytes, msg)) {
    return py::none();
  }
  py::dict d;
  d["seq"] = msg.seq;
  d["t_ns"] = msg.t_ns;
  d["valid"] = (msg.valid != 0);
  py::tuple q(4);
  q[0] = msg.q_body_to_ned[0];
  q[1] = msg.q_body_to_ned[1];
  q[2] = msg.q_body_to_ned[2];
  q[3] = msg.q_body_to_ned[3];
  py::tuple v(3);
  v[0] = msg.vel_ned_mps[0];
  v[1] = msg.vel_ned_mps[1];
  v[2] = msg.vel_ned_mps[2];
  py::tuple p(3);
  p[0] = msg.pos_ned_m[0];
  p[1] = msg.pos_ned_m[1];
  p[2] = msg.pos_ned_m[2];
  d["q_body_to_ned"] = std::move(q);
  d["vel_ned_mps"] = std::move(v);
  d["pos_ned_m"] = std::move(p);
  return std::move(d);
}

py::bytes encode_estimator_state(uint64_t seq, uint64_t t_ns, bool valid, float q0, float q1, float q2, float q3,
                                 float vx, float vy, float vz, float px, float py, float pz) {
  ExternalEstimatorStateMsg msg{};
  fill_message_header(msg, seq, t_ns);
  msg.valid = valid ? 1U : 0U;
  msg.q_body_to_ned = {q0, q1, q2, q3};
  msg.vel_ned_mps = {vx, vy, vz};
  msg.pos_ned_m = {px, py, pz};
  finalize_message_crc(msg);
  return py::bytes(reinterpret_cast<const char*>(&msg), sizeof(msg));
}

}  // namespace
}  // namespace runtime

PYBIND11_MODULE(_runtime_ipc_codec, m) {
  m.doc() = "Lightweight runtime IPC codec bindings";

  m.attr("CODEC_API_VERSION") = py::int_(1);
  m.attr("MESSAGE_MAGIC") = py::int_(runtime::kMessageMagic);
  m.attr("MESSAGE_VERSION") = py::int_(runtime::kMessageVersion);
  m.attr("SENSOR_MSG_SIZE") = py::int_(sizeof(runtime::SensorSnapshotMsg));
  m.attr("ESTIMATOR_MSG_SIZE") = py::int_(sizeof(runtime::ExternalEstimatorStateMsg));
  m.attr("CONTROLLER_MSG_SIZE") = py::int_(sizeof(runtime::ExternalControllerCommandMsg));
  m.attr("SENSOR_PAYLOAD_BYTES") = py::int_(runtime::payload_size_bytes<runtime::SensorSnapshotMsg>());
  m.attr("ESTIMATOR_PAYLOAD_BYTES") = py::int_(runtime::payload_size_bytes<runtime::ExternalEstimatorStateMsg>());
  m.attr("CONTROLLER_PAYLOAD_BYTES") = py::int_(runtime::payload_size_bytes<runtime::ExternalControllerCommandMsg>());

  m.def("decode_sensor_snapshot", &runtime::decode_sensor_snapshot, py::arg("bytes"));
  m.def("encode_sensor_snapshot", &runtime::encode_sensor_snapshot, py::arg("seq"), py::arg("t_ns"),
        py::arg("imu_valid"), py::arg("baro_valid"), py::arg("ax_mps2"), py::arg("ay_mps2"), py::arg("az_mps2"),
        py::arg("gx_rads"), py::arg("gy_rads"), py::arg("gz_rads"), py::arg("pressure_pa"),
        py::arg("temperature_c"));

  m.def("decode_controller_command", &runtime::decode_controller_command, py::arg("bytes"));
  m.def("encode_controller_command", &runtime::encode_controller_command, py::arg("seq"), py::arg("t_ns"),
        py::arg("armed"), py::arg("s0"), py::arg("s1"), py::arg("s2"), py::arg("s3"));

  m.def("decode_estimator_state", &runtime::decode_estimator_state, py::arg("bytes"));
  m.def("encode_estimator_state", &runtime::encode_estimator_state, py::arg("seq"), py::arg("t_ns"), py::arg("valid"),
        py::arg("q0"), py::arg("q1"), py::arg("q2"), py::arg("q3"), py::arg("vx"), py::arg("vy"), py::arg("vz"),
        py::arg("px"), py::arg("py"), py::arg("pz"));
}
