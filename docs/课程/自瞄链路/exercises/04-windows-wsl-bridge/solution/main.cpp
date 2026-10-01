#include <cstdint>
#include <iostream>

namespace
{
struct BridgeState
{
  bool camera_connected;
  bool serial_connected;
  bool frame_crc_ok;
  uint32_t camera_age_ms;
  uint32_t serial_age_ms;
};

bool can_forward(const BridgeState &state, uint32_t freshness_ms)
{
  return state.camera_connected && state.serial_connected && state.frame_crc_ok &&
         state.camera_age_ms <= freshness_ms && state.serial_age_ms <= freshness_ms;
}

bool check(bool condition, const char *message)
{
  if (!condition) std::cerr << "FAIL: " << message << '\n';
  return condition;
}
}  // namespace

int main()
{
  BridgeState state{true, true, true, 5, 4};
  if (!check(can_forward(state, 30), "fresh camera and serial data")) return 1;
  state.camera_connected = false;
  if (!check(!can_forward(state, 30), "camera disconnect")) return 1;
  state.camera_connected = true;
  state.serial_connected = false;
  if (!check(!can_forward(state, 30), "serial disconnect")) return 1;
  state.serial_connected = true;
  state.frame_crc_ok = false;
  if (!check(!can_forward(state, 30), "corrupt SP frame")) return 1;
  state.frame_crc_ok = true;
  state.camera_age_ms = 31;
  if (!check(!can_forward(state, 30), "stale camera frame")) return 1;
  std::cout << "Bridge disconnect, CRC, and freshness cases passed\n";
  return 0;
}
