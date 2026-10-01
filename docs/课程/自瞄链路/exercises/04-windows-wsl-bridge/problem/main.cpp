#include <cstdint>

struct BridgeState
{
  bool camera_connected;
  bool serial_connected;
  bool crc_ok;
  uint32_t frame_age_ms;
};

// TODO: allow forwarding only when both links are live and the frame is valid/fresh.
bool can_forward(const BridgeState &state)
{
  (void)state;
  return false;
}

int main() { return can_forward({true, true, true, 0}) ? 0 : 1; }
