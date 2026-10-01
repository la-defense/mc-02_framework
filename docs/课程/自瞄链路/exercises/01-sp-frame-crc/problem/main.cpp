#include <array>
#include <cstdint>

// TODO: implement a 29-byte simulated SP command frame validator.
bool valid_frame(const std::array<uint8_t, 29> &frame)
{
  (void)frame;
  return false;
}

int main() { return valid_frame({}) ? 0 : 1; }
