#include <cstdint>

struct Input
{
  bool online;
  bool target;
  uint32_t age_ms;
  float yaw;
  float pitch;
};

// TODO: return mode 1 only for fresh finite in-range target data; otherwise mode 0.
uint8_t safe_mode(const Input &input)
{
  (void)input;
  return 0;
}

int main() { return safe_mode({true, true, 0, 0.0F, 0.0F}) == 1 ? 0 : 1; }
