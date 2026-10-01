#include <array>
#include <cstdint>

// TODO: accept a new frame only when configured and no earlier frame is in flight.
class CdcTxModel
{
 public:
  bool submit(const std::array<uint8_t, 43> &frame, bool configured)
  {
    (void)frame;
    (void)configured;
    return false;
  }
};

int main() { return CdcTxModel{}.submit({}, true) ? 0 : 1; }
