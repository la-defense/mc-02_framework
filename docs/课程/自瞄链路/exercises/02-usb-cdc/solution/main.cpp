#include <array>
#include <cstdint>
#include <iostream>

namespace
{
enum class Submit { accepted, not_configured, busy, invalid_size };

class CdcTxModel
{
 public:
  Submit submit(const uint8_t *data, std::size_t size, bool configured)
  {
    if (!configured) return Submit::not_configured;
    if (busy_) return Submit::busy;
    if (data == nullptr || size != buffer_.size()) return Submit::invalid_size;
    for (std::size_t index = 0; index < size; ++index) buffer_[index] = data[index];
    busy_ = true;
    return Submit::accepted;
  }

  void complete() { busy_ = false; }
  bool busy() const { return busy_; }
  const std::array<uint8_t, 43> &buffer() const { return buffer_; }

 private:
  std::array<uint8_t, 43> buffer_{};
  bool busy_ = false;
};

bool check(bool condition, const char *message)
{
  if (!condition) std::cerr << "FAIL: " << message << '\n';
  return condition;
}
}  // namespace

int main()
{
  CdcTxModel tx;
  std::array<uint8_t, 43> first{};
  std::array<uint8_t, 43> second{};
  first[3] = 0x31;
  second[3] = 0x72;
  if (!check(tx.submit(first.data(), first.size(), false) == Submit::not_configured, "unconfigured device")) return 1;
  if (!check(tx.submit(first.data(), first.size(), true) == Submit::accepted, "first frame accepted")) return 1;
  if (!check(tx.submit(second.data(), second.size(), true) == Submit::busy, "busy frame rejected")) return 1;
  if (!check(tx.buffer()[3] == 0x31 && tx.busy(), "in-flight buffer remains unchanged")) return 1;
  tx.complete();
  if (!check(tx.submit(second.data(), second.size(), true) == Submit::accepted, "completion releases buffer")) return 1;
  std::cout << "CDC buffer lifetime and BUSY cases passed\n";
  return 0;
}
