#include <array>
#include <cstdint>
#include <iostream>
#include <vector>

namespace
{
constexpr std::size_t kCommandFrameSize = 29;

uint16_t crc16(const uint8_t *data, std::size_t length)
{
  uint16_t crc = 0xffff;
  for (std::size_t index = 0; index < length; ++index) {
    crc ^= data[index];
    for (unsigned bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) ? static_cast<uint16_t>((crc >> 1U) ^ 0x8408U)
                       : static_cast<uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

std::array<uint8_t, kCommandFrameSize> make_frame(uint8_t mode)
{
  std::array<uint8_t, kCommandFrameSize> frame{};
  frame[0] = 'S';
  frame[1] = 'P';
  frame[2] = mode;
  const uint16_t crc = crc16(frame.data(), frame.size() - 2);
  frame[frame.size() - 2] = static_cast<uint8_t>(crc & 0xffU);
  frame[frame.size() - 1] = static_cast<uint8_t>(crc >> 8U);
  return frame;
}

bool valid_frame(const std::vector<uint8_t> &frame)
{
  if (frame.size() != kCommandFrameSize || frame[0] != 'S' || frame[1] != 'P') {
    return false;
  }
  const uint16_t received = static_cast<uint16_t>(frame[kCommandFrameSize - 2]) |
                            (static_cast<uint16_t>(frame[kCommandFrameSize - 1]) << 8U);
  return crc16(frame.data(), frame.size() - 2) == received;
}

bool check(bool condition, const char *message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    return false;
  }
  return true;
}
}  // namespace

int main()
{
  const uint8_t sample[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  if (!check(crc16(sample, sizeof(sample)) == 0x6f91, "known CRC vector")) return 1;

  const auto bytes = make_frame(1);
  std::vector<uint8_t> frame(bytes.begin(), bytes.end());
  if (!check(valid_frame(frame), "valid fixed-size frame")) return 1;
  if (!check(!valid_frame(std::vector<uint8_t>(frame.begin(), frame.begin() + 12)), "short frame")) return 1;
  frame[0] = 'X';
  if (!check(!valid_frame(frame), "damaged header")) return 1;
  frame[0] = 'S';
  frame[7] ^= 0x01;
  if (!check(!valid_frame(frame), "damaged payload")) return 1;
  std::cout << "SP length, header, and CRC cases passed\n";
  return 0;
}
