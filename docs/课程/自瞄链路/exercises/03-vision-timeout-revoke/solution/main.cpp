#include <cmath>
#include <iostream>
#include <limits>

namespace
{
struct Input
{
  bool online;
  bool target;
  uint8_t mode;
  uint32_t age_ms;
  float yaw;
  float pitch;
};

struct Bounds
{
  float yaw_min;
  float yaw_max;
  float pitch_min;
  float pitch_max;
  uint32_t max_age_ms;
};

struct Command
{
  uint8_t mode;
  bool fire;
};

Command decide(const Input &input, const Bounds &bounds)
{
  const bool usable = input.online && input.mode == 0 && input.target &&
                      input.age_ms <= bounds.max_age_ms && std::isfinite(input.yaw) &&
                      std::isfinite(input.pitch) && input.yaw >= bounds.yaw_min &&
                      input.yaw <= bounds.yaw_max && input.pitch >= bounds.pitch_min &&
                      input.pitch <= bounds.pitch_max;
  return Command{static_cast<uint8_t>(usable ? 1 : 0), false};
}

bool check(bool condition, const char *message)
{
  if (!condition) std::cerr << "FAIL: " << message << '\n';
  return condition;
}
}  // namespace

int main()
{
  const Bounds bounds{-1.0F, 1.0F, -0.5F, 0.5F, 50};
  Input input{true, true, 0, 10, 0.2F, -0.1F};
  if (!check(decide(input, bounds).mode == 1, "fresh safe target controls")) return 1;
  if (!check(!decide(input, bounds).fire, "fire is always disabled")) return 1;
  input.target = false;
  if (!check(decide(input, bounds).mode == 0, "no target revokes control")) return 1;
  input.target = true;
  input.age_ms = 51;
  if (!check(decide(input, bounds).mode == 0, "stale state revokes control")) return 1;
  input.age_ms = 0;
  input.yaw = std::numeric_limits<float>::quiet_NaN();
  if (!check(decide(input, bounds).mode == 0, "NaN revokes control")) return 1;
  input.yaw = 1.01F;
  if (!check(decide(input, bounds).mode == 0, "out-of-range angle revokes control")) return 1;
  input.yaw = 0.0F;
  input.online = false;
  if (!check(decide(input, bounds).mode == 0, "link loss revokes control")) return 1;
  std::cout << "Vision timeout and neutral-revocation cases passed\n";
  return 0;
}
