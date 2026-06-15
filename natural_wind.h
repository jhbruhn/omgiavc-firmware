#pragma once
#include <algorithm>
#include <cmath>
#include <esp_random.h>

// Natural-wind algorithm reverse-engineered from zhimi.fan.za4 firmware.
// One instance lives as a global; all algorithm state is encapsulated here.
class NaturalWind {
 public:
  void activate() {
    counter_ = 0;
    prev_    = 0.0f;
    active_  = true;
  }

  void deactivate() { active_ = false; }
  bool is_active() const { return active_; }

  // One 300 ms tick. Returns an output level [0, 1] for FloatOutput::set_level().
  // base_speed: current fan component speed (0..speed_count) — acts as ceiling.
  // min_speed:  speed_count * min_power — motor start threshold.
  float tick(int base_speed, int min_speed, int speed_count) {
    const Params& p = params_for(base_speed, speed_count);

    // r1 drives jitter; r2 drives carrier frequency stretch
    float r1 = random_01();
    counter_ = (counter_ + 1) % p.wrap;
    float r2 = random_01();

    // Carrier (FUN_0800cdac): cosine with random frequency stretch → [0, 75]
    float stretch = ((r2 - 0.5f) * 0.9f + 1.0f) * 45.0f;
    float wave    = std::max(0.0f, std::min(75.0f,
                    roundf((cosf((float)counter_ * M_PI / stretch) + 1.0f) * 37.5f)));

    // Main step (FUN_0800ce22): jitter + (cur+prev)/10 smoother
    float speed_pre = wave / p.amp + p.base;
    float jitter    = (r1 - 0.5f) * p.offset;
    float combined  = (jitter >= 0.0f || speed_pre >= -jitter)
                    ? speed_pre + jitter
                    : r1 * speed_pre;  // non-negative fallback
    float cur      = std::max((float)p.floor_val, std::max(0.0f, roundf(combined)));
    float nw_motor = std::min(255.0f, (cur + prev_) / 10.0f + 0.5f);
    prev_ = cur;

    // Map nw_motor (typical range 0–50) → fan speed in [min_speed, base_speed]
    float norm  = std::min(1.0f, nw_motor / 50.0f);
    int   speed = min_speed + (int)roundf(norm * (float)(base_speed - min_speed));
    speed = std::max(min_speed, std::min(base_speed, speed));
    return (float)speed / (float)speed_count;
  }

 private:
  struct Params { float amp, offset, base; int floor_val, wrap; };

  // Parameters verified from firmware constant pool 0x0800cfd0..0x0800d020
  inline static const Params MODES[4] = {
    {1.0f,  70.0f,  50.0f,  50, 90},  // gentle   (≤25 % speed)
    {0.6f,  90.0f,  50.0f,  50, 90},  // moderate (≤50 %)
    {0.6f, 110.0f,  70.0f,  70, 45},  // stronger (≤75 %)
    {0.6f, 130.0f, 100.0f,  70, 45},  // high     (>75 %)
  };

  const Params& params_for(int base_speed, int speed_count) const {
    float pct = (float)base_speed / (float)speed_count;
    if (pct <= 0.25f) return MODES[0];
    if (pct <= 0.50f) return MODES[1];
    if (pct <= 0.75f) return MODES[2];
    return MODES[3];
  }

  static float random_01() {
    return (float)(esp_random() >> 1) / (float)0x7FFFFFFFu;
  }

  bool  active_  = false;
  int   counter_ = 0;
  float prev_    = 0.0f;
};

inline NaturalWind natural_wind;
