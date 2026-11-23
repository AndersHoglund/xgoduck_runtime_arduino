#pragma once

// Battery gauge on the UNO Q LED matrix (8 x 13).
// Source: servo bus voltage (median of the servos' own voltage readings),
// published by the control thread. 2S Li-ion pack.
//
//   7.0 - 8.4 V : battery outline with 0..8 bars
//   6.5 - 7.0 V : empty outline, slow blink (1 Hz)      -> charge soon
//   < 6.5 V     : "!" / empty outline, fast flash (4 Hz) -> dangerous
//   no reading  : matrix blank (servos unpowered / not answering)

#include <Arduino_LED_Matrix.h>
#include <zephyr/sys/atomic.h>
#include <string.h>

static constexpr uint32_t BATT_FULL_MV = 8400;
static constexpr uint32_t BATT_EMPTY_MV = 7000;
static constexpr uint32_t BATT_EMPTY_EXIT_MV = 7100;  // hysteresis
static constexpr uint32_t BATT_DANGER_MV = 6500;
static constexpr uint32_t BATT_DANGER_EXIT_MV = 6700; // hysteresis
static constexpr int BATT_LEVELS = 8;
static constexpr uint32_t BATT_STALE_MS = 2000;

// Written by the control thread, read by loop().
static atomic_t batteryBusMv = ATOMIC_INIT(0);
static atomic_t batteryBusMs = ATOMIC_INIT(0);

static inline void publishBatteryMv(uint32_t mv) {
  if (!mv) return;
  atomic_set(&batteryBusMv, (atomic_val_t)mv);
  atomic_set(&batteryBusMs, (atomic_val_t)millis());
}

enum class BatteryState : uint8_t { Unknown, Normal, Low, Danger };

struct BatteryGauge {
  float mv = 0;            // smoothed
  int bars = -1;
  BatteryState state = BatteryState::Unknown;

  // Called every ~100 ms from loop().
  void update(uint32_t nowMs) {
    const uint32_t raw = (uint32_t)atomic_get(&batteryBusMv);
    const uint32_t at = (uint32_t)atomic_get(&batteryBusMs);
    if (!raw || uint32_t(nowMs - at) > BATT_STALE_MS) {
      mv = 0; bars = -1; state = BatteryState::Unknown;
      return;
    }
    // ~1 s low-pass so motor-load sag does not flicker the gauge.
    mv = (mv <= 0) ? raw : mv + 0.1f * ((float)raw - mv);

    switch (state) {
      case BatteryState::Unknown:
        state = mv < BATT_DANGER_MV ? BatteryState::Danger
              : mv < BATT_EMPTY_MV  ? BatteryState::Low : BatteryState::Normal;
        break;
      case BatteryState::Normal:
        if (mv < BATT_DANGER_MV) state = BatteryState::Danger;
        else if (mv < BATT_EMPTY_MV) state = BatteryState::Low;
        break;
      case BatteryState::Low:
        if (mv < BATT_DANGER_MV) state = BatteryState::Danger;
        else if (mv >= BATT_EMPTY_EXIT_MV) state = BatteryState::Normal;
        break;
      case BatteryState::Danger:
        if (mv >= BATT_DANGER_EXIT_MV) state = mv < BATT_EMPTY_MV ? BatteryState::Low : BatteryState::Normal;
        break;
    }

    float level = (mv - (float)BATT_EMPTY_MV) * BATT_LEVELS / (float)(BATT_FULL_MV - BATT_EMPTY_MV);
    if (level < 0) level = 0;
    if (level > BATT_LEVELS) level = BATT_LEVELS;
    // Needs to move 3/4 of a step before a bar changes.
    if (bars < 0 || level > bars + 0.75f || level < bars - 0.75f) {
      int b = (int)(level + 0.5f);
      bars = b < 0 ? 0 : (b > BATT_LEVELS ? BATT_LEVELS : b);
    }
  }
};

static void drawBatteryOutline(uint8_t (&px)[8][13]) {
  for (unsigned c = 1; c <= 12; ++c) { px[0][c] = 1; px[7][c] = 1; }
  for (unsigned r = 0; r <= 7; ++r) { px[r][1] = 1; px[r][12] = 1; }
  for (unsigned r = 2; r <= 5; ++r) px[r][0] = 1;  // terminal
}

static void drawBatteryBars(uint8_t (&px)[8][13], int bars) {
  for (int i = 0; i < bars && i < BATT_LEVELS; ++i)
    for (unsigned r = 2; r <= 5; ++r) px[r][12 - i] = 1;  // interior cols 2..9
}

static void drawBatteryWarning(uint8_t (&px)[8][13]) {
  for (unsigned r = 0; r <= 4; ++r) { px[r][5] = 1; px[r][6] = 1; }
  px[6][5] = px[6][6] = px[7][5] = px[7][6] = 1;
}

// Fills px. Returns false only when there is no battery reading.
static bool renderBattery(const BatteryGauge& g, uint32_t nowMs, uint8_t (&px)[8][13]) {
  memset(px, 0, sizeof(px));
  switch (g.state) {
    case BatteryState::Unknown:
      return false;
    case BatteryState::Danger:
      if ((nowMs / 250) % 2) drawBatteryWarning(px); else drawBatteryOutline(px);
      return true;
    case BatteryState::Low:
      if ((nowMs / 500) % 2) drawBatteryOutline(px);
      return true;
    case BatteryState::Normal:
      drawBatteryOutline(px);
      drawBatteryBars(px, g.bars);
      return true;
  }
  return false;
}
