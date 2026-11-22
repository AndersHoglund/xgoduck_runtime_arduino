#pragma once

#include <Arduino_LED_Matrix.h>
#include <zephyr/sys/atomic.h>
#include <string.h>
#include "battery_matrix.h"

static atomic_t startupIpOctet = ATOMIC_INIT(-1);

static bool receiveStartupIp(int octet) {
  if (octet < 0 || octet > 255) return false;
  // A retry or a restart of the Linux voice process must not restart the timer.
  atomic_cas(&startupIpOctet, -1, octet);
  return true;
}

static void renderIpOctet(Arduino_LED_Matrix& matrix, unsigned octet) {
  static constexpr uint8_t digits[10][5] = {
    {7, 5, 5, 5, 7}, {2, 6, 2, 2, 7}, {7, 1, 7, 4, 7}, {7, 1, 7, 1, 7}, {5, 5, 7, 1, 1},
    {7, 4, 7, 1, 7}, {7, 4, 7, 5, 7}, {7, 1, 1, 1, 1}, {7, 5, 7, 5, 7}, {7, 5, 7, 1, 7}
  };
  uint8_t pixels[8][13] = {};
  const unsigned count = octet >= 100 ? 3 : (octet >= 10 ? 2 : 1);
  const unsigned left = (13 - (count * 4 - 1)) / 2;
  for (unsigned i = count; i > 0; --i, octet /= 10) {
    const unsigned x = left + (i - 1) * 4;
    for (unsigned row = 0; row < 5; ++row) {
      for (unsigned col = 0; col < 3; ++col) {
        pixels[1 + row][x + col] = (digits[octet % 10][row] >> (2 - col)) & 1;
      }
    }
  }
  matrix.renderBitmap(pixels, 8, 13);
}

// Called by the Arduino loop, outside the motion and Bridge threads.
// The Wi-Fi IP suffix owns the matrix for 60 s after it arrives; otherwise the
// matrix shows the battery gauge (battery_matrix.h).
static void serviceStartupIpDisplay() {
  static Arduino_LED_Matrix matrix;
  static bool begun = false, shown = false, ipVisible = false, blank = false;
  static uint32_t shownAt = 0;
  static BatteryGauge gauge;
  static uint8_t last[8][13] = {};
  const uint32_t now = millis();
  if (!begun) { matrix.begin(); matrix.clear(); begun = blank = true; }

  const int octet = atomic_get(&startupIpOctet);
  if (!shown && octet >= 0) {
    renderIpOctet(matrix, octet);
    shownAt = now;
    shown = ipVisible = true;
    blank = false;
  } else if (ipVisible && uint32_t(now - shownAt) >= 60000u) {
    ipVisible = false;
    memset(last, 0xff, sizeof(last));  // force a battery redraw
  }

  gauge.update(now);
  if (ipVisible) return;

  uint8_t px[8][13];
  if (!renderBattery(gauge, now, px)) {
    if (!blank) { matrix.clear(); blank = true; }
    return;
  }
  if (!blank && memcmp(px, last, sizeof(px)) == 0) return;
  matrix.renderBitmap(px, 8, 13);
  memcpy(last, px, sizeof(px));
  blank = false;
}
