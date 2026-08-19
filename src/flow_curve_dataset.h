#pragma once

#include <Arduino.h>

// Uploaded flow-curve rows, their metadata, and the next playback position.
// Capacity remains a firmware configuration decision at the declaration site.
template <size_t Capacity> struct FlowCurveDataset {
  uint32_t timeMs[Capacity];
  float valveCurrentMa[Capacity];
  uint8_t solenoidEnabled[Capacity];
  uint8_t triggerEnabled[Capacity];
  int receivedCount = 0;
  int loadedCount = 0;
  int durationMs = 0;
  int nextIndex = 0;
};