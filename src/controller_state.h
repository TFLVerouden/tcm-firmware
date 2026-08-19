#pragma once

#include <Arduino.h>

// Modes shared by the run, droplet-detection, and laser-test state machines.
enum class LoopMode : uint8_t {
  Idle,
  DropletDetect,
  DelayBeforeRun,
  ExecutingRun,
  LaserTest
};

// Mutable controller state shared by command handlers and mode processors.
struct ControllerState {
  LoopMode mode = LoopMode::Idle;
  bool solValveOpen = false;
  bool performingTrigger = false;
  bool belowThreshold = false;
  bool detectionBaselineReady = false;
  uint32_t delayedRunStartTime = 0;
  uint32_t detectionStartTime = 0;
  bool runAfterDropletDetection = false;
  bool pressureConfigured = false;
  uint32_t laserTestLastPrint = 0;
};