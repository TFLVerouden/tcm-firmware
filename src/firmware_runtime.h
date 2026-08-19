#pragma once

#include <Arduino.h>

#include "controller_state.h"
#include "flow_curve_dataset.h"

// Non-owning references to mutable firmware state needed by controller and
// command code. Modules receive this context instead of declaring globals.
template <size_t DatasetCapacity> struct FirmwareRuntime {
  ControllerState &controller;
  FlowCurveDataset<DatasetCapacity> &dataset;
  bool &debugEnabled;
  uint32_t &triggerStartedUs;
  uint32_t &preTriggerDelayUs;
  uint32_t &photodetectorDelayUs;
  int32_t &dropletRunsRemaining;
  uint32_t &runCallTimeUs;
};