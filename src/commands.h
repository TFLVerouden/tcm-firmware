#pragma once

#include <Arduino.h>

// Internal identifiers used by the command dispatcher after parsing a serial
// command line. The protocol parser keeps prefix precedence in commands.cpp.
enum class CommandId : uint8_t {
  IdQuery,
  ProtocolVersionQuery,
  DebugToggle,
  StatusQuery,
  Help,
  SetValve,
  SetTankPressure,
  SetNebPressure,
  OpenSolenoid,
  CloseSolenoid,
  Quit,
  TriggerOnce,
  LaserTestToggle,
  LightToggle,
  FanSpeed,
  NebuliserToggle,
  ReadTankPressure,
  ReadNebPressure,
  ReadTempHumidity,
  WaitSet,
  WaitQuery,
  ClearMemory,
  ClearLogs,
  LoadDataset,
  DatasetStatus,
  Run,
  DropletRun,
  DropletDetect,
  Other
};

CommandId parseCommandId(const char *command);