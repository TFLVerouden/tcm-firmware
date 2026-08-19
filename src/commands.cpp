#include "commands.h"

#include <string.h>

namespace {

struct CommandDefinition {
  const char *prefix;
  CommandId id;
};

// Order matters: more specific command prefixes must appear first.
const CommandDefinition COMMAND_DEFINITIONS[] = {
    {"id?", CommandId::IdQuery},
    {"ver?", CommandId::ProtocolVersionQuery},
    {"S?", CommandId::StatusQuery},
    {"P?", CommandId::ReadTankPressure},
    {"M?", CommandId::ReadNebPressure},
    {"T?", CommandId::ReadTempHumidity},
    {"W?", CommandId::WaitQuery},
    {"X!", CommandId::ClearMemory},
    {"L?", CommandId::DatasetStatus},
    {"D!", CommandId::DropletRun},
    {"B", CommandId::DebugToggle},
    {"V", CommandId::SetValve},
    {"P", CommandId::SetTankPressure},
    {"M", CommandId::SetNebPressure},
    {"O", CommandId::OpenSolenoid},
    {"C", CommandId::CloseSolenoid},
    {"Q", CommandId::Quit},
    {"G", CommandId::TriggerOnce},
    {"A", CommandId::LaserTestToggle},
    {"I", CommandId::LightToggle},
    {"F", CommandId::FanSpeed},
    {"N", CommandId::NebuliserToggle},
    {"W", CommandId::WaitSet},
    {"X", CommandId::ClearLogs},
    {"L", CommandId::LoadDataset},
    {"R", CommandId::Run},
    {"D", CommandId::DropletDetect},
    {"?", CommandId::Help},
};

} // namespace

CommandId parseCommandId(const char *command) {
  for (const CommandDefinition &definition : COMMAND_DEFINITIONS) {
    if (strncmp(command, definition.prefix, strlen(definition.prefix)) == 0) {
      return definition.id;
    }
  }
  return CommandId::Other;
}