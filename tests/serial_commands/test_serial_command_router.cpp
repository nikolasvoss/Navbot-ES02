#include <cassert>
#include <cstddef>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>
#include "../../src/ES-02/OllieFOCdrive/MotorDiagnostic.h"

enum class VerboseMode : unsigned char { nothing, on_request, user_friendly, machine_readable };
constexpr int MAX_COMMAND_LENGTH = 20;
using CommandCallback = void (*)(char *);

class Stream {
 public:
  explicit Stream(std::string input = {}) : input_(std::move(input)) {}
  int available() const { return static_cast<int>(input_.size() - offset_); }
  int read() { return offset_ < input_.size() ? input_[offset_++] : -1; }
  void print(char value) { output_ += value; }
  void print(const char *value) { output_ += value; }
  void println(const char *value) { output_ += value; output_ += '\n'; }
  void feed(const std::string &value) { input_ += value; }
  const std::string &output() const { return output_; }

 private:
  std::string input_;
  std::string output_;
  size_t offset_ = 0;
};

class Commander {
 public:
  Stream *com_port = nullptr;
  VerboseMode verbose = VerboseMode::user_friendly;
  unsigned char decimal_places = 3;
  char eol = '\n';
  bool echo = false;
  int builtins = 0;

  bool isSentinel(char value) const { return value == eol || value == '\r'; }
  void run(char *input) {
    ++builtins;
    if (input[0] == '@' && input[1] != '\n' && input[1] != '\r')
      verbose = static_cast<VerboseMode>(input[1] - '0');
    if (input[0] == '#' && input[1] != '\n' && input[1] != '\r') decimal_places = input[1] - '0';
  }
};

static std::vector<std::string> calls;
static std::vector<bool> parsedMotorCommands;
#define COMMAND_CALLBACK(name) \
  void name(char *input) { calls.emplace_back(#name ":" + std::string(input)); }
void handleM1MotionCommand(char *input) {
  calls.emplace_back("handleM1MotionCommand:" + std::string(input));
  MotorDiagnostic::Request request{};
  parsedMotorCommands.push_back(MotorDiagnostic::parseCurrentRequest(input, request));
}
void handleM1Command(char *input) {
  calls.emplace_back("handleM1Command:" + std::string(input));
  MotorDiagnostic::Request request{};
  parsedMotorCommands.push_back(MotorDiagnostic::parseVoltageRequest(input, request));
}
COMMAND_CALLBACK(zeroBiasServo1)
COMMAND_CALLBACK(zeroBiasServo2)
COMMAND_CALLBACK(zeroBiasServo3)
COMMAND_CALLBACK(zeroBiasServo4)
COMMAND_CALLBACK(setImuLowPassCutoff)
COMMAND_CALLBACK(setMahonyKp)
COMMAND_CALLBACK(setMahonyKi)
COMMAND_CALLBACK(selectDebugLog)
COMMAND_CALLBACK(selectCalibration)
COMMAND_CALLBACK(setYawPid)
COMMAND_CALLBACK(setKvSafetyFactor)
#undef COMMAND_CALLBACK

#include "../../src/ES-02/OllieFOCdrive/SerialCommandRouter.h"
#include "command_table.inc"

int main() {
  Stream output;
  Commander commander;
  commander.com_port = &output;
  SerialCommandRouter router;

  assert(sizeof(commandEntries) / sizeof(commandEntries[0]) == 13);
  const char *expectedCallbacks[] = {"handleM1MotionCommand", "handleM1Command", "zeroBiasServo1", "zeroBiasServo2",
                                    "zeroBiasServo3", "zeroBiasServo4", "setImuLowPassCutoff", "setMahonyKp",
                                    "setMahonyKi", "selectDebugLog", "selectCalibration", "setYawPid",
                                    "setKvSafetyFactor"};
  const char ids[] = "TMABCDZQIKEYJ";
  for (size_t i = 0; ids[i]; ++i) {
    Stream input(std::string(1, ids[i]) + "\n");
    router.run(input, commander, commandEntries);
    assert(calls[i] == std::string(expectedCallbacks[i]) + ":\n");
  }
  assert(calls.size() == 13);
  assert(calls[11] == "setYawPid:\n");
  assert(calls[12] == "setKvSafetyFactor:\n");

  calls.clear();
  Stream yawInput("YP?\nYI2\nYD\nYL\n");
  router.run(yawInput, commander, commandEntries);
  assert((calls == std::vector<std::string>{"setYawPid:P?\n", "setYawPid:I2\n", "setYawPid:D\n", "setYawPid:L\n"}));

  Stream helpInput("?\n");
  router.run(helpInput, commander, commandEntries);
  assert(output.output().find("Y:yawPidSettings\n") != std::string::npos);

  const int builtinCalls = commander.builtins;
  Stream builtinInput("@3\n#5\n");
  router.run(builtinInput, commander, commandEntries);
  assert(commander.builtins == builtinCalls + 2);
  assert(commander.verbose == VerboseMode::machine_readable);
  assert(commander.decimal_places == 5);
  Stream builtinQueries("@\n#\n");
  router.run(builtinQueries, commander, commandEntries);
  assert(commander.builtins == builtinCalls + 4);
  assert(commander.verbose == VerboseMode::machine_readable);
  assert(commander.decimal_places == 5);
  const std::string outputBeforeMachineCommand = output.output();
  Stream machineInput("YI\n");
  router.run(machineInput, commander, commandEntries);
  assert(output.output().substr(outputBeforeMachineCommand.size()) == "Y");
  const std::string outputBeforeMachineHelp = output.output();
  Stream machineHelpInput("?\n");
  router.run(machineHelpInput, commander, commandEntries);
  assert(output.output().substr(outputBeforeMachineHelp.size()).find("?Y:yawPidSettings\n") != std::string::npos);

  calls.clear();
  Stream partialInput("Y");
  router.run(partialInput, commander, commandEntries);
  assert(calls.empty());
  partialInput.feed("P?\n");
  router.run(partialInput, commander, commandEntries);
  assert((calls == std::vector<std::string>{"setYawPid:P?\n"}));

  calls.clear();
  Stream exactCapacityInput("Y" + std::string(18, '2') + "\n");
  router.run(exactCapacityInput, commander, commandEntries);
  assert((calls == std::vector<std::string>{"setYawPid:" + std::string(18, '2') + "\n"}));

  calls.clear();
  Stream malformedCapacityInput("Y" + std::string(18, 'x') + "\n");
  router.run(malformedCapacityInput, commander, commandEntries);
  assert((calls == std::vector<std::string>{"setYawPid:" + std::string(18, 'x') + "\n"}));

  calls.clear();
  Stream crlfInput("YI2\r\n");
  router.run(crlfInput, commander, commandEntries);
  assert((calls == std::vector<std::string>{"setYawPid:I2\r"}));

  calls.clear();
  parsedMotorCommands.clear();
  Stream currentDiagnostic("T 0.02 -0.02 1000\n");
  router.run(currentDiagnostic, commander, commandEntries);
  assert((calls == std::vector<std::string>{"handleM1MotionCommand: 0.02 -0.02 1000\n"}));
  assert(parsedMotorCommands.size() == 1 && parsedMotorCommands[0]);
  calls.clear();
  parsedMotorCommands.clear();
  Stream voltageDiagnostic("M 2 2.0 5000\r\n");
  router.run(voltageDiagnostic, commander, commandEntries);
  assert(parsedMotorCommands.size() == 1 && parsedMotorCommands[0]);
  calls.clear();
  parsedMotorCommands.clear();
  Stream invalidDiagnostic("M 1 nan 1000\n");
  router.run(invalidDiagnostic, commander, commandEntries);
  assert(parsedMotorCommands.size() == 1 && !parsedMotorCommands[0]);

  calls.clear();
  std::string continuousCommands;
  for (int i = 0; i < 30; ++i) continuousCommands += "YI\n";
  Stream continuousInput(continuousCommands);
  router.run(continuousInput, commander, commandEntries);
  assert(calls.size() == SerialCommandRouter::kByteBudgetPerRun / 3);
  assert(continuousInput.available() > 0);
  const size_t callsBeforeSecondRun = calls.size();
  router.run(continuousInput, commander, commandEntries);
  assert(calls.size() > callsBeforeSecondRun);
  while (continuousInput.available()) router.run(continuousInput, commander, commandEntries);
  assert(calls.size() == 30);

  calls.clear();
  Stream overlongInput("Y" + std::string(24, '1') + "\nYI\n");
  router.run(overlongInput, commander, commandEntries);
  assert((calls == std::vector<std::string>{"setYawPid:I\n"}));

  // Commander's 20-character limit: a 19-character T line plus its newline is dispatched, while a
  // 20-character T line plus its newline is discarded whole, so the next line still runs.
  calls.clear();
  parsedMotorCommands.clear();
  Stream maxWidthCurrent("T -0.020 -0.020 200\n");
  router.run(maxWidthCurrent, commander, commandEntries);
  assert((calls == std::vector<std::string>{"handleM1MotionCommand: -0.020 -0.020 200\n"}));
  assert(parsedMotorCommands.size() == 1 && parsedMotorCommands[0]);
  calls.clear();
  parsedMotorCommands.clear();
  Stream overWidthCurrent("T -0.020 -0.020 2000\nYI\n");
  router.run(overWidthCurrent, commander, commandEntries);
  assert((calls == std::vector<std::string>{"setYawPid:I\n"}));
  assert(parsedMotorCommands.empty());

  commander.echo = true;
  commander.verbose = VerboseMode::user_friendly;
  const std::string outputBeforeEcho = output.output();
  calls.clear();
  Stream echoInput("YI\n");
  router.run(echoInput, commander, commandEntries);
  assert(output.output().substr(outputBeforeEcho.size()) == "YI\n");
  assert((calls == std::vector<std::string>{"setYawPid:I\n"}));
  commander.echo = false;

  commander.verbose = VerboseMode::nothing;
  const std::string outputBeforeQuietHelp = output.output();
  Stream quietHelpInput("?\n");
  router.run(quietHelpInput, commander, commandEntries);
  assert(output.output() == outputBeforeQuietHelp);
  commander.echo = true;
  Stream quietEchoInput("YI\n");
  router.run(quietEchoInput, commander, commandEntries);
  assert(output.output() == outputBeforeQuietHelp);
  assert(calls.back() == "setYawPid:I\n");
  commander.echo = false;
  calls.clear();
  Stream unknownInput("X123\n");
  router.run(unknownInput, commander, commandEntries);
  assert(calls.empty());
  puts("serial command router passed");
}
