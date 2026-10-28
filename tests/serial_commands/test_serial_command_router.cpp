#include <cassert>
#include <cstddef>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

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

#include "../../src/ES-02/OllieFOCdrive/SerialCommandRouter.h"

static std::vector<std::string> calls;
void selectDebugLog(char *input) { calls.emplace_back("selectDebugLog:" + std::string(input)); }

// The sketch's command table, extracted by run_host_test.sh: the K selector is the only letter command left.
#include "command_table.inc"

int main() {
  Stream output;
  Commander commander;
  commander.com_port = &output;
  SerialCommandRouter router;

  assert(sizeof(commandEntries) / sizeof(commandEntries[0]) == 1);
  Stream input("K\n");
  router.run(input, commander, commandEntries);
  assert((calls == std::vector<std::string>{"selectDebugLog:\n"}));

  // The removed letters are unknown: they produce no call and no help text.
  for (const char *removed : {"T 0.02 0 1000", "M 1 0.2 500", "Q0.4", "I0.001", "Z25", "A1.5", "D0", "E1", "T STOP"}) {
    calls.clear();
    Stream removedInput(std::string(removed) + "\n");
    router.run(removedInput, commander, commandEntries);
    assert(calls.empty());
  }

  calls.clear();
  Stream multiLineInput("KP?\nKI2\nKD\nKL\n");
  router.run(multiLineInput, commander, commandEntries);
  assert((calls == std::vector<std::string>{"selectDebugLog:P?\n", "selectDebugLog:I2\n", "selectDebugLog:D\n", "selectDebugLog:L\n"}));

  Stream helpInput("?\n");
  router.run(helpInput, commander, commandEntries);
  assert(output.output().find("K:my Select\n") != std::string::npos);
  assert(output.output().find("T:") == std::string::npos && output.output().find("E:") == std::string::npos);

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
  Stream machineInput("KI\n");
  router.run(machineInput, commander, commandEntries);
  assert(output.output().substr(outputBeforeMachineCommand.size()) == "K");
  const std::string outputBeforeMachineHelp = output.output();
  Stream machineHelpInput("?\n");
  router.run(machineHelpInput, commander, commandEntries);
  assert(output.output().substr(outputBeforeMachineHelp.size()).find("?K:my Select\n") != std::string::npos);

  calls.clear();
  Stream partialInput("K");
  router.run(partialInput, commander, commandEntries);
  assert(calls.empty());
  partialInput.feed("P?\n");
  router.run(partialInput, commander, commandEntries);
  assert((calls == std::vector<std::string>{"selectDebugLog:P?\n"}));

  calls.clear();
  Stream exactCapacityInput("K" + std::string(18, '2') + "\n");
  router.run(exactCapacityInput, commander, commandEntries);
  assert((calls == std::vector<std::string>{"selectDebugLog:" + std::string(18, '2') + "\n"}));

  calls.clear();
  Stream malformedCapacityInput("K" + std::string(18, 'x') + "\n");
  router.run(malformedCapacityInput, commander, commandEntries);
  assert((calls == std::vector<std::string>{"selectDebugLog:" + std::string(18, 'x') + "\n"}));

  calls.clear();
  Stream crlfInput("KI2\r\n");
  router.run(crlfInput, commander, commandEntries);
  assert((calls == std::vector<std::string>{"selectDebugLog:I2\r"}));

  calls.clear();
  std::string continuousCommands;
  for (int i = 0; i < 30; ++i) continuousCommands += "KI\n";
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
  Stream overlongInput("K" + std::string(24, '1') + "\nKI\n");
  router.run(overlongInput, commander, commandEntries);
  assert((calls == std::vector<std::string>{"selectDebugLog:I\n"}));

  // Commander's 20-character limit: a 19-character K line plus its newline is dispatched, while a 20-character K line
  // plus its newline is discarded whole, so the next line still runs.
  calls.clear();
  Stream maxWidth("K" + std::string(18, '1') + "\nKI\n");
  router.run(maxWidth, commander, commandEntries);
  assert((calls == std::vector<std::string>{"selectDebugLog:" + std::string(18, '1') + "\n", "selectDebugLog:I\n"}));
  calls.clear();
  Stream overWidth("K" + std::string(19, '1') + "\nKI\n");
  router.run(overWidth, commander, commandEntries);
  assert((calls == std::vector<std::string>{"selectDebugLog:I\n"}));

  commander.echo = true;
  commander.verbose = VerboseMode::user_friendly;
  const std::string outputBeforeEcho = output.output();
  calls.clear();
  Stream echoInput("KI\n");
  router.run(echoInput, commander, commandEntries);
  assert(output.output().substr(outputBeforeEcho.size()) == "KI\n");
  assert((calls == std::vector<std::string>{"selectDebugLog:I\n"}));
  commander.echo = false;

  commander.verbose = VerboseMode::nothing;
  const std::string outputBeforeQuietHelp = output.output();
  Stream quietHelpInput("?\n");
  router.run(quietHelpInput, commander, commandEntries);
  assert(output.output() == outputBeforeQuietHelp);
  commander.echo = true;
  Stream quietEchoInput("KI\n");
  router.run(quietEchoInput, commander, commandEntries);
  assert(output.output() == outputBeforeQuietHelp);
  assert(calls.back() == "selectDebugLog:I\n");
  commander.echo = false;
  calls.clear();
  Stream unknownInput("X123\n");
  router.run(unknownInput, commander, commandEntries);
  assert(calls.empty());
  puts("serial command router passed");
}
