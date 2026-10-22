#pragma once

#include <cstring>

struct SerialCommandEntry {
  char id;
  CommandCallback callback;
  const char *label;
};

class SerialCommandRouter {
 public:
  static constexpr size_t kByteBudgetPerRun = 64;

  template <size_t Count>
  void run(Stream &input, Commander &commander, const SerialCommandEntry (&commands)[Count]) {
    size_t processed = 0;
    while (input.available() && processed < kByteBudgetPerRun) {
      const char ch = static_cast<char>(input.read());
      ++processed;
      if (commander.echo && commander.com_port && commander.verbose != VerboseMode::nothing)
        commander.com_port->print(ch);
      if (discarding_) {
        if (commander.isSentinel(ch)) discarding_ = false;
        continue;
      }

      received_[receivedCount_++] = ch;

      if (commander.isSentinel(ch)) {
        dispatch(received_, commander, commands);
        memset(received_, 0, sizeof(received_));
        receivedCount_ = 0;
      }
      if (receivedCount_ >= MAX_COMMAND_LENGTH) {
        memset(received_, 0, sizeof(received_));
        receivedCount_ = 0;
        discarding_ = true;
      }
    }
  }

 private:
  template <size_t Count>
  static void dispatch(char *input, Commander &commander, const SerialCommandEntry (&commands)[Count]) {
    const char id = input[0];
    if (id == '@' || id == '#') {
      commander.run(input);
      return;
    }
    if (id == '?') {
      printHelp(commander, commands);
      return;
    }
    for (const SerialCommandEntry &entry : commands) {
      if (entry.id != id) continue;
      if (commander.verbose == VerboseMode::machine_readable && commander.com_port)
        commander.com_port->print(id);
      entry.callback(&input[1]);
      return;
    }
  }

  template <size_t Count>
  static void printHelp(Commander &commander, const SerialCommandEntry (&commands)[Count]) {
    if (!commander.com_port || commander.verbose == VerboseMode::nothing) return;
    for (const SerialCommandEntry &entry : commands) {
      if (commander.verbose == VerboseMode::machine_readable) commander.com_port->print('?');
      commander.com_port->print(entry.id);
      commander.com_port->print(':');
      commander.com_port->println(entry.label ? entry.label : "");
    }
  }

  char received_[MAX_COMMAND_LENGTH + 1]{};
  size_t receivedCount_ = 0;
  bool discarding_ = false;
};
