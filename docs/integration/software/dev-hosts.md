# Development hosts

Two hosts take part: the desktop PC (`niko-desktop`, SSH `desktop`) and the ros2_pc (`cm5`, SSH `cm5`). Which of them has the Arduino toolchain and which has the robot's USB cable can change; `scripts/deploy.py` checks on every run. The robot's UART link is wired to the ros2_pc, whose multiplexer service shares it on `127.0.0.1:7650`.
