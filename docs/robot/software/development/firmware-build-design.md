# Firmware build command design

## Problem

Humans and agents need one command for compiling the NavBot sketch, with options for other Arduino sketches. Flashing remains a separate host operation through the configured `navbot_flash` MCP tool. The repository does not pin Arduino CLI, board core, or library versions.

## Usage

The default command is `python3 scripts/build_firmware.py`. It compiles `src/ES-02/OllieFOCdrive` for the documented ESP32-S3 FQBN into `build/flash`. Callers can supply `--sketch`, `--fqbn`, and `--output-dir`. Repeated `--build-property` values and `--clean` cover common Arduino CLI build options. To flash, call `navbot_flash` MCP `flash_firmware` with the absolute build directory from the active worktree; check serial MCP ports, close the serial MCP connection, and keep balance disabled with motor outputs safe.

## Shape

Use one Python CLI with a normalized immutable `BuildRequest`:

```python
@dataclass(frozen=True)
class BuildRequest:
    sketch: Path
    fqbn: str
    output_dir: Path
    build_path: Path
    build_properties: tuple[str, ...]
    clean: bool
```

The parser supplies repository defaults and resolves caller paths. It derives the Arduino CLI working path as a hidden sibling of the output directory, or accepts `--build-path`. Boundary validation checks the sketch directory, at least one `.ino` file, Arduino CLI availability, and directory creation. The compiler runner builds an argument list and forwards Arduino CLI's output and exit status. It never uploads.

Keep the default sketch and FQBN in `scripts/firmware_build_config.py` for the builder. The explicit working path keeps compilation inside the repository, where agents can write, instead of relying on a user-level Arduino cache directory. The script does not install or pin toolchain packages.

## Synthesis decision

Candidate 3's single Python CLI is the base. It matches the existing uploader and keeps build, upload, and board detection separate. Candidate 1's checks for an `.ino` sketch and an available compiler are included. Candidate 2's simple override model is retained as named command-line options. Its Make target was rejected because it adds a Make prerequisite and shell validation for one operation.

## Tradeoffs

- The script adds a Python entry point but no Python package dependency.
- The default output is reusable, so Arduino CLI controls the contents after a failed build.
- Toolchain versions remain environment inputs. The command is consistent, but builds are not bit-reproducible until package versions are pinned.

## Implementation reconciliation

The implementation must keep the `BuildRequest` data shape and default command above. Any change to the FQBN or sketch default must apply to both build and upload through the shared config module.

## Open questions and risks

The build command does not check the locally installed core or library versions. Use the observed working versions below as a reference when comparing build environments.

After a successful compile, the Python build command updates [`firmware-build-environment.json`](../../../../firmware-build-environment.json) in the repository root. It records the FQBN, resolved platform version, and libraries found in Arduino CLI's compilation database. Git history associates the record with source changes when you commit both files. Commit it after you confirm that the firmware balances. The build command does not stage or commit files.

## Observed working versions

On 2026-10-09, the user confirmed that balancing worked with a firmware build using these installed versions:

| Component | Version |
| --- | --- |
| ESP32 core (`esp32:esp32`) | 3.3.12 |
| Simple FOC | 2.4.0 |
| ArduinoJson | 7.4.3 |
| Queue (`cppQueue`) | 2.1 |

The source was based on commit `b198590` with a working-tree change that sets motor 2 to estimated-current torque control. These are observed build environment versions, not build constraints. `scripts/build_firmware.py` uses whichever core and libraries are installed in the local Arduino CLI environment.
