# Firmware build command design

## Problem

Humans and agents need one command for compiling the NavBot sketch, with options for other Arduino sketches. The repository already uses Python for the upload command, and flashing must remain a separate operation. The repository does not pin Arduino CLI, board core, or library versions.

## Usage

The default command is `python3 scripts/build_firmware.py`. It compiles `src/ES-02/OllieFOCdrive` for the documented ESP32-S3 FQBN into `build/flash`. Callers can supply `--sketch`, `--fqbn`, and `--output-dir`. Repeated `--build-property` values and `--clean` cover common Arduino CLI build options. Upload remains `python3 scripts/upload_firmware.py build/flash`.

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

Keep the default sketch and FQBN in `scripts/firmware_build_config.py` so the builder and upload helper use the same values. The explicit working path keeps compilation inside the repository, where agents can write, instead of relying on a user-level Arduino cache directory. The script does not install or pin toolchain packages.

## Synthesis decision

Candidate 3's single Python CLI is the base. It matches the existing uploader and keeps build, upload, and board detection separate. Candidate 1's checks for an `.ino` sketch and an available compiler are included. Candidate 2's simple override model is retained as named command-line options. Its Make target was rejected because it adds a Make prerequisite and shell validation for one operation.

## Tradeoffs

- The script adds a Python entry point but no Python package dependency.
- The default output is reusable, so Arduino CLI controls the contents after a failed build.
- Toolchain versions remain environment inputs. The command is consistent, but builds are not bit-reproducible until package versions are pinned.

## Implementation reconciliation

The implementation must keep the `BuildRequest` data shape and default command above. Any change to the FQBN or sketch default must apply to both build and upload through the shared config module.

## Open questions and risks

The documented ESP32 core version differs from the version installed in the inspected environment. The build command must report compile results without claiming a pinned core or library version.
