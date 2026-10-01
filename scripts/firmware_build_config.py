from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SKETCH = REPOSITORY_ROOT / "src/ES-02/OllieFOCdrive"
DEFAULT_FQBN = "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default"
DEFAULT_OUTPUT_DIR = REPOSITORY_ROOT / "build/flash"
