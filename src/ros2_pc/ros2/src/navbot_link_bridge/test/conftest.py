import sys
from pathlib import Path

# The bridge tests drive the same in-memory firmware as the navbot_link tests.
sys.path.insert(0, str(Path(__file__).resolve().parents[6] / "tests" / "navbot_link"))
