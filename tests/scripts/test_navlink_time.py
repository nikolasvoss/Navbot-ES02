"""`t_us` handling in navlink captures: wrap at 2^32, robot restart, corrupt values, header problems."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from analyze_drive_trace import analyze  # noqa: E402
from summarize_balance_trace import read_trace, summarize  # noqa: E402

FIXTURES = ROOT / "tests" / "scripts" / "fixtures"


def test_restart_mid_capture_continues_the_timeline_without_a_phantom_wrap():
    trace = read_trace(FIXTURES / "navlink-restart.csv")
    assert [row["time_ms"] for _, row in trace.rows] == [2998000.0, 2999000.0, 3000000.0, 3000005.0, 3001000.0]
    assert trace.malformed == 0


def test_restart_is_reported_in_the_summary():
    report = "\n".join(summarize(FIXTURES / "navlink-restart.csv"))
    assert "- Zeitspanne der gültigen Datensätze: **3.00 s**; Medianabstand zwischen gültigen Datensätzen: **997.5 ms**" in report
    assert ("- Warnung: `t_us` fiel 1-mal auf einen kleinen Wert zurück (Roboter-Neustart); "
            "die Zeitachse läuft danach weiter, die Ausfallzeit fehlt in der Zeitspanne") in report


def test_one_corrupt_t_us_is_malformed_and_leaves_span_and_median_unchanged():
    report = "\n".join(summarize(FIXTURES / "navlink-corrupt-t-us.csv"))
    assert "gültige Datensätze: **5**; beschädigte/ungültige Datensätze: **1**" in report
    assert "- Zeitspanne der gültigen Datensätze: **5.00 s**; Medianabstand zwischen gültigen Datensätzen: **1000.0 ms**" in report
    assert "Warnung" not in report


def test_real_wrap_is_still_unwrapped(tmp_path):
    log = tmp_path / "wrap.csv"
    log.write_text("t_us,x\n4294960000,1\n4294966000,2\n4000,3\n10000,4\n", encoding="utf-8")
    assert [row["time_ms"] for _, row in read_trace(log).rows] == [4294960.0, 4294966.0, 4294971.296, 4294977.296]


def test_drive_analysis_shares_the_restart_handling():
    result = analyze(FIXTURES / "drive-navlink-restart.csv")
    assert result["duration_s"] == 4.0
