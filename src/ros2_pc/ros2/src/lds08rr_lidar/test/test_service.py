from lds08rr_lidar.service import copy_saved_map


def test_bench_copies_only_the_map_and_keepout_files(tmp_path):
    live = tmp_path / "live"
    live.mkdir()
    for name in ("map.posegraph", "map.data", "map.pgm", "map.yaml", "keepout.pgm", "keepout.yaml", "slam.log"):
        (live / name).write_text(name)
    (live / "undo").mkdir()
    (live / "undo" / "snapshot-1.posegraph").write_text("undo")
    bench = tmp_path / "bench" / "slam"
    bench.mkdir(parents=True)
    (bench / "map.yaml").write_text("old bench copy")
    (bench / "nav.log").write_text("old log")

    assert copy_saved_map(live, bench) == bench

    assert sorted(path.name for path in bench.iterdir()) == [
        "keepout.pgm", "keepout.yaml", "map.data", "map.pgm", "map.posegraph", "map.yaml",
    ]
    assert (bench / "map.yaml").read_text() == "map.yaml"
    assert sorted(path.name for path in live.iterdir()) == [
        "keepout.pgm", "keepout.yaml", "map.data", "map.pgm", "map.posegraph", "map.yaml", "slam.log", "undo",
    ]
    assert (live / "undo" / "snapshot-1.posegraph").read_text() == "undo"


def test_bench_without_a_saved_map_starts_empty(tmp_path):
    bench = tmp_path / "bench"

    assert copy_saved_map(tmp_path / "missing", bench) == bench

    assert list(bench.iterdir()) == []
