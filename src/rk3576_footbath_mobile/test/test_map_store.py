from pathlib import Path

import pytest

from rk3576_footbath_mobile.map_store import MapStore


def create_map(root: Path, name: str = "old") -> Path:
    root.mkdir(parents=True, exist_ok=True)
    yaml_path = root / f"{name}.yaml"
    yaml_path.write_text(
        f"image: {name}.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n",
        encoding="utf-8",
    )
    (root / f"{name}.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    (root / f"{name}.posegraph.data").write_bytes(b"data")
    (root / f"{name}.posegraph.posegraph").write_bytes(b"graph")
    return yaml_path


def test_catalog_and_group_rename(tmp_path):
    auto_root = tmp_path / "maps"
    manual_root = tmp_path / "manual"
    old = create_map(auto_root)
    store = MapStore((auto_root, manual_root))
    entries = store.entries()
    assert entries[0]["name"] == "old"
    assert entries[0]["complete"]
    assert entries[0]["has_posegraph"]
    renamed = Path(store.rename(old, "客厅_01"))
    assert renamed.name == "客厅_01.yaml"
    assert "image: 客厅_01.pgm" in renamed.read_text(encoding="utf-8")
    for suffix in (".pgm", ".posegraph.data", ".posegraph.posegraph"):
        assert (auto_root / ("客厅_01" + suffix)).is_file()
    assert not old.exists()


def test_delete_moves_complete_group_to_recoverable_trash(tmp_path):
    auto_root = tmp_path / "maps"
    manual_root = tmp_path / "manual"
    yaml_path = create_map(manual_root, "bedroom")
    store = MapStore((auto_root, manual_root))
    trash = Path(store.trash(yaml_path))
    assert trash.parent == manual_root / ".trash"
    assert sorted(path.name for path in trash.iterdir()) == [
        "bedroom.pgm",
        "bedroom.posegraph.data",
        "bedroom.posegraph.posegraph",
        "bedroom.yaml",
    ]
    assert store.paths() == []


def test_rejects_outside_paths_and_unsafe_names(tmp_path):
    roots = (tmp_path / "maps", tmp_path / "manual")
    yaml_path = create_map(roots[0])
    outside = create_map(tmp_path / "outside")
    store = MapStore(roots)
    with pytest.raises(ValueError):
        store.checked_yaml(outside)
    with pytest.raises(ValueError):
        store.rename(yaml_path, "../escape")

def test_saved_pgm_preview_is_read_only_and_oriented(tmp_path):
    auto_root = tmp_path / "maps"
    yaml_path = create_map(auto_root, "preview")
    before = (auto_root / "preview.pgm").read_bytes()
    store = MapStore((auto_root, tmp_path / "manual"))
    data, width, height = store.preview_data(yaml_path)
    assert (width, height) == (1, 1)
    assert data == [100]
