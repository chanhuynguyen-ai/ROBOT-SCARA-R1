"""Verify the curated V13 runtime reference and original pinned inputs."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()

def verify(root):
    root = root.resolve()
    manifest = json.loads((root / "CONTENT_MANIFEST.json").read_text(encoding="utf-8"))
    expected = {item["path"]: item for item in manifest["files"]}
    observed = {p.relative_to(root).as_posix() for p in root.rglob("*")
                if p.is_file() and p.name != "CONTENT_MANIFEST.json"
                and "__pycache__" not in p.parts and p.suffix != ".pyc"}
    if set(expected) != observed:
        raise RuntimeError("V13 reference file set changed")
    for name, item in expected.items():
        path = root / name
        if path.stat().st_size != item["size_bytes"] or digest(path) != item["sha256"]:
            raise RuntimeError("V13 reference content changed: " + name)
    controller = root / "controller"
    calibration = json.loads((controller / "R1_V13_CALIBRATION.json").read_text(encoding="utf-8"))
    pinned = {
        root / "scene/R1_V09_GF01_MATED.xml": "original_V09_scene_sha256",
        root / "trajectories/v10.csv": "reference_V10_trace_sha256",
        controller / "R1_V513_ORIGINAL.zip": "original_V513_zip_sha256",
        controller / "R1_V13_TRAJECTORY_2622.csv": "proposed_csv_sha256",
    }
    for path, key in pinned.items():
        if digest(path) != calibration[key]:
            raise RuntimeError("Original source pin changed: " + path.name)
    with (controller / "R1_V13_TRAJECTORY_2622.csv").open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 2622 or len({r["phase"] for r in rows}) != 13:
        raise RuntimeError("V13 trajectory shape changed")
    if abs(float(rows[-1]["t_s"]) - 65.525) > 1e-8:
        raise RuntimeError("V13 trajectory timebase changed")
    scene = root / "scene/R1_V09_GF01_MATED.xml"
    xml = ET.parse(scene).getroot()
    meshes = [node.get("file") for node in xml.iter("mesh") if node.get("file")]
    if len(meshes) != 107 or any(not (scene.parent / name).is_file() for name in meshes):
        raise RuntimeError("V09 scene mesh dependencies are incomplete")
    print(f"V13_REFERENCE_VERIFY_PASS: {len(expected)} files, {len(meshes)} meshes, 2622 frames", flush=True)
    return {"status": "PASS", "files": len(expected), "mesh_references": len(meshes),
            "trajectory_frames": len(rows), "original_source_pins_preserved": True}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    verify(args.root)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
