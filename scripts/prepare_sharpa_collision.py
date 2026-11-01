"""Build Bench-only convex palm parts; leave canonical URDFs and meshes untouched."""
import hashlib
import importlib.metadata
import json
from pathlib import Path

import trimesh
import yaml
from sharpa_geometry import solid, union_mesh

REPO = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    config = yaml.safe_load((REPO / "config/sharpa_validation.yaml").read_text())
    assets = (REPO / config["asset_root"]).resolve()
    settings = config["palm_decomposition"]
    output = REPO / settings["directory"]
    records = {}
    for side in ("right", "left"):
        source = assets / f"meshes/{side}/{side}_hand_C_MC.STL"
        mesh = trimesh.load_mesh(source)
        parts = mesh.convex_decomposition(**settings["parameters"])
        destination = output / side
        destination.mkdir(parents=True, exist_ok=True)
        # This directory contains generated parts only; drop stale parts on rebuild.
        for stale in destination.glob("palm_*.obj"):
            stale.unlink()
        files = []
        for index, part in enumerate(parts):
            path = destination / f"palm_{index:02d}.obj"
            part.export(path)
            files.append({"path": str(path.relative_to(REPO)), "sha256": digest(path)})
        # Measure fidelity of the union, not the sum of overlapping part volumes.
        union = union_mesh(parts)
        missing = solid(mesh) - solid(union)
        extra = solid(union) - solid(mesh)
        volume = lambda value: abs(float(value.volume)) if len(value.faces) else 0.0
        records[side] = dict(
            source=str(source.relative_to(assets)), source_sha256=digest(source),
            parts=files, source_volume_m3=volume(mesh), union_volume_m3=volume(union),
            missing_volume_fraction=abs(float(missing.volume()))/volume(mesh),
            added_volume_fraction=abs(float(extra.volume()))/volume(mesh),
        )
        print(f"{side}: {len(parts)} palm parts; missing={records[side]['missing_volume_fraction']:.3%}, "
              f"added={records[side]['added_volume_fraction']:.3%}", flush=True)
    manifest = dict(parameters=settings["parameters"], hands=records,
                    versions={name: importlib.metadata.version(name) for name in ("trimesh", "vhacdx", "manifold3d")})
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
