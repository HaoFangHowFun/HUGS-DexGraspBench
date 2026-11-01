"""Source-triangle/convex-proxy diagnostics. No diagnostic repairs are saved to assets."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import manifold3d


def solid(mesh):
    value = manifold3d.Manifold(manifold3d.Mesh64(
        vert_properties=np.array(mesh.vertices, dtype=np.float64, order="C", copy=True),
        tri_verts=np.array(mesh.faces, dtype=np.uint64, order="C", copy=True)))
    if value.status() != manifold3d.Error.NoError:
        raise ValueError(f"Invalid diagnostic solid: {value.status()}")
    return value


def intersection_volume(first, second):
    result = solid(first) ^ solid(second)
    if result.status() != manifold3d.Error.NoError:
        raise ValueError(f"Failed geometry intersection: {result.status()}")
    return abs(float(result.volume()))


def convex_mesh(mesh):
    value = manifold3d.Manifold.hull_points(np.asarray(mesh.vertices))
    if value.status() != manifold3d.Error.NoError or value.is_empty():
        raise ValueError("Cannot construct a valid diagnostic convex hull")
    result = value.to_mesh64()
    return trimesh.Trimesh(vertices=result.vert_properties[:, :3], faces=result.tri_verts, process=False)


def union_mesh(meshes):
    value = solid(meshes[0])
    for mesh in meshes[1:]:
        value = value + solid(mesh)
    if value.status() != manifold3d.Error.NoError:
        raise ValueError(f"Invalid collision-parts union: {value.status()}")
    result = value.to_mesh64()
    return trimesh.Trimesh(vertices=result.vert_properties[:, :3], faces=result.tri_verts, process=False)


def volume(mesh):
    return abs(float(mesh.volume)) if len(mesh.faces) else 0.0


class GeometryAudit:
    def __init__(self, assets, side, manifest, repo):
        sys.path.insert(0, str(assets / "scripts"))
        from validate_models import urdf_fk, transform
        self.fk = urdf_fk
        self.tree = ET.parse(assets / "urdf" / f"{side}_sharpa_wave_with_flange.urdf").getroot()
        self.names = [j.get("name") for j in self.tree.findall("joint") if j.get("type") != "fixed"]
        self.meshes = {}
        self.repairs = {}
        for link in self.tree.findall("link"):
            collision = link.find("collision")
            if collision is None:
                continue
            name = link.get("name")
            mesh = trimesh.load_mesh(assets / "urdf" / collision.find("geometry/mesh").get("filename"))
            if not mesh.is_watertight:
                original_volume = mesh.volume
                original_faces = len(mesh.faces)
                if trimesh.repair.fill_holes(mesh):
                    self.repairs[name] = dict(method="diagnostic triangular/quad hole closure",
                                              added_faces=len(mesh.faces)-original_faces,
                                              relative_volume_change=abs(mesh.volume-original_volume)/abs(original_volume),
                                              source_watertight=False)
                else:
                    hull = convex_mesh(mesh)
                    delta = abs(hull.volume - mesh.volume)/abs(hull.volume)
                    if delta > 1e-6:
                        raise ValueError(f"Open nonconvex diagnostic mesh requires inspection: {name}")
                    self.repairs[name] = dict(method="diagnostic convex closure of nearly convex open mesh",
                                              relative_volume_change=float(delta), source_watertight=False)
                    mesh = hull
            if not mesh.is_winding_consistent:
                mesh.fix_normals(multibody=True)
            origin = collision.find("origin")
            mesh.apply_transform(transform(**(origin.attrib if origin is not None else {})))
            self.meshes[name] = mesh
        parts = [trimesh.load_mesh(repo / part["path"]) for part in manifest["hands"][side]["parts"]]
        self.palm = side + "_hand_C_MC"
        self.palm_proxy = union_mesh(parts)

    def compare(self, first, second, q):
        first, second = first.removeprefix("child-"), second.removeprefix("child-")
        if first not in self.meshes or second not in self.meshes:
            return dict(classification="non-mesh or external contact", source_volume_mm3=None)
        poses = self.fk(self.tree, dict(zip(self.names, q)))
        pair = [self.meshes[name].copy().apply_transform(poses[name]) for name in (first, second)]
        proxies = [(self.palm_proxy if name == self.palm else convex_mesh(self.meshes[name]))
                   .copy().apply_transform(poses[name]) for name in (first, second)]
        # Do not treat an invalid manifold as an empty intersection. Mesh64 avoids
        # collapsing fine STL topology during the default float32 conversion.
        raw = intersection_volume(*pair)*1e9
        hull = intersection_volume(*(convex_mesh(mesh) for mesh in pair))*1e9
        proxy = intersection_volume(*proxies)*1e9
        if raw > .001:
            classification = "source_geometry_overlap"
        elif proxy > .001:
            classification = "convex_approximation_overlap"
        elif hull > .001:
            classification = "single_convex_hull_artifact_removed"
        else:
            classification = "surface_touch_or_numeric_tolerance"
        return dict(classification=classification, source_volume_mm3=raw,
                    single_hull_volume_mm3=hull, current_proxy_volume_mm3=proxy,
                    diagnostic_closures={name: self.repairs[name] for name in (first, second) if name in self.repairs})

    def zero_pairs(self, side):
        return {name: self.compare(side+"_hand_C_MC", side+"_"+name, np.zeros(22))
                for name in ("thumb_MC", "index_MCP_VL", "ring_MCP_VL", "pinky_MC")}
