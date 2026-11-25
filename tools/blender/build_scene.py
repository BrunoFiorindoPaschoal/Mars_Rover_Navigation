"""Blender scene for the hero video: recorded Nav2 run on Jezero terrain (Phase 9).

Input is the folder written by tools/export_usd.py (rover.usdc, terrain.obj, texture.png, paths.npz).
Run with Blender 5.3 on Windows, headless:

    blender -b -P tools/blender/build_scene.py -- --export <dir> --save hero.blend
    blender -b -P tools/blender/build_scene.py -- --export <dir> --still 400 --out still.png --engine eevee
    blender -b -P tools/blender/build_scene.py -- --export <dir> --anim --out frames/ --engine cycles

What it builds:
    terrain + sand  from terrain.obj (the sim's own hfields), HiRISE ortho tinted Mars-red + fine noise
    rover + rocks   imported from the MuJoCo USD export (animated by a Transform Cache), new materials
    plan            the live Nav2 /plan as a glowing cyan tube: each replan replaces the previous one
    track           the true driven path (ground truth) drawn behind the rover
    goal            a flag at the goal
    sky + sun       butterscotch gradient sky, low warm sun for long shadows
    three cameras   wide drone shot -> low tracking shot beside the wheels -> close chase that rises
                    into an overview of the whole route, cut by markers
"""
import argparse
import math
import pathlib
import sys

import bmesh
import bpy
import numpy as np
from mathutils import Vector, noise

SHOTS = (0.38, 0.75)  # cut points as fractions of the run: wide | tracking | chase + overview
TRACK_SIDE = 1.0      # tracking camera on the left of travel (+1) or right (-1); rocks_01's boulders are on the right
OVERVIEW_S = 4.0      # last seconds: the chase camera rises to show the whole route


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    p = argparse.ArgumentParser()
    p.add_argument("--export", type=pathlib.Path, required=True)
    p.add_argument("--save", type=pathlib.Path)
    p.add_argument("--engine", choices=("eevee", "cycles"), default="eevee")
    p.add_argument("--samples", type=int, default=64)
    p.add_argument("--res-pct", type=int, default=100)
    p.add_argument("--still", type=int, help="render this one frame to --out")
    p.add_argument("--anim", action="store_true", help="render the frame range to --out (PNG sequence)")
    p.add_argument("--frames", help="start:end, default the whole run")
    p.add_argument("--out", type=pathlib.Path)
    return p.parse_args(argv)


# ---------- small node helpers ----------

def node(tree, kind, **props):
    n = tree.nodes.new(kind)
    for k, v in props.items():
        setattr(n, k, v)
    return n


def sock(sockets, name, kind=None):
    """Socket by name (and type, for nodes like Mix that have one 'A' per data type)."""
    return next(s for s in sockets if s.name == name and (kind is None or s.type == kind) and s.enabled)


def ramp(tree, stops):
    r = node(tree, "ShaderNodeValToRGB")
    els = r.color_ramp.elements
    els[0].position, els[0].color = stops[0][0], (*stops[0][1], 1)
    els[1].position, els[1].color = stops[-1][0], (*stops[-1][1], 1)
    for pos, col in stops[1:-1]:
        els.new(pos).color = (*col, 1)
    return r


def multiply(tree, a, b):
    m = node(tree, "ShaderNodeMix", data_type="RGBA", blend_type="MULTIPLY")
    sock(m.inputs, "Factor", "VALUE").default_value = 1.0
    tree.links.new(a, sock(m.inputs, "A", "RGBA"))
    tree.links.new(b, sock(m.inputs, "B", "RGBA"))
    return sock(m.outputs, "Result", "RGBA")


def new_material(name, color=(0.8, 0.8, 0.8), roughness=0.6, metallic=0.0, emission=None):
    mat = bpy.data.materials.new(name)
    t = mat.node_tree
    bsdf = t.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*color, 1)
    bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["Metallic"].default_value = metallic
    if emission:
        bsdf.inputs["Emission Color"].default_value = (*emission[0], 1)
        bsdf.inputs["Emission Strength"].default_value = emission[1]
    return mat, t, bsdf


def noise_detail(t, scale, lo, hi):
    """Object-space noise as a 0..1 colour multiplier between lo and hi: grain the 25 cm/px ortho lacks."""
    coord = node(t, "ShaderNodeTexCoord")
    noise = node(t, "ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value, noise.inputs["Detail"].default_value = scale, 8.0
    t.links.new(coord.outputs["Object"], noise.inputs["Vector"])
    r = ramp(t, [(0.3, (lo,) * 3), (0.7, (hi,) * 3)])
    t.links.new(noise.outputs["Fac"], r.inputs["Fac"])
    return coord, noise, r.outputs["Color"]


def bump(t, bsdf, height, strength, distance=0.05):
    b = node(t, "ShaderNodeBump")
    b.inputs["Strength"].default_value, b.inputs["Distance"].default_value = strength, distance
    t.links.new(height, b.inputs["Height"])
    t.links.new(b.outputs["Normal"], bsdf.inputs["Normal"])


# ---------- scene parts ----------

def terrain_materials(texture):
    regolith, t, bsdf = new_material("Regolith", roughness=0.95)
    img = node(t, "ShaderNodeTexImage", interpolation="Cubic")
    img.image = bpy.data.images.load(str(texture))
    # HiRISE is greyscale: map brightness onto a Mars palette (not true colour, said in the credits)
    tint = ramp(t, [(0.0, (0.16, 0.07, 0.035)), (0.55, (0.55, 0.28, 0.14)), (1.0, (0.92, 0.62, 0.42))])
    t.links.new(img.outputs["Color"], tint.inputs["Fac"])
    _, noise, grain = noise_detail(t, 6.0, 0.8, 1.05)
    t.links.new(multiply(t, tint.outputs["Color"], grain), bsdf.inputs["Base Color"])
    fine = node(t, "ShaderNodeTexNoise")
    fine.inputs["Scale"].default_value, fine.inputs["Detail"].default_value = 60.0, 10.0
    t.links.new(noise.inputs["Vector"].links[0].from_socket, fine.inputs["Vector"])
    bump(t, bsdf, fine.outputs["Fac"], 0.35)

    sand, t, bsdf = new_material("Sand", color=(0.24, 0.115, 0.055), roughness=1.0)  # renders close to the regolith
    coord = node(t, "ShaderNodeTexCoord")
    wave = node(t, "ShaderNodeTexWave", wave_type="BANDS")  # wind ripples
    wave.inputs["Scale"].default_value, wave.inputs["Distortion"].default_value = 4.0, 6.0
    t.links.new(coord.outputs["Object"], wave.inputs["Vector"])
    bump(t, bsdf, wave.outputs["Fac"], 0.08)
    return regolith, sand


def import_terrain(export, regolith, sand):
    bpy.ops.wm.obj_import(filepath=str(export / "terrain.obj"), forward_axis="Y", up_axis="Z")
    for ob in bpy.context.selected_objects:
        ob.data.materials.clear()
        ob.data.materials.append(sand if ob.name.startswith("sand") else regolith)
        for poly in ob.data.polygons:
            poly.use_smooth = True


def lumpy(mesh, seed, cuts=3, amount=0.25):
    """Turn a smooth MuJoCo ellipsoid into a rock: subdivide and push vertices out by 3D noise.

    Done on the mesh data, not with Subsurf/Displace modifiers: a Displace texture on a
    USD Transform-Cache object crashes Blender 5.3 at frame change.
    """
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bmesh.ops.subdivide_edges(bm, edges=bm.edges[:], cuts=cuts, use_grid_fill=True)
    size = min(Vector(v.co).length for v in bm.verts) or 0.1
    offset = Vector((seed * 7.3, seed * 3.1, seed * 5.7))
    bm.normal_update()
    for v in bm.verts:
        p = v.co / size + offset
        v.co += v.normal * size * amount * (noise.noise(p * 1.2) + 0.5 * noise.noise(p * 3.0))
    bm.to_mesh(mesh)
    bm.free()
    for poly in mesh.polygons:
        poly.use_smooth = True


def wheel_material():
    """Dark tyre with six light/dark sectors around the axle, so the wheel spin reads on camera."""
    mat, t, bsdf = new_material("Wheel", (0.05, 0.05, 0.05), 0.8)
    coord = node(t, "ShaderNodeTexCoord")
    sep, comb = node(t, "ShaderNodeSeparateXYZ"), node(t, "ShaderNodeCombineXYZ")
    t.links.new(coord.outputs["Object"], sep.inputs[0])
    t.links.new(sep.outputs["X"], comb.inputs["X"])  # wheel axle is the mesh's local y: angle in the x-z plane
    t.links.new(sep.outputs["Z"], comb.inputs["Y"])
    angle = node(t, "ShaderNodeTexGradient", gradient_type="RADIAL")
    t.links.new(comb.outputs[0], angle.inputs["Vector"])
    times = node(t, "ShaderNodeMath", operation="MULTIPLY")
    times.inputs[1].default_value = 6.0
    t.links.new(angle.outputs["Fac"], times.inputs[0])
    frac = node(t, "ShaderNodeMath", operation="FRACT")
    t.links.new(times.outputs[0], frac.inputs[0])
    sectors = ramp(t, [(0.0, (0.04, 0.04, 0.04)), (0.5, (0.3, 0.28, 0.25))])
    sectors.color_ramp.interpolation = "CONSTANT"
    t.links.new(frac.outputs[0], sectors.inputs["Fac"])
    t.links.new(sectors.outputs["Color"], bsdf.inputs["Base Color"])
    return mat


def import_rover(export):
    bpy.ops.wm.usd_import(filepath=str(export / "rover.usdc"), import_lights=False, import_cameras=False,
                          create_world_material=False, set_frame_range=False)
    objs = list(bpy.context.selected_objects)
    for ob in objs:  # MuJoCo sites (imu, lidar) come through as tiny meshes
        if "_imu_" in ob.name or "_lidar_id" in ob.name:
            bpy.data.objects.remove(ob)
    objs = [o for o in bpy.data.objects if o.type == "MESH" and "_id" in o.name]

    chassis = new_material("Chassis", (0.85, 0.84, 0.8), 0.45)[0]
    panel = new_material("SolarPanel", (0.02, 0.05, 0.15), 0.15, 0.6)[0]
    wheel = wheel_material()
    metal = new_material("Metal", (0.5, 0.5, 0.52), 0.35, 1.0)[0]
    rock, t, bsdf = new_material("Rock", (0.26, 0.15, 0.09), 0.85)
    _, noise, var = noise_detail(t, 3.0, 0.6, 1.2)
    base = node(t, "ShaderNodeRGB")
    base.outputs[0].default_value = (0.26, 0.15, 0.09, 1)
    t.links.new(multiply(t, base.outputs[0], var), bsdf.inputs["Base Color"])
    bump(t, bsdf, noise.outputs["Fac"], 0.5)

    rover = []
    for k, ob in enumerate(objs):
        n = ob.name
        if "rock_" in n or "boulder_" in n:
            ob.data.materials.clear()
            ob.data.materials.append(rock)
            lumpy(ob.data, seed=k)
            continue
        mat = (panel if "solar" in n else wheel if "wheel" in n else metal if ("mast" in n or "axle" in n or "lidar" in n)
               else chassis)
        ob.data.materials.clear()
        ob.data.materials.append(mat)
        if "wheel" in n:  # smooth the tread, keep the hub faces flat so the facets show the spin
            for poly in ob.data.polygons:
                poly.use_smooth = abs(poly.normal.y) < 0.5
        rover.append(ob)
    return rover


def poly_curve(name, pts, radius, mat):
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions, cu.bevel_depth, cu.bevel_resolution = "3D", radius, 3
    cu.use_fill_caps = True
    sp = cu.splines.new("POLY")
    sp.points.add(len(pts) - 1)
    sp.points.foreach_set("co", np.column_stack([pts, np.ones(len(pts))]).astype(np.float32).ravel())
    ob = bpy.data.objects.new(name, cu)
    bpy.context.collection.objects.link(ob)
    cu.materials.append(mat)
    return ob


def reveal(curve, f0, f1):
    """Grow a curve from 0 to 100 % of its points between frames f0 and f1, one key per frame.

    Two end keys are not enough: Blender 5.3 makes them Bezier (eased) despite the LINEAR keyframe
    preference, and the track then lags the rover in the first half and runs ahead of it in the second.
    """
    cu = curve.data
    for f in range(f0, f1 + 1):
        cu.bevel_factor_end = (f - f0) / (f1 - f0)
        cu.keyframe_insert("bevel_factor_end", frame=f)


def add_paths(paths, n):
    """Each Nav2 plan is its own curve, shown from the frame it was published until the next replan."""
    plan_mat = new_material("Plan", (0.1, 0.8, 1.0), 0.3, emission=((0.05, 0.6, 1.0), 1.5))[0]
    track_mat = new_material("Track", (1.0, 0.95, 0.9), 0.5, emission=((1.0, 0.9, 0.8), 0.5))[0]
    ends = np.cumsum(paths["plans_len"])
    plans = []  # (first frame, points)
    for f0, a, b in zip(paths["plans_frame"], ends - paths["plans_len"], ends):
        pts = paths["plans_xyz"][a:b]
        length = np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1).sum()
        # Hybrid-A* can return a turn-around loop in the last half metre to meet the goal heading;
        # on screen it reads as a glitch, so keep the previous plan up instead
        if f0 > n or length > 2 * np.linalg.norm(pts[-1, :2] - pts[0, :2]) + 0.5:
            continue
        if plans and plans[-1][0] == f0:  # replaced within the same frame
            plans.pop()
        plans.append((int(f0), pts))
    assert plans, "no Nav2 plan falls inside the video"
    for k, (f0, pts) in enumerate(plans):
        f1 = plans[k + 1][0] if k + 1 < len(plans) else n + 1
        ob = poly_curve(f"NavPlan_{k:02d}", pts, 0.025, plan_mat)
        for f, hide in ((1, True), (f0, False), (f1, True)):
            ob.hide_render = ob.hide_viewport = hide
            ob.keyframe_insert("hide_render", frame=f)
            ob.keyframe_insert("hide_viewport", frame=f)
    track = poly_curve("TrueTrack", paths["track_xyz"], 0.012, track_mat)
    reveal(track, 1, n)


def goal_flag(xyz, side):
    """Pole and pennant at the goal; the pennant faces sideways so the cameras behind the rover see it."""
    pole_mat = new_material("FlagPole", (0.8, 0.8, 0.8), 0.4, 1.0)[0]
    flag_mat = new_material("Flag", (1.0, 0.2, 0.05), 0.6, emission=((1.0, 0.2, 0.05), 1.0))[0]
    base = Vector(xyz)
    bpy.ops.mesh.primitive_cylinder_add(radius=0.02, depth=1.2, location=base + Vector((0, 0, 0.6)))
    bpy.context.object.name = "GoalPole"
    bpy.context.object.data.materials.append(pole_mat)
    top = base + Vector((0, 0, 1.2))
    mesh = bpy.data.meshes.new("GoalFlag")
    mesh.from_pydata([top, top - Vector((0, 0, 0.35)), top + 0.45 * side - Vector((0, 0, 0.175))], [], [(0, 1, 2)])
    mesh.materials.append(flag_mat)
    bpy.context.collection.objects.link(bpy.data.objects.new("GoalFlag", mesh))


def sky_and_sun(plan_dir):
    world = bpy.data.worlds.new("MarsSky")
    bpy.context.scene.world = world
    t = world.node_tree
    bg = t.nodes["Background"]
    coord = node(t, "ShaderNodeTexCoord")
    sep = node(t, "ShaderNodeSeparateXYZ")
    t.links.new(coord.outputs["Generated"], sep.inputs[0])  # world 'Generated' = view direction
    sky = ramp(t, [(0.0, (0.75, 0.5, 0.32)), (0.12, (0.62, 0.42, 0.28)), (0.6, (0.32, 0.22, 0.17))])
    t.links.new(sep.outputs["Z"], sky.inputs["Fac"])
    t.links.new(sky.outputs["Color"], bg.inputs["Color"])
    bg.inputs["Strength"].default_value = 0.6

    sun = bpy.data.lights.new("Sun", "SUN")
    sun.energy, sun.color, sun.angle = 4.0, (1.0, 0.88, 0.75), math.radians(0.35)  # small sun disk from Mars
    ob = bpy.data.objects.new("Sun", sun)
    bpy.context.collection.objects.link(ob)
    # low sun from the left of the drive direction: long shadows across the tracking shot
    heading = math.atan2(plan_dir.y, plan_dir.x)
    ob.rotation_euler = (math.radians(68), 0, heading - math.radians(70) + math.pi / 2)


def keyframe_path(ob, xyz, frames=None):
    """Location keys: one per frame from frame 1, or at the given frames."""
    for f, p in zip(frames or range(1, len(xyz) + 1), xyz):
        ob.location = p
        ob.keyframe_insert("location", frame=f)


def smooth(xyz, window):
    pad = np.pad(xyz, ((window, window), (0, 0)), mode="edge")
    kernel = np.ones(2 * window + 1) / (2 * window + 1)
    return np.stack([np.convolve(pad[:, k], kernel, "valid") for k in range(3)], axis=1)


def camera(name, lens, target):
    cam = bpy.data.cameras.new(name)
    cam.lens, cam.clip_start, cam.clip_end = lens, 0.05, 500
    ob = bpy.data.objects.new(name, cam)
    bpy.context.collection.objects.link(ob)
    c = ob.constraints.new("TRACK_TO")
    c.target, c.track_axis, c.up_axis = target, "TRACK_NEGATIVE_Z", "UP_Y"
    return ob


def empty(name):
    ob = bpy.data.objects.new(name, None)
    bpy.context.collection.objects.link(ob)
    return ob


def add_cameras(paths, fps, n):
    pos = paths["rover_pos"]
    plan = paths["plan_xyz"]
    d = Vector((*(plan[-1][:2] - plan[0][:2]), 0)).normalized()
    side = Vector((-d.y, d.x, 0))
    start = Vector(pos[0])
    cut1, cut2 = int(SHOTS[0] * n), int(SHOTS[1] * n)
    scene = bpy.context.scene

    # 1. wide drone shot: high behind the start, drifting forward while the plan draws itself
    up = Vector((0, 0, 1))
    wide_target = empty("WideTarget")
    keyframe_path(wide_target, [start + 3 * d, start + 5 * d], [1, cut1])
    wide = camera("CamWide", 28, wide_target)
    keyframe_path(wide, [start - 9 * d + 5 * side + 6 * up, start - 6 * d + 3 * side + 4.5 * up], [1, cut1])

    # 2. low tracking shot: camera rides with a smoothed copy of the rover position, beside the wheels,
    #    on the side away from the obstacles so they pass behind the rover
    sm = smooth(pos, int(fps)) + np.array([0, 0, 0.15])
    follow = empty("RoverFollow")
    keyframe_path(follow, sm)
    track = camera("CamTrack", 35, follow)
    track.parent = follow
    track.location = TRACK_SIDE * 2.2 * side - 1.0 * d + 0.35 * up

    # 3. close chase above and behind the rover, then rise over the last OVERVIEW_S to show the whole route
    goal = Vector(plan[-1])
    centre = (start + goal) / 2
    length = (goal - start).length
    overview = centre - 0.2 * length * d + TRACK_SIDE * 0.55 * length * side + 0.9 * length * up
    rise = n - int(OVERVIEW_S * fps)
    locs, aims = [], []
    for f in range(cut2, n + 1):
        p = Vector(sm[f - 1])
        a = min(1.0, max(0.0, (f - rise) / (n - rise)))
        a = a * a * (3 - 2 * a)  # smoothstep: ease in and out of the rise
        locs.append((p - 3.0 * d + TRACK_SIDE * 1.8 * side + 2.8 * up).lerp(overview, a))
        aims.append(p.lerp(centre, a))
    top_target = empty("TopTarget")
    keyframe_path(top_target, aims, range(cut2, n + 1))
    top = camera("CamTop", 30, top_target)
    keyframe_path(top, locs, range(cut2, n + 1))

    for f, cam in ((1, wide), (cut1, track), (cut2, top)):
        scene.timeline_markers.new(cam.name, frame=f).camera = cam
    scene.camera = wide
    return d, side


def render_settings(args, fps, n):
    scene = bpy.context.scene
    scene.render.fps = fps
    scene.frame_start, scene.frame_end = 1, n
    if args.frames:
        a, b = args.frames.split(":")
        scene.frame_start, scene.frame_end = int(a), int(b)
    scene.render.resolution_x, scene.render.resolution_y = 1920, 1080
    scene.render.resolution_percentage = args.res_pct
    scene.render.image_settings.file_format = "PNG"
    scene.view_settings.view_transform = "AgX"
    scene.view_settings.look = "AgX - Medium High Contrast"
    if args.engine == "cycles":
        scene.render.engine = "CYCLES"
        prefs = bpy.context.preferences.addons["cycles"].preferences
        for backend in ("OPTIX", "CUDA"):
            try:
                prefs.compute_device_type = backend
                prefs.get_devices()
                if any(dv.type == backend for dv in prefs.devices):
                    break
            except TypeError:
                continue
        for dv in prefs.devices:
            dv.use = dv.type != "CPU"
        scene.cycles.device = "GPU"
        scene.cycles.samples = args.samples
        scene.cycles.use_denoising = True
        print(f"cycles on {prefs.compute_device_type}: {[dv.name for dv in prefs.devices if dv.use]}")
    else:
        scene.render.engine = "BLENDER_EEVEE"
        scene.eevee.taa_render_samples = args.samples
        scene.eevee.use_shadows = True
    scene.render.use_motion_blur = args.engine == "cycles"


def main():
    args = parse_args()
    export = args.export.resolve()
    paths = dict(np.load(export / "paths.npz"))
    fps, n = int(paths["fps"]), int(paths["n_frames"])

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.context.preferences.edit.keyframe_new_interpolation_type = "LINEAR"
    regolith, sand = terrain_materials(export / "texture.png")
    import_terrain(export, regolith, sand)
    import_rover(export)
    add_paths(paths, n)
    d, side = add_cameras(paths, fps, n)
    goal_flag(paths["plan_xyz"][-1], TRACK_SIDE * side)
    sky_and_sun(d)
    render_settings(args, fps, n)

    if args.save:
        bpy.ops.wm.save_as_mainfile(filepath=str(args.save.resolve()))
    scene = bpy.context.scene
    if args.still is not None:
        scene.frame_set(args.still)
        scene.render.filepath = str(args.out.resolve())
        bpy.ops.render.render(write_still=True)
    elif args.anim:
        scene.render.filepath = str(args.out.resolve()) + "/"
        bpy.ops.render.render(animation=True)


if __name__ == "__main__":
    main()
