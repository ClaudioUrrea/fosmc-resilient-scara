"""
RoboDK digital-twin scene for:
  "Fault-tolerant fractional-order sliding-mode control of a kinematically
   redundant SCARA robot on a linear rail"

What this script does (RoboDK >= 5.6, Python API `robodk` package):
  1. Builds a dispensing work cell: floor, table, 1.8-m linear rail with carriage,
     ABB IRB 910SC-3/0.65 SCARA (from the RoboDK library), dispensing nozzle,
     workpiece plate with the lemniscate contour, cartridge feeder, camera post,
     safety fence and a fault-status beacon.
  2. Self-calibrates the kinematic mapping (link lengths, J1 axis, joint senses)
     from the RoboDK model itself, so the playback does not depend on
     vendor-specific conventions.
  3. Replays the joint trajectories exported by `run_all.py`
     (rail d, theta1, theta2 + fault flags), colouring the beacon
     (green = healthy, amber = LOE active, red = elbow locked,
      blue = reconfigured/degraded mode).
  4. Checks collisions at every frame and computes the Cartesian deviation
     between the RoboDK TCP and the simulated end-effector (independent
     forward-kinematics cross-check).
  5. Draws the desired (green) and executed (red) contours.

IMPORTANT
  * RoboDK is a kinematic simulator: it does NOT integrate the robot dynamics
    nor the torque-level controller. Dynamics/control are computed in Python
    (sim.py); RoboDK is used as a digital twin for kinematic validation,
    layout/collision verification and visualisation.
  * Download the robot first: https://robodk.com/robot/ABB/IRB-910SC-3-0-65
    (or open it from RoboDK: File > Open online library) and set ROBOT_FILE
    below, or simply load it into the open station before running the script.
  * If the link lengths measured in step 2 differ from those in sim.py
    (0.325/0.325 m), re-run the simulation with the printed values.

Usage (inside RoboDK: Program > Add Python program, or from a terminal
while RoboDK is open):
    python robodk_scene.py --csv results\\traj_S3_FOSMC.csv [--robot file.robot] [--snap figures]
"""
import argparse
import csv
import math
import os
import tempfile
import time

from robodk.robolink import (Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, COLLISION_ON,
                             COLLISION_OFF, PROJECTION_NONE)
try:
    from robodk.robolink import (FLAG_ROBODK_ALL, FLAG_ROBODK_TREE_VISIBLE, FLAG_ROBODK_REFERENCES_VISIBLE,
                                 WINDOWSTATE_CINEMA, WINDOWSTATE_MAXIMIZED)
except ImportError:  # very old API versions
    FLAG_ROBODK_ALL, FLAG_ROBODK_TREE_VISIBLE, FLAG_ROBODK_REFERENCES_VISIBLE = 0xFFFF, 8192, 16384
    WINDOWSTATE_CINEMA, WINDOWSTATE_MAXIMIZED = 5, 3
from robodk.robomath import transl, rotz, eye, Mat

# ---------------------------------------------------------------------------
# User settings
# ---------------------------------------------------------------------------
ROBOT_FILE = r""        # e.g. r"C:\RoboDK\Library\ABB-IRB-910SC-3-0-65.robot"
ROBOT_NAME_HINT = "910SC"
TABLE_H = 750.0         # table top height [mm]
RAIL_LEN = 2000.0       # rail length [mm]
RAIL_H = 120.0          # rail height [mm]
CARRIAGE_H = 60.0       # carriage thickness [mm]
PLATE_T = 12.0          # workpiece plate thickness [mm]
NOZZLE_L = 90.0         # nozzle length [mm]
CLEARANCE = 3.0         # nozzle-to-plate gap [mm]
SIM_L1, SIM_L2 = 400.0, 250.0   # link lengths used in sim.py [mm]
TCP_WORK_H = 90.0        # nozzle working height above the table top [mm]
FINAL_T = 7.3            # instant of the final presentation frame [s]
J3_WORK = 80.0          # SCARA quill (J3) stroke used during dispensing [mm]
COL = dict(floor=[0.93, 0.93, 0.94, 1], table=[0.30, 0.32, 0.35, 1], legs=[0.22, 0.23, 0.25, 1],
           rail=[0.78, 0.80, 0.82, 1], guide=[0.20, 0.20, 0.22, 1], stop=[0.80, 0.12, 0.10, 1],
           carriage=[0.95, 0.55, 0.10, 1], plate=[0.86, 0.87, 0.88, 1], fixture=[0.35, 0.36, 0.39, 1],
           nozzle=[0.15, 0.15, 0.15, 1], cabinet=[0.85, 0.86, 0.88, 1], cartridge=[0.15, 0.35, 0.75, 1],
           post=[0.25, 0.26, 0.28, 1], hmi=[0.08, 0.10, 0.14, 1], camera=[0.12, 0.12, 0.14, 1], fence=[0.55, 0.62, 0.70, 0.22], fencepost=[0.90, 0.72, 0.12, 1],
           healthy=[0.10, 0.80, 0.10, 1], loe=[1.00, 0.65, 0.00, 1],
           locked=[0.90, 0.10, 0.10, 1], reconf=[0.10, 0.40, 0.95, 1],
           path_des=[0.00, 0.60, 0.20, 1], path_exe=[0.85, 0.00, 0.00, 1])


# ---------------------------------------------------------------------------
# Simple ASCII-STL primitives (no external dependencies)
# ---------------------------------------------------------------------------
def _write_stl(path, tris):
    with open(path, "w") as f:
        f.write("solid s\n")
        for a, b, c in tris:
            f.write(" facet normal 0 0 0\n  outer loop\n")
            for v in (a, b, c):
                f.write("   vertex %.4f %.4f %.4f\n" % v)
            f.write("  endloop\n endfacet\n")
        f.write("endsolid s\n")


def stl_box(path, sx, sy, sz, cx=0.0, cy=0.0, z0=0.0):
    x0, x1, y0, y1, z1 = cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2, z0 + sz
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    q = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    tris = []
    for a, b, c, d in q:
        tris += [(v[a], v[b], v[c]), (v[a], v[c], v[d])]
    _write_stl(path, tris)
    return path


def stl_cylinder(path, r, h, n=32, z0=0.0, r_top=None):
    r_top = r if r_top is None else r_top
    tris = []
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        p0 = (r * math.cos(a0), r * math.sin(a0), z0)
        p1 = (r * math.cos(a1), r * math.sin(a1), z0)
        t0 = (r_top * math.cos(a0), r_top * math.sin(a0), z0 + h)
        t1 = (r_top * math.cos(a1), r_top * math.sin(a1), z0 + h)
        tris += [(p0, p1, t1), (p0, t1, t0), ((0, 0, z0), p1, p0), ((0, 0, z0 + h), t0, t1)]
    _write_stl(path, tris)
    return path


# ---------------------------------------------------------------------------
# Station construction
# ---------------------------------------------------------------------------
def add_obj(RDK, path, parent, name, color, pose=None):
    obj = RDK.AddFile(path, parent)
    obj.setName(name)
    obj.setColor(color)
    if pose is not None:
        obj.setPose(pose)
    return obj


def _find_robot_file(hint):
    """Look for a downloaded .robot file of the IRB 910SC-3/0.65 in usual places."""
    import glob
    home = os.path.expanduser("~")
    here = os.path.dirname(os.path.abspath(__file__))
    folders = [here, os.getcwd(), os.path.join(home, "Downloads"), os.path.join(home, "Documents"),
               r"C:\RoboDK\Library", r"C:\RoboDK\Other", r"C:\RoboDK\Stations"]
    for f in folders:
        hits = sorted(glob.glob(os.path.join(f, "**", "*910SC*0*65*.robot"), recursive=True)) if os.path.isdir(f) else []
        if hits:
            return hits[0]
    return ""


def _vec(m):
    """RoboDK Mat/list -> flat list of floats."""
    try:
        return [float(v) for v in m.list()]
    except Exception:
        return [float(v) for v in m]


def get_robot(RDK, robot_file="", robot_name=""):
    """Select the SCARA: --robot-name, else a robot whose name contains '910SC',
    else load a .robot file, else the only 4-axis robot of the station."""
    robots = RDK.ItemList(ITEM_TYPE_ROBOT)
    info = [(r.Name(), len(_vec(r.Joints()))) for r in robots]
    print("Robots in the open station:", ", ".join("%s (%d axes)" % i for i in info) or "none")
    key = robot_name or ROBOT_NAME_HINT
    for r in robots:
        if key.lower() in r.Name().lower():
            return r
    path = robot_file or ROBOT_FILE or _find_robot_file(ROBOT_NAME_HINT)
    if path and os.path.isfile(path):
        print("Loading robot file:", path)
        item = RDK.AddFile(path)
        if item.Valid():
            return item
    four = [r for r, (_, n) in zip(robots, info) if n == 4]
    if len(four) == 1:
        print("Using the only 4-axis robot of the station:", four[0].Name())
        return four[0]
    raise RuntimeError(
        "SCARA ABB IRB 910SC-3/0.65 not found in the open station (the robots listed above are not used).\n"
        "  Recommended: File > New Station, then File > Open online library > 'IRB 910SC-3/0.65' > Load,\n"
        "  and run this script again. Or pass --robot-name \"<exact robot name>\" / --robot <file.robot>.")


def _find_csv(name):
    here = os.path.dirname(os.path.abspath(__file__))
    for c in (name, os.path.join(here, name), os.path.join(here, os.path.basename(name)),
              os.path.join(os.getcwd(), os.path.basename(name)), os.path.join(here, "results", os.path.basename(name)),
              os.path.join(here, "..", "results", os.path.basename(name)), os.path.join(os.getcwd(), "results", os.path.basename(name))):
        if os.path.isfile(c):
            return os.path.abspath(c)
    raise FileNotFoundError("Trajectory file '%s' not found. Use --csv with the full path, e.g. "
                            "--csv C:\\...\\FOSMC_FractalFract\\results\\traj_S3_FOSMC.csv" % name)


def calibrate(robot):
    """Measure L1, L2, J1-axis location, joint senses and a valid quill stroke from
    the RoboDK model itself, using test joints inside the robot's own limits."""
    n = len(_vec(robot.Joints()))
    if n != 4:
        raise RuntimeError("Selected robot '%s' has %d axes; a 4-axis SCARA is required." % (robot.Name(), n))
    lo, hi, _ = robot.JointLimits()
    lo, hi = _vec(lo), _vec(hi)
    mid = [0.5 * (l + h) for l, h in zip(lo, hi)]
    clampj = lambda v, i: min(max(v, lo[i]), hi[i])
    j3 = J3_WORK if lo[2] <= J3_WORK <= hi[2] else mid[2]
    j4 = clampj(0.0, 3)
    a1 = min(45.0, 0.45 * min(abs(lo[0]), abs(hi[0])))
    b2 = min(90.0, 0.9 * hi[1]) if hi[1] > 10 else max(-90.0, 0.9 * lo[1])
    base1 = clampj(0.0, 0)
    print("Joint limits: lower %s, upper %s  -> J3 = %.1f mm, test angles J1 = +/-%.1f deg, J2 = %.1f deg"
          % ([round(v, 1) for v in lo], [round(v, 1) for v in hi], j3, a1, b2))

    def xy(j1, j2):
        P = robot.SolveFK([j1, j2, j3, j4])
        return P[0, 3], P[1, 3], P[2, 3]
    pts = [xy(base1 + a, 0) for a in (-a1, 0.0, a1)]
    (x1, y1, _), (x2, y2, _), (x3, y3, _) = pts
    D = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    cx = ((x1**2 + y1**2) * (y2 - y3) + (x2**2 + y2**2) * (y3 - y1) + (x3**2 + y3**2) * (y1 - y2)) / D
    cy = ((x1**2 + y1**2) * (x3 - x2) + (x2**2 + y2**2) * (x1 - x3) + (x3**2 + y3**2) * (x2 - x1)) / D
    x0, y0, z0 = xy(0.0, 0.0)
    ang0 = math.atan2(y0 - cy, x0 - cx)
    Ltot = math.hypot(x0 - cx, y0 - cy)
    xp, yp, _ = xy(a1, 0.0)
    dang = math.atan2(math.sin(math.atan2(yp - cy, xp - cx) - ang0), math.cos(math.atan2(yp - cy, xp - cx) - ang0))
    s1 = 1 if dang > 0 else -1
    xb, yb, _ = xy(0.0, b2)
    ex, ey = math.cos(ang0), math.sin(ang0)
    along = (xb - cx) * ex + (yb - cy) * ey
    perp = -(xb - cx) * ey + (yb - cy) * ex
    sb, cb = math.sin(math.radians(abs(b2))), math.cos(math.radians(b2))
    L2 = abs(perp) / sb
    L1 = along - L2 * cb
    s2 = 1 if perp * b2 > 0 else -1
    return dict(cx=cx, cy=cy, ang0=ang0, s1=s1, s2=s2, L1=L1, L2=L2, Ltot=Ltot, z_flange=z0,
                j3=j3, lo=lo, hi=hi)


def tcp_in_rail(robot, base_local, d_mm, tool=True):
    """TCP (or flange) pose in the rail frame, computed from LOCAL poses only:
    carriage(d) * base mounting * forward kinematics of the RoboDK model.
    This avoids reading absolute poses (PoseAbs) of items that were just
    re-parented, which some RoboDK versions report before updating them."""
    fk = robot.SolveFK(robot.Joints(), robot.PoseTool()) if tool else robot.SolveFK(robot.Joints())
    return transl(d_mm, 0, RAIL_H) * base_local * fk


def rail_abs():
    """Absolute pose of the rail frame (World cell frame is at the station origin)."""
    return transl(0, 0, TABLE_H)


def build_station(RDK, robot, cal, tmp):
    box = lambda n, *a, **k: stl_box(os.path.join(tmp, n + ".stl"), *a, **k)
    cyl = lambda n, *a, **k: stl_cylinder(os.path.join(tmp, n + ".stl"), *a, **k)
    world = RDK.AddFrame("World cell")
    add_obj(RDK, box("floor", 5000, 4000, 5, cy=250, z0=-5), world, "Floor", COL['floor'])
    # table: steel top on four legs
    TY, TW, TD = 250.0, 2400.0, 1300.0
    add_obj(RDK, box("tabletop", TW, TD, 40, cy=TY, z0=TABLE_H - 40), world, "Table top", COL['table'])
    leg = box("leg", 80, 80, TABLE_H - 40)
    for i, (lx, ly) in enumerate(((-1140, TY - 590), (1140, TY - 590), (-1140, TY + 590), (1140, TY + 590))):
        add_obj(RDK, leg, world, "Table leg %d" % (i + 1), COL['legs'], transl(lx, ly, 0))
    # light guarding: back and left panels only, so that the camera view stays clear
    add_obj(RDK, box("fence_back", 2800, 10, 1500, z0=300), world, "Guard panel back", COL['fence'], transl(0, 1080, 0))
    add_obj(RDK, box("fence_left", 10, 1600, 1500, z0=300), world, "Guard panel left", COL['fence'], transl(-1400, 280, 0))
    fp = box("fencepost", 50, 50, 1800)
    for i, (px, py) in enumerate(((-1400, 1080), (0, 1080), (1400, 1080))):
        add_obj(RDK, fp, world, "Guard post %d" % (i + 1), COL['fencepost'], transl(px, py, 0))

    rail = RDK.AddFrame("Rail frame", world)
    rail.setPose(transl(0, 0, TABLE_H))
    # rail: aluminium profile, two guide rails and end stops
    add_obj(RDK, box("rail", RAIL_LEN, 200, RAIL_H - 30), rail, "Linear rail", COL['rail'])
    g = box("guide", RAIL_LEN, 20, 30, z0=RAIL_H - 30)
    add_obj(RDK, g, rail, "Guide rail 1", COL['guide'], transl(0, -60, 0))
    add_obj(RDK, g, rail, "Guide rail 2", COL['guide'], transl(0, 60, 0))
    st_ = box("stop", 30, 200, 70, z0=RAIL_H - 30)
    add_obj(RDK, st_, rail, "End stop 1", COL['stop'], transl(-RAIL_LEN / 2 - 15, 0, 0))
    add_obj(RDK, st_, rail, "End stop 2", COL['stop'], transl(RAIL_LEN / 2 + 15, 0, 0))
    carriage = RDK.AddFrame("Carriage (d)", rail)
    carriage.setPose(transl(0, 0, RAIL_H))
    add_obj(RDK, box("carriage", 320, 300, CARRIAGE_H), carriage, "Carriage plate", COL['carriage'])
    # stack light (fault-status beacon) on a short pole
    add_obj(RDK, cyl("pole", 8, 170), carriage, "Beacon pole", COL['post'], transl(-130, -120, CARRIAGE_H))
    beacon = add_obj(RDK, cyl("beacon", 24, 70), carriage, "Fault beacon", COL['healthy'],
                     transl(-130, -120, CARRIAGE_H + 170))

    # --- mount the SCARA on the carriage so that its J1 axis is the model origin
    base = robot.Parent()
    base.setParent(carriage)
    # rotate the base so that the arm points along +X_rail at theta1 = 0, then
    # shift so that the J1 axis sits on the carriage centre
    Rz = rotz(-cal['ang0'])
    cxr = math.cos(-cal['ang0']) * cal['cx'] - math.sin(-cal['ang0']) * cal['cy']
    cyr = math.sin(-cal['ang0']) * cal['cx'] + math.cos(-cal['ang0']) * cal['cy']
    base_local = transl(-cxr, -cyr, CARRIAGE_H) * Rz
    base.setPose(base_local)
    RDK.Render(True)          # make RoboDK update the absolute poses of the re-parented items

    # --- dispensing nozzle: an STL added with the robot as parent becomes a TOOL
    # (documented RoboDK behaviour), so its geometry is rigidly fixed to the flange.
    tool = None
    noz_path = stl_nozzle(os.path.join(tmp, "nozzle.stl"))
    try:
        t = RDK.AddFile(noz_path, robot)
        if t.Valid() and t.Type() == ITEM_TYPE_TOOL:
            tool = t
            tool.setName("Dispensing nozzle")
            tool.setPoseTool(transl(0, 0, NOZZLE_L))          # TCP at the needle tip
        elif t.Valid():
            t.setVisible(False)
    except Exception as ex:
        print("Note: nozzle tool from STL failed (%s)" % ex)
    if tool is None:   # fallback: TCP only, geometry copied into the tool, template hidden
        tool = robot.AddTool(transl(0, 0, NOZZLE_L), "Dispensing nozzle")
        try:
            tmpl = RDK.AddFile(noz_path)
            tool.AddGeometry(tmpl, eye(4))
            tmpl.setName("Nozzle template (hidden)"); tmpl.setVisible(False)
        except Exception as ex:
            print("Note: nozzle geometry not added (%s); the TCP is still defined." % ex)
    try:
        tool.setColor(COL['nozzle'])
    except Exception:
        pass
    robot.setPoseTool(tool)

    # TCP height with the nozzle (measure in the rail frame)
    robot.setJoints([0, min(90.0, cal['hi'][1]), cal['j3'], min(max(0.0, cal['lo'][3]), cal['hi'][3])])
    # choose the quill stroke J3 so that the nozzle works TCP_WORK_H above the table
    j2t = min(90.0, cal['hi'][1]); j4t = min(max(0.0, cal['lo'][3]), cal['hi'][3])
    def ztcp(j3):
        robot.setJoints([0, j2t, j3, j4t]); return tcp_in_rail(robot, base_local, 0.0)[2, 3]
    ja, jb = cal['lo'][2], cal['hi'][2]
    za, zb = ztcp(ja), ztcp(jb)
    if abs(zb - za) > 1e-6:
        j3 = ja + (TCP_WORK_H - za) * (jb - ja) / (zb - za)
        cal['j3'] = min(max(j3, min(ja, jb)), max(ja, jb))
    robot.setJoints([0, j2t, cal['j3'], j4t])
    print("Quill stroke J3 = %.1f mm (TCP reachable between %.1f and %.1f mm above the table)" % (cal['j3'], min(za, zb), max(za, zb)))
    z_tcp = tcp_in_rail(robot, base_local, 0.0)[2, 3]
    print("TCP height above the table (rail frame): %.1f mm -> plate top at %.1f mm" % (z_tcp, z_tcp - CLEARANCE))

    # --- workpiece plate on two fixtures (the plate top lies CLEARANCE below the TCP)
    plate_frame = RDK.AddFrame("Workpiece", rail)
    plate_top = z_tcp - CLEARANCE
    plate_frame.setPose(transl(0, 0, plate_top - PLATE_T))
    add_obj(RDK, stl_box(os.path.join(tmp, "plate.stl"), 1400, 280, PLATE_T, cy=360), plate_frame, "Plate", COL['plate'])
    hfix = max(plate_top - PLATE_T, 10.0)
    fx = stl_box(os.path.join(tmp, "fixture.stl"), 90, 220, hfix, cy=360)
    for i, x in enumerate((-620, 0, 620)):
        add_obj(RDK, fx, rail, "Fixture %d" % (i + 1), COL['fixture'], transl(x, 0, 0))
    # auxiliary equipment outside the swept volume (checked by the collision test)
    add_obj(RDK, stl_box(os.path.join(tmp, "cabinet.stl"), 320, 240, 460), rail, "Dispensing controller",
            COL['cabinet'], transl(930, 760, 0))
    add_obj(RDK, stl_cylinder(os.path.join(tmp, "cartridge.stl"), 45, 320), rail, "Adhesive cartridge",
            COL['cartridge'], transl(700, 790, 0))
    add_obj(RDK, stl_box(os.path.join(tmp, "hmi.stl"), 200, 8, 140), rail, "Controller HMI",
            COL['hmi'], transl(930, 636, 250))
    # inspection camera on a boom from the back, hanging above the left lobe of the contour
    add_obj(RDK, stl_cylinder(os.path.join(tmp, "post.stl"), 25, 1250), rail, "Camera post",
            COL['post'], transl(-600, 1000, 0))
    add_obj(RDK, stl_box(os.path.join(tmp, "boom.stl"), 40, 660, 40), rail, "Camera boom",
            COL['post'], transl(-600, 690, 1210))
    add_obj(RDK, stl_box(os.path.join(tmp, "camera.stl"), 90, 90, 120), rail, "Inspection camera",
            COL['camera'], transl(-600, 380, 1090))

    return dict(world=world, rail=rail, carriage=carriage, beacon=beacon,
                plate_top=plate_top, tool=tool, base=base, base_local=base_local)


# ---------------------------------------------------------------------------
# Trajectory playback
# ---------------------------------------------------------------------------
def load_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return [{k: float(v) for k, v in r.items()} for r in rows]


def to_robodk_joints(r, cal):
    """Model joints (rad) -> RoboDK joints (deg / mm), kept inside the joint limits."""
    j1 = cal['s1'] * math.degrees(r['theta1'])
    j2 = cal['s2'] * math.degrees(r['theta2'])
    j4 = -(j1 + j2)                  # keep nozzle orientation constant
    q = [j1, j2, cal['j3'], j4]
    return [min(max(v, l), h) for v, l, h in zip(q, cal['lo'], cal['hi'])]


def _cyl_tris(r0, r1, z0, h, n=32):
    tris = []
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        p0 = (r0 * math.cos(a0), r0 * math.sin(a0), z0); p1 = (r0 * math.cos(a1), r0 * math.sin(a1), z0)
        t0 = (r1 * math.cos(a0), r1 * math.sin(a0), z0 + h); t1 = (r1 * math.cos(a1), r1 * math.sin(a1), z0 + h)
        tris += [(p0, p1, t1), (p0, t1, t0), ((0, 0, z0), p1, p0), ((0, 0, z0 + h), t0, t1)]
    return tris


def stl_nozzle(path):
    """Dispensing valve: flange adapter, valve body and tapered needle, total NOZZLE_L
    along +Z of the robot flange (the TCP is the needle tip)."""
    tris = (_cyl_tris(30, 30, 0, 12) + _cyl_tris(20, 20, 12, 48) +
            _cyl_tris(9, 9, 60, 10) + _cyl_tris(4, 0.8, 70, NOZZLE_L - 70))
    _write_stl(path, tris)
    return path


def stl_ribbon(path, pts, width=3.0):
    """Flat ribbon (triangle strip) along a polyline, for drawing contours as geometry.
    Independent of the AddCurve API, whose accepted point format varies across versions."""
    tris = []
    w = 0.5 * width
    for (x0, y0, z0), (x1, y1, z1) in zip(pts[:-1], pts[1:]):
        dx, dy = x1 - x0, y1 - y0
        n = math.hypot(dx, dy)
        if n < 1e-9:
            continue
        ox, oy = -dy / n * w, dx / n * w
        a, b = (x0 + ox, y0 + oy, z0), (x0 - ox, y0 - oy, z0)
        c, d = (x1 - ox, y1 - oy, z1), (x1 + ox, y1 + oy, z1)
        tris += [(a, b, c), (a, c, d), (a, c, b), (a, d, c)]      # both faces visible
    _write_stl(path, tris)
    return path


def add_curve(RDK, pts, frame, name, color, width=3.0):
    """Draw a contour (points in `frame` coordinates, mm) as a thin ribbon object."""
    path = os.path.join(tempfile.gettempdir(), "fosmc_%s.stl" % name.replace(" ", "_"))
    return add_obj(RDK, stl_ribbon(path, pts, width), frame, name, color)


def look_at_view(V0, eye_pt, target, up=(0.0, 0.0, 1.0)):
    """View pose (world w.r.t. camera) for a camera at eye_pt looking at target.
    The camera convention (looking along -Z as in OpenGL, or +Z) is inferred from
    the current view V0, in which the station origin lies in front of the camera."""
    def sub(a, b): return [a[i] - b[i] for i in range(3)]
    def nrm(a):
        n = math.sqrt(sum(v * v for v in a)); return [v / n for v in a]
    def cross(a, b): return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
    opengl = V0[2, 3] < 0          # origin at negative z in camera coords -> camera looks along -Z
    if opengl:
        zc = nrm(sub(eye_pt, target)); xc = nrm(cross(up, zc)); yc = cross(zc, xc)
    else:
        zc = nrm(sub(target, eye_pt)); xc = nrm(cross(zc, up)); yc = cross(zc, xc)
    C = Mat([[xc[0], yc[0], zc[0], eye_pt[0]], [xc[1], yc[1], zc[1], eye_pt[1]],
             [xc[2], yc[2], zc[2], eye_pt[2]], [0, 0, 0, 1]])
    return C.inv()


def presentation_mode(RDK, on):
    """Hide the station tree and reference frames and remove menus for clean snapshots."""
    try:
        if on:
            RDK.setFlagsRoboDK(FLAG_ROBODK_ALL & ~FLAG_ROBODK_TREE_VISIBLE & ~FLAG_ROBODK_REFERENCES_VISIBLE)
            RDK.setWindowState(WINDOWSTATE_CINEMA)
        else:
            RDK.setFlagsRoboDK(FLAG_ROBODK_ALL)
            RDK.setWindowState(WINDOWSTATE_MAXIMIZED)
    except Exception as ex:
        print("Note: presentation mode not available (%s)" % ex)


def snapshot(RDK, fn):
    for h in (None, 0):
        try:
            if RDK.Cam2D_Snapshot(os.path.abspath(fn), h):
                return True
        except Exception:
            pass
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="traj_S3_FOSMC.csv")
    ap.add_argument("--speed", type=float, default=1.0, help="real-time factor")
    ap.add_argument("--stride", type=int, default=10, help="use every n-th sample (1 kHz data)")
    ap.add_argument("--snap", default="", help="folder to save PNG snapshots of key instants")
    ap.add_argument("--robot", default="", help="path to the IRB 910SC-3/0.65 .robot file (optional)")
    ap.add_argument("--robot-name", default="", help="exact/partial name of the SCARA in the station (optional)")
    ap.add_argument("--view", default="auto", choices=["auto", "keep"],
                    help="auto: set a presentation camera; keep: keep the current RoboDK view")
    ap.add_argument("--zoom", type=float, default=1.0, help="camera DISTANCE factor: >1 moves the camera away (smaller scene), <1 closer (larger scene)")
    args = ap.parse_args()
    if args.snap:
        os.makedirs(args.snap, exist_ok=True)

    RDK = Robolink()
    V0 = RDK.ViewPose()
    RDK.Render(False)
    csv_path = _find_csv(args.csv)
    robot = get_robot(RDK, args.robot, args.robot_name)
    cal = calibrate(robot)
    print("Measured SCARA kinematics: L1 = %.1f mm, L2 = %.1f mm, reach = %.1f mm, s1=%d, s2=%d"
          % (cal['L1'], cal['L2'], cal['Ltot'], cal['s1'], cal['s2']))
    if abs(cal['L1'] - SIM_L1) > 2 or abs(cal['L2'] - SIM_L2) > 2:
        print(">> Link lengths differ from sim.py (%.0f/%.0f mm). Set l1/l2 in sim.py NOMINAL to %.4f/%.4f m "
              "and re-run the pipeline." % (SIM_L1, SIM_L2, cal['L1'] / 1000, cal['L2'] / 1000))

    tmp = tempfile.mkdtemp(prefix="fosmc_cell_")
    st = build_station(RDK, robot, cal, tmp)
    data = load_csv(csv_path)

    # desired contour (green) in the rail frame, at the plate surface
    z = st['plate_top'] + 0.5
    des = [[float(r['xd'] * 1000), float(r['yd'] * 1000), float(z)] for r in data[::20]]
    add_curve(RDK, des, st['rail'], "Desired contour", COL['path_des'], width=5.0)
    if args.snap:
        presentation_mode(RDK, True)
    if args.view == "auto":
        T = [0.0, 330.0, TABLE_H + 180.0]
        E = [T[0] + 1450.0 * args.zoom, T[1] - 2150.0 * args.zoom, T[2] + 1250.0 * args.zoom]
        try:
            RDK.setViewPose(look_at_view(V0, E, T))
        except Exception as ex:
            print("Note: camera not set (%s); use --view keep and frame the scene manually." % ex)

    RDK.setCollisionActive(COLLISION_ON)
    RDK.Render(True)
    state = None
    max_dev, n_coll, executed = 0.0, 0, []
    t_prev_wall, t_prev_sim = time.time(), data[0]['t']
    snaps = {1.0: "healthy", 4.05: "lock", 5.0: "reconfigured", 8.0: "degraded"}
    for r in data[::args.stride]:
        st['carriage'].setPose(transl(r['d'] * 1000, 0, RAIL_H))
        robot.setJoints(to_robodk_joints(r, cal))
        # fault beacon
        new_state = ('reconf' if r['reconf'] > 0.5 else 'locked' if r['locked'] > 0.5
                     else 'loe' if r['loe'] > 0.5 else 'healthy')
        if new_state != state:
            st['beacon'].setColor(COL[new_state]); state = new_state
            RDK.ShowMessage("t = %.2f s : %s" % (r['t'], new_state.upper()), False)
        # independent FK cross-check (RoboDK TCP vs simulated end-effector)
        P = tcp_in_rail(robot, st['base_local'], r['d'] * 1000)
        dev = math.hypot(P[0, 3] - r['x'] * 1000, P[1, 3] - r['y'] * 1000)
        max_dev = max(max_dev, dev)
        executed.append([float(P[0, 3]), float(P[1, 3]), float(z + 0.3)])
        n_coll += 1 if RDK.Collisions() > 0 else 0
        if args.snap:
            for ts, name in snaps.items():
                if abs(r['t'] - ts) < 0.5 * args.stride * 1e-3:
                    RDK.Render(True)
                    snapshot(RDK, os.path.join(args.snap, "robodk_%s.png" % name))
        # real-time pacing
        dt_sim = (r['t'] - t_prev_sim) / args.speed
        sleep = dt_sim - (time.time() - t_prev_wall)
        if sleep > 0:
            time.sleep(sleep)
        t_prev_wall, t_prev_sim = time.time(), r['t']

    add_curve(RDK, executed, st['rail'], "Executed contour", COL['path_exe'], width=2.0)
    RDK.setCollisionActive(COLLISION_OFF)
    # final presentation frame: reconfigured (degraded) mode, both contours visible
    rf = min(data, key=lambda r: abs(r['t'] - FINAL_T))
    st['carriage'].setPose(transl(rf['d'] * 1000, 0, RAIL_H))
    robot.setJoints(to_robodk_joints(rf, cal))
    st['beacon'].setColor(COL['reconf'] if rf['reconf'] > 0.5 else COL['healthy'])
    RDK.Render(True)
    if args.snap:
        ok = snapshot(RDK, os.path.join(args.snap, "robodk_final.png"))
        print("Final snapshot:", os.path.abspath(os.path.join(args.snap, "robodk_final.png")) if ok else "failed")
        presentation_mode(RDK, False)
    msg = ("Playback finished. Max TCP deviation RoboDK vs simulation = %.3f mm; "
           "frames in collision = %d" % (max_dev, n_coll))
    print(msg); RDK.ShowMessage(msg, False)


if __name__ == "__main__":
    main()
