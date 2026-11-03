"""
Room Sensor LITE case (XIAO ESP32C6 + HLK-LD2410C + SHT40 + BH1750) - parametric (Python + manifold3d)

Run:   pip install manifold3d trimesh numpy
       python room_sensor_case_lite.py
Out:   lite_base.stl, lite_lid.stl, lite_foot.stl, lite_corner_mount.stl

Coordinates (seen from the room):  X = right, Y = up, Z = out of the wall towards the room.
Board data comes from Room-Sensor-lowcost.kicad_pcb (board 43.3 x 22 mm, KiCad x 147.5-190.8, y 96-118).

Screws: ONE type, 2 x M3 x 12 (JLCMC EDLA-J2-M3-L12) per sensor. They go in from the back,
through the base and thread into the lid posts at the two ends. The board is NOT screwed:
its three 2.2 mm holes sit on pins in the base and the lid presses it down.
"""
import manifold3d as m
from manifold3d import Manifold as M, CrossSection as CS, JoinType
import numpy as np
import trimesh

m.set_circular_segments(48)

# ------------------------------------------------------------------ PARAMETERS
WALL, BACK_T, LID_T = 2.0, 2.5, 1.6
FIT = 0.25
CORNER_R = 4.0
MATERIAL = "resin"
M3_PILOT = 2.7 if MATERIAL == "resin" else 2.5
M3_CLEAR, M3_HEAD_D, M3_HEAD_H = 3.4, 6.0, 3.2
END_POST_D = 7.5          # lid posts at the two ends (the M3 threads in here)
END_BOSS_H = 6.5          # base boss height under the end posts

BOARD_T = 1.6
STANDOFF_H = 4.5          # room under the board for the radar's through-hole pins + wall-screw heads
LID_GAP = 6.0             # board top -> inside of lid (XIAO + USB-C is ~4.5 mm tall)

# HLK-LD2410C-P: CHECK when the boards arrive (header spacer height / module thickness)
RADAR_LIFT = 2.5          # plastic spacer of its pin header: module underside above the board
RADAR_T = 1.0             # module PCB thickness

OY = 2.0                  # gap under the board's bottom edge (XIAO USB-C sticks out ~1.3 mm)
RADAR_U0, RADAR_U1 = -10.36, 5.64      # radar module, board-local u (from the KiCad courtyard)
RADAR_V0, RADAR_V1 = 0.08, 22.08
POST_ZONE = 9.0           # space at each end for the lid posts
OX = POST_ZONE - RADAR_U0                # board left edge, inner X
BOARD_W, BOARD_H = 43.3, 22.0
INNER_W = OX + BOARD_W + 1.0 + POST_ZONE - 0.5
INNER_H = OY + RADAR_V1 + 0.6

USB_U, USB_OPEN_W, USB_OPEN_H = 18.72, 13.0, 7.4

# board-local (u right from left edge, v up from bottom edge)
def U(x): return x - 147.5
def V(y): return 118.0 - y
HOLES = [(U(188.4), V(98.4)), (U(188.4), V(115.6)), (U(179.0), V(115.6))]   # 2.2 mm NPTH
PIN_D = 1.9
EXTRA_RESTS = [(1.5, 2.0), (1.5, 20.0)]          # plain supports at the left end of the board
U4 = (U(186.43), V(111.9))                        # BH1750
U1 = (U(186.08), V(106.4))                        # SHT40
CH_U, CH_V = 32.0, 9.0                            # SHT40 chamber: u > CH_U, v > CH_V

KEY_U = (-4.5, 22.0)      # keyholes (board-local u): under the radar overhang, and under the XIAO
KEY_V_HOLE, KEY_SLOT_LEN, KEY_HEAD_D, KEY_SLOT_W = 5.0, 6.0, 8.0, 3.6
STAND_LIFT = 32.0

# ------------------------------------------------------------------ DERIVED
OUT_W, OUT_H = INNER_W + 2 * WALL, INNER_H + 2 * WALL
Z_PCB_BOT = BACK_T + STANDOFF_H
Z_PCB_TOP = Z_PCB_BOT + BOARD_T
Z_LID_IN = Z_PCB_TOP + LID_GAP
OUT_D = Z_LID_IN + LID_T
Z_RADAR_BOT = Z_PCB_TOP + RADAR_LIFT
Z_RADAR_TOP = Z_RADAR_BOT + RADAR_T

def ix(x): return WALL + x
def iy(y): return WALL + y
def bx(u): return WALL + OX + u
def by(v): return WALL + OY + v
END_POSTS = [(ix(POST_ZONE / 2 - 0.25), OUT_H / 2), (ix(INNER_W - POST_ZONE / 2 + 0.25), OUT_H / 2)]

# ------------------------------------------------------------------ HELPERS
def box(x0, y0, z0, x1, y1, z1):
    return M.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])

def cyl(x, y, z0, z1, d, d2=None):
    return M.cylinder(z1 - z0, d / 2, (d2 if d2 else d) / 2).translate([x, y, z0])

def rrect(w, h, r, t, dx=0, dy=0):
    r = max(r, 0.01)
    return CS.square([w - 2 * r, h - 2 * r]).translate([r, r]).offset(r, JoinType.Round).extrude(t).translate([dx, dy, 0])

def slots(x0, x1, y0, y1, z0, z1, n, axis="x", w=1.6):
    out = M()
    for i in range(n):
        c = (x0 if axis == "x" else y0) + (i + 0.5) / n * ((x1 - x0) if axis == "x" else (y1 - y0))
        out += box(c - w / 2, y0, z0, c + w / 2, y1, z1) if axis == "x" else box(x0, c - w / 2, z0, x1, c + w / 2, z1)
    return out

def usb_cut():
    ux, uz = bx(USB_U), Z_PCB_TOP + 2.6
    return box(ux - USB_OPEN_W / 2, -1, uz - USB_OPEN_H / 2, ux + USB_OPEN_W / 2, WALL + 2.0, uz + USB_OPEN_H / 2)

# ------------------------------------------------------------------ BASE
def make_base():
    b = rrect(OUT_W, OUT_H, CORNER_R, Z_LID_IN) - rrect(INNER_W, INNER_H, CORNER_R - WALL, Z_LID_IN, WALL, WALL).translate([0, 0, BACK_T])

    # board supports: standoffs with locating pins in the three 2.2 mm holes + two plain rests
    for (u, v) in HOLES:
        b += cyl(bx(u), by(v), BACK_T - 0.1, Z_PCB_BOT, 4.5)
        b += cyl(bx(u), by(v), Z_PCB_BOT - 0.1, Z_PCB_BOT + BOARD_T - 0.25, PIN_D, PIN_D - 0.3)
    for (u, v) in EXTRA_RESTS:
        b += cyl(bx(u), by(v), BACK_T - 0.1, Z_PCB_BOT, 3.0)

    # radar overhang: two pillars under its far corners
    for v in (RADAR_V0 + 1.2, RADAR_V1 - 1.2):
        b += cyl(bx(RADAR_U0 + 1.2), by(v), BACK_T - 0.1, Z_RADAR_BOT - 0.3, 2.4)

    # end bosses for the two M3 screws (head recessed in the back)
    for (x, y) in END_POSTS:
        b += cyl(x, y, BACK_T - 0.1, END_BOSS_H, END_POST_D + 0.5)
        b -= cyl(x, y, -1, END_BOSS_H + 0.1, M3_CLEAR)
        b -= cyl(x, y, -1, M3_HEAD_H, M3_HEAD_D)

    b -= usb_cut()

    zv0, zv1 = BACK_T + 1.5, Z_LID_IN - 1.5
    ux = bx(USB_U)
    # bottom wall: inlets (left of USB under the radar, right of USB, SHT40 chamber)
    b -= slots(bx(-8.5), ux - USB_OPEN_W / 2 - 2, -1, WALL + 1, zv0, zv1, 6)
    b -= slots(ux + USB_OPEN_W / 2 + 2, bx(CH_U) - 2, -1, WALL + 1, zv0, zv1, 3)
    # top wall: warm air out above radar/XIAO; separate slots over the SHT40 chamber
    b -= slots(bx(-8.5), bx(CH_U) - 2.5, OUT_H - WALL - 1, OUT_H + 1, zv0, zv1, 12)
    b -= slots(bx(CH_U) + 2.0, bx(BOARD_W) - 1.5, OUT_H - WALL - 1, OUT_H + 1, zv0, zv1, 3)
    # right wall: fresh air straight into the SHT40 chamber
    b -= slots(OUT_W - WALL - 1, OUT_W + 1, by(CH_V) + 2, OUT_H - WALL - 2, zv0, zv1, 3, axis="y")

    # keyholes
    for ku in KEY_U:
        x, y = bx(ku), by(KEY_V_HOLE)
        b -= cyl(x, y, -1, BACK_T + 1, KEY_HEAD_D)
        b -= box(x - KEY_SLOT_W / 2, y, -1, x + KEY_SLOT_W / 2, y + KEY_SLOT_LEN, BACK_T + 1)
        b -= cyl(x, y + KEY_SLOT_LEN, -1, BACK_T + 1, KEY_SLOT_W)
    return b

# ------------------------------------------------------------------ LID
def make_lid():
    z0 = Z_LID_IN
    lid = rrect(OUT_W, OUT_H, CORNER_R, LID_T).translate([0, 0, z0])
    lip = rrect(INNER_W - 2 * FIT, INNER_H - 2 * FIT, CORNER_R - WALL - FIT, 2.0, WALL + FIT, WALL + FIT) - \
          rrect(INNER_W - 2 * FIT - 2.4, INNER_H - 2 * FIT - 2.4, 0.5, 2.0, WALL + FIT + 1.2, WALL + FIT + 1.2)
    lid += lip.translate([0, 0, z0 - 2.0])

    # end posts (M3 threads in)
    for (x, y) in END_POSTS:
        lid += cyl(x, y, END_BOSS_H + FIT, z0 + 0.1, END_POST_D)
        lid -= cyl(x, y, END_BOSS_H, z0 + 0.6, M3_PILOT)
        lid -= cyl(x, y, END_BOSS_H + FIT - 0.01, END_BOSS_H + FIT + 0.6, 3.3, M3_PILOT)

    # press-down posts over the three board holes (recess for the locating pins)
    for (u, v) in HOLES:
        lid += cyl(bx(u), by(v), Z_PCB_TOP, z0 + 0.1, 4.0)
        lid -= cyl(bx(u), by(v), Z_PCB_TOP - 1, Z_PCB_TOP + 0.8, 2.4)

    # radar: small pads hold the module's corners (keep plastic off the antenna patches)
    for (u, v) in [(RADAR_U0 + 1.5, RADAR_V0 + 1.5), (RADAR_U0 + 1.5, RADAR_V1 - 1.5),
                   (RADAR_U1 - 1.2, RADAR_V0 + 1.0), (RADAR_U1 - 1.2, RADAR_V1 - 1.0)]:
        lid += cyl(bx(u), by(v), Z_RADAR_TOP + 0.3, z0 + 0.1, 2.4)

    # BH1750 light tube + window
    lx, ly = bx(U4[0]), by(U4[1])
    tube_bot = Z_PCB_TOP + 1.25
    lid += cyl(lx, ly, tube_bot, z0 + 0.1, 5.2)
    lid -= cyl(lx, ly, tube_bot - 1, z0 + LID_T + 1, 3.0)

    # SHT40 chamber walls (keep XIAO/radar heat away), stop 0.8 mm above the board
    rib_bot = Z_PCB_TOP + 0.8
    lid += box(bx(CH_U) - 0.6, by(CH_V) - 0.6, rib_bot, bx(CH_U) + 0.6, iy(INNER_H) - FIT, z0 + 0.1)
    lid += box(bx(CH_U) - 0.6, by(CH_V) - 0.6, rib_bot, ix(INNER_W) - FIT, by(CH_V) + 0.6, z0 + 0.1)
    # front vents over the SHT40 (clear of the press post)
    lid -= slots(bx(CH_U) + 1.8, bx(BOARD_W) + 0.5, by(CH_V) + 2.0, by(17.0), z0 - 1, z0 + LID_T + 1, 5)

    lid -= usb_cut()
    return lid

# ------------------------------------------------------------------ MOUNTS
def peg(x, y, z):
    return cyl(x, y, z - 0.1, z + BACK_T + 0.4, KEY_SLOT_W - 0.5) + \
           cyl(x, y, z + BACK_T + 0.4, z + BACK_T + 2.4, KEY_HEAD_D - 0.6, KEY_HEAD_D - 1.6)

def peg_positions():
    return [(bx(ku), by(KEY_V_HOLE + KEY_SLOT_LEN)) for ku in KEY_U]

def make_foot():
    T = 4.0
    up_h = STAND_LIFT + peg_positions()[0][1] + 8
    base_d, w = 45.0, OUT_W - 10
    foot = rrect(w, up_h, 3, T, 5, -STAND_LIFT).translate([0, 0, -T])
    for (x, y) in peg_positions():
        foot += peg(x, y, 0)
    foot += rrect(w, base_d, 3, T, 5, 0).rotate([90, 0, 0]).translate([0, -STAND_LIFT + T, -base_d + 0.01])
    foot += M.hull_points([[x, -STAND_LIFT + T, -T] for x in (OUT_W / 2 - 1.5, OUT_W / 2 + 1.5)] +
                          [[x, -STAND_LIFT + T + 22, -T] for x in (OUT_W / 2 - 1.5, OUT_W / 2 + 1.5)] +
                          [[x, -STAND_LIFT + T, -T - 22] for x in (OUT_W / 2 - 1.5, OUT_W / 2 + 1.5)])
    return foot

def make_corner():
    W, H = 64.0, 45.0
    d = W / 2
    wedge = CS([[[0, 0], [W, 0], [W / 2, d]]]).extrude(H).rotate([-90, 0, 0])
    pp = peg_positions()
    cx = (pp[0][0] + pp[1][0]) / 2
    for (x, y) in pp:
        wedge += peg(x - cx + W / 2, H / 2, 0)
    s2 = np.sqrt(0.5)
    for side, ys in ((-1, (9, 36)), (1, (15, 30))):
        px = W / 2 + side * W / 4
        out = np.array([side * s2, 0, -s2])
        rot = [0, -135 if side < 0 else 135, 0]
        for hy in ys:
            P = np.array([px, hy, -d / 2])
            wedge -= M.cylinder(60, 2.2).rotate(rot).translate((P - out * 30).tolist())
            wedge -= M.cylinder(40, 3.8).rotate(rot).translate((P - out * 44).tolist())
    return wedge

# ------------------------------------------------------------------ FIT CHECK MODELS
def board_solid():
    return box(bx(0), by(0), Z_PCB_BOT, bx(BOARD_W), by(BOARD_H), Z_PCB_TOP)

def parts_solids():
    t = Z_PCB_TOP
    P = {}
    P["XIAO"] = box(bx(U(157.32)), by(V(117.98)), t, bx(U(175.12)), by(V(96.98)), t + 4.0)
    P["USB-C"] = box(bx(USB_U) - 4.5, by(-1.3), t + 1.0, bx(USB_U) + 4.5, by(7.3), t + 4.2)
    P["radar"] = box(bx(RADAR_U0), by(RADAR_V0), Z_RADAR_BOT, bx(RADAR_U1), by(RADAR_V1), Z_RADAR_TOP + 0.3)
    P["radar header"] = box(bx(3.1) - 1.27, by(V(112.0)) - 1.27, t, bx(3.1) + 1.27, by(V(101.84)) + 1.27, Z_RADAR_BOT)
    P["radar pins"] = box(bx(3.1) - 0.6, by(V(112.0)) - 0.6, Z_PCB_BOT - 3.0, bx(3.1) + 0.6, by(V(101.84)) + 0.6, Z_PCB_BOT)
    P["U4 BH1750"] = box(bx(U4[0]) - 1.5, by(U4[1]) - 0.8, t, bx(U4[0]) + 1.5, by(U4[1]) + 0.8, t + 0.75)
    P["U1 SHT40"] = box(bx(U1[0]) - 0.75, by(U1[1]) - 0.75, t, bx(U1[0]) + 0.75, by(U1[1]) + 0.75, t + 0.5)
    P["C7"] = box(bx(U(155.2)) - 0.65, by(V(113.8)) - 1.0, t, bx(U(155.2)) + 0.65, by(V(113.8)) + 1.0, t + 1.25)
    for name, (x, y, rot) in {"R8": (183.49, 110, 0), "R9": (183.4, 108.4, 0), "C12": (180.72, 110.98, 90),
                              "C11": (187.88, 106.4, 90), "C9": (189.43, 111.82, 90)}.items():
        hw, hh = (0.5, 0.25) if rot == 0 else (0.25, 0.5)
        P[name] = box(bx(U(x)) - hw, by(V(y)) - hh, t, bx(U(x)) + hw, by(V(y)) + hh, t + 0.55)
    return P

def to_trimesh(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3], faces=np.asarray(mesh.tri_verts), process=True)

if __name__ == "__main__":
    base, lid, foot, corner = make_base(), make_lid(), make_foot(), make_corner()
    for name, part in [("lite_base", base), ("lite_lid", lid), ("lite_foot", foot), ("lite_corner_mount", corner)]:
        to_trimesh(part).export(f"{name}.stl")
        bb = part.bounding_box()
        print(f"{name:18s} {bb[3]-bb[0]:6.1f} x {bb[4]-bb[1]:6.1f} x {bb[5]-bb[2]:6.1f} mm  vol {part.volume()/1000:5.1f} cm3  genus {part.genus()}")
    print(f"outer case: {OUT_W:.1f} x {OUT_H:.1f} x {OUT_D:.1f} mm")
