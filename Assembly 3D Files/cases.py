# Sensor head (EVAL-ADXL355Z) and FT232H enclosures.
# Run: freecadcmd cases.py   (writes next to this file, or to the folder in env CASE_OUT)
# All sizes in mm. z = 0 is the bottom face of each part's base.
import math, os
import FreeCAD as App
import Part
from FreeCAD import Vector as V

OUT = os.environ.get("CASE_OUT", os.path.dirname(os.path.abspath(__file__)))

# ---------------------------------------------------------------- shared
WALL = 2.0          # side wall
R_OUT = 3.0         # vertical outer edges
TOP_CHAMFER = 1.0   # lid top edges; chamfer, not fillet, since the lid prints top face down
SEAM_CHAMFER = 0.4  # both sides of the parting line (small V groove)
FOOT_CHAMFER = 0.4  # bottom edge, hides first-layer squish
LIP_T = 1.0         # base lip thickness (inner half of the wall)
LIP_H = 1.6         # base lip height above the parting line
LIP_CLR = 0.15      # lid skirt clearance over the lip
SEAM_GAP = 0.2      # lid rests on screws/standoffs, not on the base rim
CABLE_D = 6.0       # cable jacket OD (measure)
CABLE_SQUEEZE = 0.7 # bore is this much smaller than the jacket
COLLAR_LEN = 2.0    # 45 deg conical collar outside the wall, prints without support either way up
COLLAR_WALL = 2.6   # collar wall at the base of the cone
RIB_H = 0.2         # two grip rings inside the bore
HOUSING_TOP = 16.5  # PCB top to top of a Dupont housing on a 2.54 header (measure)
WIRE_BEND = 4.5     # room above the housings for the wires to turn
CB_WALL = 1.0       # wall around a counterbore where it reaches past a lid plate

BORE_R = (CABLE_D - CABLE_SQUEEZE) / 2
LAYER = 0.2         # horizontal planes sit on whole layers


# ---------------------------------------------------------------- helpers
def rbox(x0, y0, x1, y1, z0, z1, r):
    b = Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))
    if r > 0:
        b = b.makeFillet(r, [e for e in b.Edges if abs(e.BoundBox.ZLength - (z1 - z0)) < 1e-6])
    return b


def on_layer(z):
    """Round a height up to the next whole layer."""
    return math.ceil(z / LAYER - 1e-6) * LAYER


def cyl(x, y, z0, h, d):
    return Part.makeCylinder(d / 2, h, V(x, y, z0))


def cbore(x, y, z_face, depth, d_head, d_hole, up=True):
    """Counterbore opening on a face that prints on the bed (up: it goes +z from z_face).
    Two layers past its ceiling step down to the hole, first a slot then a square,
    so the ceiling prints as bridges instead of a ring hanging in the air."""
    s = 1 if up else -1
    zc = z_face + s * depth
    r, h = d_head / 2, d_hole / 2

    def step(k, dx, dy):
        z0 = min(zc + s * k * LAYER, zc + s * (k + 1) * LAYER)
        return Part.makeBox(2 * dx, 2 * dy, LAYER + 0.02, V(x - dx, y - dy, z0 - 0.01))
    bore = cyl(x, y, min(z_face - 1, zc), depth + 1, d_head)
    return bore.fuse([step(0, r, h).common(cyl(x, y, zc - 1, 2, d_head)), step(1, h, h)])


def hex_prism(x, y, z0, h, af, ang=30):
    rc = af / math.sqrt(3)
    pts = [V(x + rc * math.cos(math.radians(ang + 60 * i)),
             y + rc * math.sin(math.radians(ang + 60 * i)), z0) for i in range(7)]
    return Part.Face(Part.makePolygon(pts)).extrude(V(0, 0, h))


def poly_prism(pts, z0, h):
    p = [V(x, y, z0) for x, y in pts] + [V(pts[0][0], pts[0][1], z0)]
    return Part.Face(Part.makePolygon(p)).extrude(V(0, 0, h))


def stroke(p0, p1, w, z0, h):
    (x0, y0), (x1, y1) = p0, p1
    L = math.hypot(x1 - x0, y1 - y0)
    b = Part.makeBox(L, w, h, V(0, -w / 2, 0))
    b.rotate(V(0, 0, 0), V(0, 0, 1), math.degrees(math.atan2(y1 - y0, x1 - x0)))
    b.translate(V(x0, y0, z0))
    return b.fuse([cyl(x0, y0, z0, h, w), cyl(x1, y1, z0, h, w)])


def edges_at_z(shape, z):
    return [e for e in shape.Edges if abs(e.BoundBox.ZMin - z) < 1e-4 and abs(e.BoundBox.ZMax - z) < 1e-4]


def below(z, big=500):
    return Part.makeBox(2 * big, 2 * big, big, V(-big, -big, z - big))


def above(z, big=500):
    return Part.makeBox(2 * big, 2 * big, big, V(-big, -big, z))


def fuse(a, items):
    return a.fuse(items).removeSplitter() if items else a


def cut(a, items):
    return a.cut(items).removeSplitter() if items else a


def shell_parts(x0, y0, x1, y1, z_split, z_top):
    """Outer blocks of base and lid with edge treatment, before any cuts."""
    base = rbox(x0, y0, x1, y1, 0, z_split, R_OUT)
    base = base.makeChamfer(FOOT_CHAMFER, edges_at_z(base, 0))
    base = base.makeChamfer(SEAM_CHAMFER, edges_at_z(base, z_split))
    lid = rbox(x0, y0, x1, y1, z_split + SEAM_GAP, z_top, R_OUT)
    lid = lid.makeChamfer(TOP_CHAMFER, edges_at_z(lid, z_top))
    lid = lid.makeChamfer(SEAM_CHAMFER, edges_at_z(lid, z_split + SEAM_GAP))
    return base, lid


def lip(px0, py0, px1, py1, z_split, r_in):
    """Lip ring on the base (inner part of the wall) and the matching recess in the lid."""
    ring = rbox(px0 - LIP_T, py0 - LIP_T, px1 + LIP_T, py1 + LIP_T, z_split - 0.01, z_split + LIP_H, R_OUT - WALL + LIP_T)
    ring = ring.cut(rbox(px0, py0, px1, py1, z_split - 1, z_split + LIP_H + 1, r_in))
    c = LIP_T + LIP_CLR
    recess = rbox(px0 - c, py0 - c, px1 + c, py1 + c, z_split - 1, z_split + LIP_H + SEAM_GAP + 0.2, R_OUT - WALL + c)
    return ring, recess


def cable_exit(px, py, z, direction, wall_out, wall_in):
    """Collar split at the cable axis, and the bore. direction is +-X or +-Y unit tuple."""
    dx, dy = direction
    d = V(dx, dy, 0)
    if dx:
        p_out, p_in = V(wall_out, py, z), V(wall_in, py, z)
    else:
        p_out, p_in = V(px, wall_out, z), V(px, wall_in, z)
    r0 = BORE_R + COLLAR_WALL
    collar = Part.makeCone(r0 + 0.3, r0 - COLLAR_LEN, COLLAR_LEN + 0.3, p_out - d * 0.3, d)
    wall = abs((p_out - p_in).dot(d))
    bore = Part.makeCylinder(BORE_R, wall + COLLAR_LEN + 2, p_in - d * 0.01, d)
    mouth = Part.makeCone(BORE_R, BORE_R + 0.5, 0.5, p_out + d * (COLLAR_LEN - 0.5), d)
    bore = bore.fuse(mouth)
    for s in (0.8, wall + COLLAR_LEN - 1.0):
        rib = Part.makeCylinder(BORE_R + 0.1, 0.8, p_in + d * s, d).cut(
            Part.makeCylinder(BORE_R - RIB_H, 0.8, p_in + d * s, d))
        bore = bore.cut(rib)
    return collar, bore


# ================================================================ sensor head
# EVAL-ADXL355Z: 20.32 square PCB, origin at its corner. Hole and pocket sizes
# are taken from the existing base that the board already fits.
S_PCB = 20.32
S_PCB_T = 1.6
S_CLR = 0.4                     # pocket clearance per side (existing base)
S_HOLE = 2.8                    # hole centre from PCB edges
S_HOLE_D = 3.4
S_UNDER = 2.4                   # under-PCB cavity for solder tails (existing base)
S_BOSS_R = 2.51                 # corner support around each hole (existing base)
S_CBORE_D, S_CBORE_H = 6.2, 3.2 # M3 socket head
S_MAG_D, S_MAG_T = 20.0, 5.0    # pot magnet held in a pocket in the base bottom (measure)
S_MAG_CLR = 0.15                # pocket clearance per side
S_MAG_PROUD = 0.2               # magnet face stands out so the magnet, not the plastic, meets the surface
S_MAG_SCREW = "M3"              # the 20 mm magnet's countersink takes M4; M3 until M4 flat heads are on hand
# magnet screw: clearance hole, nut pocket across flats and depth (nut held from inside)
S_MAG_NUT = {"M3": (3.4, 5.6, 2.6), "M4": (4.5, 7.1, 3.4)}[S_MAG_SCREW]
S_FLOOR = S_MAG_NUT[2] + 1.0    # floor over the magnet: nut pocket and 1 mm under it
S_STANDOFF = 12.0               # M3 F-F standoff (nylon, threaded through) on the two lid corners (measure)
S_LID_CORNERS = [(0, 0), (1, 1)]  # which hole corners carry the lid screws
S_BASE_SCREW = 8.0              # M3 socket head from below into the standoff (length under the head)
S_LID_SCREW = 16.0              # M3 socket head from the top into the standoff, head in a counterbore like the base
TIP_GAP = 1.0                   # the two screw tips stay this far apart inside the standoff
S_LID_TOP = 2.0
S_AX_X = (1, 0)                 # +X and +Y directions on the lid (cable exits toward -Y); set from a tilt test
S_AX_Y = (0, 1)
ENGRAVE = 0.4                   # two 0.2 layers

s_holes = [(S_HOLE, S_HOLE), (S_PCB - S_HOLE, S_HOLE), (S_HOLE, S_PCB - S_HOLE), (S_PCB - S_HOLE, S_PCB - S_HOLE)]
s_pk0, s_pk1 = -S_CLR, S_PCB + S_CLR
s_o0, s_o1 = s_pk0 - WALL, s_pk1 + WALL
s_z_mag = on_layer(S_MAG_T - S_MAG_PROUD)   # magnet pocket ceiling
s_z_floor = s_z_mag + S_FLOOR
s_z_head = s_z_floor - 2 * LAYER            # base screw heads, at the end of counterbores past the magnet
s_z_pcb_bot = s_z_floor + S_UNDER
s_z_pcb_top = s_z_pcb_bot + S_PCB_T
s_z_split = on_layer(s_z_pcb_top + BORE_R + 0.4)   # cable rests just above the PCB top
s_z_ceil = on_layer(s_z_pcb_top + HOUSING_TOP + WIRE_BEND)
s_z_top = s_z_ceil + S_LID_TOP
# a long lid screw raises the top so its tip stays clear of the base screw
s_z_top = max(s_z_top, on_layer(s_z_head + S_BASE_SCREW + TIP_GAP + S_LID_SCREW + S_CBORE_H))
s_mid = S_PCB / 2


def corner_region(hx, hy, r, z0, h, pc):
    sx = -1 if hx < s_mid else 1
    sy = -1 if hy < s_mid else 1
    px, py = pc * sx + (S_PCB if sx > 0 else 0), pc * sy + (S_PCB if sy > 0 else 0)

    def box(xa, xb, ya, yb):
        return Part.makeBox(abs(xb - xa), abs(yb - ya), h, V(min(xa, xb), min(ya, yb), z0))
    return box(px, hx - sx * r, py, hy).fuse([box(px, hx, py, hy - sy * r), cyl(hx, hy, z0, h, 2 * r)])


def axis_mark(z_top):
    cx = cy = s_mid
    w, h = 0.8, ENGRAVE + 1
    z0 = z_top - ENGRAVE
    parts = []
    # Z out of the top: ring and dot, at the sensor position
    ring = cyl(cx, cy, z0, h, 4.4).cut(cyl(cx, cy, z0 - 1, h + 2, 4.4 - 2 * w))
    parts += [ring, cyl(cx, cy, z0, h, 1.4)]
    r0, r1, head_l, head_w = 3.4, 9.2, 2.2, 2.6

    def arrow(ux, uy):
        vx, vy = -uy, ux
        tip = (cx + ux * r1, cy + uy * r1)
        base = (cx + ux * (r1 - head_l), cy + uy * (r1 - head_l))
        sh = stroke((cx + ux * r0, cy + uy * r0), (base[0] - ux * 0.2, base[1] - uy * 0.2), w, z0, h)
        hd = poly_prism([tip, (base[0] + vx * head_w / 2, base[1] + vy * head_w / 2),
                         (base[0] - vx * head_w / 2, base[1] - vy * head_w / 2)], z0, h)
        return [sh, hd]

    def letter_x(x, y, s=1.3):
        return [stroke((x - s * 0.75, y - s), (x + s * 0.75, y + s), 0.7, z0, h),
                stroke((x - s * 0.75, y + s), (x + s * 0.75, y - s), 0.7, z0, h)]

    def letter_y(x, y, s=1.3):
        return [stroke((x - s * 0.75, y + s), (x, y), 0.7, z0, h),
                stroke((x + s * 0.75, y + s), (x, y), 0.7, z0, h),
                stroke((x, y), (x, y - s), 0.7, z0, h)]

    (ux, uy), (vx, vy) = S_AX_X, S_AX_Y
    parts += arrow(ux, uy) + arrow(vx, vy)
    # letters stay upright, beside each arrow head on the side away from the other axis
    parts += letter_x(cx + 7.6 * ux - 3.0 * vx, cy + 7.6 * uy - 3.0 * vy)
    parts += letter_y(cx + 7.6 * vx - 3.0 * ux, cy + 7.6 * vy - 3.0 * uy)
    return parts[0].fuse(parts[1:])


def sensor_head():
    base, lid = shell_parts(s_o0, s_o0, s_o1, s_o1, s_z_split, s_z_top)
    ring, recess = lip(s_pk0, s_pk0, s_pk1, s_pk1, s_z_split, 0.5)
    collar, bore = cable_exit(s_mid, None, s_z_split, (0, -1), s_o0, s_pk0)

    # ---- base
    base = fuse(base, [ring, collar.common(below(s_z_split))])
    pocket = rbox(s_pk0, s_pk0, s_pk1, s_pk1, s_z_pcb_bot, s_z_split + 5, 0.5)
    cavity = rbox(s_pk0, s_pk0, s_pk1, s_pk1, s_z_floor, s_z_pcb_bot + 0.01, 0.5)
    cavity = cavity.cut([corner_region(hx, hy, S_BOSS_R, s_z_floor - 1, S_UNDER + 2, S_CLR) for hx, hy in s_holes])
    cuts = [pocket, cavity, bore]
    # base screws only where the standoffs are; their heads sit above the magnet
    for i, j in S_LID_CORNERS:
        hx, hy = s_holes[i + 2 * j]
        cuts += [cyl(hx, hy, -1, s_z_pcb_bot + 2, S_HOLE_D), cbore(hx, hy, 0, s_z_head, S_CBORE_D, S_HOLE_D)]
    # magnet in a pocket, flat head screw through it into a nut held from inside
    hole_d, nut_af, nut_h = S_MAG_NUT
    mag_d = S_MAG_D + 2 * S_MAG_CLR
    cuts += [cbore(s_mid, s_mid, 0, s_z_mag, mag_d, hole_d),
             Part.makeCone(mag_d / 2 + FOOT_CHAMFER + 0.01, mag_d / 2, FOOT_CHAMFER + 0.01, V(s_mid, s_mid, -0.01)),
             cyl(s_mid, s_mid, -1, s_z_floor + 2, hole_d),
             hex_prism(s_mid, s_mid, s_z_floor - nut_h, nut_h + 1, nut_af)]
    # the lip stops where the cable passes
    cuts.append(Part.makeBox(2 * (BORE_R + 0.6), LIP_T + 1, LIP_H + 1,
                             V(s_mid - BORE_R - 0.6, s_pk0 - LIP_T - 0.5, s_z_split)))
    base = cut(base, cuts)

    # ---- lid
    z_so_top = s_z_pcb_top + S_STANDOFF
    z_cb = s_z_top - S_CBORE_H - 2 * LAYER   # lid counterbore ceiling and its bridging layers
    bosses = []
    for i, j in S_LID_CORNERS:
        hx, hy = s_holes[i + 2 * j]
        bosses.append(corner_region(hx, hy, 3.2, z_so_top, s_z_ceil - z_so_top + 0.5, S_CLR))
        # the counterbore reaches below the lid plate, so the boss top widens to wall it in
        if z_cb < s_z_ceil:
            bosses.append(cyl(hx, hy, z_cb, s_z_ceil - z_cb + 0.5, S_CBORE_D + 2 * CB_WALL))
    lid = fuse(lid, [collar.common(above(s_z_split + SEAM_GAP))] + bosses)
    inner = rbox(s_pk0, s_pk0, s_pk1, s_pk1, s_z_split - 1, s_z_ceil, 0.5)
    inner = inner.cut(bosses)
    cuts = [inner, recess, bore, axis_mark(s_z_top)]
    for i, j in S_LID_CORNERS:
        hx, hy = s_holes[i + 2 * j]
        cuts += [cyl(hx, hy, z_so_top - 1, s_z_top, S_HOLE_D),
                 cbore(hx, hy, s_z_top, S_CBORE_H, S_CBORE_D, S_HOLE_D, up=False)]
    lid = cut(lid, cuts)
    return base, lid


# ================================================================ FT232H
# Adafruit FT232H (USB-C, product 2264). Positions from the board file
# Adafruit-FT232H-Breakout-PCB/Adafruit FT232H.brd, origin at the PCB corner.
F_W, F_H, F_T = 38.1, 22.86, 1.6
F_CLR = 0.3
F_USB_FACE = -0.635              # USB-C receptacle mouth overhangs the PCB edge
F_HOLES = [(2.54, 2.54), (35.56, 2.54), (2.54, 20.32), (35.56, 20.32)]
F_ROWS_Y = [1.27, 21.59]         # JP1, JP2, 11 pins each from x = 6.35
F_FLOOR = 3.0                    # holds the head counterbore and its two bridging layers
F_UNDER = 2.4                    # header tails under the board
F_BOSS_D = 5.0
F_SCREW_D = 2.7                  # M2.5 clearance
F_CBORE_D, F_CBORE_H = 4.9, 2.6  # M2.5 socket head from below
F_POST_D = 4.0                   # lid posts press the PCB at each hole (0.5 from the end housings)
F_PILOT_D = 2.1                  # M2.5 self-tapping into the post
F_PILOT_L = 10.0
F_USB_W, F_USB_HT, F_USB_R = 13.0, 7.2, 2.5     # opening for the plug overmold, edges on whole layers
F_USB_ZC = 1.6                   # receptacle centre above PCB top
F_STEMMA_TOP = 4.3               # cable stays above the Stemma QT connector
F_LID_TOP = 2.0

f_px0, f_px1 = F_USB_FACE - 0.2, F_W + F_CLR
f_py0, f_py1 = -F_CLR, F_H + F_CLR
f_ox0, f_ox1, f_oy0, f_oy1 = f_px0 - WALL, f_px1 + WALL, f_py0 - WALL, f_py1 + WALL
f_z_pcb_bot = F_FLOOR + F_UNDER
f_z_pcb_top = f_z_pcb_bot + F_T
f_z_split = on_layer(f_z_pcb_top + F_STEMMA_TOP + BORE_R + 0.3)
f_z_ceil = on_layer(f_z_pcb_top + HOUSING_TOP + WIRE_BEND)
f_z_top = f_z_ceil + F_LID_TOP
f_yc = F_H / 2


def ft232h_case():
    base, lid = shell_parts(f_ox0, f_oy0, f_ox1, f_oy1, f_z_split, f_z_top)
    ring, recess = lip(f_px0, f_py0, f_px1, f_py1, f_z_split, 0.5)
    collar, bore = cable_exit(None, f_yc, f_z_split, (1, 0), f_ox1, f_px1)

    # ---- tray
    base = fuse(base, [ring, collar.common(below(f_z_split))])
    base = cut(base, [rbox(f_px0, f_py0, f_px1, f_py1, F_FLOOR, f_z_split + 5, 0.5)])
    base = fuse(base, [cyl(x, y, F_FLOOR - 0.01, F_UNDER + 0.01, F_BOSS_D) for x, y in F_HOLES])
    cuts = [bore]
    for x, y in F_HOLES:
        cuts += [cyl(x, y, -1, f_z_pcb_bot + 2, F_SCREW_D), cbore(x, y, 0, F_CBORE_H, F_CBORE_D, F_SCREW_D)]
    zc = f_z_pcb_top + F_USB_ZC
    usb = rbox(-F_USB_HT / 2, -F_USB_W / 2, F_USB_HT / 2, F_USB_W / 2, 0, WALL + 2, F_USB_R)
    usb.rotate(V(0, 0, 0), V(0, 1, 0), -90)
    usb.translate(V(f_px0 + 0.5, f_yc, zc))
    cuts.append(usb)
    cuts.append(Part.makeBox(LIP_T + 1, 2 * (BORE_R + 0.6), LIP_H + 1,
                             V(f_px1 - 0.5, f_yc - BORE_R - 0.6, f_z_split)))
    base = cut(base, cuts)

    # ---- lid
    posts = [cyl(x, y, f_z_pcb_top, f_z_ceil - f_z_pcb_top + 0.5, F_POST_D) for x, y in F_HOLES]
    lid = fuse(lid, [collar.common(above(f_z_split + SEAM_GAP))])
    inner = rbox(f_px0, f_py0, f_px1, f_py1, f_z_split - 1, f_z_ceil, 0.5)
    lid = cut(lid, [inner, recess, bore])
    lid = fuse(lid, posts)
    cuts = [cyl(x, y, f_z_pcb_top - 1, F_PILOT_L + 1, F_PILOT_D) for x, y in F_HOLES]
    lid = cut(lid, cuts)
    return base, lid


# ================================================================ dummies for fit checks
def sensor_dummy():
    pcb = Part.makeBox(S_PCB, S_PCB, S_PCB_T, V(0, 0, s_z_pcb_bot))
    for hx, hy in s_holes:
        pcb = pcb.cut(cyl(hx, hy, 0, 50, 3.2))
    chip = Part.makeBox(6, 5.6, 2.2, V(s_mid - 3, s_mid - 2.8, s_z_pcb_top))
    so = [hex_prism(*s_holes[i + 2 * j], s_z_pcb_top, S_STANDOFF, 5.5).cut(cyl(*s_holes[i + 2 * j], 0, 50, 3))
          for i, j in S_LID_CORNERS]
    # envelope where housings may stand: whole PCB except the four corner squares, up to HOUSING_TOP
    env = Part.makeBox(S_PCB, S_PCB, HOUSING_TOP - 0.01, V(0, 0, s_z_pcb_top + 0.01))
    env = env.cut([corner_region(hx, hy, 3.3, s_z_pcb_top - 1, HOUSING_TOP + 2, 0) for hx, hy in s_holes])
    # cable drawn at the bore size: any overlap here means the bore is misplaced
    cable = Part.makeCylinder(BORE_R - 0.01, 30, V(s_mid, s_pk0 + 0.5, s_z_split), V(0, -1, 0))
    magnet = cyl(s_mid, s_mid, s_z_mag - S_MAG_T, S_MAG_T, S_MAG_D)
    return {"pcb": pcb, "chip": chip, "standoffs": so[0].fuse(so[1:]), "housing_zone": env, "cable": cable,
            "magnet": magnet}


def ft232h_dummy():
    pcb = Part.makeBox(F_W, F_H, F_T, V(0, 0, f_z_pcb_bot))
    for x, y in F_HOLES:
        pcb = pcb.cut(cyl(x, y, 0, 50, 2.5))
    t = f_z_pcb_top
    hdr = []
    for y in F_ROWS_Y:
        hdr.append(Part.makeBox(11 * 2.54, 2.54, 2.5, V(6.35 - 1.27, y - 1.27, t)))
        hdr.append(Part.makeBox(11 * 2.54, 2.54, HOUSING_TOP - 2.5, V(6.35 - 1.27, y - 1.27, t + 2.5)))
    usb = Part.makeBox(6.715 - F_USB_FACE, 8.94, 3.26, V(F_USB_FACE, f_yc - 4.47, t))
    stemma = Part.makeBox(37.65 - 33.4, 6.1, 2.9, V(33.4, 8.43, t))
    sw = Part.makeBox(3.5, 7.2, 1.6, V(28.194 - 1.75, 11.43 - 3.6, t))
    cable = Part.makeCylinder(BORE_R - 0.01, 30, V(f_px1 - 12, f_yc, f_z_split), V(1, 0, 0))
    return {"pcb": pcb, "headers": hdr[0].fuse(hdr[1:]), "usb": usb, "stemma": stemma, "switch": sw,
            "cable": cable}


# ================================================================ output
def report(label, part, dummies):
    print("%s: %.1f x %.1f x %.1f mm, %.2f cm3, valid=%s" % (
        label, part.BoundBox.XLength, part.BoundBox.YLength, part.BoundBox.ZLength, part.Volume / 1000, part.isValid()))
    for k, d in dummies.items():
        v = part.common(d).Volume
        if v > 1e-3:
            print("   OVERLAP with %s: %.3f mm3" % (k, v))


def main():
    os.makedirs(OUT, exist_ok=True)
    sb, sl = sensor_head()
    fb, fl = ft232h_case()
    sd, fd = sensor_dummy(), ft232h_dummy()
    report("sensor base", sb, sd)
    report("sensor lid", sl, sd)
    report("ft232h tray", fb, fd)
    report("ft232h lid", fl, fd)
    print("sensor: split %.2f, PCB top %.2f, ceiling %.2f, top %.2f" % (s_z_split, s_z_pcb_top, s_z_ceil, s_z_top))
    print("sensor magnet: pocket %.1f deep, %s nut %.1f..%.1f, PCB bottom %.1f" % (
        s_z_mag, S_MAG_SCREW, s_z_floor - S_MAG_NUT[2], s_z_floor, s_z_pcb_bot))
    tip_b, tip_l = s_z_head + S_BASE_SCREW, s_z_top - S_CBORE_H - S_LID_SCREW
    print("sensor screws: base M3x%g %.1f into standoff, lid M3x%g %.1f into standoff, tips %.1f apart" % (
        S_BASE_SCREW, tip_b - s_z_pcb_top, S_LID_SCREW, s_z_pcb_top + S_STANDOFF - tip_l, tip_l - tip_b))
    print("ft232h: split %.2f, PCB top %.2f, ceiling %.2f, top %.2f" % (f_z_split, f_z_pcb_top, f_z_ceil, f_z_top))
    # STEP only (slicers read it), in assembly position; the lids print upside down, so flip them in the slicer
    for name, shp in [("sensor_base", sb), ("sensor_lid", sl), ("ft232h_tray", fb), ("ft232h_lid", fl)]:
        shp.exportStep(os.path.join(OUT, name + ".step"))
    shift = V(40, 0, 0)
    Part.makeCompound([sb, sl, fb.translated(shift), fl.translated(shift)]).exportStep(os.path.join(OUT, "assembly.step"))
    Part.makeCompound(list(sd.values()) + [d.translated(shift) for d in fd.values()]).exportStep(
        os.path.join(OUT, "fit_dummies.step"))


# freecadcmd runs scripts with __name__ set to the file name, so call main() unconditionally
main()
