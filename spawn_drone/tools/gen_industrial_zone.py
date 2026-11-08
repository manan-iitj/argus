#!/usr/bin/env python3
"""Generator for the ARGUS industrial_zone.sdf world.

Layout is inspired by real industrial estates: a road grid of blocks, big
distribution warehouses with dock doors, sawtooth-roof manufacturing halls,
a petrochemical tank farm with bund walls and pipe racks, a power plant
with cooling towers, a cement plant with silos, a rail yard, a container
yard, an electrical substation, parking lots, a perimeter fence and a tree
belt.  Colours are deliberately muted (weathered cladding, concrete,
galvanised steel, faded paint).

The drone spawns at the origin, which is kept clear (central apron).

Usage:
    python3 gen_industrial_zone.py industrial_zone.sdf             # procedural
    python3 gen_industrial_zone.py industrial_zone_real.sdf --real # Fuel models

With --real the SAME layout is produced, but wherever a suitable real 3D
model exists on Gazebo Fuel (https://app.gazebosim.org) it is pulled in
directly from the internet via <include><uri>https://fuel.gazebosim.org/...
(buildings, cooling towers, power plant, water tower, tower crane,
transformers, cars/trucks, forklifts, lamp posts, poles, trees, barriers ...).
Structures with no usable Fuel model (oil tanks, silos, distillation columns,
pipe racks, shipping containers, rails, fence, roads) stay procedural.
"""

import math
import random
import sys
import urllib.parse

random.seed(7)

REAL = '--real' in sys.argv
WORLD_NAME = 'industrial_zone_real' if REAL else 'industrial_zone'

# --------------------------------------------------------------------------
# Muted palette (r, g, b)
# --------------------------------------------------------------------------
C = {
    'grass': (0.46, 0.49, 0.36),
    'dirt': (0.44, 0.41, 0.35),
    'slab': (0.50, 0.50, 0.48),
    'asphalt': (0.17, 0.17, 0.17),
    'asphalt_lot': (0.22, 0.22, 0.21),
    'mark_w': (0.72, 0.72, 0.69),
    'mark_y': (0.66, 0.59, 0.33),
    'concrete': (0.62, 0.61, 0.58),
    'concrete_dark': (0.48, 0.47, 0.45),
    'clad_light': (0.72, 0.72, 0.70),
    'clad_beige': (0.68, 0.65, 0.58),
    'clad_bluegray': (0.50, 0.54, 0.58),
    'clad_greengray': (0.47, 0.51, 0.47),
    'clad_sand': (0.63, 0.59, 0.50),
    'brick': (0.48, 0.33, 0.27),
    'roof_dark': (0.33, 0.34, 0.36),
    'roof_mid': (0.46, 0.47, 0.48),
    'roof_light': (0.58, 0.58, 0.57),
    'steel': (0.55, 0.56, 0.58),
    'steel_dark': (0.36, 0.37, 0.39),
    'rust': (0.43, 0.30, 0.22),
    'door': (0.30, 0.31, 0.33),
    'glass': (0.24, 0.29, 0.33),
    'tank_white': (0.78, 0.78, 0.76),
    'tank_gray': (0.64, 0.65, 0.65),
    'pipe_y': (0.62, 0.56, 0.34),
    'pipe_g': (0.50, 0.52, 0.50),
    'crane': (0.66, 0.58, 0.32),
    'chim_band_r': (0.58, 0.32, 0.29),
    'chim_band_w': (0.78, 0.77, 0.74),
    'tree': (0.20, 0.27, 0.17),
    'tree2': (0.24, 0.30, 0.19),
    'trunk': (0.30, 0.24, 0.19),
    'gravel': (0.47, 0.45, 0.42),
    'rail': (0.35, 0.33, 0.31),
    'sleeper': (0.30, 0.26, 0.22),
    'coal': (0.16, 0.16, 0.16),
    'sand': (0.60, 0.54, 0.42),
    'ore': (0.42, 0.33, 0.27),
}

CONTAINER_COLS = [
    (0.45, 0.25, 0.20), (0.30, 0.36, 0.45), (0.52, 0.52, 0.50),
    (0.28, 0.35, 0.29), (0.55, 0.45, 0.30), (0.40, 0.22, 0.22),
    (0.62, 0.62, 0.60), (0.35, 0.40, 0.44), (0.50, 0.33, 0.22),
]
CAR_COLS = [
    (0.75, 0.75, 0.74), (0.15, 0.15, 0.16), (0.50, 0.51, 0.53),
    (0.62, 0.63, 0.64), (0.25, 0.27, 0.32), (0.40, 0.18, 0.17),
    (0.82, 0.82, 0.80), (0.30, 0.32, 0.30),
]

INERTIAL = ('<inertial><mass>1</mass><inertia><ixx>1</ixx><ixy>0</ixy><ixz>0</ixz>'
            '<iyy>1</iyy><iyz>0</iyz><izz>1</izz></inertia></inertial>')

out = []
_names = set()


def uname(base):
    n, i = base, 1
    while n in _names:
        i += 1
        n = f'{base}_{i}'
    _names.add(n)
    return n


def f(v):
    s = f'{v:.3f}'.rstrip('0').rstrip('.')
    return '0' if s in ('-0', '') else s


def mat(c, a=1.0, emissive=None):
    r, g, b = c
    s = (f'<material><ambient>{f(r*0.85)} {f(g*0.85)} {f(b*0.85)} {f(a)}</ambient>'
         f'<diffuse>{f(r)} {f(g)} {f(b)} {f(a)}</diffuse>'
         f'<specular>0.08 0.08 0.08 1</specular>')
    if emissive:
        s += f'<emissive>{f(emissive[0])} {f(emissive[1])} {f(emissive[2])} 1</emissive>'
    return s + '</material>'


class Model:
    """A static model with one link holding many visuals/collisions."""

    def __init__(self, name, x=0.0, y=0.0, yaw=0.0):
        self.name = uname(name)
        self.x, self.y, self.yaw = x, y, yaw
        self.parts = []
        self.k = 0

    def _geom(self, kind, dims):
        if kind == 'box':
            return f'<box><size>{f(dims[0])} {f(dims[1])} {f(dims[2])}</size></box>'
        if kind == 'cyl':
            return f'<cylinder><radius>{f(dims[0])}</radius><length>{f(dims[1])}</length></cylinder>'
        if kind == 'sph':
            return f'<sphere><radius>{f(dims[0])}</radius></sphere>'
        if kind == 'ell':
            return f'<ellipsoid><radii>{f(dims[0])} {f(dims[1])} {f(dims[2])}</radii></ellipsoid>'
        raise ValueError(kind)

    def add(self, kind, dims, pose, color, collide=True, alpha=1.0, emissive=None):
        self.k += 1
        x, y, z = pose[:3]
        r, p, yw = (list(pose[3:]) + [0, 0, 0])[:3]
        ps = f'<pose>{f(x)} {f(y)} {f(z)} {f(r)} {f(p)} {f(yw)}</pose>'
        g = f'<geometry>{self._geom(kind, dims)}</geometry>'
        self.parts.append(f'<visual name="v{self.k}">{ps}{g}{mat(color, alpha, emissive)}</visual>')
        if collide:
            if kind == 'ell':
                # DART has no ellipsoid primitive; use an inscribed cylinder instead.
                g = f'<geometry>{self._geom("cyl", (min(dims[0], dims[1]) * 0.8, dims[2] * 1.6))}</geometry>'
            self.parts.append(f'<collision name="c{self.k}">{ps}{g}</collision>')

    def box(self, size, pos, color, collide=True, rpy=(0, 0, 0), **kw):
        self.add('box', size, tuple(pos) + tuple(rpy), color, collide, **kw)

    def cyl(self, r, length, pos, color, collide=True, rpy=(0, 0, 0), **kw):
        self.add('cyl', (r, length), tuple(pos) + tuple(rpy), color, collide, **kw)

    def emit(self):
        if not self.parts:
            return
        out.append(
            f'    <model name="{self.name}"><static>true</static><link name="link">\n      '
            + '\n      '.join(self.parts)
            + f'\n      {INERTIAL}\n    </link><pose>{f(self.x)} {f(self.y)} 0 0 0 {f(self.yaw)}</pose></model>')


def section(title):
    out.append(f'\n    <!-- ============ {title} ============ -->')


# --------------------------------------------------------------------------
# Gazebo Fuel models (only used with --real)
# --------------------------------------------------------------------------
FUEL = 'https://fuel.gazebosim.org/1.0'

# name -> (owner, footprint centre offset in the model frame [m], yaw offset
# so that the model's long axis lies along +x).  Offsets were measured from
# the downloaded meshes so that models can be placed by their footprint centre.
FUEL_MODELS = {
    'Office Building': ('OpenRobotics', (0.0, 0.0), 0.0),
    'Fire Station': ('OpenRobotics', (0.0, 0.19), 0.0),
    'Police station': ('OpenRobotics', (0.0, 0.0), 0.0),
    'Gas Station': ('OpenRobotics', (0.0, -9.2), 0.0),
    'Depot': ('OpenRobotics', (0.0, -0.13), 0.0),
    'Warehouse': ('OpenRobotics', (0.0, 0.0), 0.0),        # 30 x 50 m hall, ridge along y
    'Reactor': ('OpenRobotics', (0.0, 0.0), 0.0),          # cooling tower mesh
    'Water tower': ('OpenRobotics', (0.0, 0.0), 0.0),
    'Tower crane': ('OpenRobotics', (0.0, 0.0), 0.0),      # placed by mast
    'Telephone pole': ('OpenRobotics', (0.0, 0.0), 0.0),
    'Lamp Post': ('OpenRobotics', (0.0, 0.0), 0.0),        # arm points -y
    'Transformer': ('kevinhalim9', (1.1, 0.3), 0.0),
    'Hatchback': ('OpenRobotics', (0.3, 0.0), 0.0),
    'Hatchback red': ('OpenRobotics', (0.3, 0.0), 0.0),
    'Hatchback blue': ('OpenRobotics', (0.3, 0.0), 0.0),
    'SUV': ('OpenRobotics', (0.1, 0.0), 0.0),
    'Pickup': ('OpenRobotics', (-0.12, 0.0), 0.0),
    'TruckBox': ('OpenRobotics', (0.0, 0.0), 0.0),
    'TruckDelivery': ('OpenRobotics', (0.0, 0.0), 0.0),
    'Bus': ('OpenRobotics', (0.0, -0.05), -math.pi / 2),
    'Fire truck': ('OpenRobotics', (0.0, -0.3), -math.pi / 2),
    'Forklift': ('OpenRobotics', (0.0, 0.7), -math.pi / 2),
    'Oak tree': ('OpenRobotics', (-0.6, 0.0), 0.0),
    'Pine Tree': ('OpenRobotics', (0.0, 0.0), 0.0),
    'Jersey Barrier': ('OpenRobotics', (0.0, 0.0), 0.0),
    'Dumpster': ('OpenRobotics', (0.0, 0.0), 0.0),
    'Dumpster_Large': ('OpenRobotics', (0.0, 0.0), 0.0),
    'Construction Cone': ('OpenRobotics', (0.0, 0.0), 0.0),
    'Construction Barrel': ('OpenRobotics', (0.0, 0.0), 0.0),
    'Electrical Box': ('OpenRobotics', (0.0, 0.0), 0.0),
    'Fire hydrant': ('OpenRobotics', (0.0, 0.0), 0.0),
}
CAR_MODELS = ['Hatchback', 'Hatchback red', 'Hatchback blue', 'SUV', 'Pickup',
              'Hatchback', 'SUV']
fuel_count = {}


def fuel_uri(name):
    owner = FUEL_MODELS[name][0]
    return f'{FUEL}/{owner}/models/{urllib.parse.quote(name)}'


def fuel(name, x, y, yaw=0.0, z=0.0):
    """Include a Fuel model so that its footprint centre lands on (x, y)."""
    _, (cx, cy), yoff = FUEL_MODELS[name]
    a = yaw + yoff
    ox = x - (cx * math.cos(a) - cy * math.sin(a))
    oy = y - (cx * math.sin(a) + cy * math.cos(a))
    slug = name.lower().replace(' ', '_')
    fuel_count[name] = fuel_count.get(name, 0) + 1
    out.append(f'    <include><name>{uname(slug)}</name><uri>{fuel_uri(name)}</uri>'
               f'<pose>{f(ox)} {f(oy)} {f(z)} 0 0 {f(a)}</pose><static>true</static></include>')
    add_roof(name, ox, oy, a)


def add_roof(name, x, y, yaw):
    """The Fuel Warehouse / Depot are indoor robotics sets with no roof, which
    looks wrong from a drone.  Cap them with a roof matching the measured
    eave / ridge heights of the meshes."""
    if name == 'Warehouse':
        r = Model(f'roof_{slug_of(name)}', x, y, yaw)
        half, eave, ridge = 15.6, 10.1, 12.6
        pitch = math.atan2(ridge - eave, 15.0)
        pw = half / math.cos(pitch)
        for sgn in (-1, 1):   # ridge runs along model y
            r.box((pw, 51.0, 0.25), (sgn * half / 2, 0, eave + (ridge - eave) / 2 + 0.15),
                  C['roof_mid'], rpy=(0, sgn * pitch, 0))
        r.emit()
    elif name == 'Depot':
        r = Model(f'roof_{slug_of(name)}', x, y, yaw)
        r.box((30.4, 15.4, 0.3), (-0.04, -0.13, 10.95), C['roof_dark'])
        for k in range(4):   # roof lights / vents
            r.box((2.2, 2.2, 0.6), (-11 + k * 7.3, -0.13, 11.35), C['steel'], collide=False)
        r.emit()


def slug_of(name):
    return name.lower().replace(' ', '_')


def fuel_l(m, name, lx, ly, lyaw=0.0, z=0.0):
    """Same as fuel() but (lx, ly, lyaw) are given in the frame of Model m."""
    c, s = math.cos(m.yaw), math.sin(m.yaw)
    fuel(name, m.x + lx * c - ly * s, m.y + lx * s + ly * c, m.yaw + lyaw, z)


# --------------------------------------------------------------------------
# Building primitives (all in model-local frame, long axis along x)
# --------------------------------------------------------------------------

def building(m, cx, cy, L, W, H, wall, roof, roof_type='flat', pitch=0.18,
             docks=0, dock_side=-1, windows=0, hvac=0, parapet=True):
    """Generic industrial building.  roof_type: flat | pitched | sawtooth."""
    m.box((L, W, H), (cx, cy, H / 2), wall)
    if roof_type == 'flat':
        m.box((L + 0.2, W + 0.2, 0.4), (cx, cy, H + 0.2), roof, collide=False)
        if parapet:
            for s in (-1, 1):
                m.box((L + 0.2, 0.3, 0.9), (cx, cy + s * (W / 2), H + 0.45), wall, collide=False)
                m.box((0.3, W + 0.2, 0.9), (cx + s * (L / 2), cy, H + 0.45), wall, collide=False)
        for i in range(hvac):
            hx = cx + (i - (hvac - 1) / 2) * (L / max(hvac, 1)) * 0.8
            m.box((3, 2.2, 1.6), (hx, cy + W * 0.15, H + 1.2), C['steel'], collide=False)
    elif roof_type == 'pitched':
        half = W / 2 + 0.6
        pw = half / math.cos(pitch)
        rise = half * math.tan(pitch)
        for s in (-1, 1):
            m.box((L + 1.0, pw, 0.3), (cx, cy + s * half / 2, H + rise / 2),
                  roof, rpy=(-s * pitch, 0, 0))
        # gable ends (fill triangle approx with a thin box)
        for s in (-1, 1):
            m.box((0.3, W * 0.6, rise * 0.7), (cx + s * L / 2, cy, H + rise * 0.35), wall, collide=False)
    elif roof_type == 'sawtooth':
        m.box((L, W, 0.3), (cx, cy, H + 0.15), roof, collide=False)
        p = 8.0
        n = max(1, int(L // p))
        p = L / n
        a = 0.45
        t = p * math.tan(a) * 0.55
        sl = math.hypot(p, t)
        ang = math.atan2(t, p)
        for i in range(n):
            x0 = cx - L / 2 + i * p
            m.box((sl, W, 0.25), (x0 + p / 2, cy, H + 0.3 + t / 2), roof, rpy=(0, -ang, 0))
            m.box((0.2, W, t), (x0 + p, cy, H + 0.3 + t / 2), C['glass'], collide=False)
    # loading docks: dark dock doors + canopy on one long side
    if docks:
        sy = cy + dock_side * (W / 2 + 0.05)
        span = L * 0.85
        for i in range(docks):
            dx = cx - span / 2 + (i + 0.5) * span / docks
            m.box((3.0, 0.15, 3.4), (dx, sy, 2.4), C['door'], collide=False)
            m.box((3.6, 2.5, 0.12), (dx, sy + dock_side * 1.2, 1.1), C['concrete_dark'], collide=False)
        m.box((span + 4, 3.0, 0.25), (cx, sy + dock_side * 1.5, 5.0), C['roof_mid'], collide=False)
        m.box((span + 4, 3.0, 1.1), (cx, sy + dock_side * 1.5, 0.55), C['concrete'])
    # ribbon windows (office part)
    if windows:
        for k in range(windows):
            z = 3.0 + k * 3.5
            if z > H - 1:
                break
            m.box((L * 0.9, W + 0.12, 1.2), (cx, cy, z), C['glass'], collide=False)


def chimney(m, x, y, h, r, color=C['concrete'], bands=True):
    m.cyl(r, h, (x, y, h / 2), color)
    m.cyl(r * 1.25, 1.2, (x, y, 0.6), C['concrete_dark'], collide=False)
    if bands:
        for k, z in enumerate((h - 1.5, h - 4.5, h - 7.5)):
            col = C['chim_band_r'] if k % 2 == 0 else C['chim_band_w']
            m.cyl(r * 1.02, 2.5, (x, y, z), col, collide=False)
    # top platform ring
    m.cyl(r * 1.18, 0.4, (x, y, h - 6), C['steel_dark'], collide=False)


def tank(m, x, y, r, h, color=C['tank_white'], dome=True):
    m.cyl(r, h, (x, y, h / 2), color)
    if dome:
        m.add('ell', (r, r, r * 0.18), (x, y, h, 0, 0, 0), color, collide=False)
    # wind girder + spiral stair hint
    m.cyl(r * 1.02, 0.3, (x, y, h - 0.5), C['steel'], collide=False)
    m.box((1.0, 0.5, h), (x + r + 0.3, y, h / 2), C['steel_dark'], collide=False)


def hyperbolic_tower(m, x, y, h, rb, rt, rw):
    """Cooling tower approximated by stacked cylinder rings."""
    n = 14
    for i in range(n):
        z0 = h * i / n
        zc = z0 + h / n / 2
        t = zc / h
        # throat at ~78% height
        zt = 0.78
        if t < zt:
            r = rw + (rb - rw) * ((zt - t) / zt) ** 1.6
        else:
            r = rw + (rt - rw) * ((t - zt) / (1 - zt)) ** 1.6
        m.cyl(r, h / n + 0.02, (x, y, zc), C['concrete'], collide=True)


def pipe_x(m, x, y, z, length, r, color):
    m.cyl(r, length, (x, y, z), color, collide=False, rpy=(0, math.pi / 2, 0))


def pipe_y(m, x, y, z, length, r, color):
    m.cyl(r, length, (x, y, z), color, collide=False, rpy=(math.pi / 2, 0, 0))


def pipe_rack(m, x0, x1, y, z=6.0, nlev=2, pipes=4):
    L = x1 - x0
    n = max(2, int(L // 6) + 1)
    for i in range(n):
        px = x0 + i * L / (n - 1)
        for s in (-1, 1):
            m.box((0.4, 0.4, z + 2), (px, y + s * 2.0, (z + 2) / 2), C['steel_dark'])
        for lv in range(nlev):
            m.box((0.3, 4.4, 0.3), (px, y, z + lv * 2), C['steel_dark'], collide=False)
    cols = [C['pipe_g'], C['steel'], C['pipe_y'], C['tank_gray']]
    for lv in range(nlev):
        for k in range(pipes):
            py = y - 1.6 + k * 3.2 / max(pipes - 1, 1)
            r = 0.18 + 0.12 * ((k + lv) % 3)
            pipe_x(m, (x0 + x1) / 2, py, z + lv * 2 + 0.15 + r, L, r, cols[(k + lv) % 4])


def car(m, x, y, yaw, col):
    if REAL:
        fuel_l(m, random.choice(CAR_MODELS), x, y, yaw)
        return
    c, s = math.cos(yaw), math.sin(yaw)
    m.box((4.4, 1.8, 0.8), (x, y, 0.55), col, rpy=(0, 0, yaw))
    m.box((2.3, 1.6, 0.6), (x - 0.2 * c, y - 0.2 * s, 1.25), col, collide=False, rpy=(0, 0, yaw))
    m.box((2.32, 1.62, 0.4), (x - 0.2 * c, y - 0.2 * s, 1.25), C['glass'], collide=False, rpy=(0, 0, yaw))


def semi_truck(m, x, y, yaw, cab_col, trailer_col=C['clad_light']):
    c, s = math.cos(yaw), math.sin(yaw)
    if REAL:
        # Fuel has no articulated lorry; use a box truck where the trailer was.
        fuel_l(m, random.choice(('TruckBox', 'TruckDelivery', 'TruckBox')), x + 4 * c, y + 4 * s, yaw)
        return
    # trailer 13.6 m
    m.box((13.6, 2.5, 2.9), (x, y, 2.6), trailer_col, rpy=(0, 0, yaw))
    m.box((13.0, 2.0, 0.5), (x, y, 0.7), C['steel_dark'], collide=False, rpy=(0, 0, yaw))
    # cab
    cx, cy = x + 8.4 * c, y + 8.4 * s
    m.box((2.4, 2.5, 3.0), (cx, cy, 2.0), cab_col, rpy=(0, 0, yaw))
    m.box((3.5, 2.3, 0.6), (cx - 1.2 * c, cy - 1.2 * s, 0.8), C['steel_dark'], collide=False, rpy=(0, 0, yaw))


def container(m, x, y, z, yaw, col, length=12.2):
    m.box((length, 2.44, 2.59), (x, y, z + 1.295), col, rpy=(0, 0, yaw))


def parking_lot(m, cx, cy, L, W, rows, fill=0.7):
    m.box((L, W, 0.03), (cx, cy, 0.015), C['asphalt_lot'], collide=False)
    bays = int((L - 4) // 2.6)
    for r in range(rows):
        ry = cy - W / 2 + 3 + r * (W - 6) / max(rows - 1, 1)
        sgn = 1 if r % 2 == 0 else -1
        for b in range(bays + 1):
            bx = cx - L / 2 + 2 + b * 2.6
            m.box((0.12, 5.0, 0.01), (bx, ry + sgn * 2.5, 0.035), C['mark_w'], collide=False)
        for b in range(bays):
            if random.random() < fill:
                bx = cx - L / 2 + 2 + (b + 0.5) * 2.6
                car(m, bx, ry + sgn * 2.5, math.pi / 2 + random.uniform(-0.05, 0.05),
                    random.choice(CAR_COLS))


def light_pole(m, x, y, yaw=0.0, h=10.0):
    m.cyl(0.14, h, (x, y, h / 2), C['steel'])
    c, s = math.cos(yaw), math.sin(yaw)
    m.box((1.6, 0.15, 0.15), (x + 0.8 * c, y + 0.8 * s, h - 0.1), C['steel'], collide=False, rpy=(0, 0, yaw))
    m.box((0.7, 0.35, 0.18), (x + 1.5 * c, y + 1.5 * s, h - 0.25), C['mark_w'], collide=False,
          rpy=(0, 0, yaw), emissive=(0.55, 0.5, 0.38))


def tree(m, x, y, h=None):
    if REAL:
        fuel_l(m, 'Oak tree' if random.random() < 0.45 else 'Pine Tree', x, y,
               random.uniform(0, 2 * math.pi))
        return
    h = h or random.uniform(6, 11)
    m.cyl(0.25, h * 0.45, (x, y, h * 0.225), C['trunk'], collide=False)
    col = random.choice((C['tree'], C['tree2']))
    rr = h * random.uniform(0.22, 0.3)
    m.add('ell', (rr, rr, h * 0.33), (x, y, h * 0.62, 0, 0, 0), col, collide=True)


# ==========================================================================
# WORLD
# ==========================================================================
XS = [-190, -110, -30, 30, 110, 190]   # N-S roads
YS = [-140, -30, 30, 140]              # E-W roads
ROAD_W = 10.0
SITE_X, SITE_Y = 205, 155              # fence half-extents

section('TERRAIN')
g = Model('terrain')
g.box((1200, 1200, 0.02), (0, 0, -0.01), C['grass'], collide=False)
# rough dirt patches outside the fence
for _ in range(14):
    px = random.choice((-1, 1)) * random.uniform(230, 420)
    py = random.uniform(-400, 400)
    g.box((random.uniform(30, 90), random.uniform(20, 70), 0.02), (px, py, 0.0), C['dirt'],
          collide=False, rpy=(0, 0, random.uniform(0, 3)))
# site concrete / compacted gravel slab inside fence
g.box((2 * SITE_X, 2 * SITE_Y, 0.02), (0, 0, 0.005), C['slab'], collide=False)
g.emit()

out.append('''    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1200 1200</size></plane></geometry>
          <surface><friction><ode/></friction><bounce/><contact/></surface>
        </collision>
        <inertial><mass>1</mass><inertia><ixx>1</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1</iyy><iyz>0</iyz><izz>1</izz></inertia></inertial>
      </link>
    </model>''')

section('ROAD GRID + MARKINGS')
rd = Model('road_network')
for y in YS:
    rd.box((2 * SITE_X + 20, ROAD_W, 0.02), (0, y, 0.02), C['asphalt'], collide=False)
for x in XS:
    rd.box((ROAD_W, YS[-1] - YS[0] + ROAD_W, 0.02), (x, (YS[0] + YS[-1]) / 2, 0.021), C['asphalt'], collide=False)
# access road from the main gate (south) out to the public road
rd.box((ROAD_W + 2, 60, 0.02), (0, YS[0] - 35, 0.02), C['asphalt'], collide=False)
rd.box((900, 12, 0.02), (0, -SITE_Y - 60, 0.02), C['asphalt'], collide=False)
# dashed centre lines and edge lines
for y in YS:
    x = -SITE_X
    while x < SITE_X:
        if all(abs(x - xx) > ROAD_W for xx in XS):
            rd.box((4.0, 0.15, 0.005), (x, y, 0.033), C['mark_w'], collide=False)
        x += 12.0
    for s in (-1, 1):
        rd.box((2 * SITE_X, 0.15, 0.005), (0, y + s * (ROAD_W / 2 - 0.4), 0.033), C['mark_w'], collide=False)
for x in XS:
    y = YS[0]
    while y < YS[-1]:
        if all(abs(y - yy) > ROAD_W for yy in YS):
            rd.box((0.15, 4.0, 0.005), (x, y, 0.034), C['mark_w'], collide=False)
        y += 12.0
x = -440
while x < 440:
    rd.box((4.0, 0.18, 0.005), (x, -SITE_Y - 60, 0.033), C['mark_y'], collide=False)
    x += 16
rd.emit()

# --------------------------------------------------------------------------
section('CENTRAL APRON + HELIPAD (drone spawn at origin)')
ap = Model('central_apron')
ap.box((46, 46, 0.02), (0, 0, 0.012), C['concrete'], collide=False)
# expansion joints
for k in range(-2, 3):
    ap.box((46, 0.08, 0.004), (0, k * 9.2, 0.024), C['concrete_dark'], collide=False)
    ap.box((0.08, 46, 0.004), (k * 9.2, 0, 0.024), C['concrete_dark'], collide=False)
# faded painted helipad circle + H (very thin, flush)
for i in range(36):
    a = 2 * math.pi * i / 36
    ap.box((1.1, 0.3, 0.004), (7 * math.cos(a), 7 * math.sin(a), 0.026), C['mark_y'],
           collide=False, rpy=(0, 0, a + math.pi / 2))
ap.box((0.6, 5.0, 0.004), (-1.6, 0, 0.027), C['mark_w'], collide=False)
ap.box((0.6, 5.0, 0.004), (1.6, 0, 0.027), C['mark_w'], collide=False)
ap.box((2.6, 0.6, 0.004), (0, 0, 0.027), C['mark_w'], collide=False)
# windsock mast at the apron corner
ap.cyl(0.08, 6, (19, 19, 3), C['steel'])
ap.add('cyl', (0.35, 2.0), (19.9, 19, 5.7, 0, math.pi / 2, 0), C['chim_band_r'], collide=False)
ap.emit()

# small ops building + parking on the north side of central block
if REAL:
    fuel('Fire Station', -12, 18.5)          # on-site fire / ops station
    fuel('Fire truck', -26, 12, math.pi / 2)
else:
    ops = Model('ops_building', 0, 22)
    building(ops, -12, 0, 14, 8, 4.0, C['clad_light'], C['roof_mid'], windows=1, hvac=1)
    ops.emit()
pl = Model('parking_central', 10, 22)
parking_lot(pl, 0, 0, 22, 12, 2, fill=0.55)
pl.emit()

# --------------------------------------------------------------------------
section('ADMIN / OFFICE BLOCK')
if REAL:
    fuel('Office Building', -84, 10)
    fuel('Office Building', -56, 10, math.pi / 2)
else:
    of = Model('admin_office', -70, 0)
    building(of, 0, 8, 34, 14, 16.0, C['concrete'], C['roof_dark'], windows=4, hvac=3)
    # entrance canopy and stair core
    of.box((8, 4, 0.3), (0, -0.6, 4.2), C['steel_dark'], collide=False)
    of.box((5, 5, 19), (-15, 8, 9.5), C['concrete_dark'])
    of.emit()
pk = Model('parking_admin', -70, -16)
parking_lot(pk, 0, 0, 60, 14, 2, fill=0.75)
pk.emit()

# --------------------------------------------------------------------------
section('MAINTENANCE WORKSHOPS + STEEL LAYDOWN')
ws = Model('workshops', 70, 0)
if REAL:
    fuel_l(ws, 'Depot', -17.5, 12)
    building(ws, 20, 12, 20, 12, 7.0, C['clad_beige'], C['roof_mid'], roof_type='pitched', pitch=0.2)
else:
    building(ws, -18, 10, 30, 16, 9.0, C['clad_greengray'], C['roof_dark'], roof_type='pitched',
             pitch=0.16)
    for i in range(4):
        ws.box((4.2, 0.15, 5.0), (-29 + i * 7.5, 1.95, 2.5), C['door'], collide=False)
    building(ws, 20, 12, 20, 12, 7.0, C['clad_beige'], C['roof_mid'], roof_type='pitched', pitch=0.2)
# laydown: stacked steel beams / pipe bundles
for i in range(5):
    for j in range(3):
        ws.box((12, 0.4, 0.5), (12 + i * 0.02, -12 + i * 1.2, 0.25 + j * 0.5), C['rust'])
for i in range(6):
    pipe_x(ws, -10, -16 + i * 0.9, 0.45, 10, 0.4, C['steel_dark'])
if REAL:
    # tower crane: mast at the east end, jib swung west over the laydown
    fuel_l(ws, 'Tower crane', 28, -14, -math.pi / 2)
    fuel_l(ws, 'Forklift', -20, -8, 0.4)
    fuel_l(ws, 'Forklift', 4, -20, -1.2)
    fuel_l(ws, 'Dumpster_Large', -30, -20, math.pi / 2)
else:
    # gantry crane over laydown
    for sx in (-2, 26):
        for sy in (-20, -4):
            ws.box((0.8, 0.8, 11), (sx, sy, 5.5), C['crane'])
    for sx in (-2, 26):
        ws.box((0.9, 17, 1.0), (sx, -12, 11.4), C['crane'])
    ws.box((29, 1.2, 1.4), (12, -12, 12.2), C['crane'])
    ws.box((2.5, 2.0, 1.5), (14, -12, 11.0), C['steel_dark'], collide=False)
ws.emit()

# --------------------------------------------------------------------------
section('ELECTRICAL SUBSTATION')
ss = Model('substation', -150, 0)
ss.box((64, 44, 0.05), (0, 0, 0.03), C['gravel'], collide=False)
for i in range(3):
    tx = -18 + i * 16
    if REAL:
        fuel_l(ss, 'Transformer', tx, -8)
        continue
    ss.box((6, 4, 4.5), (tx, -8, 2.25), C['steel_dark'])          # transformer tank
    for k in range(4):
        ss.box((0.25, 3.6, 3.6), (tx - 3.4 - 0.4 * k, -8, 2.4), C['steel'], collide=False)  # radiators
    for k in range(3):
        ss.cyl(0.2, 2.5, (tx - 1.5 + 1.5 * k, -8, 5.7), C['tank_white'], collide=False)  # bushings
if REAL:
    for k in range(4):
        fuel_l(ss, 'Electrical Box', 14 + k * 1.2, -19)
# busbar gantries
for gx in (-24, -8, 8, 24):
    for gy in (6, 16):
        ss.box((0.5, 0.5, 12), (gx, gy, 6), C['steel'])
    ss.box((0.5, 11, 0.6), (gx, 11, 12), C['steel'], collide=False)
for gy in (8, 11, 14):
    pipe_x(ss, 0, gy, 11.4, 50, 0.06, C['steel'])
# control room
building(ss, 22, -14, 10, 6, 4, C['brick'], C['roof_dark'], hvac=1)
# security fence
for s in (-1, 1):
    ss.box((64, 0.1, 2.5), (0, s * 22, 1.25), C['steel'], alpha=0.55)
    ss.box((0.1, 44, 2.5), (s * 32, 0, 1.25), C['steel'], alpha=0.55)
ss.emit()

# transmission line leaving the substation to the west (outside the fence)
tl = Model('transmission_line')
for i, x in enumerate((-178, -240, -300, -360, -420)):
    h = 32
    tl.box((5, 5, 1), (x, 0, 0.5), C['concrete'], collide=False)
    tl.box((1.0, 1.0, h), (x, 0, h / 2), C['steel'])
    tl.box((3.0, 3.0, h * 0.4), (x, 0, h * 0.2), C['steel'], alpha=0.35, collide=False)
    for z, w in ((h - 1, 14), (h - 7, 18), (h - 13, 14)):
        tl.box((0.6, w, 0.6), (x, 0, z), C['steel'], collide=False)
for z, w in ((31, 14), (25, 18), (19, 14)):
    for s in (-1, 1):
        pipe_x(tl, -299, s * (w / 2 - 0.5), z - 1.2, 242, 0.04, C['steel_dark'])
tl.emit()
if REAL:
    # wooden distribution poles along the public road
    for x in range(-420, 421, 45):
        fuel('Telephone pole', x, -SITE_Y - 68)

# --------------------------------------------------------------------------
section('TRUCK MARSHALLING YARD')
ty = Model('truck_yard', 150, 0)
ty.box((70, 46, 0.02), (0, 0, 0.026), C['asphalt_lot'], collide=False)
for i in range(9):
    yy = -18 + i * 4.5
    ty.box((16, 0.15, 0.005), (-16, yy, 0.04), C['mark_w'], collide=False)
    if random.random() < 0.75:
        semi_truck(ty, -18, yy + 2.25, 0, random.choice(CAR_COLS[:6]),
                   random.choice((C['clad_light'], C['tank_gray'], C['clad_bluegray'])))
for i in range(6):
    container(ty, 20, -15 + i * 3.0, 0, 0, random.choice(CONTAINER_COLS))
# weighbridge hut
building(ty, 24, 16, 6, 4, 3, C['clad_light'], C['roof_mid'])
ty.box((18, 3.5, 0.15), (0, 18, 0.08), C['steel_dark'], collide=False)
ty.emit()

# --------------------------------------------------------------------------
section('TANK FARM (north-west)')
tf = Model('tank_farm', -150, 85)
tf.box((70, 100, 0.03), (0, 0, 0.03), C['gravel'], collide=False)
tanks = [(-20, -30, 10, 14), (6, -30, 10, 14), (-20, 0, 9, 12), (6, 0, 9, 12),
         (-20, 28, 7, 10), (2, 28, 7, 10), (22, 28, 5, 8)]
for (x, y, r, h) in tanks:
    # bund wall around each tank
    b = r + 4
    for s in (-1, 1):
        tf.box((2 * b, 0.6, 1.4), (x, y + s * b, 0.7), C['concrete'])
        tf.box((0.6, 2 * b, 1.4), (x + s * b, y, 0.7), C['concrete'])
    tank(tf, x, y, r, h, random.choice((C['tank_white'], C['tank_white'], C['tank_gray'])))
# spherical LPG tanks
for (x, y) in ((26, -30), (26, -10)):
    for k in range(6):
        a = k * math.pi / 3
        tf.box((0.5, 0.5, 7), (x + 5 * math.cos(a), y + 5 * math.sin(a), 3.5), C['steel_dark'])
    tf.add('sph', (6.5,), (x, y, 9), C['tank_white'])
pipe_rack(tf, -30, 30, 15, z=5.0, nlev=2, pipes=4)
# loading gantry for road tankers
for i in range(4):
    tf.box((0.5, 0.5, 7), (30 - i * 6, -48, 3.5), C['steel_dark'])
tf.box((22, 6, 0.4), (21, -48, 7.2), C['roof_mid'], collide=False)
semi_truck(tf, 18, -48, math.pi, CAR_COLS[6], C['tank_gray'])
tf.emit()

# water tower in the tank farm corner
if REAL:
    fuel('Water tower', -178, 130)
else:
    wt = Model('water_tower', -178, 130)
    for k in range(4):
        a = k * math.pi / 2 + math.pi / 4
        wt.cyl(0.45, 22, (3.5 * math.cos(a), 3.5 * math.sin(a), 11), C['steel'])
    wt.cyl(0.6, 22, (0, 0, 11), C['steel_dark'])
    wt.cyl(6, 7, (0, 0, 25.5), C['tank_gray'])
    wt.add('ell', (6, 6, 2.2), (0, 0, 29, 0, 0, 0), C['tank_gray'], collide=False)
    wt.cyl(6.4, 0.3, (0, 0, 22), C['steel_dark'], collide=False)
    wt.emit()

# --------------------------------------------------------------------------
section('CHEMICAL PROCESS UNIT')
cp = Model('process_unit', -70, 85)
cp.box((70, 100, 0.03), (0, 0, 0.03), C['concrete_dark'], collide=False)
# distillation columns with platforms
for (x, y, r, h) in ((-18, 20, 2.2, 42), (-8, 20, 1.8, 34), (2, 20, 1.5, 28), (12, 22, 2.6, 24)):
    cp.cyl(r, h, (x, y, h / 2), C['tank_white'])
    cp.add('ell', (r, r, r * 0.6), (x, y, h, 0, 0, 0), C['tank_white'], collide=False)
    for z in range(6, int(h), 7):
        cp.cyl(r + 0.9, 0.2, (x, y, z), C['steel_dark'], collide=False)
    cp.box((0.6, 0.6, h), (x + r + 0.8, y, h / 2), C['steel_dark'], collide=False)
# horizontal drums and heat exchangers
for i in range(4):
    cp.add('cyl', (1.5, 9), (-16 + i * 9, 2, 3.0, 0, math.pi / 2, 0), C['tank_gray'])
    for s in (-1, 1):
        cp.box((1.0, 2.5, 1.6), (-16 + i * 9 + s * 3, 2, 0.8), C['concrete'])
# steel structure (multi-level skid)
for sx in range(4):
    for sy in range(3):
        cp.box((0.5, 0.5, 18), (-20 + sx * 8, -22 + sy * 7, 9), C['steel_dark'])
for z in (6, 12, 18):
    cp.box((25, 15, 0.25), (-8, -15, z), C['steel'], collide=False, alpha=0.9)
for i in range(3):
    cp.cyl(1.2, 6, (-14 + i * 6, -15, 15), C['tank_white'], collide=False)
pipe_rack(cp, -30, 30, -40, z=6.0, nlev=3, pipes=5)
pipe_rack(cp, -30, 30, 38, z=6.0, nlev=2, pipes=4)
# flare stack
cp.cyl(0.9, 55, (26, -20, 27.5), C['steel'])
for k in range(3):
    a = k * 2 * math.pi / 3
    cp.cyl(0.05, 52, (26 + 6 * math.cos(a), -20 + 6 * math.sin(a), 26), C['steel_dark'],
           collide=False, rpy=(-0.115 * math.sin(a), 0.115 * math.cos(a), 0))
cp.cyl(1.2, 2.0, (26, -20, 56), C['steel_dark'], collide=False)
building(cp, 22, 40, 12, 8, 4.5, C['clad_bluegray'], C['roof_dark'], hvac=2)
cp.emit()
if REAL:
    for k in range(6):
        fuel_l(cp, 'Jersey Barrier', -30 + k * 4.2, -47)
    for k in range(5):
        fuel_l(cp, 'Construction Barrel', 30, -10 + k * 1.2)
    fuel_l(cp, 'Fire hydrant', -32, 0)
    fuel_l(cp, 'Fire hydrant', 32, 30)

# --------------------------------------------------------------------------
section('DISTRIBUTION WAREHOUSE (north-centre)')
dw = Model('distribution_centre', 0, 85)
if REAL:
    fuel_l(dw, 'Warehouse', -16, 8)
    fuel_l(dw, 'Warehouse', 16, 8)
    # delivery trucks queued in the yard south of the building
    for i in range(7):
        if random.random() < 0.75:
            fuel_l(dw, random.choice(('TruckBox', 'TruckDelivery')), -27 + i * 9, -42, math.pi / 2)
else:
    building(dw, 0, 8, 54, 60, 13.0, C['clad_light'], C['roof_light'], docks=12, dock_side=-1, hvac=5)
    # trailers backed into docks
    for i in range(12):
        if random.random() < 0.6:
            dx = -54 * 0.85 / 2 + (i + 0.5) * 54 * 0.85 / 12
            semi_truck(dw, dx, -32, -math.pi / 2, random.choice(CAR_COLS[:6]))
    for k in range(5):
        light_pole(dw, -24 + k * 12, -45, math.pi / 2)
dw.emit()

# --------------------------------------------------------------------------
section('SAWTOOTH MANUFACTURING HALL (north-east)')
mh = Model('manufacturing_hall', 70, 85)
if REAL:
    fuel_l(mh, 'Warehouse', 8, 10)
    fuel_l(mh, 'Police station', -24, -20, math.pi / 2)   # plant admin / security
    chimney(mh, 28, -32, 30, 1.2, C['brick'], bands=False)
    fuel_l(mh, 'Dumpster', -30, 20, math.pi / 2)
    fuel_l(mh, 'Dumpster', -30, 23, math.pi / 2)
else:
    building(mh, 0, 10, 64, 44, 10.0, C['brick'], C['roof_mid'], roof_type='sawtooth')
    building(mh, -14, -22, 30, 12, 8.0, C['concrete'], C['roof_dark'], windows=2, hvac=2)
    chimney(mh, 26, -22, 30, 1.2, C['brick'], bands=False)
    for i in range(4):
        mh.box((4.5, 0.15, 5.5), (-24 + i * 14, -12.1, 2.75), C['door'], collide=False)
    # extraction ducts on the roof
    for i in range(3):
        mh.cyl(0.6, 4, (-20 + i * 20, 20, 14), C['steel'], collide=False)
mh.emit()

# --------------------------------------------------------------------------
section('CONTAINER TERMINAL (north-east)')
ct = Model('container_yard', 150, 85)
ct.box((70, 100, 0.03), (0, 0, 0.03), C['asphalt_lot'], collide=False)
for row in range(5):
    for col in range(6):
        x = -24 + col * 13.0
        y = -38 + row * 16.0
        for bay in range(2):
            yy = y + bay * 2.7
            stack = random.choice((1, 2, 2, 3, 3, 4))
            for lvl in range(stack):
                container(ct, x, yy, lvl * 2.59, 0, random.choice(CONTAINER_COLS))
# rubber-tyred gantry cranes
for gy in (-26, 6, 38):
    for sx in (-32, 32):
        for sy in (-6, 6):
            ct.box((1.0, 1.0, 18), (sx, gy + sy, 9), C['crane'])
        ct.box((1.2, 13, 1.2), (sx, gy, 18.5), C['crane'], collide=False)
    for sy in (-6, 6):
        ct.box((66, 1.0, 1.4), (0, gy + sy, 18.6), C['crane'])
    ct.box((4, 13, 2.5), (random.uniform(-20, 20), gy, 17.5), C['steel_dark'], collide=False)
# reach stacker
ct.box((9, 3.5, 3), (30, -46, 1.5), C['crane'])
ct.box((10, 0.8, 0.8), (26, -46, 5), C['crane'], collide=False, rpy=(0, -0.45, 0))
ct.emit()

# --------------------------------------------------------------------------
section('POWER PLANT + COOLING TOWERS (south-west)')
pp = Model('power_plant', -150, -85)
pp.box((70, 100, 0.03), (0, 0, 0.03), C['gravel'], collide=False)
if REAL:
    # Fuel "Reactor" is a hyperbolic cooling tower (21 m dia, 26 m high).
    fuel_l(pp, 'Reactor', -13, 26)
    fuel_l(pp, 'Reactor', 13, 26)
    # Fuel "Power Plant" (turbine hall + 5 stacks).  Its mesh collision crashes
    # DART/ODE, so the Fuel mesh is used as a visual only, scaled to fit the
    # block, with simple box/cylinder collisions measured from the mesh.
    S = 0.65
    PP_MESH = (f'{FUEL}/openrobotics/models/power%20plant/3/files/meshes/powerplant.dae')
    ox, oy = 1.9, -27.1   # model origin in block frame (centres the footprint)
    gx, gy = pp.x + ox, pp.y + oy
    cols = [('box', (85, 57, 16), (0, -1, 8)), ('box', (70, 24, 24), (-3, 19, 12)),
            ('cyl', (2.1, 48), (-31, 33.4, 24)), ('cyl', (1.4, 41), (-15.5, 29.5, 20.5)),
            ('cyl', (1.4, 41), (0, 29.6, 20.5)), ('cyl', (1.4, 24), (15.3, 29.6, 12)),
            ('cyl', (1.4, 24), (30.7, 29.6, 12))]
    cs = ''
    for i, (k, d, p) in enumerate(cols):
        g = (f'<box><size>{f(d[0]*S)} {f(d[1]*S)} {f(d[2]*S)}</size></box>' if k == 'box' else
             f'<cylinder><radius>{f(d[0]*S)}</radius><length>{f(d[1]*S)}</length></cylinder>')
        cs += (f'<collision name="c{i}"><pose>{f(p[0]*S)} {f(p[1]*S)} {f(p[2]*S)} 0 0 0</pose>'
               f'<geometry>{g}</geometry></collision>')
    out.append(f'    <model name="{uname("fuel_power_plant")}"><static>true</static><link name="link">'
               f'<visual name="v"><geometry><mesh><uri>{PP_MESH}</uri><scale>{S} {S} {S}</scale>'
               f'</mesh></geometry></visual>{cs}{INERTIAL}</link>'
               f'<pose>{f(gx)} {f(gy)} 0 0 0 0</pose></model>')
    fuel_count['Power Plant (mesh)'] = 1
else:
    hyperbolic_tower(pp, -14, 22, 48, 17, 11.5, 10)
    hyperbolic_tower(pp, 18, 26, 42, 15, 10, 8.8)
    # boiler house + turbine hall
    building(pp, -6, -22, 30, 22, 34.0, C['clad_bluegray'], C['roof_dark'], hvac=0)
    building(pp, 22, -24, 18, 30, 18.0, C['clad_light'], C['roof_mid'], roof_type='pitched', pitch=0.1)
    chimney(pp, -28, -40, 110, 3.6)
    # coal / biomass conveyor gallery to boiler house
    pp.box((40, 3, 3), (-34, -6, 22), C['clad_greengray'], rpy=(0, 0.45, 0.4), collide=False)
    for k in range(3):
        pp.box((0.6, 0.6, 8 + k * 7), (-44 + k * 9, -10 + k * 3.5, (8 + k * 7) / 2), C['steel_dark'])
pp.emit()

# --------------------------------------------------------------------------
section('STEEL WORKS HALL (south)')
sw = Model('steel_works', -70, -85)
if REAL:
    # row of brick industrial halls (Fuel "Depot")
    fuel_l(sw, 'Depot', -16, 14)
    fuel_l(sw, 'Depot', 16, 14)
    building(sw, -6, -24, 46, 18, 14.0, C['clad_greengray'], C['roof_mid'], roof_type='pitched',
             pitch=0.15, docks=0)
else:
    building(sw, 0, 10, 62, 30, 24.0, C['rust'], C['roof_dark'], roof_type='pitched', pitch=0.12)
    # monitor roof (clerestory) along ridge
    sw.box((58, 6, 3), (0, 10, 28.2), C['clad_greengray'], collide=False)
    sw.box((60, 7, 0.4), (0, 10, 29.8), C['roof_dark'], collide=False)
    building(sw, -6, -24, 46, 18, 14.0, C['clad_greengray'], C['roof_mid'], roof_type='pitched',
             pitch=0.15, docks=0)
    for i in range(5):
        sw.box((6, 0.15, 9), (-26 + i * 13, -5.1, 4.5), C['door'], collide=False)
for i in range(3):
    chimney(sw, 14 + i * 7, -38, 40 + i * 6, 1.6 + 0.2 * i, C['steel'])
# slab / coil storage outside
for i in range(8):
    sw.cyl(1.0, 1.5, (-28 + i * 2.6, -42, 1.0), C['steel'], rpy=(math.pi / 2, 0, 0))
sw.emit()

# --------------------------------------------------------------------------
section('WAREHOUSES + PARKING (south-centre)')
wh = Model('warehouses_south', 0, -85)
if REAL:
    fuel_l(wh, 'Warehouse', -12, 20)
    fuel_l(wh, 'Warehouse', 18, -18)
    fuel_l(wh, 'Police station', 18, 24)
    fuel_l(wh, 'Dumpster_Large', 30, 12, math.pi / 2)
else:
    building(wh, -12, 20, 32, 28, 11.0, C['clad_sand'], C['roof_mid'], docks=6, dock_side=1, hvac=3)
    building(wh, 14, -18, 24, 26, 9.0, C['clad_bluegray'], C['roof_dark'], docks=5, dock_side=-1,
             roof_type='pitched', pitch=0.1)
    building(wh, 16, 22, 14, 20, 8.0, C['clad_light'], C['roof_mid'], hvac=1)
parking_lot(wh, -14, -22, 26, 26, 3, fill=0.65)
wh.emit()

# --------------------------------------------------------------------------
section('CEMENT PLANT / SILOS (south-east)')
cm = Model('cement_plant', 70, -85)
cm.box((70, 100, 0.03), (0, 0, 0.03), C['gravel'], collide=False)
for i in range(4):
    for j in range(2):
        x, y = -22 + i * 9, 18 + j * 9
        cm.cyl(4.2, 28, (x, y, 14), C['concrete'])
        cm.cyl(4.3, 0.6, (x, y, 28.3), C['concrete_dark'], collide=False)
# silo top gallery
cm.box((32, 4, 3), (-8.5, 22.5, 30), C['clad_light'], collide=False)
# preheater tower
for sx in (-1, 1):
    for sy in (-1, 1):
        cm.box((0.8, 0.8, 58), (22 + sx * 6, 20 + sy * 6, 29), C['steel_dark'])
for z in range(8, 58, 10):
    cm.box((13, 13, 0.4), (22, 20, z), C['steel'], collide=False, alpha=0.95)
for z in (14, 28, 42):
    cm.cyl(2.6, 7, (20, 20, z), C['tank_gray'], collide=False)
cm.box((10, 10, 6), (22, 20, 58), C['clad_light'])
# rotary kiln on piers
cm.add('cyl', (2.4, 58), (-4, -10, 6, 0, math.pi / 2 - 0.035, 0), C['steel_dark'])
for k in range(4):
    cm.box((2.5, 6, 4.5 - 0.4 * k), (-26 + k * 15, -10, (4.5 - 0.4 * k) / 2), C['concrete'])
# conveyor from quarry pile
cm.box((48, 2.4, 2.4), (6, -30, 12), C['clad_greengray'], rpy=(0, -0.38, 0), collide=False)
for k in range(4):
    cm.box((0.6, 0.6, 3 + k * 5), (-12 + k * 12, -30, (3 + k * 5) / 2), C['steel_dark'])
cm.add('ell', (14, 10, 7), (-26, -38, 0, 0, 0, 0), C['sand'])
cm.emit()
if REAL:
    fuel_l(cm, 'TruckBox', 26, -2, math.pi / 2)
    fuel_l(cm, 'Dumpster_Large', 10, 40)
    fuel_l(cm, 'Forklift', -30, 2, 0.8)

# --------------------------------------------------------------------------
section('RAIL YARD + BULK STORAGE (south-east)')
ry = Model('rail_yard', 150, -85)
ry.box((70, 100, 0.03), (0, 0, 0.03), C['dirt'], collide=False)
for t in range(4):
    tx = -24 + t * 6
    ry.box((3.2, 100, 0.3), (tx, 0, 0.15), C['gravel'], collide=False)
    for s in (-1, 1):
        ry.box((0.08, 100, 0.15), (tx + s * 0.72, 0, 0.38), C['rail'], collide=False)
    ry.box((2.5, 100, 0.1), (tx, 0, 0.32), C['sleeper'], collide=False)
    # freight wagons
    if t < 3:
        y = -40 + t * 4
        while y < 42:
            if random.random() < 0.8:
                if t == 0:   # hopper wagons
                    ry.box((2.9, 13.5, 3.2), (tx, y, 2.6), C['rust'])
                elif t == 1:  # tank wagons
                    ry.add('cyl', (1.4, 13), (tx, y, 2.6, math.pi / 2, 0, 0), C['tank_gray'])
                    ry.box((2.2, 14, 0.4), (tx, y, 1.0), C['steel_dark'], collide=False)
                else:         # flat wagons with containers
                    ry.box((2.6, 14, 0.5), (tx, y, 1.1), C['steel_dark'])
                    container(ry, tx, y, 1.35, math.pi / 2, random.choice(CONTAINER_COLS))
            y += 15
# bulk piles
ry.add('ell', (12, 18, 8), (16, 20, 0, 0, 0, 0), C['coal'])
ry.add('ell', (11, 14, 7), (16, -18, 0, 0, 0, 0), C['ore'])
ry.add('ell', (8, 9, 5), (24, -42, 0, 0, 0, 0), C['sand'])
# stacker/reclaimer boom
ry.box((4, 4, 3), (4, 0, 1.5), C['crane'])
ry.box((30, 1.5, 1.5), (16, 0, 9), C['crane'], rpy=(0, -0.25, 0.2), collide=False)
ry.box((1.0, 1.0, 8), (4, 0, 7), C['crane'])
ry.emit()

# --------------------------------------------------------------------------
section('STREET LIGHTS')
sl = Model('street_lights')
for y in YS:
    x = -SITE_X + 15
    while x < SITE_X:
        if all(abs(x - xx) > 8 for xx in XS):
            side = 1 if (int(x) // 40) % 2 == 0 else -1
            if REAL:
                # Fuel lamp post arm points along -y: face it toward the road
                fuel('Lamp Post', x, y + side * (ROAD_W / 2 + 1), 0.0 if side > 0 else math.pi)
                x += 40
                continue
            sl.cyl(0.14, 10, (x, y + side * (ROAD_W / 2 + 1), 5), C['steel'])
            sl.box((0.7, 0.35, 0.18), (x, y + side * (ROAD_W / 2 - 0.5), 9.8), C['mark_w'],
                   collide=False, emissive=(0.55, 0.5, 0.38))
        x += 40
sl.emit()

# --------------------------------------------------------------------------
section('PERIMETER FENCE + MAIN GATE')
fe = Model('perimeter_fence')
H = 2.6
gate_half = 9
# (Fuel has no usable chain-link fence, so this stays procedural in both modes)
fe.box((2 * SITE_X, 0.08, H), (0, SITE_Y, H / 2), C['steel'], alpha=0.5)
for s in (-1, 1):
    fe.box((0.08, 2 * SITE_Y, H), (s * SITE_X, 0, H / 2), C['steel'], alpha=0.5)
seg = SITE_X - gate_half
for s in (-1, 1):
    fe.box((seg, 0.08, H), (s * (gate_half + seg / 2), -SITE_Y, H / 2), C['steel'], alpha=0.5)
# fence posts
for x in range(-SITE_X, SITE_X + 1, 20):
    for y in (-SITE_Y, SITE_Y):
        if y == -SITE_Y and abs(x) < gate_half:
            continue
        fe.cyl(0.06, H + 0.3, (x, y, (H + 0.3) / 2), C['steel_dark'], collide=False)
for y in range(-SITE_Y, SITE_Y + 1, 20):
    for x in (-SITE_X, SITE_X):
        fe.cyl(0.06, H + 0.3, (x, y, (H + 0.3) / 2), C['steel_dark'], collide=False)
# gate: boom barriers + guard house
for s in (-1, 1):
    fe.box((0.4, 0.4, 1.1), (s * gate_half, -SITE_Y, 0.55), C['concrete_dark'])
    fe.box((7.5, 0.12, 0.12), (s * (gate_half - 3.8), -SITE_Y, 1.0), C['mark_w'], collide=False)
building(fe, gate_half + 6, -SITE_Y + 6, 6, 4, 3.0, C['clad_light'], C['roof_dark'], windows=1)
fe.box((5, 14, 0.3), (0, -SITE_Y + 2, 5.2), C['roof_mid'], collide=False)
for s in (-1, 1):
    fe.box((0.4, 0.4, 5.2), (s * 2.3, -SITE_Y - 4, 2.6), C['steel_dark'])
fe.emit()
if REAL:
    # traffic calming at the gate
    for s in (-1, 1):
        fuel('Jersey Barrier', s * 11.5, -SITE_Y - 10, math.pi / 2)
        fuel('Jersey Barrier', s * 11.5, -SITE_Y - 15, math.pi / 2)
        for k in range(3):
            fuel('Construction Cone', s * 4.5, -SITE_Y - 9 - k * 3)
    fuel('Fire hydrant', 20, -SITE_Y + 3)

# --------------------------------------------------------------------------
section('TREE BELT + OUTSIDE VEGETATION')
tr = Model('trees')
TSTEP = 12 if REAL else 9
for x in range(-SITE_X - 10, SITE_X + 11, TSTEP):
    for y in (SITE_Y + 8, -SITE_Y - 8):
        if y < 0 and abs(x) < 20:
            continue
        tree(tr, x + random.uniform(-2, 2), y + random.uniform(-2, 2))
for y in range(-SITE_Y, SITE_Y + 1, TSTEP):
    for x in (SITE_X + 8, -SITE_X - 8):
        if x < 0 and abs(y) < 8:
            continue  # transmission line corridor
        tree(tr, x + random.uniform(-2, 2), y + random.uniform(-2, 2))
# scattered groves further out
for _ in range(40 if REAL else 60):
    x = random.uniform(-450, 450)
    y = random.uniform(-450, 450)
    if abs(x) < SITE_X + 25 and abs(y) < SITE_Y + 25:
        continue
    if abs(y + SITE_Y + 60) < 10 or abs(y) < 10 and x < 0:
        continue
    tree(tr, x, y, random.uniform(7, 14))
tr.emit()

# --------------------------------------------------------------------------
section('NEIGHBOURING SITES (distant context, outside fence)')
nb = Model('neighbour_sites')
for (x, y, L, W, H, wall, roof) in (
        (330, 60, 70, 50, 12, C['clad_light'], C['roof_mid']),
        (330, -80, 50, 60, 10, C['clad_beige'], C['roof_dark']),
        (-340, 120, 60, 40, 9, C['clad_bluegray'], C['roof_dark']),
        (-330, -110, 80, 45, 14, C['clad_light'], C['roof_light']),
        (60, 280, 90, 50, 13, C['clad_sand'], C['roof_mid']),
        (-120, 280, 60, 40, 10, C['clad_greengray'], C['roof_dark']),
        (180, -290, 70, 40, 11, C['clad_light'], C['roof_mid'])):
    if REAL:
        continue
    building(nb, x, y, L, W, H, wall, roof, hvac=2)
nb.emit()
if REAL:
    for (n, x, y, yaw) in (('Warehouse', 330, 60, math.pi / 2), ('Warehouse', 330, -90, 0),
                           ('Office Building', -340, 120, 0), ('Fire Station', -330, -110, 0),
                           ('Warehouse', 60, 280, math.pi / 2), ('Warehouse', -120, 280, math.pi / 2),
                           ('Office Building', 180, -290, 0), ('Gas Station', 120, -240, 0)):
        fuel(n, x, y, yaw)
    # a little traffic on the public road
    for (n, x, yaw) in (('Bus', -150, 0.0), ('TruckDelivery', -60, math.pi), ('SUV', 40, 0.0),
                        ('Hatchback red', 95, math.pi), ('TruckBox', 210, 0.0), ('Pickup', 300, math.pi)):
        fuel(n, x, -SITE_Y - 60 + (-3 if yaw == 0.0 else 3), yaw)

# ==========================================================================
HEADER = '''<?xml version="1.0" encoding="UTF-8"?>
<!--
  ARGUS industrial_zone world.
  ~410 m x 310 m fenced industrial estate (road grid of 15 blocks) with a
  distribution centre, sawtooth manufacturing hall, steel works, power plant
  with cooling towers, cement plant, chemical process unit, tank farm,
  container terminal, rail yard, substation, offices, parking and a tree belt.
  The drone spawns at the origin on the central apron / helipad.
-->
<sdf version="1.9">
  <world name="industrial_zone">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.68 0.72 0.76 1</background>
      <shadows>true</shadows>
    </scene>
'''

FOOTER = '''
    <!-- SUN (slightly warm, low-ish afternoon sun for readable shadows) -->
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>-0.35 0.55 -0.76</direction>
      <diffuse>0.88 0.86 0.82 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>3000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic></attenuation>
      <spot><inner_angle>0</inner_angle><outer_angle>0</outer_angle><falloff>0</falloff></spot>
    </light>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>488</elevation>
    </spherical_coordinates>
  </world>
</sdf>
'''

REAL_NOTE = '''<!--
  REAL-MODEL VARIANT (industrial_zone_real): identical layout to
  industrial_zone.sdf, but buildings, cooling towers, the power plant,
  water tower, tower crane, transformers, vehicles, forklifts, lamp posts,
  utility poles, trees, dumpsters and barriers are real 3D models included
  directly from Gazebo Fuel (https://fuel.gazebosim.org).  Gazebo downloads
  them on first load and caches them in ~/.gz/fuel.  Pre-fetch with:
      gz fuel download -u "<uri>"   (for each <include><uri> below)
  Structures with no suitable Fuel model (oil tanks, silos, distillation
  columns, pipe racks, shipping containers, rails, fence, roads) remain
  procedural.
  Generated by spawn_drone/tools/gen_industrial_zone.py (real mode)
-->
'''

if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    path = args[0] if args else f'{WORLD_NAME}.sdf'
    header = HEADER.replace('<world name="industrial_zone">', f'<world name="{WORLD_NAME}">')
    if REAL:
        header = header.replace('<sdf version="1.9">', REAL_NOTE + '<sdf version="1.9">')
    with open(path, 'w') as fh:
        fh.write(header + '\n'.join(out) + '\n' + FOOTER)
    nv = sum(s.count('<visual') for s in out)
    nc = sum(s.count('<collision') for s in out)
    print(f'wrote {path}: {len(_names)} entities, {nv} procedural visuals, {nc} collisions')
    if fuel_count:
        print(f'Fuel includes: {sum(fuel_count.values())} ->',
              ', '.join(f'{k} x{v}' for k, v in sorted(fuel_count.items())))
