"""The robot: every part instance, on its link, plus the 30 joints.

This is the one description that the STEP, URDF and MJCF exporters read, so
the CAD, the kinematic model and the physics model cannot drift apart.

    links   base_link, 8 x (coxa, femur, tibia), sensor_pan, sensor_tilt,
            lidar_link, imu_link, jetson_link, front_camera_link
    joints  24 leg revolutes, 2 gimbal revolutes, 4 fixed sensor mounts
"""
from __future__ import annotations

import functools
import math
from dataclasses import dataclass, field

import numpy as np

from spider import config as C
from spider import layout as LY
from spider.parts import (antenna, ball_bearing, battery_clamp, battery_pack, bhcs,
                          bulkhead, bumper, cable_assembly, cable_tie, camera_bracket,
                          can_interface, carry_handle, chassis_plate, circlip, coxa_yoke,
                          dcdc_converter, dowel_pin, driver_cage, enclosure_cover,
                          estop_button, ethernet_switch, femur_side_plate, foot_core,
                          foot_load_cell, foot_pad, gimbal_motor, hex_nut, htd_pulley,
                          idler_roller, imu, jetson_orin, knee_axle, knee_drive_shaft,
                          lidar_3d, lidar_mast, motor_driver, pan_bracket, power_board,
                          qdd_actuator, sensor_cable, shcs, standoff, status_light,
                          stereo_camera, tibia_ferrule, tibia_knuckle, tibia_tube,
                          tilt_cradle, timing_belt)
from spider.parts._base import Variant
from spider.tf import chain, rotx, roty, rotz, trans

# Screw/nut frames: z axis points out of the surface, away from the material.
UP, DOWN = np.eye(4), rotx(180)
PY, NY = rotx(-90), rotx(90)          # z -> +y, z -> -y
PX, NX = roty(90), roty(-90)          # z -> +x, z -> -x


@dataclass
class Instance:
    variant: Variant
    link: str
    T: np.ndarray          # pose in link frame, mm
    name: str


@dataclass
class Joint:
    name: str
    type: str              # "revolute" | "fixed"
    parent: str
    child: str
    origin: np.ndarray     # child frame in parent frame, mm
    axis: tuple = (0.0, 0.0, 1.0)
    lower: float = 0.0
    upper: float = 0.0
    effort: float = 0.0
    velocity: float = 0.0
    driver: str | None = None   # motor-driver instance that powers it


@dataclass
class Robot:
    links: list[str] = field(default_factory=list)
    joints: list[Joint] = field(default_factory=list)
    instances: list[Instance] = field(default_factory=list)

    # -- building helpers -------------------------------------------------
    def add(self, variant: Variant, link: str, T, name: str):
        self.instances.append(Instance(variant, link, np.asarray(T, float), name))

    def link(self, name: str):
        self.links.append(name)
        return name

    def screw(self, v, link, xyz, frame, name, nut=None, nut_depth=0.0):
        """A screw whose head sits at xyz with z of ``frame`` pointing away from
        the part.  Optional lock nut on the far side, nut_depth below the head."""
        T = chain(trans(*xyz), frame)
        self.add(v, link, T, name)
        if nut is not None:
            self.add(nut, link, chain(T, trans(0, 0, -nut_depth), rotx(180)), name + "/nut")

    def cable(self, link, p0, p1, name):
        d = np.subtract(p1, p0, dtype=float)
        L = int(round(np.linalg.norm(d) / 10.0) * 10)
        x = d / np.linalg.norm(d)
        z = np.cross(x, [0, 0, 1.0]) if abs(x[2]) < 0.9 else np.cross(x, [1.0, 0, 0])
        z /= np.linalg.norm(z)
        y = np.cross(z, x)
        T = np.eye(4)
        T[:3, :3] = np.column_stack([x, y, z])
        T[:3, 3] = p0
        self.add(cable_assembly.make(L=L), link, T, name)

    # -- queries ------------------------------------------------------------
    def joint_by_child(self) -> dict:
        return {j.child: j for j in self.joints}

    def link_poses(self, q: dict | None = None) -> dict:
        """World pose (mm) of every link for joint angles q (rad)."""
        q = q or {}
        by_child = self.joint_by_child()
        poses = {"base_link": np.eye(4)}

        def pose(link):
            if link in poses:
                return poses[link]
            j = by_child[link]
            T = pose(j.parent) @ j.origin
            if j.type == "revolute":
                from spider.tf import axis_angle
                T = T @ axis_angle(j.axis, q.get(j.name, 0.0))
            poses[link] = T
            return T

        for ln in self.links:
            pose(ln)
        return poses

    def instances_on(self, link):
        return [i for i in self.instances if i.link == link]


# ------------------------------------------------------------------------
# Standard hardware shorthands
# ------------------------------------------------------------------------
# Cable-tie stations along the leg (sensor cable dressing).
LEG_TIES_FEMUR = (100.0, 220.0)
LEG_TIES_TIBIA = (150.0, 260.0)


def S(d, L):
    return shcs.make(d=d, L=L)


def B(d, L):
    return bhcs.make(d=d, L=L)


def N(d):
    return hex_nut.make(d=d)


# ------------------------------------------------------------------------
def _chassis(r: Robot):
    b = "base_link"
    t, gap = C.PLATE_T, C.PLATE_GAP
    top = LY.TOP_Z
    r.add(chassis_plate.make(which="top"), b, trans(0, 0, gap), "chassis/top_plate")
    r.add(chassis_plate.make(which="bottom"), b, trans(0, 0, -t), "chassis/bottom_plate")
    for i, (x, y) in enumerate(LY.STANDOFFS):
        r.add(standoff.make(d=5, L=gap), b, trans(x, y, 0), f"chassis/standoff{i}")
        r.screw(S(5, 10), b, (x, y, top), UP, f"chassis/standoff{i}/top_screw")
        r.screw(S(5, 10), b, (x, y, -t), DOWN, f"chassis/standoff{i}/bottom_screw")
    for k, sx in enumerate((1, -1)):
        nm = "front" if sx > 0 else "rear"
        R = rotz(0 if sx > 0 else 180)
        r.add(bulkhead.make(), b, chain(trans(sx * LY.BULKHEAD_X, 0, 0), R), f"chassis/bulkhead_{nm}")
        for y in LY.BULKHEAD_SCREW_Y:
            r.screw(B(4, 8), b, (sx * LY.BULKHEAD_X, y, top), UP, f"chassis/bulkhead_{nm}/top{y:+.0f}")
            r.screw(B(4, 8), b, (sx * LY.BULKHEAD_X, y, -t), DOWN, f"chassis/bulkhead_{nm}/bot{y:+.0f}")
        bx = sx * (C.CHASSIS_LEN / 2 + 1)
        r.add(bumper.make(), b, chain(trans(bx, 0, 0), R), f"chassis/bumper_{nm}")
        for y in (-80.0, 0.0, 80.0):
            r.screw(B(4, 30) if y else B(4, 35), b, (bx + sx * (16 if y else 30), y, gap / 2 + 5),
                    PX if sx > 0 else NX, f"chassis/bumper_{nm}/screw{y:+.0f}")
    # battery
    r.add(battery_pack.make(P=4), b, trans(*LY.BATTERY_XY, 0), "power/battery")
    for k, x in enumerate(LY.BATTERY_CLAMP_X):
        R = rotz(0 if k == 0 else 180)
        r.add(battery_clamp.make(), b, chain(trans(x, 0, 0), R), f"power/battery_clamp{k}")
        for y in (-25.0, 25.0):
            r.screw(B(4, 10), b, (x, y, -t), DOWN, f"power/battery_clamp{k}/screw{y:+.0f}")


def _top_deck(r: Robot):
    b = "base_link"
    top, gap = LY.TOP_Z, C.PLATE_GAP
    boards = [("jetson", None, 0), ("pdb", power_board.make(), 0), ("can", can_interface.make(), 90),
              ("eth", ethernet_switch.make(), 0)]
    xy = {"jetson": LY.JETSON_XY, "pdb": LY.PDB_XY, "can": LY.CAN_XY, "eth": LY.ETH_XY}
    for key, v, yaw in boards:
        bx, by = xy[key]
        _, _, hx, hy = LY.BOARDS[key]
        if v is not None:
            r.add(v, b, chain(trans(bx, by, top + 10), rotz(yaw)), f"electronics/{key}")
        for px, py in LY.rect_pattern(bx, by, hx, hy):
            # male-female standoffs thread into the tapped top plate
            r.add(standoff.make(d=3, L=10.0, af=5.5), b, trans(px, py, top), f"electronics/{key}/so{px:.0f}{py:.0f}")
            r.screw(S(3, 6), b, (px, py, top + 10 + 1.6), UP, f"electronics/{key}/screw{px:.0f}{py:.0f}")
    gx, gy = LY.GIMBAL_CAGE_XY
    r.add(driver_cage.make(slots=2), b, trans(gx, gy, top), "electronics/gimbal_driver_cage")
    for k in range(2):
        r.add(motor_driver.make(), b, trans(gx, gy, top + driver_cage.BASE + k * driver_cage.PITCH),
              f"electronics/gdrv{k}")
    for x, y in LY.cage_holes((gx, gy)):
        r.screw(S(3, 8), b, (x, y, top + driver_cage.BASE), UP, f"electronics/gimbal_driver_cage/screw{x:.0f}")
    for k, (x, y) in enumerate(LY.DCDC_XY):
        r.add(dcdc_converter.make(Vout=(12, 5)[k]), b, trans(x, y, top), f"electronics/dcdc{k}")
        for px, py in LY.rect_pattern(x, y, 50, 48):
            r.screw(S(3, 8), b, (px, py, gap), DOWN, f"electronics/dcdc{k}/screw{px:.0f}{py:.0f}")
    for x, y in LY.imu_holes():
        r.screw(S(3, 8), b, (x, y, top + 3), UP, f"sensors/imu/screw{x:+.0f}")
    cx, cy = LY.ENCLOSURE_XY
    r.add(enclosure_cover.make(), b, trans(cx, cy, top), "electronics/enclosure_cover")
    for x, y in LY.enclosure_flange_holes():
        r.screw(B(4, 8), b, (x, y, top + 1), UP, f"electronics/enclosure/screw{x:.0f}{y:.0f}",
                nut=N(4), nut_depth=C.PLATE_T + 1)
    for k, (x, y) in enumerate(LY.HANDLE_XY):
        r.add(carry_handle.make(), b, trans(x, y, top), f"chassis/handle{k}")
    for x, y in LY.handle_holes():
        # into PEM CLS-M6 clinch nuts in the top plate (lift load bears the nut flange)
        r.screw(S(6, 12), b, (x, y, top + 8), UP, f"chassis/handle/screw{x:.0f}{y:.0f}")
    r.add(estop_button.make(), b, trans(*LY.ESTOP_XY, top), "electronics/estop")
    r.add(status_light.make(), b, trans(*LY.BEACON_XY, top), "electronics/beacon")
    for k, (x, y) in enumerate(LY.ANTENNA_XY):
        r.add(antenna.make(), b, trans(x, y, top), f"electronics/antenna{k}")
    # sensor mast
    mx, my = LY.MAST_XY
    r.add(lidar_mast.make(), b, trans(mx, my, top), "sensor_head/mast")
    for x, y in LY.mast_holes():
        # PEM CLS-M5 clinch nuts are pressed into the top plate (part of SP-1001-top)
        r.screw(S(5, 12), b, (x, y, top + 6), UP, f"sensor_head/mast/screw{x:.0f}{y:.0f}")
    seat = top + C.MAST_H - C.GIMBAL_T
    r.add(gimbal_motor.make(), b, chain(trans(mx, my, C.PAN_Z), rotx(180)), "sensor_head/pan_motor")
    for x, y in LY.bolt_circle(C.GIMBAL_D / 2 - 7, 4, 45):
        r.screw(S(3, 8), b, (mx + x, my + y, seat - 6), DOWN, f"sensor_head/pan_motor/screw{x:+.0f}{y:+.0f}")
    # camera
    r.add(camera_bracket.make(), b, trans(LY.CAMERA_X, 0, top), "sensors/camera_bracket")
    for x, y in LY.camera_holes():
        r.screw(S(4, 10), b, (x, y, top + 4), UP, f"sensors/camera_bracket/screw{y:+.0f}",
                nut=N(4), nut_depth=4 + C.PLATE_T)
    # main power harness: battery -> PDB, PDB -> 4 leg-pair buses
    r.cable(b, (140, 60, 20), (95, 55, top + 12), "power/main_harness")
    for k, (x, y) in enumerate([(250, 150), (-250, 150), (250, -150), (-250, -150)]):
        r.cable(b, (95, 55, 30), (x, y, 30), f"power/bus{k}")


def _leg(r: Robot, leg: str):
    hx, hy, yaw = C.hip_pose(leg)
    b = "base_link"
    coxa, femur, tibia = (r.link(f"{leg}_{s}") for s in ("coxa", "femur", "tibia"))
    H = chain(trans(hx, hy, 0), rotz(yaw))    # hip frame in base_link
    top = LY.TOP_Z
    fx, fz = C.COXA_X, C.COXA_Z
    QB = C.QDD_BOLT_CIRCLE / 2
    QO = C.QDD_OUT_BOLT_CIRCLE / 2
    py, pt = femur_side_plate.PLATE_Y, C.FEMUR_PLATE_T

    # ---- base_link side of the hip ------------------------------------
    r.add(qdd_actuator.make(), b, chain(H, trans(0, 0, 2)), f"{leg}/coxa_actuator")
    for k, (x, y) in enumerate(LY.bolt_circle(QB, 6, 30)):
        p = H @ np.array([x, y, top, 1.0])
        r.screw(S(4, 10), b, p[:3], UP, f"{leg}/coxa_actuator/screw{k}")
    cx, cy = LY.cage_positions()[leg]
    r.add(driver_cage.make(slots=3), b, trans(cx, cy, 0), f"{leg}/driver_cage")
    for x, y in LY.cage_holes((cx, cy)):
        r.screw(S(3, 8), b, (x, y, -C.PLATE_T), DOWN, f"{leg}/driver_cage/screw{x:.0f}")
    for k, joint in enumerate(("coxa", "femur", "knee")):
        z = driver_cage.BASE + k * driver_cage.PITCH
        r.add(motor_driver.make(), b, trans(cx, cy, z), f"{leg}/{joint}_driver")
        conn = H @ np.array([-C.QDD_D / 2 - 4, 0, 30, 1.0]) if joint == "coxa" else H @ np.array([0, 0, 20, 1.0])
        r.cable(b, (cx, cy - np.sign(cy) * 42, z + 6), conn[:3], f"{leg}/{joint}_harness_body")

    # ---- coxa link (hip frame) ------------------------------------------
    r.add(coxa_yoke.make(), coxa, np.eye(4), f"{leg}/coxa_yoke")
    for k, (x, y) in enumerate(LY.bolt_circle(QO, 6, 0)):
        r.screw(S(4, 20), coxa, (x, y, -14), DOWN, f"{leg}/coxa_yoke/rotor_screw{k}")
    # hip actuators by their back faces: femur on -y (rotor flush with the
    # femur plate), knee on +y (rotor drives the belt through the drive shaft)
    for joint, ys, cheek, R, Fs in (("femur", -py - pt / 2, coxa_yoke.CHEEK_N[0], rotx(90), NY),
                                    ("knee", 32.0, coxa_yoke.CHEEK_P[1], rotx(-90), PY)):
        r.add(qdd_actuator.make(), coxa, chain(trans(fx, ys, fz), R), f"{leg}/{joint}_actuator")
        for k, (x, z) in enumerate(LY.bolt_circle(QB, 6, 30)):
            r.screw(S(4, 12), coxa, (fx + x, cheek, fz + z), Fs, f"{leg}/{joint}_actuator/screw{k}")
        conn_y = ys + (-1 if joint == "femur" else 1) * (C.QDD_T - 12)
        r.cable(coxa, (0, np.sign(conn_y) * 6, -14), (fx - C.QDD_D / 2 - 4, conn_y, fz), f"{leg}/{joint}_harness_leg")
    F = trans(fx, 0, fz)   # femur-axis frame in coxa
    r.add(knee_drive_shaft.make(), coxa, F, f"{leg}/knee_drive_shaft")
    for k, (x, z) in enumerate(LY.bolt_circle(QO, 6, 0)):
        r.screw(S(4, 10), coxa, (fx + x, 24, fz + z), NY, f"{leg}/knee_drive_shaft/screw{k}")
    r.add(htd_pulley.make(bore=25), coxa, F, f"{leg}/hip_pulley")

    # ---- femur link (femur frame) ---------------------------------------
    r.add(femur_side_plate.make(side="hub"), femur, trans(0, -py, 0), f"{leg}/femur_plate_hub")
    r.add(femur_side_plate.make(side="bearing"), femur, trans(0, py, 0), f"{leg}/femur_plate_bearing")
    for k, (x, z) in enumerate(LY.bolt_circle(QO, 6, 0)):
        r.screw(S(4, 10), femur, (x, -py + pt / 2, z), PY, f"{leg}/femur_plate_hub/rotor_screw{k}")
    brg = ball_bearing.make(series="6805")
    r.add(brg, femur, chain(trans(0, 16.5, 0), rotx(90)), f"{leg}/hip_bearing")
    for sy in (1, -1):
        r.add(brg, femur, chain(trans(C.FEMUR_L, sy * 16.5, 0), rotx(90)), f"{leg}/knee_bearing{sy:+d}")
    for k, (x, z) in enumerate(femur_side_plate.SPACERS_XZ):
        r.add(standoff.make(d=5, L=C.FEMUR_INNER_W, af=10.0), femur,
              chain(trans(x, -C.FEMUR_INNER_W / 2, z), rotx(-90)), f"{leg}/femur_spacer{k}")
        for sy in (1, -1):
            r.screw(B(5, 10), femur, (x, sy * (py + pt / 2), z), PY if sy > 0 else NY,
                    f"{leg}/femur_spacer{k}/screw{sy:+d}")
    ix, iz = femur_side_plate.IDLER_XZ
    r.add(idler_roller.make(), femur, trans(ix, 0, iz), f"{leg}/idler_roller")
    r.add(ball_bearing.make(series="625"), femur, chain(trans(ix, 0, iz), rotx(90)), f"{leg}/idler_bearing")
    r.screw(S(5, 45), femur, (ix, py + pt / 2, iz), PY, f"{leg}/idler_bolt",
            nut=N(5), nut_depth=2 * py + pt)
    r.add(timing_belt.make(), femur, np.eye(4), f"{leg}/belt")
    r.add(sensor_cable.make(L=260), femur, trans(30, 0, 0), f"{leg}/foot_sensor_cable")
    for k, x in enumerate(LEG_TIES_FEMUR):
        r.add(cable_tie.make(D=12.0), femur, chain(trans(x, 0, 0), roty(90)), f"{leg}/femur_tie{k}")

    # ---- tibia link (tibia frame) ---------------------------------------
    r.add(tibia_knuckle.make(), tibia, np.eye(4), f"{leg}/tibia_knuckle")
    r.add(htd_pulley.make(bore=25), tibia, np.eye(4), f"{leg}/knee_pulley")
    for k, (x, z) in enumerate(LY.bolt_circle(17, 4, 45)):
        r.screw(S(3, 25), tibia, (x, 13, z), PY, f"{leg}/knee_pulley/screw{k}")
    r.add(knee_axle.make(), tibia, np.eye(4), f"{leg}/knee_axle")
    r.add(dowel_pin.make(d=4, L=12), tibia, chain(trans(0, 11.5, 0), rotx(90)), f"{leg}/knee_axle/pin")
    r.add(circlip.make(d=25), tibia, chain(trans(0, 20, 0), rotx(-90)), f"{leg}/knee_axle/ring+")
    r.add(circlip.make(d=25), tibia, chain(trans(0, -20, 0), rotx(90)), f"{leg}/knee_axle/ring-")
    r.add(tibia_tube.make(), tibia, np.eye(4), f"{leg}/tibia_tube")
    for x in tibia_knuckle.CLAMP_BOLTS_X:
        r.screw(S(5, 25), tibia, (x, 9, 23), PY, f"{leg}/tibia_clamp{x:.0f}", nut=N(5), nut_depth=18)
    r.add(tibia_ferrule.make(), tibia, np.eye(4), f"{leg}/foot_ferrule")
    r.add(foot_load_cell.make(), tibia, chain(trans(368, 0, 0), roty(90)), f"{leg}/foot_load_cell")
    for z in (-9.0, 9.0):
        r.screw(S(3, 8), tibia, (360, 0, z), NX, f"{leg}/foot_load_cell/screw{z:+.0f}")
    r.add(foot_core.make(), tibia, np.eye(4), f"{leg}/foot_core")
    r.add(foot_pad.make(), tibia, np.eye(4), f"{leg}/foot_pad")
    for k, x in enumerate(LEG_TIES_TIBIA):
        r.add(cable_tie.make(D=36.0), tibia, chain(trans(x, 0, 0), roty(90)), f"{leg}/tibia_tie{k}")

    # ---- joints ---------------------------------------------------------
    eff, vel = C.QDD_TAU_PEAK, C.QDD_SPEED
    r.joints += [
        Joint(f"{leg}_coxa_joint", "revolute", b, coxa, H, (0, 0, 1), *C.COXA_LIMIT, eff, vel,
              driver=f"{leg}/coxa_driver"),
        Joint(f"{leg}_femur_joint", "revolute", coxa, femur, F, (0, 1, 0), *C.FEMUR_LIMIT, eff, vel,
              driver=f"{leg}/femur_driver"),
        Joint(f"{leg}_knee_joint", "revolute", femur, tibia, trans(C.FEMUR_L, 0, 0), (0, 1, 0),
              *C.KNEE_LIMIT, eff, vel, driver=f"{leg}/knee_driver"),
    ]


def _sensor_head(r: Robot):
    pan, tilt = r.link("sensor_pan"), r.link("sensor_tilt")
    lidar, imu_l, jet, cam = (r.link(n) for n in ("lidar_link", "imu_link", "jetson_link", "front_camera_link"))
    top = LY.TOP_Z
    mx, my = LY.MAST_XY
    r.add(pan_bracket.make(), pan, np.eye(4), "sensor_head/pan_bracket")
    for x, y in LY.bolt_circle(C.GIMBAL_D / 2 - 11, 4, 0):
        r.screw(S(3, 8), pan, (x, y, 5), UP, f"sensor_head/pan_bracket/screw{x:+.0f}{y:+.0f}")
    r.add(gimbal_motor.make(), pan, chain(trans(0, -27, C.TILT_DZ), rotx(90)), "sensor_head/tilt_motor")
    for x, z in [(-21, 0), (21, 0), (0, 21), (0, -21)]:
        r.screw(S(3, 8), pan, (x, -58, C.TILT_DZ + z), NY, f"sensor_head/tilt_motor/screw{x:+d}{z:+d}")
    r.add(tilt_cradle.make(), tilt, np.eye(4), "sensor_head/tilt_cradle")
    for x, z in LY.bolt_circle(20, 4, 45):
        r.screw(S(3, 8), tilt, (x, -22, z), PY, f"sensor_head/tilt_cradle/screw{x:+.0f}{z:+.0f}")
    for x, y in LY.rect_pattern(0, 0, 50, 50):
        r.screw(S(4, 10), tilt, (x, y, 31), DOWN, f"sensor_head/lidar/screw{x:+.0f}{y:+.0f}")
    r.add(lidar_3d.make(), lidar, np.eye(4), "sensors/lidar")
    r.add(imu.make(), imu_l, np.eye(4), "sensors/imu")
    r.add(jetson_orin.make(), jet, np.eye(4), "electronics/jetson")
    r.add(stereo_camera.make(), cam, np.eye(4), "sensors/front_camera")
    r.cable(pan, (0, 0, 8), (0, -40, C.TILT_DZ - 30), "sensor_head/tilt_harness")
    r.cable("base_link", (LY.GIMBAL_CAGE_XY[0] + 38, LY.GIMBAL_CAGE_XY[1], top + 20), (mx, my, top + 40),
            "sensor_head/pan_harness")
    r.cable("base_link", (LY.ETH_XY[0], LY.ETH_XY[1], top + 20), (mx, my, top + 60), "sensor_head/lidar_eth")
    r.joints += [
        Joint("sensor_pan_joint", "revolute", "base_link", pan, trans(mx, my, C.PAN_Z), (0, 0, 1),
              -math.radians(170), math.radians(170), C.GIMBAL_TAU_PEAK, 20.0, driver="electronics/gdrv0"),
        Joint("sensor_tilt_joint", "revolute", pan, tilt, trans(0, 0, C.TILT_DZ), (0, 1, 0),
              -math.radians(35), math.radians(60), C.GIMBAL_TAU_PEAK, 20.0, driver="electronics/gdrv1"),
        Joint("lidar_joint", "fixed", tilt, lidar, trans(0, 0, 77)),
        Joint("imu_joint", "fixed", "base_link", imu_l, trans(*LY.IMU_XY, top)),
        Joint("jetson_joint", "fixed", "base_link", jet, trans(*LY.JETSON_XY, top + 10)),
        Joint("front_camera_joint", "fixed", "base_link", cam,
              chain(trans(LY.CAMERA_X + 16, 0, top + 44), roty(15))),
    ]


@functools.lru_cache(maxsize=1)
def build() -> Robot:
    r = Robot()
    r.link("base_link")
    _chassis(r)
    _top_deck(r)
    for leg in C.LEG_NAMES:
        _leg(r, leg)
    _sensor_head(r)
    names = [i.name for i in r.instances]
    assert len(names) == len(set(names)), "duplicate instance names"
    return r


def standing_pose() -> dict:
    """Joint angles for the nominal stance (used for the STEP assembly)."""
    from spider.kinematics import leg_ik
    q = {}
    for leg in C.LEG_NAMES:
        a = leg_ik(np.array([C.STAND_RADIUS, 0.0, C.COXA_Z - C.STAND_HEIGHT]) / 1000.0)
        q[f"{leg}_coxa_joint"], q[f"{leg}_femur_joint"], q[f"{leg}_knee_joint"] = a
    q["sensor_tilt_joint"] = math.radians(20)
    return q
