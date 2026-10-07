"""Autonomous rough-terrain walking controller for the 8-leg spider.

Pipeline (runs at 500 Hz, LiDAR at 10 Hz):

  LiDAR scan -> elevation map -> foothold choice + swing apex
  gait clock (8-leg wave gait, duty 0.75: 6 feet always down)
  desired body pose (level, height over the stance feet, heading to goal)
  world-fixed stance footholds + swing trajectories -> body frame -> leg IK
  joint PD + gravity feed-forward -> belt-coupled motor torques -> limits

State estimation uses the simulator's base pose (a stand-in for the
LiDAR-inertial odometry the Jetson would run); everything else - contact,
terrain, joint state - comes from the robot's own simulated sensors.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import mujoco
import numpy as np

from sim.perception import HeightMap, Lidar
from spider import config as C
from spider.kinematics import leg_fk, leg_ik, leg_jacobian

LEGS = C.LEG_NAMES
FOOT_R = C.FOOT_R_M
HIP_PLANE_Z = 0.0                  # hip plane in base frame


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def smooth(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


@dataclass
class GaitParams:
    period: float = 1.5            # s
    duty: float = 0.75             # stance fraction: 6 of 8 feet loaded at all times
    speed: float = 0.16            # m/s forward
    body_height: float = (C.STAND_HEIGHT - C.COXA_Z) / 1000   # hip plane above foot centres (m)
    stand_radius: float = C.STAND_RADIUS / 1000
    clearance: float = 0.075       # swing apex above the highest terrain on the path
    touch_threshold: float = 25.0  # N
    # back-to-front waves, sides half a cycle apart: L4+R2, L3+R1, L2+R4, L1+R3
    offsets: dict = field(default_factory=lambda: {
        "L4": 0.0, "L3": 0.25, "L2": 0.5, "L1": 0.75,
        "R2": 0.0, "R1": 0.25, "R4": 0.5, "R3": 0.75})


@dataclass
class LegState:
    mode: str = "stance"           # stance | swing | search
    p_world: np.ndarray = None     # stance foothold (foot-sphere centre)
    p0: np.ndarray = None          # swing lift-off point
    pT: np.ndarray = None          # swing target
    apex: float = 0.0
    replanned: bool = False
    stumbled: bool = False
    search_z: float = 0.0
    last_phase: float = 0.0


class SpiderController:
    def __init__(self, m: mujoco.MjModel, gait: GaitParams = GaitParams(), goal_y=0.0, seed=0,
                 use_perception=True):
        self.m, self.g = m, gait
        self.goal_y = goal_y
        self.use_perception = use_perception
        self.qadr = np.array([[m.joint(f"{l}_{j}_joint").qposadr[0] for j in ("coxa", "femur", "knee")] for l in LEGS])
        self.dadr = np.array([[m.joint(f"{l}_{j}_joint").dofadr[0] for j in ("coxa", "femur", "knee")] for l in LEGS])
        self.act = np.array([[m.actuator(f"{l}_{j}_motor").id for j in ("coxa", "femur", "knee")] for l in LEGS])
        self.touch = np.array([m.sensor(f"{l}_foot_touch").adr[0] for l in LEGS])
        self.gimbal_q = [m.joint(n).qposadr[0] for n in ("sensor_pan_joint", "sensor_tilt_joint")]
        self.gimbal_d = [m.joint(n).dofadr[0] for n in ("sensor_pan_joint", "sensor_tilt_joint")]
        self.gimbal_a = [m.actuator(n).id for n in ("sensor_pan_motor", "sensor_tilt_motor")]
        hp = [C.hip_pose(l) for l in LEGS]
        self.hip_xy = np.array([[x / 1000, y / 1000] for x, y, _ in hp])
        self.hip_yaw = np.radians([a for _, _, a in hp])
        self.hip_R = np.array([rotz(a) for a in self.hip_yaw])
        self.mass = float(m.body_subtreemass[m.body("base_link").id])
        self.kp = np.array([320.0, 380.0, 380.0])
        self.kd = np.array([6.0, 8.0, 8.0])
        self.legs = [LegState() for _ in LEGS]
        self.lidar = Lidar(m, seed=seed)
        self.map = HeightMap()
        self.t_scan = -1.0
        self.started = False
        self.walking = False
        self.t_walk0 = 0.0
        self.log = dict(max_tau=0.0, touchdowns=0, early_td=0, searches=0, stumbles=0, energy=0.0)

    # ------------------------------------------------------------------ utils
    def base_pose(self, d):
        R = d.xmat[self.m.body("base_link").id].reshape(3, 3).copy()
        p = d.xpos[self.m.body("base_link").id].copy()
        return p, R

    def foot_world(self, d, p, R):
        q = d.qpos[self.qadr]
        f = leg_fk(q)                                    # hip frame
        fb = np.einsum("lij,lj->li", self.hip_R, f)
        fb[:, :2] += self.hip_xy
        return p + fb @ R.T

    def nominal_body(self):
        r = self.g.stand_radius
        n = np.zeros((8, 3))
        n[:, 0] = self.hip_xy[:, 0] + r * np.cos(self.hip_yaw)
        n[:, 1] = self.hip_xy[:, 1] + r * np.sin(self.hip_yaw)
        n[:, 2] = HIP_PLANE_Z - self.g.body_height
        return n

    def ground_ref(self):
        z = [s.p_world[2] for s in self.legs if s.mode == "stance" and s.p_world is not None]
        return (np.mean(z) - FOOT_R) if z else 0.0

    # --------------------------------------------------------------- planning
    def plan_swing(self, i, d, t_left):
        g = self.g
        ground = self.ground_ref()
        v = np.array([g.speed if self.walking else 0.0, 0.0, 0.0])
        yaw_p = self.yaw_d + self.omega * t_left
        p_pred = self.pd + rotz(self.yaw_d) @ v * t_left
        t_stance = g.period * g.duty
        nb = self.nominal_body()[i] + np.array([0.5 * v[0] * t_stance, 0, 0])
        xy = (p_pred + rotz(yaw_p) @ nb)[:2]
        if self.use_perception:
            xy, z_surf, info = self.map.best_foothold(xy, ground)
            if info.get("known", 0) < 0.3:
                z_surf = max(z_surf, ground)
            path_top = self.map.max_along(self.legs[i].p0[:2], xy, fallback=ground)
        else:
            z_surf, path_top = ground, ground
        s = self.legs[i]
        s.pT = np.array([xy[0], xy[1], z_surf + FOOT_R])
        top = max(path_top, z_surf, s.p0[2] - FOOT_R)
        s.apex = max(s.apex if s.stumbled else 0.0, top + FOOT_R + g.clearance)

    def swing_target(self, s: LegState, u):
        a = smooth((u - 0.12) / 0.7)
        xy = s.p0[:2] + (s.pT[:2] - s.p0[:2]) * a
        if u < 0.35:
            z = s.p0[2] + (s.apex - s.p0[2]) * smooth(u / 0.35)
        elif u < 0.62:
            z = s.apex
        else:
            z = s.apex + (s.pT[2] - 0.015 - s.apex) * smooth((u - 0.62) / 0.3)
        return np.array([xy[0], xy[1], z])

    # ------------------------------------------------------------------- step
    def __call__(self, d, t):
        m, g = self.m, self.g
        dt = m.opt.timestep
        p, R = self.base_pose(d)
        yaw = math.atan2(R[1, 0], R[0, 0])
        feet = self.foot_world(d, p, R)
        touch = d.sensordata[self.touch]

        if not self.started:
            self.started = True
            self.pd, self.yaw_d, self.omega = p.copy(), yaw, 0.0
            for i, s in enumerate(self.legs):
                s.p_world = feet[i].copy()

        # perception
        if self.use_perception and t - self.t_scan >= 0.1:
            self.t_scan = t
            self.map.update(self.lidar.scan(m, d))

        # desired body pose
        v = g.speed if self.walking else 0.0
        head_des = math.atan2(-(p[1] - self.goal_y) * 0.8, 1.0) if self.walking else self.yaw_d
        err = (head_des - self.yaw_d + math.pi) % (2 * math.pi) - math.pi
        self.omega = float(np.clip(1.2 * err, -0.25, 0.25))
        self.yaw_d += self.omega * dt
        self.yaw_d = yaw + float(np.clip((self.yaw_d - yaw + math.pi) % (2 * math.pi) - math.pi, -0.12, 0.12))
        self.pd[:2] += (rotz(self.yaw_d) @ np.array([v, 0, 0]))[:2] * dt
        self.pd[:2] = p[:2] + np.clip(self.pd[:2] - p[:2], -0.06, 0.06)
        z_target = self.ground_ref() + FOOT_R + g.body_height
        self.pd[2] += (z_target - self.pd[2]) * min(1.0, dt / 0.35)
        Rd = rotz(self.yaw_d)

        # gait clock
        phase = ((t - self.t_walk0) / g.period) % 1.0 if self.walking else None
        sw_frac = 1.0 - g.duty
        t_sw = sw_frac * g.period
        targets = np.zeros((8, 3))
        n_stance = 0
        for i, (leg, s) in enumerate(zip(LEGS, self.legs)):
            if phase is not None:
                ph = (phase + g.offsets[leg]) % 1.0
                if ph < sw_frac and s.last_phase >= sw_frac:      # lift-off
                    s.mode, s.p0, s.stumbled, s.replanned = "swing", feet[i].copy(), False, False
                    s.apex = 0.0
                    self.plan_swing(i, d, t_sw)
                s.last_phase = ph
            if s.mode == "swing":
                u = s.last_phase / sw_frac if s.last_phase < sw_frac else 1.0
                if not s.replanned and u > 0.45:
                    s.replanned = True
                    self.plan_swing(i, d, (1 - u) * t_sw)
                if touch[i] > g.touch_threshold and u > 0.55:
                    s.mode, s.p_world = "stance", feet[i].copy()
                    self.log["touchdowns"] += 1
                    self.log["early_td"] += int(u < 0.98)
                elif touch[i] > g.touch_threshold and 0.2 < u < 0.5 and not s.stumbled:
                    s.stumbled = True                             # hit a rock face: lift higher
                    s.apex += 0.06
                    self.log["stumbles"] += 1
                    targets[i] = self.swing_target(s, u)
                elif u >= 1.0:
                    s.mode, s.search_z = "search", s.pT[2] - 0.015
                    self.log["searches"] += 1
                    targets[i] = self.swing_target(s, 1.0)
                else:
                    targets[i] = self.swing_target(s, u)
            if s.mode == "search":
                s.search_z -= 0.35 * dt
                if touch[i] > g.touch_threshold or s.search_z < s.pT[2] - 0.14:
                    s.mode, s.p_world = "stance", feet[i].copy()
                    self.log["touchdowns"] += 1
                else:
                    targets[i] = np.array([s.pT[0], s.pT[1], s.search_z])
            if s.mode == "stance":
                targets[i] = s.p_world
                n_stance += 1

        # world -> desired body -> hip frames -> IK
        body = (targets - self.pd) @ Rd
        hip = body.copy()
        hip[:, :2] -= self.hip_xy
        hip = np.einsum("lji,lj->li", self.hip_R, hip)
        q_des = leg_ik(hip)
        q_des[:, 0] = np.clip(q_des[:, 0], *C.COXA_LIMIT)
        q_des[:, 1] = np.clip(q_des[:, 1], *C.FEMUR_LIMIT)

        q, qd = d.qpos[self.qadr], d.qvel[self.dadr]
        tau = self.kp * (q_des - q) - self.kd * qd
        # gravity feed-forward on stance legs: share body weight equally
        if n_stance:
            Fw = np.array([0, 0, self.mass * 9.81 / n_stance])
            J = leg_jacobian(q)
            for i, s in enumerate(self.legs):
                if s.mode == "stance" and touch[i] > 1.0:
                    F_hip = self.hip_R[i].T @ (R.T @ Fw)
                    tau[i] += -J[i].T @ F_hip

        # belt coupling: knee motor acts on (femur + knee); femur motor = rest
        motor = np.stack([tau[:, 0], tau[:, 1] - tau[:, 2], tau[:, 2]], axis=1)
        w_motor = np.stack([qd[:, 0], qd[:, 1], qd[:, 1] + qd[:, 2]], axis=1)
        # QDD-96 torque-speed envelope (linear above 40 % of no-load speed)
        lim = C.QDD_TAU_PEAK * np.clip((1 - np.abs(w_motor) / C.QDD_SPEED) / 0.6, 0.0, 1.0)
        motor = np.clip(motor, -lim, lim)
        d.ctrl[self.act] = motor
        self.log["max_tau"] = max(self.log["max_tau"], float(np.abs(motor).max()))
        self.log["energy"] += float(np.clip(motor * w_motor, 0, None).sum()) * dt

        # sensor head: hold pan 0, tilt the LiDAR 22 deg down
        gq, gd = d.qpos[self.gimbal_q], d.qvel[self.gimbal_d]
        d.ctrl[self.gimbal_a] = np.clip(8.0 * (np.array([0.0, math.radians(22)]) - gq) - 0.3 * gd, -2.5, 2.5)
        return q_des
