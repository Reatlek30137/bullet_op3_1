#!/usr/bin/env python
import time
from threading import Thread
import math

from op3 import OP3


class WJFunc:
    """
    Walk Joint Function CPG style
    Provides parameterized sine wave functions as y=offset+scale*(in_offset+in_scale*x)
    """

    def __init__(self):
        """
        初始化 WJFunc 類別。
        - offset: 正弦波的偏移量。
        - scale: 正弦波的幅度。
        - in_offset: 正弦波的相位偏移。
        - in_scale: 正弦波的頻率縮放。
        """
        self.offset = 0
        self.scale = 1
        self.in_offset = 0
        self.in_scale = 1

    def get(self, x):
        """
        計算正弦波函數的值。
        :param x: 輸入值，範圍為 0 到 1。
        :return: 正弦波函數的輸出值。
        """
        f = math.sin(self.in_offset + self.in_scale * x)
        return self.offset + self.scale * f

    def clone(self):
        """
        複製當前的 WJFunc 實例。
        :return: 新的 WJFunc 實例，具有相同的參數。
        """
        z = WJFunc()
        z.offset = self.offset
        z.scale = self.scale
        z.in_offset = self.in_offset
        z.in_scale = self.in_scale
        return z

    def mirror(self):
        """
        創建一個鏡像版本的 WJFunc。
        :return: 鏡像的 WJFunc 實例。
        """
        z = self.clone()
        z.offset *= -1
        z.scale *= -1
        return z

    def __str__(self):
        """
        返回正弦波函數的字串表示。
        :return: 字串形式的函數表示。
        """
        return "y=%f+%f*sin(%f+%f*x)" % (self.offset, self.scale, self.in_offset, self.in_scale)


class WFunc:
    """
    Multi-joint walk function for Darwin
    提供多關節的步態函數，基於 CPG（中央模式生成器）。
    """

    def __init__(self, **kwargs):
        """
        初始化 WFunc 類別。
        :param kwargs: 可選參數，用於覆蓋默認的步態參數。
        - swing_scale: 擺動幅度的縮放因子。
        - step_scale: 步幅的縮放因子。
        - step_offset: 步態的偏移量。
        - ankle_offset: 腳踝的偏移量。
        - vx_scale: 前進速度的縮放因子。
        - vy_scale: 側向速度的縮放因子。
        - vt_scale: 旋轉速度的縮放因子。
        """
        self.parameters = {}

        self.parameters["swing_scale"] = 0.0
        self.parameters["step_scale"] = 0.1
        self.parameters["step_offset"] = 0.25
        self.parameters["ankle_offset"] = 0
        self.parameters["vx_scale"] = 0.2
        self.parameters["vy_scale"] = 0.2
        self.parameters["vt_scale"] = 0.1

        for k, v in kwargs.items():
            self.parameters[k] = v

        self.generate()

    def generate(self):
        """
        Build CPG functions for walk-on-spot (no translation or rotation, only legs up/down)
        構建步態函數，用於原地步態（無平移或旋轉，僅腿部上下移動）。
        """
        # f1=THIGH1=ANKLE1=L=R in phase
        self.pfn = {}  # phase joint functions
        self.afn = {}  # anti phase joint functions

        # ~ print f
        f1 = WJFunc()
        f1.in_scale = math.pi
        f1.scale = self.parameters["swing_scale"]
        self.pfn["l_ank_roll"] = f1
        self.pfn["l_hip_roll"] = f1

        # f2=mirror f1 in antiphase
        f2 = f1.mirror()
        # ~ f2=WJFunc()
        self.afn["l_ank_roll"] = f2
        self.afn["l_hip_roll"] = f2

        f3 = WJFunc()
        f3.in_scale = math.pi
        f3.scale = self.parameters["step_scale"]
        f3.offset = -self.parameters["step_offset"]
        self.pfn["l_hip_pitch"] = f3
        f33 = f3.mirror()
        f33.offset += self.parameters["ankle_offset"]
        self.pfn["l_ank_pitch"] = f33

        f4 = f3.mirror()
        f4.offset *= 2
        f4.scale *= 2
        self.pfn["l_knee"] = f4

        s2 = 0
        f5 = f3.clone()
        f5.in_scale *= 2
        f5.scale = s2
        self.afn["l_hip_pitch"] = f5

        f6 = f3.mirror()
        f6.in_scale *= 2
        f6.scale = f5.scale
        f6.offset += self.parameters["ankle_offset"]
        self.afn["l_ank_pitch"] = f6

        f7 = f4.clone()
        f7.scale = 0
        self.afn["l_knee"] = f7

        self.forward = [f5, f6]

        self.generate_right()
        self.joints = self.pfn.keys()

        self.show()

    def generate_right(self):
        """
        Mirror CPG functions from left to right and antiphase right
        從左側鏡像生成右側的步態函數。
        """
        l = [v[2:] for v in self.pfn.keys()]
        for j in l:
            self.pfn["r_" + j] = self.afn["l_" + j].mirror()
            self.afn["r_" + j] = self.pfn["l_" + j].mirror()

    def get(self, phase, x, velocity):
        """
        Obtain the joint angles for a given phase, position in cycle (x 0,1)) and velocity parameters 
        獲取給定相位、步態位置和速度參數的關節角度。
        :param phase: 布林值，表示當前是否處於同相位。
        :param x: 步態循環中的位置（0 到 1）。
        :param velocity: 速度向量 [vx, vy, vt]。
        :return: 字典形式的關節角度。
        """
        angles = {}
        for j in self.pfn.keys():
            if phase:
                v = self.pfn[j].get(x)
                angles[j] = v
            else:
                angles[j] = self.afn[j].get(x)
        self.apply_velocity(angles, velocity, phase, x)
        return angles

    def show(self):
        """
        Display the CPG functions used
        顯示使用的步態函數。
        """
        for j in self.pfn.keys():
            print(j, "p", self.pfn[j], "a", self.afn[j])

    def apply_velocity(self, angles, velocity, phase, x):
        """ 
        Modify on the walk-on-spot joint angles to apply the velocity vector
        根據速度向量修改步態關節角度。
        :param angles: 當前的關節角度字典。
        :param velocity: 速度向量 [vx, vy, vt]。
        :param phase: 布林值，表示當前是否處於同相位。
        :param x: 步態循環中的位置（0 到 1）。
        """

        # VX
        v = velocity[0] * self.parameters["vx_scale"]
        d = (x * 2 - 1) * v
        if phase:
            angles["l_hip_pitch"] += d
            angles["l_ank_pitch"] += d
            angles["r_hip_pitch"] += d
            angles["r_ank_pitch"] += d
        else:
            angles["l_hip_pitch"] -= d
            angles["l_ank_pitch"] -= d
            angles["r_hip_pitch"] -= d
            angles["r_ank_pitch"] -= d

        # VY
        v = velocity[1] * self.parameters["vy_scale"]
        d = (x) * v
        d2 = (1 - x) * v
        if v >= 0:
            if phase:
                angles["l_hip_roll"] -= d
                angles["l_ank_roll"] -= d
                angles["r_hip_roll"] += d
                angles["r_ank_roll"] += d
            else:
                angles["l_hip_roll"] -= d2
                angles["l_ank_roll"] -= d2
                angles["r_hip_roll"] += d2
                angles["r_ank_roll"] += d2
        else:
            if phase:
                angles["l_hip_roll"] += d2
                angles["l_ank_roll"] += d2
                angles["r_hip_roll"] -= d2
                angles["r_ank_roll"] -= d2
            else:
                angles["l_hip_roll"] += d
                angles["l_ank_roll"] += d
                angles["r_hip_roll"] -= d
                angles["r_ank_roll"] -= d

        # VT
        v = velocity[2] * self.parameters["vt_scale"]
        d = (x) * v
        d2 = (1 - x) * v
        if v >= 0:
            if phase:
                angles["l_hip_yaw"] = -d
                angles["r_hip_yaw"] = d
            else:
                angles["l_hip_yaw"] = -d2
                angles["r_hip_yaw"] = d2
        else:
            if phase:
                angles["l_hip_yaw"] = d2
                angles["r_hip_yaw"] = -d2
            else:
                angles["l_hip_yaw"] = d
                angles["r_hip_yaw"] = -d


class Walker:
    """
    Class for making Darwin walk
    """

    def __init__(self, op3):
        self.op3 = op3
        self.running = False

        self.velocity = [0, 0, 0]
        self.walking = False
        self.func = WFunc()

        # ~ self.ready_pos=get_walk_angles(10)
        self.ready_pos = self.func.get(True, 0, [0, 0, 0])
        self._th_walk = None

    def cmd_vel(self, vx, vy, vt):
        print("cmdvel", (vx, vy, vt))
        self.start()
        self.set_velocity(vx, vy, vt)

    def init_walk(self):
        """
        If not there yet, go to initial walk position
        """
        if self.get_dist_to_ready() > 0.02:
            self.updateready_pos()
            self.op3.set_angles_slow(self.ready_pos)


    def start(self):
        if not self.running:
            self.running = True
            self.init_walk()
            self._th_walk = Thread(target=self._do_walk)
            self._th_walk.start()
            self.walking = True

    def stop(self):
        if self.running:
            self.walking = False
            self.running = False

    def set_velocity(self, x, y, t):
        self.velocity = [x, y, t]

    def _do_walk(self):
        """
        Main walking loop, smoothly update velocity vectors and apply corresponding angles
        """
        func = self.func

        # Global walk loop
        n = 50
        p = True
        i = 0
        self.current_velocity = [0, 0, 0]
        while self.walking or i < n or self.is_walking():
            if not self.walking:
                self.velocity = [0, 0, 0]
            if not self.is_walking() and i == 0:  # Do not move if nothing to do and already at 0
                self.update_velocity(self.velocity, n)
                time.sleep(1. / 240.)
                continue
            x = float(i) / n
            angles = func.get(p, x, self.current_velocity)
            self.update_velocity(self.velocity, n)
            self.op3.set_angles(angles)
            i += 1
            if i > n:
                i = 0
                p = not p
            time.sleep(1. / 240.)

        self._th_walk = None

    def updateready_pos(self):
        self.ready_pos['l_sho_roll'] = 0.8
        self.ready_pos['r_sho_roll'] = -0.8
        self.ready_pos['l_el'] = -0.7
        self.ready_pos['r_el'] = 0.7

    def is_walking(self):
        e = 0.02
        for v in self.current_velocity:
            if abs(v) > e: return True
        return False

    def rescale(self, angles, coef):
        z = {}
        for j, v in angles.items():
            offset = self.ready_pos[j]
            v -= offset
            v *= coef
            v += offset
            z[j] = v
        return z

    def update_velocity(self, target, n):
        a = 3 / float(n)
        b = 1 - a
        self.current_velocity = [a * t + b * v for (t, v) in zip(target, self.current_velocity)]

    def get_dist_to_ready(self):
        angles = self.op3.get_angles()
        return get_distance(self.ready_pos, angles)


def interpolate(anglesa, anglesb, coefa):
    z = {}
    joints = anglesa.keys()
    for j in joints:
        z[j] = anglesa[j] * coefa + anglesb[j] * (1 - coefa)
    return z


def get_distance(anglesa, anglesb):
    d = 0
    joints = anglesa.keys()
    if len(joints) == 0: return 0
    for j in joints:
        d += abs(anglesb[j] - anglesa[j])
    d /= len(joints)
    return d


if __name__ == "__main__":

    op3 = OP3()
    op3.update_data_th(log_interval=0.25, log_file="./torque_log.csv") #log torque data to file, interval should less than 0.25s
    walker = Walker(op3)
    time.sleep(1)
    walker.start()
    walker.set_velocity(1, 0, 0)

    while True:
        # print("Joint Torques:", op3.joint_torques)
        time.sleep(0.25)
