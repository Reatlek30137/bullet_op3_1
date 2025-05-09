import time
from threading import Thread

import pybullet_data
import pybullet as p

op3_joints = ['l_hip_yaw',
              'l_hip_roll',
              'l_hip_pitch',
              'l_knee',
              'l_ank_pitch',
              'l_ank_roll',
              'r_hip_yaw',
              'r_hip_roll',
              'r_hip_pitch',
              'r_knee',
              'r_ank_pitch',
              'r_ank_roll',
              'l_sho_pitch',
              'l_sho_roll',
              'l_el',
              'r_sho_pitch',
              'r_sho_roll',
              'r_el',
              'head_pan',
              'head_tilt']

class OP3:
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setGravity(0, 0, -9.8)
        op3StartPos = [0, 0, 0.3]
        op3StartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.planeId = p.loadURDF("plane.urdf")
        self.robot = p.loadURDF("../models/robotis_op3.urdf", op3StartPos, op3StartOrientation)
        self.numJoints = p.getNumJoints(self.robot)
        self.targetVel = 0
        self.maxForce = 100

        self.update_data_th()
        # We go real-time simulation rather than call stepSimulation
        self.run_sim_th()
        self._set_joint()

        self.joints = op3_joints
        self.angles = None

    def get_angles(self):
        if self.joints is None: return None
        if self.angles is None: return None
        return dict(zip(self.joints, self.angles))

    def set_angles(self, angles):
        for j, v in angles.items():
            if j not in self.joints:
                AssertionError("Invalid joint name " + j)
                continue
            p.setJointMotorControl(self.robot, op3_joints.index(j), p.POSITION_CONTROL, v, self.maxForce)

    def set_angles_slow(self, stop_angles, delay=2):
        start_angles = self.get_angles()
        start = time.time()
        stop = start + delay
        while True:
            t = time.time()
            if t > stop: break
            ratio = (t - start) / delay
            angles = interpolate(stop_angles, start_angles, ratio)
            self.set_angles(angles)
            time.sleep(0.1)

    def update_data_th(self, log_interval=None, log_file="torque_log.csv"):
        """
        更新角度與扭矩資訊，並可選擇性地將扭矩資訊寫入檔案。
        :param log_interval: 紀錄扭矩的時間間隔（秒），若為 None 則不紀錄。
        :param log_file: 紀錄扭矩的檔案名稱。
        """
        def _cb_datas():
            last_log_time = time.time()
            with open(log_file, "w") as f:
                # 寫入 CSV 標頭，第一列為時間，後續列為關節名稱
                f.write("time," + ",".join(self.joints) + "\n")

                while True:
                    angles = []
                    joint_torques = []
                    for joint in range(self.numJoints):
                        joint_state = p.getJointState(self.robot, joint)
                        angles.append(joint_state[0])
                        joint_torques.append(joint_state[3])

                    self.angles = angles
                    self.joint_torques = joint_torques

                    # 如果設定了 log_interval，則進行紀錄
                    if log_interval is not None:
                        current_time = time.time()
                        if current_time - last_log_time >= log_interval:
                            # 寫入當前時間和所有關節的扭矩值
                            f.write(f"{current_time}," + ",".join(map(str, self.joint_torques)) + "\n")
                            last_log_time = current_time
                            f.flush()  # 確保即時寫入檔案

                    time.sleep(0.001)

        Thread(target=_cb_datas).start()

    def _set_joint(self):
        for joint in range(self.numJoints):
            print(p.getJointInfo(self.robot, joint))
            p.setJointMotorControl(self.robot, joint, p.POSITION_CONTROL, self.targetVel, self.maxForce)

    def run_sim_th(self):
        def _cb_sim():
            while True:
                p.stepSimulation()
                time.sleep(1.0 / (240.0))
        Thread(target=_cb_sim).start()

    def run(self):
        try:
            while True:
                # p.stepSimulation()
                time.sleep(1./240.)
        finally:
            OP3Pos, OP3Orn = p.getBasePositionAndOrientation(self.robot)
            print(OP3Pos, OP3Orn)
            p.disconnect()


def interpolate(anglesa, anglesb, coefa):
    z = {}
    joints = anglesa.keys()
    for j in joints:
        z[j] = anglesa[j] * coefa + anglesb[j] * (1 - coefa)
    return z


if __name__ == '__main__':
    op3 = OP3()
    op3.run()
    pass
