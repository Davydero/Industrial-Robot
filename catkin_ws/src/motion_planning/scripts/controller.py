import math
import copy
import rospy
import numpy as np
import kinematics
import control_msgs.msg
import trajectory_msgs.msg
from pyquaternion import Quaternion


def get_controller_state(controller_topic, timeout=None):
    return rospy.wait_for_message(
        f"{controller_topic}/state",
        control_msgs.msg.JointTrajectoryControllerState,
        timeout=timeout)


class ArmController:
    def __init__(self, gripper_state=0, controller_topic="/trajectory_controller"):
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.gripper_state = gripper_state

        self.controller_topic = controller_topic
        self.default_joint_trajectory = trajectory_msgs.msg.JointTrajectory()#es el mensaje de tipo trayectoria que define a que punto se movera??
        self.default_joint_trajectory.joint_names = self.joint_names #al anterior mensaje se le asignan los joint names

        joint_states = get_controller_state(controller_topic).actual.positions #obtiene las posiciones actuales de los joints??
        x, y, z, rot = kinematics.get_pose(joint_states) #en base a la cinematica directa se devuelve las posiciones X,y y z
        self.gripper_pose = (x, y, z), Quaternion(matrix=rot)

        # Create an action client for the joint trajectory        Crea el publicador en el topico /trajectory_controller/command
        self.joints_pub = rospy.Publisher(
            f"{self.controller_topic}/command",
            trajectory_msgs.msg.JointTrajectory, queue_size=10)

    def move(self, dx=0, dy=0, dz=0, delta_quat=Quaternion(1, 0, 0, 0), blocking=True):
        (sx, sy, sz), start_quat = self.gripper_pose

        tx, ty, tz = sx + dx, sy + dy, sz + dz
        target_quat = start_quat * delta_quat

        self.move_to(tx, ty, tz, target_quat, blocking=blocking)

    def move_to(self, x=None, y=None, z=None, target_quat=None, z_raise=0.0, blocking=True): #aqui se pasa el argumento de a donde quieres que se mueva el brazo
        """
        Move the end effector to target_pos with target_quat as orientation
        :param x:
        :param y:
        :param z:
        :param start_quat:
        :param target_pos:
        :param target_quat:
        :param z_raise:
        :param blocking:
        :return:
        """

        def smooth(percent_value, period=math.pi):
            return (1 - math.cos(percent_value * period)) / 2

        (sx, sy, sz), start_quat = self.gripper_pose #sx , sy sz y start quat son la posicion del gripper en el estado antes de que se realice el movimiento

        if x is None:
            x = sx
        if y is None:
            y = sy
        if z is None:
            z = sz
        if target_quat is None:
            target_quat = start_quat

        dx, dy, dz = x - sx, y - sy, z - sz #diferencia que hay entre el objetivo y la posicion inicial
        length = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2) * 300 + 80 #se calcula la distancia entre 2 puntos en un espacio 3D
        speed = length

        steps = int(length)
        step = 1 / steps #es el paso en forma de porcentaje 

        for i in np.arange(0, 1 + step, step):
            i_2 = smooth(i, 2 * math.pi)  # from 0 to 1 to 0
            i_1 = smooth(i)  # from 0 to 1

            grip = Quaternion.slerp(start_quat, target_quat, i_1) #retorna un cuaternion unitario que enlaza los dos cuaterniones que entran como argumentos
            self.send_joints( #se envia los joints paso a paso
                sx + i_1*dx, sy + i_1*dy, sz + i_1*dz + i_2*z_raise,
                grip,
                duration=1/speed*0.9)
            rospy.sleep(1/speed)

        if blocking:
            self.wait_for_position(tol_pos=0.005, tol_vel=0.08)

        self.gripper_pose = (x, y, z), target_quat

    def send_joints(self, x, y, z, quat, duration=1.0):  # x,y,z and orientation of lego block (del punto de destino en si)
        # Solve for the joint angles, select the 5th solution
        joint_states = kinematics.get_joints(x, y, z, quat.rotation_matrix) #se obtiene la cinematica inversa (los angulos de los joints para llegar a X y z)

        traj = copy.deepcopy(self.default_joint_trajectory) #default_joint_trajectory hasta este punto solo tiene los nombres de los joints, es para copiar objetos que tienen otros objetos en su interior

        for _ in range(0, 2):
            pts = trajectory_msgs.msg.JointTrajectoryPoint()
            pts.positions = joint_states
            pts.velocities = [0, 0, 0, 0, 0, 0]
            pts.time_from_start = rospy.Time(duration)
            # Set the points to the trajectory
            traj.points = [pts]
            # Publish the message
            self.joints_pub.publish(traj) #se publica para la simulacion? publica en el topico /trajectory_controller/command

    def wait_for_position(self, timeout=2, tol_pos=0.01, tol_vel=0.01):
        end = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < end:
            msg = get_controller_state(self.controller_topic, timeout=10)
            v = np.sum(np.abs(msg.actual.velocities), axis=0)
            if v < tol_vel:
                for actual, desired in zip(msg.actual.positions, msg.desired.positions):
                    if abs(actual - desired) > tol_pos:
                        break
                    return
        rospy.logwarn("Timeout waiting for position")