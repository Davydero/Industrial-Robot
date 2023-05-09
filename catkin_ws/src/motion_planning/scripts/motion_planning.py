#!/usr/bin/python3

import os
import math
import copy
import json
import actionlib #para crear action servers y clients
import control_msgs.msg
from controller import ArmController
from gazebo_msgs.msg import ModelStates
import rospy
from pyquaternion import Quaternion as PyQuaternion
import numpy as np
from gazebo_ros_link_attacher.srv import SetStatic, SetStaticRequest, SetStaticResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

PKG_PATH = os.path.dirname(os.path.abspath(__file__))
CONT_PAQ = 0 

MODELS_INFO = {
    "X1-Y2-Z1": {
        "home": [0.264589, -0.293903, 0.777] #puntos donde se colocan de acuerdo al tipo de pieza
    },
    "X2-Y2-Z2": {
        "home": [0.277866, -0.724482, 0.777] 
    },
    "X1-Y3-Z2": {
        "home": [0.268053, -0.513924, 0.777]  
    },
    "X1-Y2-Z2": {
        "home": [0.429198, -0.293903, 0.777] 
    },
    "X1-Y2-Z2-CHAMFER": {
        "home": [0.592619, -0.293903, 0.777]  
    },
    "X1-Y4-Z2": {
        "home": [0.108812, -0.716057, 0.777] 
    },
    "X1-Y1-Z2": {
        "home": [0.088808, -0.295820, 0.777] 
    },
    "X1-Y2-Z2-TWINFILLET": {
        "home": [0.103547, -0.501132, 0.777] 
    },
    "X1-Y3-Z2-FILLET": {
        "home": [0.433739, -0.507130, 0.777]  
    },
    "X1-Y4-Z1": {
        "home": [0.589908, -0.501033, 0.777]  
    },
    "X2-Y2-Z2-FILLET": {
        "home": [0.442505, -0.727271, 0.777] 
    },
    "BARRASIM": {
        "home": [0.3559086, -0.58, 0.89]
        #"home": [0.5899086+0.062, -0.58, 0.89]  #originalmente era como del X1-Y4-Z1
    }
}

for model, model_info in MODELS_INFO.items():
    pass
    #MODELS_INFO[model]["home"] = model_info["home"] + np.array([0.0, 0.10, 0.0])

for model, info in MODELS_INFO.items():
    model_json_path = os.path.join(PKG_PATH, "..", "models", f"lego_{model}", "model.json")
    # make path absolute
    model_json_path = os.path.abspath(model_json_path)
    # check path exists
    if not os.path.exists(model_json_path):
        raise FileNotFoundError(f"Model file {model_json_path} not found")

    model_json = json.load(open(model_json_path, "r"))
    corners = np.array(model_json["corners"])

    size_x = (np.max(corners[:, 0]) - np.min(corners[:, 0]))
    size_y = (np.max(corners[:, 1]) - np.min(corners[:, 1]))
    size_z = (np.max(corners[:, 2]) - np.min(corners[:, 2]))

    #print(f"{model}: {size_x:.3f} x {size_y:.3f} x {size_z:.3f}")

    MODELS_INFO[model]["size"] = (size_x, size_y, size_z)

# Compensate for the interlocking height
INTERLOCKING_OFFSET = 0.019
INT_OFF_X = 0.0155
TAM_X = 0.031
SAFE_X = -0.40
SAFE_Y = -0.13
SURFACE_Z = 0.774

# Resting orientation of the end effector
DEFAULT_QUAT = PyQuaternion(axis=(0, 1, 0), angle=math.pi)
# Resting position of the end effector
DEFAULT_POS = (-0.1, -0.2, 1.2)

DEFAULT_PATH_TOLERANCE = control_msgs.msg.JointTolerance()
DEFAULT_PATH_TOLERANCE.name = "path_tolerance"
DEFAULT_PATH_TOLERANCE.velocity = 10

def get_gazebo_model_name(model_name, vision_model_pose):
    """
        Get the name of the model inside gazebo. It is needed for link attacher plugin.
    """
    models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
    epsilon = 0.05
    for gazebo_model_name, model_pose in zip(models.name, models.pose):
        if model_name not in gazebo_model_name:
            continue
        # Get everything inside a square of side epsilon centered in vision_model_pose
        ds = abs(model_pose.position.x - vision_model_pose.position.x) + abs(model_pose.position.y - vision_model_pose.position.y)
        if ds <= epsilon:
            return gazebo_model_name
    raise ValueError(f"Model {model_name} at position {vision_model_pose.position.x} {vision_model_pose.position.y} was not found!")


def get_model_name(gazebo_model_name):
    return gazebo_model_name.replace("lego_", "").split("_", maxsplit=1)[0]


def get_legos_pos(vision=False):
    #get legos position reading vision topic
    if vision:
        #legos = rospy.wait_for_message("/lego_detections", ModelStates, timeout=None) #wait_for_message hace que se suscriba a un topico, se reciba el mensaje y luego se desuscriba, en legos se almacena el mensaje ModelStates consta de nombre, pose y twist en el mundo de gazebo
        legos = rospy.wait_for_message("/detections", ModelStates, timeout=None)
    else:
        models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
        legos = ModelStates()

        for name, pose in zip(models.name, models.pose):
            if "X" not in name:
                continue
            name = get_model_name(name)

            legos.name.append(name)
            legos.pose.append(pose)
    return [(lego_name, lego_pose) for lego_name, lego_pose in zip(legos.name, legos.pose)]


def straighten(model_pose, gazebo_model_name):
    x = model_pose.position.x
    y = model_pose.position.y
    z = model_pose.position.z
    model_quat = PyQuaternion(
        x=model_pose.orientation.x,
        y=model_pose.orientation.y,
        z=model_pose.orientation.z,
        w=model_pose.orientation.w)
    print('model quat', model_quat)

    model_size = MODELS_INFO[get_model_name(gazebo_model_name)]["size"]

    """
        Calculate approach quaternion and target quaternion
    """

    facing_direction = get_axis_facing_camera(model_quat)
    #facing_direction = (0,0,1) #probablemente tenga que ser en formato quaternion y por eso se tiene algun problema
    approach_angle = get_approach_angle(model_quat, facing_direction)

    print(f"Lego is facing {facing_direction}")
    print(f"Angle of approaching measures {approach_angle:.2f} deg")
    # Calculate approach quat
    approach_quat = get_approach_quat(facing_direction, approach_angle)#PROBABLEMENTE ESTE SEA EL QUE CHUEQUEA

    # Get above the object
    dist_adicional=0.03
    adX =  dist_adicional*math.cos(approach_angle)
    adY =  dist_adicional*math.sin(approach_angle)
    controller.move_to(x+adX, y+adY, target_quat=approach_quat) #primero el efector se mueve a la posicion X Y encima del objeto, aqui ya llega chueco REVISAR
    facing_direction = (0, 0, 1) #linea solo para probar CON 001 AGARRA CHUECO
    # Calculate target quat
    regrip_quat = DEFAULT_QUAT #DEFAULT_QUAT = PyQuaternion(axis=(0, 1, 0), angle=math.pi)
    if facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):  # Side
        target_quat = DEFAULT_QUAT 
        pitch_angle = -math.pi/2 + 0.2

        if abs(approach_angle) < math.pi/2:
            target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi/2)
        else:
            target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi/2)
        target_quat = PyQuaternion(axis=(0, 1, 0), angle=pitch_angle) * target_quat

        if facing_direction == (0, 1, 0):
            regrip_quat = PyQuaternion(axis=(0, 0, 1), angle=math.pi/2) * regrip_quat

    elif facing_direction == (0, 0, -1):
        """
            Pre-positioning
        """
        controller.move_to(z=z, target_quat=approach_quat)
        close_gripper(gazebo_model_name, model_size[0])

        tmp_quat = PyQuaternion(axis=(0, 0, 1), angle=2*math.pi/6) * DEFAULT_QUAT
        controller.move_to(SAFE_X, SAFE_Y, z+0.05, target_quat=tmp_quat, z_raise=0.1)  # Move to safe position
        controller.move_to(z=z)
        open_gripper(gazebo_model_name)

        approach_quat = tmp_quat * PyQuaternion(axis=(1, 0, 0), angle=math.pi/2)

        target_quat = approach_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi)  # Add a yaw rotation of 180 deg

        regrip_quat = tmp_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi)
    else:
        target_quat = DEFAULT_QUAT
        target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi/2)

    """
        Grip the model
    """
    #target_quat = DEFAULT_QUAT #esta linea fue anadida solo para probar
    if facing_direction == (0, 0, 1) or facing_direction == (0, 0, -1):
        closure = model_size[0]
        z = SURFACE_Z + model_size[2] / 8
    elif facing_direction == (1, 0, 0):
        closure = model_size[1]
        z = SURFACE_Z + model_size[0] / 8
    elif facing_direction == (0, 1, 0):
        closure = model_size[0]
        z = SURFACE_Z + model_size[1] / 8 
    controller.move_to(z=z, target_quat=approach_quat)
    close_gripper(gazebo_model_name, closure)

    """
        Straighten model if needed
    """
    if facing_direction != (0, 0, 1):
        z = SURFACE_Z + model_size[2]/2

        controller.move_to(z=z+0.05, target_quat=target_quat, z_raise=0.1)
        controller.move(dz=-0.05)
        open_gripper(gazebo_model_name)

        # Re grip the model
        controller.move_to(z=z, target_quat=regrip_quat, z_raise=0.1)
        close_gripper(gazebo_model_name, model_size[0])


def close_gripper(gazebo_model_name, closure=0):
    #set_gripper(0.81-closure*10)
    set_gripper(0.81-closure*9.5)
    rospy.sleep(0.5)
    # Create dynamic joint
    if gazebo_model_name is not None:
        req = AttachRequest()
        req.model_name_1 = gazebo_model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        attach_srv.call(req)


def open_gripper(gazebo_model_name=None):
    set_gripper(0.0)

    # Destroy dynamic joint
    if gazebo_model_name is not None:
        req = AttachRequest()
        req.model_name_1 = gazebo_model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        detach_srv.call(req)


def set_model_fixed(model_name):
    req = AttachRequest()
    req.model_name_1 = model_name
    req.link_name_1 = "link"
    req.model_name_2 = "ground_plane"
    req.link_name_2 = "link"
    attach_srv.call(req)

    req = SetStaticRequest()
    print("{} TO HOME".format(model_name))
    req.model_name = model_name
    req.link_name = "link"
    req.set_static = True #originalmente era True

    setstatic_srv.call(req)


def get_approach_quat(facing_direction, approach_angle):
    quat = DEFAULT_QUAT
    if facing_direction == (0, 0, 1):
        pitch_angle = 0
        yaw_angle = 0 #original
        #yaw_angle = math.pi/2 
    elif facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):
        #pitch_angle = + 0.2 #este es el angulo que se chuequeaba original
        pitch_angle = 0
        if abs(approach_angle) < math.pi/2:
            yaw_angle = math.pi/2 #original
            #yaw_angle = 0
        else:
            yaw_angle = -math.pi/2 #original
            #yaw_angle = 0
    elif facing_direction == (0, 0, -1):
        pitch_angle = 0
        yaw_angle = 0
    else:
        raise ValueError(f"Invalid model state {facing_direction}")

    quat = quat * PyQuaternion(axis=(0, 1, 0), angle=pitch_angle)
    quat = quat * PyQuaternion(axis=(0, 0, 1), angle=yaw_angle)
    #quat = quat * PyQuaternion(axis=(0, 1, 0), angle=0)#estas dos lineas solo para probar si el efector no se chuequea
    #quat = quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi/2)
    quat = PyQuaternion(axis=(0, 0, 1), angle=approach_angle+math.pi/2) * quat

    return quat


def get_axis_facing_camera(quat):
    axis_x = np.array([1, 0, 0])
    axis_y = np.array([0, 1, 0])
    axis_z = np.array([0, 0, 1])
    new_axis_x = quat.rotate(axis_x)
    new_axis_y = quat.rotate(axis_y)
    new_axis_z = quat.rotate(axis_z)
    # get angle between new_axis and axis_z
    angle = np.arccos(np.clip(np.dot(new_axis_z, axis_z), -1.0, 1.0))
    # get if model is facing up, down or sideways
    if angle < np.pi / 3:
        return 0, 0, 1
    elif angle < np.pi / 3 * 2 * 1.2:
        if abs(new_axis_x[2]) > abs(new_axis_y[2]):
            return 1, 0, 0
        else:
            return 0, 1, 0
        #else:
        #    raise Exception(f"Invalid axis {new_axis_x}")
    else:
        return 0, 0, -1


def get_approach_angle(model_quat, facing_direction):#get gripper approach angle
    if facing_direction == (0, 0, 1): #funcion original todo lo comentado
        return model_quat.yaw_pitch_roll[0] - math.pi/2 #rotate gripper
    elif facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):
        axis_x = np.array([0, 1, 0])
        axis_y = np.array([-1, 0, 0])
        new_axis_z = model_quat.rotate(np.array([0, 0, 1])) #get z axis of lego
        # get angle between new_axis and axis_x
        dot = np.clip(np.dot(new_axis_z, axis_x), -1.0, 1.0) #sin angle between lego z axis and x axis in fixed frame
        det = np.clip(np.dot(new_axis_z, axis_y), -1.0, 1.0) #cos angle between lego z axis and x axis in fixed frame
        return math.atan2(det, dot) #get angle between lego z axis and x axis in fixed frame
    elif facing_direction == (0, 0, -1):
        return -(model_quat.yaw_pitch_roll[0] - math.pi/2) % math.pi - math.pi
    else:
        raise ValueError(f"Invalid model state {facing_direction}") 
    """if facing_direction == (0, 0, 1) or facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0): #con esto si agarra del mismo angulo todos los bloques pero no es el angulo correcto
        axis_y = np.array([0, 1, 0])
        axis_x = np.array([-1, 0, 0])
        new_axis_z = model_quat.rotate(np.array([0, 0, 1])) #get z axis of lego
        # get angle between new_axis and axis_x
        dot = np.clip(np.dot(new_axis_z, axis_x), -1.0, 1.0) #sin angle between lego z axis and x axis in fixed frame
        det = np.clip(np.dot(new_axis_z, axis_y), -1.0, 1.0) #cos angle between lego z axis and x axis in fixed frame
        return math.atan2(det, dot) #get angle between lego z axis and x axis in fixed frame
    elif facing_direction == (0, 0, -1):
        return -(model_quat.yaw_pitch_roll[0] - math.pi/2) % math.pi - math.pi
    else:
        raise ValueError(f"Invalid model state {facing_direction}")"""
def set_gripper(value):
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = value  # From 0.0 to 0.8
    goal.command.max_effort = -1  # # Do not limit the effort
    action_gripper.send_goal_and_wait(goal, rospy.Duration(10))

    return action_gripper.get_result()


if __name__ == "__main__":
    print("Initializing node of kinematics")
    rospy.init_node("send_joints")

    controller = ArmController() # el objeto que contiene los joints, la posicion inicial, el topico, etc

    # Create an action client for the gripper
    action_gripper = actionlib.SimpleActionClient(
        "/gripper_controller/gripper_cmd", #este parametro es el nombre del servidor de acciones 
        control_msgs.msg.GripperCommandAction #la estructura del mensaje
    )
    print("Waiting for action of gripper controller")
    action_gripper.wait_for_server() #solo se espera por el servidor, para realizar la accion se debe ver la linea de send goal

    setstatic_srv = rospy.ServiceProxy("/link_attacher_node/setstatic", SetStatic) #inicializador de servicios, llamada a un servicio
    attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
    setstatic_srv.wait_for_service() #bloquea hasta que el servicio este disponible
    attach_srv.wait_for_service()
    detach_srv.wait_for_service()

    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT) #se envia posicion y orientacion de reposo del efector final DEFAULT_POS = (-0.1, -0.2, 1.2) VVVV DEFAULT_QUAT = PyQuaternion(axis=(0, 1, 0), angle=math.pi)

    print("Waiting for detection of the models")
    rospy.sleep(0.5)
    legos = get_legos_pos(vision=True) #si se manda True hace uso del sistema de vision, en caso contrario obtiene la posicion en base a la simulacion de Gazebo directamente, retorna los nombres y las posiciones de los bloques
    legos.sort(reverse=True, key=lambda a: (a[1].position.x, a[1].position.y)) #ordena los bloques primero en base a su coordenada X y luego a su Y

    for model_name, model_pose in legos:
        open_gripper()
        try:
            model_home = MODELS_INFO[model_name]["home"] #buscar el parametro home del bloquei
            model_size = MODELS_INFO[model_name]["size"]
            #print('model size: ', model_size)
        except ValueError as e:
            print(f"Model name {model_name} was not recognized!")
            continue

        # Get actual model_name at model xyz coordinates
        try:
            gazebo_model_name = get_gazebo_model_name(model_name, model_pose)#nos retorna el nombre del modelo en gazebo si fue encontrado en la simulacion
        except ValueError as e:
            print(e)
            continue

        # Straighten lego
        straighten(model_pose, gazebo_model_name) #basicamente hasta aqui se hace el agarre del objeto
        controller.move(dz=0.15)

        """
            Go to destination
        """
        x, y, z = model_home
        z += model_size[2] / 2 +0.004
        print(f"Moving model {model_name} to {x} {y} {z}")

        #controller.move_to(x, y, target_quat=DEFAULT_QUAT * PyQuaternion(axis=[0, 0, 1], angle=math.pi / 2)) #este angulo que se manda es el que gira sobre el eje z
        #controller.move_to(y=y-0.08, target_quat=DEFAULT_QUAT * PyQuaternion(axis=[0, 0, 1], angle=math.pi / 2)) 
        controller.move_to(x=0.11,y=y-0.08, target_quat=DEFAULT_QUAT * PyQuaternion(axis=[0, 0, 1], angle=0)) 
        if CONT_PAQ<5:
            controller.move_to(x)
            controller.move_to(x, y, target_quat=DEFAULT_QUAT * PyQuaternion(axis=[1, 0, 0], angle=math.pi / 4))
            dif_imag = 0.02
            difY = dif_imag*math.cos(math.pi / 4)
            difZ = dif_imag*math.sin(math.pi / 4)
            controller.move_to(y=y-difY, z=z-difZ)
            # Lower the object and release
            #controller.move_to(x+0.014,y, z)
            controller.move_to(x+0.014)
            CONT_PAQ = CONT_PAQ+1
        else:
            controller.move_to(x+0.01)
            controller.move_to(x+0.01, y, target_quat=DEFAULT_QUAT * PyQuaternion(axis=[1, 0, 0], angle=math.pi / 4))
            dif_imag = 0.02
            difY = dif_imag*math.cos(math.pi / 4)
            difZ = dif_imag*math.sin(math.pi / 4)
            controller.move_to(y=y-difY, z=z-difZ)
            # Lower the object and release
            #controller.move_to(x+0.014,y, z)
            #controller.move_to(x+0.014)
            CONT_PAQ = 0

        
        #set_model_fixed(gazebo_model_name) #esto hace que el bloque se quede fijo en la simulacion hay que analizar si se deshabilitara
        open_gripper(gazebo_model_name)
        rospy.sleep(0.5)#no habia este sleep
        controller.move(dz=0.15)

        if controller.gripper_pose[0][1] > -0.3 and controller.gripper_pose[0][0] > 0:
            controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)

        # increment z in order to stack lego correctly #en aqui debemos poner el patron de matriz con el que el robot ira acomodando los paquetes 
        #MODELS_INFO[model_name]["home"][2] += model_size[2] - INTERLOCKING_OFFSET original
        #MODELS_INFO[model_name]["home"][0] += model_size[0] - INT_OFF_X*2
        MODELS_INFO[model_name]["home"][0] -= model_size[0]
        
    print("Moving to Default Position")
    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
    open_gripper()
    rospy.sleep(0.4)