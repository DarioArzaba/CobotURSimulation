from robolink import *    # RoboDK API
from robodk import *
import robotiq_gripper
import math
import numpy as np
import time

#Funciones para gripper (NO ALTERAR)
def log_info(gripper):
    print(f"Pos: {str(gripper.get_current_position()): >3} "
          f"Open: {gripper.is_open(): <2} "
          f"Closed: {gripper.is_closed(): <2} ")


def dt_gripper_position(current_position): #(NO ALTERAR)
    dt_pos = [0]
    dt_pos[0] = 85 - (current_position / gMax_Pos) * 85
    return dt_pos

# Function to move real gripper and see it in RoboDK simulation (Open) (NO ALTERAR)
def open_gripper(): 
    if RDK.RunMode() > 1:
        RDK.setRunMode(1) #Set RunMode to see movement in robot and simulation
    time.sleep(0.5)
    gripper.move_and_wait_for_pos(int(gMin_Pos), 255, 255) #Move real gripper
    log_info(gripper) #Print info about gripper position
    dt_pos = dt_gripper_position(gripper.get_current_position()) # give actual position of gripper in milimeters
    dt_gripper.MoveJ(dt_pos) #move simulated gripper
    time.sleep(0.5)
    RDK.setRunMode(6) #Set Run mode to PC-Real Robot connection
    time.sleep(0.5)

# Function to move real gripper and see it in RoboDK simulation (Close) (NO ALTERAR)
def close_gripper():
    if RDK.RunMode() > 1:
        RDK.setRunMode(1)
    gripper.move_and_wait_for_pos(int(gMax_Pos), 255, 191)
    log_info(gripper)
    dt_pos = dt_gripper_position(gripper.get_current_position())
    dt_gripper.MoveJ(dt_pos)
    #input("Press 'Enter' to continue...")
    gripper.move_and_wait_for_pos(gMax_Pos, 255, 191)
    log_info(gripper)
    dt_pos = dt_gripper_position(gripper.get_current_position())
    dt_gripper.MoveJ(dt_pos)
    #input("Press 'Enter' to continue...")
    time.sleep(0.5)
    RDK.setRunMode(6)
    time.sleep(0.5)

#Suministar los datos de posicion y rotacion ROLL PITCH YAW 
"""
VTCP: Vector Tool Pose
X : Posicion en X -- en milimetros [mm]
Y : Posicion en Y -- en milimetros [mm]
Z : Posicion en Z -- en milimetros [mm]
RX : Rotacion en X - Yaw -- En grados [°]
RY : Rotacion en Y - Pitch -- En grados [°]
RZ : Rotacion en Z - Roll -- En grados [°]
VTCP = [X,Y,Z,RX,RY,RZ]
"""
VTCP=[1000,300,500,90,-90,90] #Ejemplo

#Conexion a RoboDK y robot
robot_ip = "192.168.1.10"  # Replace with your robot ip
rdk_ip = 'localhost'  # Or the explicit ip of the computer running RoboDK
### Connect to the RoboDK API ###
RDK = Robolink(robodk_ip=rdk_ip, port=20500)

rdk_robot = RDK.Item('UR10e')  # retrieve the robot by name
dt_gripper = RDK.Item('RobotiQ 2F85')  # the robotiq gripper inside RoboDK, It is not TCP attached to robot
"""
#Este codigo es ESENCIAL para asegurar la conexion al robot
status, status_msg = rdk_robot.ConnectedState()
while status != ROBOTCOM_READY:
    print(status_msg)
    rdk_robot.Disconnect()
    time.sleep(2)
    try:
        success = rdk_robot.Connect(robot_ip=robot_ip) ##IP UR10e PolyScope
        #success = Robot.Connect(robot_ip="192.168.1.10") ##IP UR10e real
        status, status_msg = rdk_robot.ConnectedState()
    except e:
        print(status_msg)
    if status == ROBOTCOM_READY:
        break

# Uses robotiq_gripper class to connect to gripper via sockets
print("Creating gripper...")
gripper = robotiq_gripper.RobotiqGripper()
print("Connecting to gripper...")
gripper.connect(robot_ip, 63352)  # The port on the URe family is 63352
print("Activating gripper...")
# When this executes the gripper will close and open again. Be careful.
# This function gets the max and min values of the gripper
gripper.activate()
# The gripper not always closes using the default max. It depends on the calibration
# So its is useful to know the current max of the system
gMax_Pos = gripper.get_max_position() #Fully Close
gMin_Pos = gripper.get_min_position() #Fully Open 
"""
#Longitudes de links
d1= 180.7 #En milimetros
a2 = -612.7 #En mm
a3= -571.55 #En mm
d4= 174.15 #En mm
d5= 119.85 #En mm
d6= 116.55+211.5 #En mm. Se suma el valor del TCP de la herramienta

#Inserte algoritmo de cinematica inversa
def sen(grados):
    return math.sin(math.radians(grados))

def cos(grados):
    return math.cos(math.radians(grados))
    
def CInversa(VTCP):
    PX=VTCP[0]
    PY=VTCP[1]
    PZ=VTCP[2]
    Psi=VTCP[3]
    Theta=VTCP[4]
    phi=VTCP[5]
    T=np.array([[cos(phi)*cos(Theta),(sen(Psi)*cos(phi)*sen(Theta))-(cos(Psi)*sen(phi)),(sen(Psi)*sen(phi))+(cos(Psi)*cos(phi)*sen(Theta)),PX],
                [cos(Theta)*sen(phi), (cos(Psi)*cos(phi))+(sen(Psi)*sen(phi)*sen(Theta)),(cos(Psi)*sen(phi)*sen(Theta))-(sen(Psi)*cos(phi)),PY],
                [-sen(Theta),sen(Psi)*cos(Theta),cos(Psi)*cos(Theta),PZ],
                [0, 0, 0, 1]])
    for i in range(4):
        for j in range(4):
            T[i][j]=round(T[i][j],2)

    #Calculo del angulo theta1
    #Vector posicion de 0 a 5
    Z06=[[T[0][2]],[T[1][2]],[T[2][2]]]
    P06=[[T[0][3]],[T[1][3]],[T[2][3]]]
    P05=P06-np.dot(d6,Z06) #Se resta a la posicion en el SC6 la distancia al SC5 considerando la rotacion Z06 
    phi1=math.degrees(math.atan2(P05[1],P05[0]))
    P05XY=math.sqrt(math.pow(P05[1],2)+math.pow(P05[0],2))
    phi2=math.degrees(math.acos(d4/P05XY)) 
    theta1=phi1+phi2+90
    theta1=round(theta1,2)

    #INICIO DE CODIGO DE CINEMATICA INVERSA (Del Angulo 2 al 6)


    #IMPRESION DE CADA ANGULO (NO TOCAR)
    print("\ntheta1:",round(theta1,2))
    print("theta2:",round(theta2,2))
    print("theta3:",round(theta3,2))
    print("theta4:",round(theta4,2))
    print("theta5:",round(theta5,2))
    print("theta6:",round(theta6,2))
    
    return theta1,theta2,theta3,theta4,theta5,theta6

#MOVER AL ROBOT AL PUNTO CALCULADO PARA CORROBORAR
rdk_robot.MoveJ(list(CInversa(VTCP)))
#close_gripper()
#open_gripper()

rdk_robot.Stop()
rdk_robot.Disconnect()

