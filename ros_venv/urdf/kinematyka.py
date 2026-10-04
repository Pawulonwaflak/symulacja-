import roboticstoolbox as rtb
import numpy as np

robot = rtb.ERobot.URDF("/home/pawla/ros2_ws/ros_venv/urdf/Rover.urdf")

q0 = np.zeros(robot.n)

# znajdź WSZYSTKIE linki, które są "liśćmi" (końcami gałęzi) - to zwykle koła
# albo po prostu policz fkine dla każdego linku i zobacz który pasuje
for link in robot.links:
    T = robot.fkine(q0, end=link.name)
    x, y, z = T.t
    print(f"{link.name:30s}  x={x:7.4f}  y={y:7.4f}  z={z:7.4f}")