"""epuck_avoid_collision controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor
import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)


motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')

motor_left.setPosition(float('Inf'))
motor_right.setPosition(float('Inf'))


ds = []
for i in range(8):
    ds.append(robot.getDevice('ps' + str(i)))
    ds[-1].enable(timestep)
    
    
ls = []
for i in range(8):
    ls.append(robot.getDevice('ls' + str(i)))
    ls[-1].enable(timestep)


print("All sensors are enabled")



state = "FORWARD_1"

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    d = []
    for dist in ds:
        d.append(dist.getValue())
        
    d = np.asarray(d)
    d = (d/1000)*3.14
    print(d)
    
    
    l = []
    for lightSensor in ls:
        l.append(lightSensor.getValue())
        
    # print(f"l before scaling down - {l}")
        
    l = np.asarray(l)
    l = (l/9000)*3.14
    # print(l)
        

    if state == "FORWARD_1":
        phi_l = 3.14 - d[0] - d[1] - d[2]  
        phi_r = 3.14 - d[7] - d[6] - d[5]
        if d[0] > 0.35:
            # print("start turning")
            state = "TURN_180"
    elif state == "TURN_180":
        phi_l = -3.14
        phi_r = 3.14
        # print("still turning")
        if d[4] > 0.33:
            state = "FORWARD_2"
            print("turn complete, moving backward")
    elif state == "FORWARD_2":
        phi_l = 3.14 - d[0] - d[1] - d[2]  
        phi_r = 3.14 - d[7] - d[6] - d[5]
        if d[0] > 0.35:
            print("right turn")
            state = "TURN_90"
    elif state == "TURN_90":
        phi_l = 3.14
        phi_r = -3.14
        if d[5] > 0.38:
            state = "FORWARD_3"
            print("turn complete, moving right")
            print(d[5])
    elif state == "FORWARD_3":
        phi_l = 3.14 - d[0] - d[1] - d[2]  
        phi_r = 3.14 - d[7] - d[6] - d[5]
        if d[5] < 0.22:
            state = "STOP"
            print("passed edge, stopping")
    elif state == "STOP":
         phi_l = 0.0
         phi_r = 0.0
      
           
    # initialize motor speeds at 50% of MAX_SPEED.   
    # write actuators inputs
    motor_left.setVelocity(phi_l)
    motor_right.setVelocity(phi_r)
    
    pass

# Enter here exit cleanup code.
