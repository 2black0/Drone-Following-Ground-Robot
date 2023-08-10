from controller import Robot, Keyboard
from simple_pid import PID
import numpy as np
import cv2
from cv2 import aruco
import math

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

def run_robot(robot):
    ################################################################
    ## initialize all
    ################################################################
    timestep = 32
    
    # Keyboard
    robot.keyboard = robot.getKeyboard()
    robot.keyboard.enable(10 * timestep)
    
    # Motor
    robot.motorFrontLeft = robot.getDevice("front left propeller")
    robot.motorFrontRight = robot.getDevice("front right propeller")
    robot.motorBackLeft = robot.getDevice("rear left propeller")
    robot.motorBackRight = robot.getDevice("rear right propeller")
    robot.motorFrontLeft.setPosition(float('inf'))
    robot.motorFrontRight.setPosition(float('inf'))
    robot.motorBackLeft.setPosition(float('inf'))
    robot.motorBackRight.setPosition(float('inf'))
    robot.motorFrontLeft.setVelocity(0.0)
    robot.motorFrontRight.setVelocity(0.0)
    robot.motorBackLeft.setVelocity(0.0)
    robot.motorBackRight.setVelocity(0.0)
    
    # Gimbal
    robot.gimbalRoll = robot.getDevice("camera roll")
    robot.gimbaPitch = robot.getDevice("camera pitch")
    robot.gimbalYaw = robot.getDevice("camera yaw")
    robot.gimbalRoll.setPosition(0.0)
    robot.gimbaPitch.setPosition(0.0)
    robot.gimbalYaw.setPosition(0.0)
    
    # IMU
    robot.imu = robot.getDevice("inertial unit")    
    robot.imu.enable(timestep)
    
    # Gyro
    robot.gyro = robot.getDevice("gyro")
    robot.gyro.enable(timestep)
    
    # Compass
    robot.compass = robot.getDevice("compass")
    robot.compass.enable(timestep)
    
    # GPS
    robot.gps = robot.getDevice("gps")
    robot.gps.enable(timestep)
    
    # Camera
    robot.camera = robot.getDevice("camera")
    robot.camera.enable(timestep)
    ################################################################
    
    ################################################################
    # PID Controller
    ################################################################
    altitudePID = PID(3, 0.05, 9, setpoint=0.00)    
    yawPID = PID(0.03, 0, 0.015, setpoint=0.00)    
    #pitchPID = PID(0.1, 0.08, 0.018, setpoint=0.00)
    pitchPID = PID(0.2, 0.07, 0.04, setpoint=0.00)       
    rollPID = PID(0.095, 0.05, 0.012, setpoint=0.00)
    
    statusTakeoff = False
    
    ################################################################
    # Main loop:
    ################################################################
    while robot.step(timestep) != -1:
        maxSpeed = 576
        motorMultiplier = 1
        
        key = robot.keyboard.getKey()
        
        ################################################################
        # Read all sensor
        ################################################################       
        roll = ((robot.imu.getRollPitchYaw()[0] + math.pi / 2.0) * 180 / math.pi) - 90 #roll_right = positive; roll_left = negative
        pitch = robot.imu.getRollPitchYaw()[1] * 180 / math.pi # nose_up = negative; nose_down = positive

        rollAccel = robot.gyro.getValues()[0] * 180 / math.pi #roll_right = positive; roll_left = negative
        pitchAccel = robot.gyro.getValues()[1] * 180 / math.pi # nose_up = positive; nose_down = negative

        heading = math.atan2(robot.compass.getValues()[0], robot.compass.getValues()[1]) - (math.pi / 2)
        if heading < -math.pi:
            heading = heading + (2 * math.pi) # ccw = positive; cw = negative
        heading = heading * 180 / math.pi
        
        xPos = robot.gps.getValues()[0] # forward = positive; backward = negative
        yPos = robot.gps.getValues()[1] # left = positive; right = negative
        zPos = robot.gps.getValues()[2] # up = positive; down = negative
        
        image = np.frombuffer(robot.camera.getImage(), dtype=np.uint8).reshape((robot.camera.getHeight(), robot.camera.getWidth(), 4))
        
        cv2.imshow("Camera", image)
        cv2.waitKey(1)
        
        ################################################################
        # Drone Motor Test
        ################################################################    
        if key == ord("1"):
            robot.motorFrontLeft.setVelocity(maxSpeed * motorMultiplier)
            robot.motorFrontRight.setVelocity(-maxSpeed * motorMultiplier)
            robot.motorBackLeft.setVelocity(-maxSpeed * motorMultiplier)
            robot.motorBackRight.setVelocity(maxSpeed * motorMultiplier)
            print("Motor Test")
        elif key == ord('2'):
            robot.gimbalRoll.setPosition(0.0) # min = -0.5, max = 0.5
            robot.gimbaPitch.setPosition(1.6) # min = -0.5, max = 1.7
            robot.gimbalYaw.setPosition(0.0) # min = -1.7, max = 1.7
            print("Gimbal Test")
        elif key == ord('3'):
            statusTakeoff = True
            print("Takeoff Run")
        elif key == ord('4'):
            statusTakeoff = False
            print("Takeoff Run")
    
        if statusTakeoff == True:
            altitudePID.setpoint = 2.25
            verticalInput = altitudePID(zPos)
            
            yawPID.setpoint = 0.00
            yawInput = yawPID(heading)
            
            yTarget = 0.0
            pitchError = clamp(yPos - yTarget, -1.5, 1.5)
            pitchPID.setpoint = 0.00
            pitchInput = pitchPID(pitch) - pitchError
            
            xTarget = 0.0
            rollError = clamp(xPos - xTarget, -1.5, 1.5)
            rollPID.setpoint = 0.00
            rollInput = rollPID(roll) - rollError
            
            motorInputFrontLeft = 68.5 + verticalInput - yawInput - pitchInput + rollInput
            motorInputFrontRight = 68.5 + verticalInput + yawInput - pitchInput - rollInput
            motorInputBackLeft = 68.5 + verticalInput + yawInput + pitchInput + rollInput
            motorInputBackRight = 68.5 + verticalInput - yawInput + pitchInput - rollInput
            
            print("roll={:+.2f}|rollA={:+.2f}|pitch={:+.2f}|pitchA={:+.2f}|head={:+.2f}|x={:+.2f}|y={:+.2f}|z={:+.2f}|MFL={:+.2f}|MFR={:+.2f}|MBL={:+.2f}|MBR={:+.2f}".format(roll, rollAccel, pitch, pitchAccel, heading, xPos, yPos, zPos, motorInputFrontLeft, motorInputFrontRight, motorInputBackLeft, motorInputBackRight))
            
        elif statusTakeoff == False:
            motorInputFrontLeft = 0
            motorInputFrontRight = 0
            motorInputBackLeft = 0
            motorInputBackRight = 0
        
        robot.motorFrontLeft.setVelocity(motorInputFrontLeft)
        robot.motorFrontRight.setVelocity(-motorInputFrontRight)
        robot.motorBackLeft.setVelocity(-motorInputBackLeft)
        robot.motorBackRight.setVelocity(motorInputBackRight)
        
    cv2.destroyAllWindows()
    
      
if __name__ == "__main__":
    # create the Robot instance.
    my_robot = Robot()
    run_robot(my_robot)