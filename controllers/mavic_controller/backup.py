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
    timestep = int(robot.getBasicTimeStep())
    
    verticalThrust = 68.5
    xParameter = [0.3, 0.12, 0.25]
    yParameter = [0.3, 0.1, 0.21]
    altitudeParameter = [0.5, 0.01, 0.15]
    rollParameter = [2.1, 1.1, 1.1]#[3.5, 1.7, 4.2]
    pitchParameter = [2.5, 1.5, 1.2]#[3.9, 1.9, 4.0]
    yawParameter = [0.55, 0.0008, 3]

    counter = 0

    xTarget = 0.0
    yTarget = 0.0
    altitudeTarget = 5.0
    rollTarget = 0.0
    pitchTarget = 0.0
    yawTarget = 0.0

    xTargetMarker = 0.0
    yTargetMarker = 0.0
    altitudeTargetMarker = 0.0

    rollGimbal = 0.0
    pitchGimbal = 0.0
    yawGimbal = 0.0

    statusTakeoff = False
    statusLanding = False
    statusGimbal = False
    statusHome = False
    statusAruco = False

    ################################################################
    # PID Controller
    ################################################################
    altitudePID = PID(float(altitudeParameter[0]), float(altitudeParameter[1]), float(altitudeParameter[2]), setpoint=float(altitudeTarget))
    altitudePID.output_limits = (-1.5, 1.5)    
    yawPID = PID(float(yawParameter[0]), float(yawParameter[1]), float(yawParameter[2]), setpoint=float(yawTarget))    
    pitchPID = PID(float(pitchParameter[0]), float(pitchParameter[1]), float(pitchParameter[2]), setpoint=float(pitchTarget))       
    rollPID = PID(float(rollParameter[0]), float(rollParameter[1]), float(rollParameter[2]), setpoint=float(rollTarget))
    xPID = PID(float(xParameter[0]), float(xParameter[1]), float(xParameter[2]), setpoint=float(xTarget))
    yPID = PID(float(yParameter[0]), float(yParameter[1]), float(yParameter[2]), setpoint=float(yTarget))
    xPID.output_limits = (-0.5, 0.5)
    yPID.output_limits = (-0.5, 0.5)
    
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
    robot.motorFrontLeft.setVelocity(0.00)
    robot.motorFrontRight.setVelocity(0.00)
    robot.motorBackLeft.setVelocity(0.00)
    robot.motorBackRight.setVelocity(0.00)
    
    # Gimbal
    robot.gimbalRoll = robot.getDevice("camera roll")
    robot.gimbaPitch = robot.getDevice("camera pitch")
    robot.gimbalYaw = robot.getDevice("camera yaw")
    robot.gimbalRoll.setPosition(0.00)
    robot.gimbaPitch.setPosition(0.00)
    robot.gimbalYaw.setPosition(0.00)
    
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
    # Main loop:
    ################################################################
    while robot.step(timestep) != -1:
        maxSpeed = 576
        motorMultiplier = 1
        
        key = robot.keyboard.getKey()
        
        ################################################################
        # Read all sensor
        ################################################################       
        roll, pitch, yaw = robot.imu.getRollPitchYaw()
        rollAccel, pitchAccel, yawAccel = robot.gyro.getValues()
        xPos, yPos, altitudePos = robot.gps.getValues()
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
            yawPID.setpoint = yawTarget
            yawInput = yawPID(yaw)
            
            altitudePID.setpoint = altitudeTarget
            verticalInput = altitudePID(altitudePos)
            
            rollInput = rollPID(roll) - yPID(yPos)
            pitchInput = pitchPID(pitch) + xPID(xPos)
            
            motorInputFrontLeft = verticalThrust + verticalInput - yawInput - pitchInput + rollInput
            motorInputFrontRight = verticalThrust + verticalInput + yawInput - pitchInput - rollInput
            motorInputBackLeft = verticalThrust + verticalInput + yawInput + pitchInput + rollInput
            motorInputBackRight = verticalThrust + verticalInput - yawInput + pitchInput - rollInput
            
            print("roll={:+.2f}|pitch={:+.2f}|yaw={:+.2f}|RollA={:+.2f}|pitchA={:+.2f}|yawA={:+.2f}|xPos={:+.2f}|yPos={:+.2f}|altitudePos={:+.2f}|MFL={:+.2f}|MFR={:+.2f}|MBL={:+.2f}|MBR={:+.2f}".format(roll, pitch, yaw, rollAccel, pitchAccel, yawAccel, xPos, yPos, altitudePos, motorInputFrontLeft, motorInputFrontRight, motorInputBackLeft, motorInputBackRight))
            
        elif statusTakeoff == False:
            motorInputFrontLeft = 0
            motorInputFrontRight = 0
            motorInputBackLeft = 0
            motorInputBackRight = 0
        
        robot.motorFrontLeft.setVelocity(clamp(motorInputFrontLeft, -576, 576))
        robot.motorFrontRight.setVelocity(clamp(-motorInputFrontRight, -576, 576))
        robot.motorBackLeft.setVelocity(clamp(-motorInputBackLeft, -576, 576))
        robot.motorBackRight.setVelocity(clamp(motorInputBackRight, -576, 576))
        
    cv2.destroyAllWindows()
    
      
if __name__ == "__main__":
    # create the Robot instance.
    my_robot = Robot()
    run_robot(my_robot)