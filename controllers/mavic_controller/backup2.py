from controller import Robot, Keyboard
from simple_pid import PID
import numpy as np
import cv2
from cv2 import aruco
import math

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

verticalThrust = 68.5
xParameter = [2, 2, 4]
yParameter = [1.5, 2, 3]
altitudeParameter = [4, 0.05, 10]
rollParameter = [45, 1, 7]
pitchParameter = [35, 1, 7]
yawParameter = [0.5, 0.0075, 3]

counter = 0

xTarget = 0.0
ytarget = 0.0
altituteTarget = 0.0
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
altitudePID = PID(float(altitudeParameter[0], float(altitudeParameter[1], float(altitudeParameter[2]), setpoint=float(altituteTarget))    
yawPID = PID(float(yawParameter[0], float(yawParameter[1], float(yawParameter[2]), setpoint=float(yawTarget))    
pitchPID = PID(float(pitchParameter[0], float(pitchParameter[1], float(pitchParameter[2]), setpoint=float(pitchTarget))       
rollPID = PID(float(yawParameter[0], float(yawParameter[1], float(yawParameter[2]), setpoint=float(yawTarget))

def run_robot(robot):
    ################################################################
    ## initialize all
    ################################################################
    timestep = robot.getBasicTimeStep()
    
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
            altitudePID.setpoint = 2.77
            verticalInput = altitudePID(zPos)
            
            yawPID.setpoint = 0.00
            yawInput = yawPID(heading)
            
            yTarget = 0.00
            pitchError = clamp(yPos - yTarget, -1.55, 1.55)
            pitchPID.setpoint = 0.00
            pitchPIDValue = pitchPID(pitch)
            pitchInput = pitchPIDValue - pitchError
            
            xTarget = 1.00
            rollError = clamp(xPos - xTarget, -1.55, 1.55)
            rollPID.setpoint = 0.00
            rollPIDValue = rollPID(roll)
            rollInput = rollPIDValue - rollError
            
            motorInputFrontLeft = 68.5 + verticalInput - yawInput - pitchInput + rollInput
            motorInputFrontRight = 68.5 + verticalInput + yawInput - pitchInput - rollInput
            motorInputBackLeft = 68.5 + verticalInput + yawInput + pitchInput + rollInput
            motorInputBackRight = 68.5 + verticalInput - yawInput + pitchInput - rollInput
            
            #print("RollE={:+.2f}|RollPID={:+.2f}|rollInput={:+.2f}|PitchE={:+.2f}|PitchPID={:+.2f}|pitchInput={:+.2f}".format(rollError, rollPIDValue, rollInput, pitchError, pitchPIDValue, pitchInput))
            #print("rollInput={:+.2f}|pitchInput={:+.2f}|MFL={:+.2f}|MFR={:+.2f}|MBL={:+.2f}|MBR={:+.2f}".format(rollInput, pitchInput, motorInputFrontLeft, motorInputFrontRight, motorInputBackLeft, motorInputBackRight))
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