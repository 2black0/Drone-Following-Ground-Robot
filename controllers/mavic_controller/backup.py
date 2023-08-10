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
    
    #altitudePID = PID(3.15, 0.07, 4.3, setpoint=0.00)
    altitudePID = PID(4.0, 0.05, 10, setpoint=0.00)
    altitudePID.output_limits = (-1.5, 1.5)
    
    #yawPID = PID(1.35, 0.0025, 10.5, setpoint=0.00)
    yawPID = PID(0.5, 0.0075, 3, setpoint=0.00)
    yawPID.output_limits = (-0.5, 0.5)
    
    pitchPID = PID(2, 0.1, 2, setpoint=0.00)
    pitchPID.output_limits = (-1.5, 1.5)
    
    rollPID = PID(2, 0.1, 2, setpoint=0.00)
    rollPID.output_limits = (-1.5, 1.5)
    
    '''
    altitudePID = PID(10, 0.1, 5, setpoint=0.00)    
    yawPID = PID(2, 0, 2, setpoint=0.00)    
    pitchPID = PID(2, 0.1, 2, setpoint=0.00)    
    rollPID = PID(2, 0.1, 2, setpoint=0.00)
    '''
    
    statusTakeoff = False
    
    ################################################################
    # Main loop:
    ################################################################
    while robot.step(timestep) != -1:
        maxSpeed = 576
        takeoffMultiplier = 1
        landingMultiplier = 0.0
        
        key = robot.keyboard.getKey()
        
        ################################################################
        # Read all sensor
        ################################################################
        '''
        roll, pitch, yaw = robot.imu.getRollPitchYaw() # roll left=negative, right=positive; pitch nose_up=negative, nose_down=positive; yaw ccw=negative, cw=positive
        rollAccel, pitchAccel, yawAccel = robot.gyro.getValues() # roll left=negative, right=positive; pitch nose_up=positive, nose_down=negative; yaw ccw=positive, cw=negative
        heading = math.atan2(robot.compass.getValues()[0], robot.compass.getValues()[1]) - (math.pi / 2)
        if heading < -math.pi:
            heading = heading + (2 * math.pi) # heading ccw=positive 0->3.14, cw=negative 0->-3.14
        xPos, yPos, zPos = robot.gps.getValues() # x forward=positive, backward=negative; y left=positive, right=negative; z up=positive, down=negative
        image = np.frombuffer(robot.camera.getImage(), dtype=np.uint8).reshape((robot.camera.getHeight(), robot.camera.getWidth(), 4))
        '''
        
        roll = ((robot.imu.getRollPitchYaw()[0] + math.pi / 2.0) * 180 / math.pi) - 90 #roll_right = positive(1.57); roll_left = negative(-1.57)
        pitch = robot.imu.getRollPitchYaw()[1] * 180 / math.pi # nose_up = negative(1.5); nose_down = positive(-1.5)

        rollAccel = robot.gyro.getValues()[0] * 180 / math.pi #roll_right = positive(3.14); roll_left = negative(-3.14)
        pitchAccel = robot.gyro.getValues()[1] * 180 / math.pi # nose_up = positive(3.14); nose_down = negative(-3.14)

        heading = math.atan2(robot.compass.getValues()[0], robot.compass.getValues()[1]) - (math.pi / 2)
        if heading < -math.pi:
            heading = heading + (2 * math.pi) # ccw = positive(3.14); cw = negative(-3.14)
        heading = heading * 180 / math.pi
        
        xPos = robot.gps.getValues()[0] # forward = positive; backward = negative
        yPos = robot.gps.getValues()[1] # left = positive; kanan = negative
        zPos = robot.gps.getValues()[2] # up = positive; down = negative
        
        image = np.frombuffer(robot.camera.getImage(), dtype=np.uint8).reshape((robot.camera.getHeight(), robot.camera.getWidth(), 4))
        
        if key == ord("6"):
            print("roll={:+.2f}|pitch={:+.2f}|rollA={:+.2f}|pitchA={:+.2f}|head={:+.2f}|x={:+.2f}|y={:+.2f}|z={:+.2f}".format(roll, pitch, rollAccel, pitchAccel, heading, xPos, yPos, zPos))
        
        cv2.imshow("Camera", image)
        cv2.waitKey(1)
        
        ################################################################
        # Drone Motor Test
        ################################################################    
        if key == ord("1"):
            robot.motorFrontLeft.setVelocity(maxSpeed * takeoffMultiplier)
            robot.motorFrontRight.setVelocity(-maxSpeed * takeoffMultiplier)
            robot.motorBackLeft.setVelocity(-maxSpeed * takeoffMultiplier)
            robot.motorBackRight.setVelocity(maxSpeed * takeoffMultiplier)
            print("Motor Run")
        elif key == ord("2"):
            statusTakeoff = False
            robot.motorFrontLeft.setVelocity(maxSpeed * landingMultiplier)
            robot.motorFrontRight.setVelocity(-maxSpeed * landingMultiplier)
            robot.motorBackLeft.setVelocity(-maxSpeed * landingMultiplier)
            robot.motorBackRight.setVelocity(maxSpeed * landingMultiplier)
            print("Motor Stop")
        elif key == ord('3'):
            robot.gimbalRoll.setPosition(0.0) # min = -0.5, max = 0.5
            robot.gimbaPitch.setPosition(1.6) # min = -0.5, max = 1.7
            robot.gimbalYaw.setPosition(0.0) # min = -1.7, max = 1.7
            print("Gimbal Run")
        elif key == ord('4'):
            robot.gimbalRoll.setPosition(0.0) # min = -0.5, max = 0.5
            robot.gimbaPitch.setPosition(0.0) # min = -0.5, max = 1.7
            robot.gimbalYaw.setPosition(0.0) # min = -1.7, max = 1.7
            print("Gimbal Reset")
        elif key == ord('5'):
            statusTakeoff = True
            print("Takeoff Run")
    
        if statusTakeoff == True:
            altitudePID.setpoint = 2.00
            verticalInput = altitudePID(zPos)
            
            yawPID.setpoint = 0.00
            yawInput = yawPID(heading)
            
            #yTarget = 0.00
            pitchPID.setpoint = 0.00
            #pitchError = clamp(yPos + yTarget, -1, 1)
            pitchInput = pitchPID(-pitch)# + clamp(pitchError,-1,1)) 
            
            #yTarget = 0.0
            #pitchPID = [33, 1, 7]
            #pitchError = clamp(yPos + yTarget, -1, 1)
            #pitchInput = - clamp((pitchPID[0] * clamp(pitch, -1, 1)) + (pitchPID[2] * pitchAccel) - pitchError,-1.5, 1.5)
            #print("pitch={:+.2f}|yPos={:+.2f}|pitchAccel={:+.2f}|pitchError={:+.2f}|pitchInput={:+.2f}".format(pitch, yPos, pitchAccel, pitchError, pitchInput))

            #xTarget = 0.00                  
            rollPID.setpoint = 0.00
            rollInput = rollPID(roll)# + clamp(xPos + xTarget,-1,1))
            
            #xTarget = 0.0
            #rollPID = [5, 0, 2]
            #rollError = clamp(xPos + xTarget, -1.5, 1.5)
            #rollInput = -(rollPID[0] * clamp(roll, -1, 1)) + (rollPID[2] * rollAccel) - rollError
            '''
            
            altitudePID.setpoint = 2.15
            verticalInput = altitudePID(zPos)
            
            yawPID.setpoint = 0.00
            yawInput = yawPID(heading)
            
            #pitchInput = 10 * pitch + pitchAccel + pitchPID(xPos)
            #rollInput = 10 * roll + rollAccel + rollPID(yPos)
            
            pitchInput = 0.5 * pitch #+ pitchPID(xPos)
            rollInput = 0.5 * roll #+ rollPID(yPos)
            
            '''
            
            motorInputFrontLeft = 68.5 + verticalInput - yawInput + pitchInput - rollInput
            motorInputFrontRight = 68.5 + verticalInput + yawInput + pitchInput + rollInput
            motorInputBackLeft = 68.5 + verticalInput + yawInput - pitchInput - rollInput
            motorInputBackRight = 68.5 + verticalInput - yawInput - pitchInput + rollInput
            
            motorInputFrontLeft = clamp(motorInputFrontLeft, -78.5, 78.5)
            motorInputFrontRight = clamp(motorInputFrontRight, -78.5, 78.5)
            motorInputBackLeft = clamp(motorInputBackLeft, -78.5, 78.5)
            motorInputBackRight = clamp(motorInputBackRight, -78.5, 78.5)
            
            #print("pitch={:+.2f}|y={:+.2f}|pitchInput={:+.2f}".format(pitch, yPos, pitchInput))
            #print("roll={:+.2f}|rollE={:+.2f}|rollAccel={:+.2f}|x={:+.2f}|rollInput={:+.2f}".format(roll, rollError, rollAccel, xPos, rollInput))
            #print("verticalInput={:+.2f}|yawInput={:+.2f}|pitchInput={:+.2f}|rollInput={:+.2f}".format(verticalInput, yawInput, pitchInput, rollInput))
            
            print("roll={:+.2f}|rollA={:+.2f}|pitch={:+.2f}|pitchA={:+.2f}|head={:+.2f}|x={:+.2f}|y={:+.2f}|z={:+.2f}|MFL={:+.2f}|MFR={:+.2f}|MBL={:+.2f}|MBR={:+.2f}".format(roll, rollAccel, pitch, pitchAccel, heading, xPos, yPos, zPos, motorInputFrontLeft, motorInputFrontRight, motorInputBackLeft, motorInputBackRight))
            
            #robot.motorFrontLeft.setVelocity(motorInputFrontLeft)
            #robot.motorFrontRight.setVelocity(-motorInputFrontRight)
            #robot.motorBackLeft.setVelocity(-motorInputBackLeft)
            #robot.motorBackRight.setVelocity(motorInputBackRight)
    
        
    cv2.destroyAllWindows()
    
      
if __name__ == "__main__":
    # create the Robot instance.
    my_robot = Robot()
    run_robot(my_robot)