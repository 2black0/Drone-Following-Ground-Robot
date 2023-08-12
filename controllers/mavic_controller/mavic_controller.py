from controller import Robot, Keyboard
from simple_pid import PID
import numpy as np
import cv2
from cv2 import aruco

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

def find_aruco(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    arucoParameters = aruco.DetectorParameters_create()
    arucoCorner, arucoId, arucoReject = aruco.detectMarkers(gray, arucoDict, parameters=arucoParameters)
    return arucoCorner, arucoId, arucoReject

def run_robot(robot):
    ################################################################
    ## initialize all
    ################################################################
    timestep = int(robot.getBasicTimeStep())
    
    verticalThrust = 68.5
    
    xParameter = [0.3, 0.12, 0.25]
    yParameter = [0.3, 0.1, 0.21]
    altitudeParameter = [0.6, 0.04, 0.15]
    rollParameter = [2.1, 1.1, 1.1]
    pitchParameter = [2.5, 1.5, 1.2]
    yawParameter = [0.55, 0.0008, 3]

    xTarget = 0.00
    yTarget = 0.00
    altitudeTarget = 3.00
    rollTarget = 0.00
    pitchTarget = 0.00
    yawTarget = 0.00

    xTargetMarker = 0.00
    yTargetMarker = 0.00
    altitudeTargetMarker = 0.00

    rollGimbal = 0.00
    pitchGimbal = 0.00
    yawGimbal = 0.00

    statusTakeoff = False
    statusLanding = False
    statusGimbal = False
    statusHome = False
    statusAruco = False

    maxSpeed = 576
    motorMultiplier = 1.00

    ################################################################
    # PID Controller
    ################################################################
    xPID = PID(float(xParameter[0]), float(xParameter[1]), float(xParameter[2]), setpoint=float(xTarget))
    yPID = PID(float(yParameter[0]), float(yParameter[1]), float(yParameter[2]), setpoint=float(yTarget))
    altitudePID = PID(float(altitudeParameter[0]), float(altitudeParameter[1]), float(altitudeParameter[2]), setpoint=float(altitudeTarget))
    yPID.output_limits = (-0.5, 0.5)
    xPID.output_limits = (-0.5, 0.5)
    altitudePID.output_limits = (-1.5, 1.5)
    
    rollPID = PID(float(rollParameter[0]), float(rollParameter[1]), float(rollParameter[2]), setpoint=float(rollTarget))
    pitchPID = PID(float(pitchParameter[0]), float(pitchParameter[1]), float(pitchParameter[2]), setpoint=float(pitchTarget))       
    yawPID = PID(float(yawParameter[0]), float(yawParameter[1]), float(yawParameter[2]), setpoint=float(yawTarget))
    rollPID.output_limits = (-1.5, 1.5)
    pitchPID.output_limits = (-1.5, 1.5)
    yawPID.output_limits = (-1.5, 1.5)
    
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
    #robot.motorFrontLeft.setVelocity(0.00)
    #robot.motorFrontRight.setVelocity(0.00)
    #robot.motorBackLeft.setVelocity(0.00)
    #robot.motorBackRight.setVelocity(0.00)
    
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
    cameraWidth = int(robot.camera.getWidth())  # 800
    cameraHeight = int(robot.camera.getHeight())  # 480
    ################################################################
    
    ################################################################
    # Main loop:
    ################################################################
    while robot.step(timestep) != -1:      
        key = robot.keyboard.getKey()
        
        ################################################################
        # Read all sensor
        ################################################################       
        roll, pitch, yaw = robot.imu.getRollPitchYaw()
        #rollAccel, pitchAccel, yawAccel = robot.gyro.getValues()
        xPos, yPos, altitudePos = robot.gps.getValues()
        image = np.frombuffer(robot.camera.getImage(), dtype=np.uint8).reshape((robot.camera.getHeight(), robot.camera.getWidth(), 4))
               
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
            print("All Motor Stop")
        
        elif key == ord('5'):
            statusAruco = True
            print("Aruco Detection Run")
        
        elif key == ord('6'):
            statusAruco = False
            print("Aruco Detection Stop")
    
        if statusTakeoff == True:
            xPosMarker = 0.00
            yPosMarker = 0.00
            
            yawPID.setpoint = 0.00
            altitudePID.setpoint = 3.10
            yPID.setpoint = 0.00
            xPID.setpoint = 0.00
            
            yPIDValue = 0.00 #yPID(yPos)
            xPIDValue = 0.00 # xPID(xPos)
            
            if statusAruco == True:
                arucoCorner, arucoId, _ = find_aruco(image)
                if arucoId is not None:
                    image = cv2.line(image, (int(cameraWidth / 2), cameraHeight), (int(cameraWidth / 2), 0), (255, 255, 0), 1)
                    image = cv2.line(image, (0, int(cameraHeight / 2)), (cameraWidth, int(cameraHeight / 2)), (255, 255, 0), 1)
                    arucoId = arucoId.flatten()
                    
                    for(markerCorner, markerId) in zip(arucoCorner, arucoId):
                        corners = markerCorner.reshape((4, 2))
                        (topLeft, topRight, bottomRight, bottomLeft) = corners
                        
                        topRight = (int(topRight[0]), int(topRight[1]))
                        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                        topLeft = (int(topLeft[0]), int(topLeft[1]))

                        shapes = image.copy()
                        cv2.rectangle(shapes, topLeft, bottomRight, (0, 255, 0), -1)
                        alpha = 0.4
                        image = cv2.addWeighted(shapes, alpha, image, 1 - alpha, 0)

                        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                        cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                        xPosMarker = cY / 100
                        yPosMarker = cX / 100
                        print("xPosMarker={:+.2f}|yPosMarker={:+.2f}".format(xPosMarker, yPosMarker))

                        yPIDValue = yPID(yPosMarker)
                        #xPIDValue = xPID(xPosMarker)
                        
            
            yawInput = yawPID(yaw)
            verticalInput = altitudePID(altitudePos)
            rollInput = rollPID(roll) - yPIDValue
            pitchInput = pitchPID(pitch) + xPIDValue
            
            motorInputFrontLeft = verticalThrust + verticalInput - yawInput - pitchInput + rollInput
            motorInputFrontRight = verticalThrust + verticalInput + yawInput - pitchInput - rollInput
            motorInputBackLeft = verticalThrust + verticalInput + yawInput + pitchInput + rollInput
            motorInputBackRight = verticalThrust + verticalInput - yawInput + pitchInput - rollInput
            
            #print("roll={:+.2f}|pitch={:+.2f}|yaw={:+.2f}|xPos={:+.2f}|yPos={:+.2f}|altitudePos={:+.2f}|MFL={:+.2f}|MFR={:+.2f}|MBL={:+.2f}|MBR={:+.2f}|xPosMarker={:+.2f}|yPosMarker={:+.2f}".format(roll, pitch, yaw, xPos, yPos, altitudePos, motorInputFrontLeft, motorInputFrontRight, motorInputBackLeft, motorInputBackRight, xPosMarker, yPosMarker))
            
        elif statusTakeoff == False:
            motorInputFrontLeft = 0
            motorInputFrontRight = 0
            motorInputBackLeft = 0
            motorInputBackRight = 0
                    
        cv2.imshow("Camera", image)
        cv2.waitKey(1)
                
        robot.motorFrontLeft.setVelocity(clamp(motorInputFrontLeft, -576, 576))
        robot.motorFrontRight.setVelocity(clamp(-motorInputFrontRight, -576, 576))
        robot.motorBackLeft.setVelocity(clamp(-motorInputBackLeft, -576, 576))
        robot.motorBackRight.setVelocity(clamp(motorInputBackRight, -576, 576))
        
    cv2.destroyAllWindows()
    
      
if __name__ == "__main__":
    # create the Robot instance.
    my_robot = Robot()
    run_robot(my_robot)