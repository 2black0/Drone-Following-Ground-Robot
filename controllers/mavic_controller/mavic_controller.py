from tracemalloc import start
from controller import Robot, Keyboard, Receiver
from simple_pid import PID
from time import sleep
import numpy as np
import cv2
from cv2 import aruco
from csv_logger import CsvLogger
import json

filename = "logger.csv"
header = [
    "time",
    "counter",
    "roll",
    "pitch",
    "yaw",
    "rollAccel",
    "pitchAccel",
    "yawAccel",
    "xPosition",
    "yPosition",
    "altitude",
    "rollError",
    "pitchError",
    "rollInput",
    "pitchInput",
    "yawInput",
    "verticalInput",
    "motorFrontLeftInput",
    "motorFrontRightInput",
    "motorRearLeftInput",
    "motorRearRightInput",
    "Speed_X",
    "Speed_Y",
    "Speed_Z",
    "statusTakeoff",
    "statusHome",
    "statusAruco",
    "statusLanding",
]
csvlogger = CsvLogger(filename=filename, header=header)


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class Mavic(Robot):
    verticalThrust = 68.5
    xPIDParameter = [2, 2, 4]
    yPIDParameter = [1.5, 2, 3]
    altitudePIDParameter = [4, 0.05, 10]
    rollPIDParameter = [45, 1, 7]
    pitchPIDParameter = [35, 1, 7]
    yawPIDParameter = [0.5, 0.0075, 3]

    counter = 0

    xTarget = 0.0
    yTarget = 0.0
    yawTarget = 0.0
    altitudeTarget = 0.0

    xTargetAruco = 0.0
    yTargetAruco = 0.0

    rollAngleGimbal = 0.0
    pitchAngleGimbal = 0.0
    yawAngleGimbal = 0.0

    statusTakeoff = False
    statusLanding = False
    statusGimbal = False
    statusHome = False
    statusAruco = False

    statusHomeA = False
    statusHomeB = False
    statusHomeC = False

    yawPID = PID(float(yawPIDParameter[0]), float(yawPIDParameter[1]), float(yawPIDParameter[2]), setpoint=float(yawTarget))
    altiPID = PID(float(altitudePIDParameter[0]), float(altitudePIDParameter[1]), float(altitudePIDParameter[2]), setpoint=float(altitudeTarget))

    yawPID.output_limits = (-0.5, 0.5)
    altiPID.output_limits = (-1.5, 1.5)

    def __init__(self):
        Robot.__init__(self)
        self.timeStep = int(self.getBasicTimeStep())

        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)

        self.camera = self.getDevice("camera")
        self.camera.enable(self.timeStep)
        
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.timeStep)
        
        self.gps = self.getDevice("gps")
        self.gps.enable(self.timeStep)
        
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.timeStep)

        self.receiver = self.getDevice("receiver")
        self.receiver.enable(self.timeStep)

        self.motorFrontLeft = self.getDevice("front left propeller")
        self.motorFrontRight = self.getDevice("front right propeller")
        self.motorRearLeft = self.getDevice("rear left propeller")
        self.motorRearRight = self.getDevice("rear right propeller")
        motors = [self.motorFrontLeft, self.motorFrontRight, self.motorRearLeft, self.motorRearRight]
        for motor in motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(1)

        self.gimbalRoll = self.getDevice("camera roll")
        self.gimbalPitch = self.getDevice("camera pitch")
        self.gimbalYaw = self.getDevice("camera yaw")

    # for opencv 4.6.0 and below
    def find_aruco(self, image):
        self.image = image
        self.gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters_create()
        self.markerCorner, self.id, self.reject = aruco.detectMarkers(self.gray, self.aruco_dict, parameters=self.parameters)
        return self.markerCorner, self.id, self.reject

    def run(self, show=False, log=False, save=False):
        counter = 0

        videoWriter = 0
        if save is True:
            fourcc = cv2.VideoWriter_fourcc("X", "V", "I", "D")
            videoWriter = cv2.VideoWriter("./video.avi", fourcc, 20, (self.camera.getWidth(), self.camera.getHeight()))

        while self.step(self.timeStep) != -1:
            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            rollAccel, pitchAccel, yawAccel = self.gyro.getValues()
            xPosition, yPosition, altitude = self.gps.getValues()
            image = np.frombuffer(self.camera.getImage(), dtype=np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4))

            key = self.keyboard.getKey()

            # Receive data from ground robot
            while self.receiver.getQueueLength() > 0:
                data = self.receiver.getString()
                message = json.loads(data)
                print(message)
                self.receiver.nextPacket()

            # Command
            ## takeoff & landing
            if key == ord("T"):
                self.statusTakeoff = True
                self.statusGimbal = True
                self.altitudeTarget = 3.0
                self.pitchAngleGimbal = 1.6
                print("Takeoff")
                sleep(0.15)
            
            ## landing
            elif key == ord("L"):
                self.statusLanding = True
                self.altitudeTarget = 0.1
                print("Landing")
                sleep(0.15)
            
            ## gimbal
            elif key == ord("G"):
                self.statusGimbal = not self.statusGimbal
                if self.statusGimbal == True:
                    self.rollAngleGimbal = 0.0
                    self.pitchAngleGimbal = 1.6
                    self.yawAngleGimbal = 0.0
                print("Gimbal Stabilize", self.statusGimbal)
                sleep(0.15)

            # moving
            elif key == ord("W"):
                self.xTarget += 0.1
                print("target x:{: .2f}[m]".format(self.xTarget))
            elif key == ord("S"):
                self.xTarget -= 0.1
                print("target x:{: .2f}[m]".format(self.xTarget))
            elif key == ord("A"):
                self.yTarget += 0.1
                print("target y:{: .2f}[m]".format(self.yTarget))
            elif key == ord("D"):
                self.yTarget -= 0.1
                print("target y:{: .2f}[m]".format(self.yTarget))
            elif key == (Keyboard.SHIFT + ord("W")):
                self.altitudeTarget += 0.05
                print("target altitude:{: .2f}[m]".format(self.altitudeTarget))
            elif key == (Keyboard.SHIFT + ord("S")):
                self.altitudeTarget -= 0.05
                print("target altitude:{: .2f}[m]".format(self.altitudeTarget))
            elif key == (Keyboard.SHIFT + ord("A")):
                self.yawTarget += 0.05
                print("target yaw:{: .2f}[rad]".format(self.yawTarget))
            elif key == (Keyboard.SHIFT + ord("D")):
                self.yawTarget -= 0.05
                print("target yaw:{: .2f}[rad]".format(self.yawTarget))
                
            ## home
            elif key == ord("H"):
                self.statusHome = not self.statusHome
                if self.statusHome == True:
                    self.xTarget = 0.0
                    self.yTarget = 0.0
                    self.yawTarget = 0.0
                    self.altitudeTarget = 10.0
                print("Status Home:", self.statusHome)
                sleep(0.15)
            
            ## aruco
            elif key == ord("M"):
                self.statusAruco = not self.statusAruco
                print("Status Aruco:", self.statusAruco)
                sleep(0.15)
            
            ## rth
            elif key == ord("R"):
                self.statusHomeA = not self.statusHomeA
                self.statusAruco = not self.statusAruco
                if self.statusHomeA == True:
                    self.yawTarget = 0.0
                    self.altitudeTarget = 20.0
                print("Return To Home:", self.statusHomeA)
                sleep(0.15)

            if self.statusHomeA == True:
                altitudeError = altitude - self.altitudeTarget
                if altitudeError < 0.25 and altitudeError > -0.25:
                    self.xTarget = 0.0
                    self.yTarget = 0.0

            cameraHeight = int(self.camera.getHeight())
            cameraWidth = int(self.camera.getWidth())

            if self.statusAruco == True:
                markerCorner, markerId, _ = self.find_aruco(image=image)
                if markerId is not None:
                    image = cv2.line(image, (int(cameraWidth / 2), cameraHeight), (int(cameraWidth / 2), 0), (255, 255, 0), 1)
                    image = cv2.line(image, (0, int(cameraHeight / 2)), (cameraWidth, int(cameraHeight / 2)), (255, 255, 0), 1)

                    for(markerCorner, markerId) in zip(markerCorner, markerId):
                        corners = markerCorner.reshape((4, 2))
                        (topLeft, topRight, bottomRight, bottomLeft) = corners

                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))

                    centerX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    centerY = int((topLeft[1] + bottomRight[1]) / 2.0)

                    shapes = image.copy()
                    cv2.rectangle(shapes, topLeft, bottomRight, (0, 255, 0), -1)
                    alpha = 0.4
                    image = cv2.addWeighted(shapes, alpha, image, 1 - alpha, 0)
                    
                    self.xTargetAruco = -4 * ((centerY - (cameraHeight / 2)) / cameraHeight)
                    self.yTargetAruco = 4 * ((centerX - (cameraWidth / 2)) / cameraWidth)
                    rollError = clamp(-self.yTargetAruco + 0.06, -1.0, 1.0)
                    pitchError = clamp(self.xTargetAruco - 0.13, -1.0, 1.0)

                    altitudeError = altitude - self.altitudeTarget
                    #if ((self.xTargetAruco < 0.1 and self.xTargetAruco > -0.1) and (self.yTargetAruco < 0.1 and self.yTargetAruco > -0.1) and (altitudeError < 0.5 and altitudeError > -0.5) and self.statusLanding == False):
                    #    self.statusLanding = True
                    #    self.altitudeTarget = 0.0
                    #    print("Landing")
                else:
                    rollError = clamp(-yPosition + 0.06 + self.yTarget, -1.0, 1.0)
                    pitchError = clamp(-xPosition - 0.13 + self.xTarget, -1.0, 1.0)
            else:
                rollError = clamp(-yPosition + 0.06 + self.yTarget, -1.0, 1.0)
                pitchError = clamp(-xPosition - 0.13 + self.xTarget, -1.0, 1.0)

            self.yawPID.setpoint = self.yawTarget
            self.altiPID.setpoint = self.altitudeTarget

            yawInput = self.yawPID(yaw)
            verticalInput = self.altiPID(altitude)
            
            rollInput = (self.rollPIDParameter[0] * clamp(roll, -1.0, 1.0)) + (self.rollPIDParameter[2] * rollAccel) + rollError
            pitchInput = (self.pitchPIDParameter[0] * clamp(pitch, -1.0, 1.0)) + (self.pitchPIDParameter[2] * pitchAccel) - pitchError
            
            motorFrontLeftInput = self.verticalThrust + verticalInput - yawInput + pitchInput - rollInput
            motorFrontRightInput = self.verticalThrust + verticalInput + yawInput + pitchInput + rollInput
            motorRearLeftInput = self.verticalThrust + verticalInput + yawInput - pitchInput - rollInput
            motorRearRightInput = self.verticalThrust + verticalInput - yawInput - pitchInput + rollInput

            if self.statusTakeoff == False or (self.statusLanding == True and altitude <= 0.1):
                self.statusTakeoff = False
                self.statusLanding = False
                self.statusGimbal = False
                self.statusHome = False
                motorFrontLeftInput = 0.0
                motorFrontRightInput = 0.0
                motorRearLeftInput = 0.0
                motorRearRightInput = 0.0

            self.motorFrontLeft.setVelocity(motorFrontLeftInput)
            self.motorFrontRight.setVelocity(-motorFrontRightInput)
            self.motorRearLeft.setVelocity(-motorRearLeftInput)
            self.motorRearRight.setVelocity(motorRearRightInput)

            if self.statusGimbal == True:
                rollGimbalInput = clamp((-0.001 * rollAccel + self.rollAngleGimbal), -0.5, 0.5)
                pitchGimbalInput = clamp(((-0.001 * pitchAccel) + self.pitchAngleGimbal), -0.5, 1.7)
                yawGimbalInput = clamp((-0.001 * yawAccel + self.yawAngleGimbal), -1.7, 1.7)
                self.gimbalRoll.setPosition(rollGimbalInput)
                self.gimbalPitch.setPosition(pitchGimbalInput)
                self.gimbalYaw.setPosition(yawGimbalInput)

            speed = self.gps.getSpeedVector()

            logs = [
                roll,
                pitch,
                yaw,
                rollAccel,
                pitchAccel,
                yawAccel,
                xPosition,
                yPosition,
                altitude,
                rollError,
                pitchError,
                rollInput,
                pitchInput,
                yawInput,
                verticalInput,
                motorFrontLeftInput,
                motorFrontRightInput,
                motorRearLeftInput,
                motorRearRightInput,
                speed[0],
                speed[1],
                speed[2],
                self.statusTakeoff,
                self.statusHome,
                self.statusAruco,
                self.statusLanding,
            ]

            dlogs = []
            for i in logs:
                dlogs.append(float("{:.2f}".format(i)))

            debug_mode = show
            if debug_mode == True:
                print(
                    "r={:+.2f}|p={:+.2f}|y={:+.2f}|ra={:+.2f}|pa={:+.2f}|ya={:+.2f}|x={:+.2f}|y={:+.2f}|z={:+.2f}|re={:+.2f}|pe={:+.2f}|ri={:+.2f}|pi={:+.2f}|yi={:+.2f}|vi={:+.2f}|fl={:+.2f}|fr={:+.2f}|rl={:+.2f}|rr={:+.2f}|sx={:+.2f}|sy={:+.2f}|sz={:+.2f}|st={}|sh={}|sa={}|sl={}".format(
                        dlogs[0],
                        dlogs[1],
                        dlogs[2],
                        dlogs[3],
                        dlogs[4],
                        dlogs[5],
                        dlogs[6],
                        dlogs[7],
                        dlogs[8],
                        dlogs[9],
                        dlogs[10],
                        dlogs[11],
                        dlogs[12],
                        dlogs[13],
                        dlogs[14],
                        dlogs[15],
                        dlogs[16],
                        dlogs[17],
                        dlogs[18],
                        dlogs[19],
                        dlogs[20],
                        dlogs[21],
                        int(dlogs[22]),
                        int(dlogs[23]),
                        int(dlogs[24]),
                        int(dlogs[25]),
                    )
                )

            cv2.imshow("Camera", image)
            if save is True:
                videoWriter.write(image)
            cv2.waitKey(1)

            log_mode = log
            if log_mode == True:
                dlogs.insert(0, counter)
                csvlogger.critical(dlogs)

            counter += 1
        if save is True:
            videoWriter.release()
        cv2.destroyAllWindows()


robot = Mavic()
robot.run(show=False, log=False, save=False)