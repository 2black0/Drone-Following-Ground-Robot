from controller import Robot, Keyboard, Emitter
import json
from csv_logger import CsvLogger

filename = "logger_mobile.csv"
header = [
    "time",
    "counter",
    "xPosition",
    "yPosition",
    "altitude",
]
csvlogger = CsvLogger(filename=filename, header=header)

def run_robot(robot):
    counter = 0
    log = True
    
    timestep = int(robot.getBasicTimeStep())
    maxSpeed = 6.28
    multiplier = 1.0
    multiplier_turn = 0.25
    
    # Keyboard
    robot.keyboard = robot.getKeyboard()
    robot.keyboard.enable(10 * timestep)
    
    # Motor
    wheelFrontLeft = robot.getDevice("wheel1")
    wheelFrontRight = robot.getDevice("wheel2")
    wheelRearLeft = robot.getDevice("wheel3")
    wheelRearRight = robot.getDevice("wheel4")
    
    wheelFrontLeft.setPosition(float('inf'))
    wheelFrontRight.setPosition(float('inf'))
    wheelRearLeft.setPosition(float('inf'))
    wheelRearRight.setPosition(float('inf'))
    
    wheelFrontLeft.setVelocity(0.0)
    wheelFrontRight.setVelocity(0.0)
    wheelRearLeft.setVelocity(0.0)
    wheelRearRight.setVelocity(0.0)
      
    # GPS
    gps = robot.getDevice("gps_ground")
    gps.enable(timestep)
    
    # Accelerometer
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
      
    # Distance Sensor
    dsLeft = robot.getDevice("ds_left")
    dsRight = robot.getDevice("ds_right")
    dsLeft.enable(timestep)
    dsRight.enable(timestep)
    
    # Emitter
    emitter = robot.getDevice("emitter_ground")
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        roll, pitch, yaw = imu.getRollPitchYaw()
        xPosition, yPosition, altitude = gps.getValues()
        #print("xPosition={:+.2f}|yPosition={:+.2f}|altitude={:+.2f}".format(xPosition, yPosition, altitude))        

        xPositionSend = int(xPosition * 100)
        yPositionSend = int(yPosition * 100)
        altitudeSend = int(altitude  * 100)
        
        message = {'yaw': yaw, 'xPosition': xPositionSend, 'yPosition': yPositionSend, 'altitude': altitudeSend}
        emitter.send(json.dumps(message))
        
        key = robot.keyboard.getKey()
        
        if key == ord("1"):            
            wheelFrontLeft.setVelocity(maxSpeed * multiplier)
            wheelFrontRight.setVelocity(maxSpeed * multiplier)
            wheelRearLeft.setVelocity(maxSpeed * multiplier)
            wheelRearRight.setVelocity(maxSpeed * multiplier)
        if key == ord("2"):            
            wheelFrontLeft.setVelocity(maxSpeed * multiplier_turn)
            wheelFrontRight.setVelocity(maxSpeed * multiplier)
            wheelRearLeft.setVelocity(maxSpeed * multiplier_turn)
            wheelRearRight.setVelocity(maxSpeed * multiplier)
        if key == ord("3"):            
            wheelFrontLeft.setVelocity(maxSpeed * multiplier)
            wheelFrontRight.setVelocity(maxSpeed * multiplier_turn)
            wheelRearLeft.setVelocity(maxSpeed * multiplier)
            wheelRearRight.setVelocity(maxSpeed * multiplier_turn)
        elif key == ord("4"):
            wheelFrontLeft.setVelocity(0.0)
            wheelFrontRight.setVelocity(0.0)
            wheelRearLeft.setVelocity(0.0)
            wheelRearRight.setVelocity(0.0)
        
        
        logs = [
                xPosition,
                yPosition,
                altitude,
            ]
        
        dlogs = []
        for i in logs:
            dlogs.append(float("{:.2f}".format(i)))
        
        log_mode = log
        if log_mode == True:
            dlogs.insert(0, counter)
            csvlogger.critical(dlogs)

            counter += 1

if __name__ == "__main__":
    # create the Robot instance.
    my_robot = Robot()
    run_robot(my_robot)