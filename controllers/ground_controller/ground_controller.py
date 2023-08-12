from controller import Robot, Keyboard, Emitter
import json

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28
    
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
        xPosition, yPosition, altitude = gps.getValues()
        print("xPosition={:+.2f}|yPosition={:+.2f}".format(xPosition, yPosition))        

        xPositionSend = int(xPosition * 100)
        yPositionSend = int(yPosition * 100)
        
        message = {'xPosition': xPositionSend, 'yPosition': yPositionSend}
        emitter.send(json.dumps(message))
        
        key = robot.keyboard.getKey()
        
        if key == ord("1"):            
            wheelFrontLeft.setVelocity(max_speed * 0.5)
            wheelFrontRight.setVelocity(max_speed * 0.5)
            wheelRearLeft.setVelocity(max_speed * 0.5)
            wheelRearRight.setVelocity(max_speed * 0.5)
        elif key == ord("2"):
            wheelFrontLeft.setVelocity(0.0)
            wheelFrontRight.setVelocity(0.0)
            wheelRearLeft.setVelocity(0.0)
            wheelRearRight.setVelocity(0.0)    

if __name__ == "__main__":
    # create the Robot instance.
    my_robot = Robot()
    run_robot(my_robot)