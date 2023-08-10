from controller import Robot, Keyboard

def run_robot(robot):
    timestep = 32
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
    gps = robot.getDevice("gps")
    gps.enable(timestep)
      
    # Distance Sensor
    dsLeft = robot.getDevice("ds_left")
    dsRight = robot.getDevice("ds_right")
    dsLeft.enable(timestep)
    dsRight.enable(timestep)
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        gps_value = gps.getValues()
        #print(gps_value)
        
        key = robot.keyboard.getKey()
        
        msg = "GPS_Value:"
        for each_val in gps_value:
            msg += " {0:0.5f}".format(each_val)
        #print(msg)
        
        if key == ord("Q"):
            wheelFrontLeft.setVelocity(max_speed * 0.25)
            wheelFrontRight.setVelocity(max_speed * 0.25)
            wheelRearLeft.setVelocity(max_speed * 0.25)
            wheelRearRight.setVelocity(max_speed * 0.25)
        elif key == ord("Z"):
            wheelFrontLeft.setVelocity(0.0)
            wheelFrontRight.setVelocity(0.0)
            wheelRearLeft.setVelocity(0.0)
            wheelRearRight.setVelocity(0.0)
        
    

if __name__ == "__main__":
    # create the Robot instance.
    my_robot = Robot()
    run_robot(my_robot)