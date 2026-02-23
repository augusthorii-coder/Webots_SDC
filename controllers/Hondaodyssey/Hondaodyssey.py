"""
Honda Odyssey is the cars name
August Horii
ATCS PROJECT for a self driving car
"""
import math
from vehicle import Driver
from controller import Keyboard, Lidar, Camera




#Constants
TIME_STEP = 10
MAX_SPEED = 150.0
TURN_ANGLE = 0.5

#Initialization
driver = Driver()
driver.setSteeringAngle(0.0)
driver.setCruisingSpeed(0.0)

keyboard = Keyboard()
keyboard.enable(TIME_STEP)


#Lidar
lidar = driver.getDevice("lidar") 
if lidar is not None:
    lidar.enable(TIME_STEP)
    lidar.enablePointCloud()
else:
    print("Twin your lidar is NOT there")

camera = driver.getDevice("camera")
if camera is not None:
    camera.enable(TIME_STEP)
else:
    print("Your camera is NOT plugged in Twin")

#to start in manual mode:
current_speed = 0.0
autodrive = False

print("Press A to start the AUTOPILOT")

#Main Loop
while driver.step() != -1:
    key = keyboard.getKey()

    
    current_steering = 0.0
    #Identifying keys
    up = False; down = False; right = False; left = False  
    
    while key != -1:
        #Adding purpose to these delinquites
        if key == Keyboard.UP: up = True
        if key == Keyboard.DOWN: down = True
        if key == Keyboard.LEFT: left = True
        if key == Keyboard.RIGHT: right = True
        #Toggle on and off for the autopilot:
        if key == ord('A'): autodrive = not autodrive
        key = keyboard.getKey()
    
    if not autodrive:
        #boring manual coding part
        if up:
            current_speed += 5
            if current_speed > MAX_SPEED:
                current_speed = MAX_SPEED
        elif down:
            current_speed -= 5
            if current_speed < -MAX_SPEED:
                current_speed = -MAX_SPEED
        else:
            current_speed *= 0.9
            if abs(current_speed) < 0.1:
                current_speed = 0.0 
    
        
        if left:
            current_steering = -TURN_ANGLE
        elif right:
            current_steering = TURN_ANGLE
    
    else:
        #the AUTOPILOT area
        current_speed = 30.0 # Constant cruising speed
            
        if camera is not None:
            image = camera.getImageArray()
            width = camera.getWidth()
            height = camera.getHeight()
            left_line_pixels = 0
            right_line_pixels = 0
            y = int(height * 0.8)
            
            for x in range(width):
                #Finding the pixel colors:
                r, g, b = image[x][y]
                brightness = r + g + b
                
                #if the pixels are very bright:
                if brightness > 600: 
                    if x < width / 2:
                        left_line_pixels += 1
                    else:
                        right_line_pixels += 1
            
            #left lane keeping logic
            if left_line_pixels > right_line_pixels: 
                current_steering = 0.15 #steer right
            elif right_line_pixels > left_line_pixels:
                current_steering = -0.15 #steer left
            else:
                current_steering = 0.0 #go straight
     
     
        
    driver.setCruisingSpeed(current_speed)
    driver.setSteeringAngle(current_steering)
       
       
"""
    elif up and right:
        current_steering = TURN_ANGLE
        current_speed = MAX_SPEED
    elif up and left:
        current_steering = -TURN_ANGLE
        current_speed = MAX_SPEED
    elif down and right:
        current_steering = TURN_ANGLE
        current_speed = -MAX_SPEED
    elif down and left:
        current_steering = -TURN_ANGLE
        current_speed = -MAX_SPEED

    
    # Check speed (UP/DOWN)
    if key == Keyboard.UP:
        current_speed = MAX_SPEED
    elif key == Keyboard.DOWN: 
        current_speed = -MAX_SPEED
        
    # Check steering (LEFT/RIGHT)
    if key == Keyboard.LEFT:
        current_steering = -TURN_ANGLE
    elif key == Keyboard.RIGHT:
        current_steering = TURN_ANGLE
"""
       
       

        